#include "logger_module.h"

#include <esp_timer.h>  // для мс timestamp
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "app_state.h"
#include "build_info.h"
#include "log.h"
#include "target.h"

static const char *TAG = "LOGR";

static hal_err_t copy_config_file(const char *dest_path);
static void build_log_plan(logger_module_t *module);

// -------------------- helpers для буферизації --------------------

static inline void logger_flush_active_buffer(logger_module_t *m) {
    if (!m->sd_card_ok || m->active_buffer_idx == 0) return;
    logger_chunk_t chunk = {
        .data = m->active_buffer,
        .size = m->active_buffer_idx,
    };
    // Віддати заповнений пінг/понг у чергу на запис
    xQueueSend(m->buffer_queue, &chunk, portMAX_DELAY);
    // Перемикаємо буфер
    m->active_buffer = (m->active_buffer == m->ping_buffer) ? m->pong_buffer : m->ping_buffer;
    m->active_buffer_idx = 0;
}

static inline void logger_append_bytes(logger_module_t *m, const uint8_t *data, size_t len) {
    if (!m->sd_card_ok || len == 0) return;
    if (len > LOGGER_BUFFER_SIZE) {
        // дуже великий шматок — пишемо порціями
        size_t off = 0;
        while (off < len) {
            size_t chunk = len - off;
            if (chunk > LOGGER_BUFFER_SIZE) chunk = LOGGER_BUFFER_SIZE;
            memcpy(m->active_buffer, data + off, chunk);
            m->active_buffer_idx = chunk;
            logger_flush_active_buffer(m);
            off += chunk;
        }
        return;
    }
    if (m->active_buffer_idx + len > LOGGER_BUFFER_SIZE) {
        logger_flush_active_buffer(m);
    }
    memcpy(m->active_buffer + m->active_buffer_idx, data, len);
    m->active_buffer_idx += len;
}

static inline void logger_append_crlf(logger_module_t *m) {
    static const uint8_t crlf[2] = {'\r', '\n'};
    logger_append_bytes(m, crlf, 2);
}

// -------------------- формування CSV-рядка --------------------

static void write_csv_line(logger_module_t *module) {
    if (!module->sd_card_ok) return;

    app_state_t *state = app_state_get_instance();

    char line[512];
    int offset = 0;

    uint64_t tms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    offset += snprintf(line + offset, sizeof(line) - offset, "%llu,%d,%d,%d,%u", tms, state->accel_x, state->accel_y, state->accel_z, state->piezo_mask);

    for (int i = 0; i < module->log_map_count; i++) {
        log_param_map_t *entry = &module->log_map[i];
        const void *ptr = ((const uint8_t *)state) + entry->offset;

        if (offset >= (int)sizeof(line) - 32) break;

        offset += snprintf(line + offset, sizeof(line) - offset, ",");

        switch (entry->type) {
            case LOG_DTYPE_INT16:
                offset += snprintf(line + offset, sizeof(line) - offset, "%d", *(const int16_t *)ptr);
                break;
            case LOG_DTYPE_INT32:
                offset += snprintf(line + offset, sizeof(line) - offset, "%ld", *(const int32_t *)ptr);
                break;
            case LOG_DTYPE_FLOAT:
                offset += snprintf(line + offset, sizeof(line) - offset, "%.3f", *(const float *)ptr);
                break;
            default:
                break;
        }
    }

    offset += snprintf(line + offset, sizeof(line) - offset, "\r\n");

    logger_append_bytes(module, (const uint8_t *)line, offset);

    if (module->active_buffer_idx >= LOGGER_BUFFER_SIZE) {
        logger_flush_active_buffer(module);
    }
}

// -------------------- реакція на зміни стану --------------------

static void on_app_state_change(Notifier *notifier, Observer *observer, void *data) {
    logger_module_t *module = (logger_module_t *)observer->context;
    uint64_t *changed_mask = (uint64_t *)data;

    // Тригеримося на акселі/п’єзо — як і було задумано
    uint64_t trigger_mask = APP_STATE_FIELD_ACCEL_X | APP_STATE_FIELD_ACCEL_Y | APP_STATE_FIELD_ACCEL_Z | APP_STATE_FIELD_PIEZO_MASK;

    if (!(*changed_mask & trigger_mask)) {
        return;
    }
    write_csv_line(module);
}

// -------------------- задача запису на SD --------------------

static void logger_task(void *arg) {
    logger_module_t *module = (logger_module_t *)arg;

    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Logger task started with NULL buffer queue. Task exiting.");
        vTaskDelete(NULL);
        return;
    }

    logger_chunk_t chunk;
    while (1) {
        if (xQueueReceive(module->buffer_queue, &chunk, portMAX_DELAY) == pdTRUE) {
            if (module->sd_card_ok && chunk.size > 0) {
                int written = sdcard_write(module->log_file, chunk.data, chunk.size);
                if (written != (int)chunk.size) {
                    LOG_E(TAG, "Failed to write log data (%d/%d)", written, (int)chunk.size);
                }
                sdcard_fsync(module->log_file);
            }
        }
    }
}

// -------------------- ініціалізація --------------------

static hal_err_t copy_config_file(const char *dest_path) {
    FILE *src = fopen(SD_MOUNT_PATH "/configuration.ini", "r");
    if (!src) {
        return HAL_ERR_FAILED;
    }
    FILE *dst = fopen(dest_path, "w");
    if (!dst) {
        fclose(src);
        return HAL_ERR_FAILED;
    }
    char buffer[128];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), src)) > 0) {
        fwrite(buffer, 1, bytes, dst);
    }
    fclose(src);
    fclose(dst);
    return HAL_ERR_NONE;
}

static void build_log_plan(logger_module_t *module) {
    app_state_t *state = app_state_get_instance();
    module->log_map_count = 0;
    module->dynamic_record_size = 0;

    strcpy(module->csv_header, "timestamp_ms,accel_x,accel_y,accel_z,piezo_mask");

    for (int i = 0; i < g_app_config.telemetry_params_count; ++i) {
        const char *name = g_app_config.telemetry_params[i];
        log_param_map_t *entry = &module->log_map[module->log_map_count];
        entry->type = LOG_DTYPE_UNKNOWN;

        if (strcmp(name, "plane_speed") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_SPEED;
            entry->offset = offsetof(app_state_t, plane_speed);
            entry->size = sizeof(state->plane_speed);
            entry->type = LOG_DTYPE_INT16;
        } else if (strcmp(name, "plane_fused_altitude") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_FUSED_ALTITUDE;
            entry->offset = offsetof(app_state_t, plane_fused_altitude);
            entry->size = sizeof(state->plane_fused_altitude);
            entry->type = LOG_DTYPE_INT32;
        } else if (strcmp(name, "plane_baro_altitude") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_BARO_ALTITUDE;
            entry->offset = offsetof(app_state_t, plane_baro_altitude);
            entry->size = sizeof(state->plane_baro_altitude);
            entry->type = LOG_DTYPE_INT16;
        } else if (strcmp(name, "plane_vspeed") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_VSPEED;
            entry->offset = offsetof(app_state_t, plane_vspeed);
            entry->size = sizeof(state->plane_vspeed);
            entry->type = LOG_DTYPE_INT16;
        } else {
            continue;
        }

        strncpy(entry->name, name, sizeof(entry->name) - 1);
        entry->name[sizeof(entry->name) - 1] = '\0';

        strcat(module->csv_header, ",");
        strcat(module->csv_header, name);

        module->dynamic_record_size += entry->size;
        module->log_map_count++;
        if (module->log_map_count >= MAX_LOG_TELEMETRY_PARAMS) break;
    }
}

hal_err_t logger_module_init(logger_module_t *module, bool is_sd_card_ok) {
    memset(module, 0, sizeof(logger_module_t));

    module->sd_card_ok = is_sd_card_ok;
    if (!module->sd_card_ok) {
        LOG_E(TAG, "SD card not available. Logger disabled.");
        return HAL_ERR_FAILED;
    }

    build_log_plan(module);

    // Папка /N з першим вільним індексом
    int log_num = 0;
    char log_dir_path[32];
    struct stat st;
    while (1) {
        sprintf(log_dir_path, SD_MOUNT_PATH "/%d", log_num);
        if (stat(log_dir_path, &st) != 0) break;
        log_num++;
    }

    if (mkdir(log_dir_path, 0755) != 0) {
        LOG_E(TAG, "Failed to create log directory %s", log_dir_path);
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    // Скопіювати конфіг у лог-папку (як є)
    char config_path[64];
    sprintf(config_path, "%s/configuration.ini", log_dir_path);
    copy_config_file(config_path);

    // Main log file in CSV format
    char log_file_path[64];
    sprintf(log_file_path, "%s/data_%d.csv", log_dir_path, log_num);
    hal_err_t err = sdcard_open_file(log_file_path, "w", &module->log_file);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to create log file %s", log_file_path);
        module->sd_card_ok = false;
        return err;
    }

    char file_header[1024];
    int len = snprintf(file_header, sizeof(file_header), "# Firmware: %s (git %s), built %s\r\n# Fields: %s\r\n", FIRMWARE_VERSION, GIT_HASH, BUILD_TIMESTAMP,
                       module->csv_header);
    sdcard_write(module->log_file, file_header, len);
    sdcard_fsync(module->log_file);

    // Черга під шматки для запису
    module->buffer_queue = xQueueCreate(4, sizeof(logger_chunk_t));
    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Failed to create buffer queue. Logger disabled.");
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    module->active_buffer = module->ping_buffer;
    module->active_buffer_idx = 0;

    // Підписка на зміни стану
    module->observer = ObserverCreate("LOGGER_APP_STATE", module, on_app_state_change, NULL);
    if (module->observer) {
        Notifier *app_state_notifier = app_state_get_notifier();
        if (app_state_notifier) {
            app_state_notifier->subject.attach(app_state_notifier, module->observer);
        }
    }

    LOG_I(TAG, "Logger initialized. CSV data to %s", log_file_path);
    module->initialized = true;
    return HAL_ERR_NONE;
}

void logger_module_create_task(logger_module_t *module) {
    xTaskCreatePinnedToCore(logger_task, "LOGGER", 4096, module, TASK_PRIORITY_LOGGER, &module->task_handle, 0);
}
