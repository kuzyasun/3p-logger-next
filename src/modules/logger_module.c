#include "logger_module.h"

#include <esp_system.h>
#include <esp_timer.h>
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

    if (xQueueSend(m->buffer_queue, &chunk, pdMS_TO_TICKS(10)) != pdTRUE) {
        // drop: черга забита -> не блокуємось
        ++m->dropped_chunks;
    }

    // xQueueSend(m->buffer_queue, &chunk, portMAX_DELAY);

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

// -------------------- логування за запитом --------------------

void logger_module_trigger_snapshot(logger_module_t *module, uint8_t piezo_mask) {
    if (!module->initialized || app_state_get_instance()->current_mode != APP_MODE_LOGGING) {
        return;
    }

    app_state_t *state = app_state_get_instance();
    log_data_snapshot_t snapshot;

    snapshot.timestamp_us = esp_timer_get_time();
    snapshot.accel_x = state->accel_x;
    snapshot.accel_y = state->accel_y;
    snapshot.accel_z = state->accel_z;
    snapshot.piezo_mask = piezo_mask;

    for (int i = 0; i < module->log_map_count; i++) {
        log_param_map_t *entry = &module->log_map[i];
        const void *ptr = ((const uint8_t *)state) + entry->offset;

        switch (entry->type) {
            case LOG_DTYPE_INT16:
                snapshot.telemetry_values[i].val_i16 = *(const int16_t *)ptr;
                break;
            case LOG_DTYPE_INT32:
                snapshot.telemetry_values[i].val_i32 = *(const int32_t *)ptr;
                break;
            case LOG_DTYPE_FLOAT:
                snapshot.telemetry_values[i].val_f = *(const float *)ptr;
                break;
            default:
                memset(&snapshot.telemetry_values[i], 0, sizeof(log_snapshot_value_u));
                break;
        }
    }

    if (module->data_queue) {
        xQueueSend(module->data_queue, &snapshot, pdMS_TO_TICKS(5));
    }
}

// -------------------- Task for writing to SD --------------------

static void logger_sd_write_task(void *arg) {
    logger_module_t *module = (logger_module_t *)arg;

    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Logger task started with NULL buffer queue. Task exiting.");
        vTaskDelete(NULL);
        return;
    }

    logger_chunk_t chunk;
    int chunks_since_sync = 0;
    while (xQueueReceive(module->buffer_queue, &chunk, portMAX_DELAY) == pdTRUE) {
        if (module->sd_card_ok && chunk.size > 0) {
            int written = sdcard_write(module->log_file, chunk.data, chunk.size);
            if (written != (int)chunk.size) LOG_E(TAG, "Write fail %d/%d", written, (int)chunk.size);
            LOG_I(TAG, "Write %d", written);
            if (++chunks_since_sync >= 8) {
                LOG_I(TAG, "Sync");
                fflush((FILE *)module->log_file);
                sdcard_fsync(module->log_file);
                chunks_since_sync = 0;
                vTaskDelay(1);  // yield
            }
        }
    }
}

// -------------------- задача формування CSV --------------------

static void logger_processing_task(void *arg) {
    logger_module_t *module = (logger_module_t *)arg;
    log_data_snapshot_t snapshot;

    while (1) {
        if (xQueueReceive(module->data_queue, &snapshot, portMAX_DELAY) == pdTRUE) {
            char line[512];
            int offset = 0;

            offset += snprintf(line + offset, sizeof(line) - offset, "%llu,%d,%d,%d,%u", snapshot.timestamp_us, snapshot.accel_x, snapshot.accel_y,
                               snapshot.accel_z, snapshot.piezo_mask);

            for (int i = 0; i < module->log_map_count; i++) {
                log_param_map_t *entry = &module->log_map[i];

                if (offset >= (int)sizeof(line) - 32) break;

                offset += snprintf(line + offset, sizeof(line) - offset, ",");

                switch (entry->type) {
                    case LOG_DTYPE_INT16:
                        offset += snprintf(line + offset, sizeof(line) - offset, "%d", snapshot.telemetry_values[i].val_i16);
                        break;
                    case LOG_DTYPE_INT32:
                        offset += snprintf(line + offset, sizeof(line) - offset, "%ld", snapshot.telemetry_values[i].val_i32);
                        break;
                    case LOG_DTYPE_FLOAT:
                        offset += snprintf(line + offset, sizeof(line) - offset, "%.3f", snapshot.telemetry_values[i].val_f);
                        break;
                    default:
                        break;
                }
            }

            offset += snprintf(line + offset, sizeof(line) - offset, "\r\n");
            logger_append_bytes(module, (const uint8_t *)line, offset);
        }
    }
}

// -------------------- Initialization --------------------

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

    int offset = 0;
    size_t header_buf_size = sizeof(module->csv_header);

    offset += snprintf(module->csv_header + offset, header_buf_size - offset, "timestamp_us,accel_x,accel_y,accel_z,piezo_mask");

    for (int i = 0; i < g_app_config.telemetry_params_count; ++i) {
        if (module->log_map_count >= MAX_LOG_TELEMETRY_PARAMS) {
            LOG_W(TAG, "Reached max log telemetry params limit (%d).", MAX_LOG_TELEMETRY_PARAMS);
            break;
        }

        if (offset >= (int)header_buf_size - 32) {
            LOG_W(TAG, "CSV header buffer is full, some telemetry fields may be skipped.");
            break;
        }

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

        offset += snprintf(module->csv_header + offset, header_buf_size - offset, ",%s", name);

        module->dynamic_record_size += entry->size;
        module->log_map_count++;
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
    hal_err_t err = sdcard_open_file(log_file_path, "a", &module->log_file);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to create log file %s", log_file_path);
        module->sd_card_ok = false;
        return err;
    }

    setvbuf((FILE *)module->log_file, NULL, _IONBF, 0);

    char file_header[1024];
    int len = snprintf(file_header, sizeof(file_header), "# Firmware: %s (git %s), built %s\r\n# Fields: %s\r\n", FIRMWARE_VERSION, GIT_HASH, BUILD_TIMESTAMP,
                       module->csv_header);
    int written = sdcard_write(module->log_file, file_header, len);
    if (written != len) {
        LOG_E(TAG, "HEADER WRITE FAILED! Tried to write %d bytes, but wrote %d.", len, written);
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    if (fflush((FILE *)module->log_file) != 0) {
        LOG_E(TAG, "HEADER fflush failed");
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    hal_err_t sync_status = sdcard_fsync(module->log_file);
    if (sync_status != HAL_ERR_NONE) {
        LOG_E(TAG, "HEADER FSYNC FAILED! Error code: %d", sync_status);
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    // Queue for data snapshots
    module->data_queue = xQueueCreate(16, sizeof(log_data_snapshot_t));
    if (module->data_queue == NULL) {
        LOG_E(TAG, "Failed to create data queue. Logger disabled.");
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    // Queue for chunks to write to SD
    module->buffer_queue = xQueueCreate(1, sizeof(logger_chunk_t));
    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Failed to create buffer queue. Logger disabled.");
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    module->active_buffer = module->ping_buffer;
    module->active_buffer_idx = 0;

    LOG_I(TAG, "Logger initialized. CSV data to %s", log_file_path);
    module->initialized = true;
    return HAL_ERR_NONE;
}

void logger_module_create_task(logger_module_t *module) {
    BaseType_t status1 =
        xTaskCreatePinnedToCore(logger_processing_task, "LOG_PROC", 4096, module, TASK_PRIORITY_LOGGER - 1, &module->processing_task_handle, 0);
    if (status1 != pdPASS) {
        LOG_E(TAG, "Failed to create LOG_PROC task!");
        return;
    }

    BaseType_t status2 = xTaskCreatePinnedToCore(logger_sd_write_task, "LOG_WRITE", 4096, module, TASK_PRIORITY_LOGGER, &module->writer_task_handle, 0);
    if (status2 != pdPASS) {
        LOG_E(TAG, "Failed to create LOG_WRITE task!");
        return;
    }

    LOG_I(TAG, "Logger tasks created successfully.");
}
