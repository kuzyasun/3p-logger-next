#include "logger_module.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "app_state.h"
#include "log.h"
#include "target.h"

static const char *TAG = "LOGR";

static hal_err_t copy_config_file(const char *dest_path);
static void build_log_plan(logger_module_t *module);

static void on_app_state_change(Notifier *notifier, Observer *observer, void *data) {
    logger_module_t *module = (logger_module_t *)observer->context;
    uint64_t *changed_mask = (uint64_t *)data;

    uint64_t trigger_mask = APP_STATE_FIELD_ACCEL_X | APP_STATE_FIELD_ACCEL_Y | APP_STATE_FIELD_ACCEL_Z | APP_STATE_FIELD_PIEZO_MASK;
    if (!(*changed_mask & trigger_mask)) {
        return;
    }
    if (!module->sd_card_ok) {
        return;
    }

    app_state_t *state = app_state_get_instance();
    uint8_t record[module->dynamic_record_size];
    size_t current_offset = 0;

    memcpy(&record[current_offset], &state->accel_x, 2);
    current_offset += 2;
    memcpy(&record[current_offset], &state->accel_y, 2);
    current_offset += 2;
    memcpy(&record[current_offset], &state->accel_z, 2);
    current_offset += 2;
    memcpy(&record[current_offset], &state->piezo_mask, 1);
    current_offset += 1;

    for (int i = 0; i < module->log_map_count; i++) {
        log_param_map_t *entry = &module->log_map[i];
        memcpy(&record[current_offset], (uint8_t *)state + entry->offset, entry->size);
        current_offset += entry->size;
    }

    if (module->active_buffer_idx + module->dynamic_record_size > LOGGER_BUFFER_SIZE) {
        uint8_t *full_buffer = module->active_buffer;
        xQueueSend(module->buffer_queue, &full_buffer, portMAX_DELAY);
        module->active_buffer = (module->active_buffer == module->ping_buffer) ? module->pong_buffer : module->ping_buffer;
        module->active_buffer_idx = 0;
    }

    memcpy(module->active_buffer + module->active_buffer_idx, record, module->dynamic_record_size);
    module->active_buffer_idx += module->dynamic_record_size;

    if (module->active_buffer_idx >= LOGGER_BUFFER_SIZE) {
        uint8_t *full_buffer = module->active_buffer;
        xQueueSend(module->buffer_queue, &full_buffer, portMAX_DELAY);
        module->active_buffer = (module->active_buffer == module->ping_buffer) ? module->pong_buffer : module->ping_buffer;
        module->active_buffer_idx = 0;
    }
}

static void logger_task(void *arg) {
    logger_module_t *module = (logger_module_t *)arg;
    uint8_t *buffer = NULL;

    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Logger task started with NULL buffer queue. Task exiting.");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (xQueueReceive(module->buffer_queue, &buffer, portMAX_DELAY) == pdTRUE) {
            if (module->sd_card_ok) {
                int written = sdcard_write(module->log_file, buffer, LOGGER_BUFFER_SIZE);
                if (written != LOGGER_BUFFER_SIZE) {
                    LOG_E(TAG, "Failed to write log data");
                }
                sdcard_fsync(module->log_file);
            }
        }
    }
}

hal_err_t logger_module_init(logger_module_t *module, hal_spi_bus_t sd_spi_bus) {
    memset(module, 0, sizeof(logger_module_t));

    hal_err_t err = sdcard_init(sd_spi_bus, SD_SPI_CS_GPIO);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize SD card HAL. Logger disabled.");
        module->sd_card_ok = false;
        return err;
    }

    module->sd_card_ok = true;

    int log_num = 0;
    char log_dir_path[32];
    struct stat st;
    while (1) {
        sprintf(log_dir_path, "/sdcard/%d", log_num);
        if (stat(log_dir_path, &st) != 0) {
            break;
        }
        log_num++;
    }

    if (mkdir(log_dir_path, 0755) != 0) {
        LOG_E(TAG, "Failed to create log directory %s", log_dir_path);
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }

    char config_path[64];
    sprintf(config_path, "%s/configuration.ini", log_dir_path);
    copy_config_file(config_path);

    char log_file_path[64];
    sprintf(log_file_path, "%s/%d.log", log_dir_path, log_num);
    err = sdcard_open_file(log_file_path, "w", &module->log_file);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to create log file %s", log_file_path);
        module->sd_card_ok = false;
        return err;
    }

    module->buffer_queue = xQueueCreate(2, sizeof(uint8_t *));
    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Failed to create buffer queue. Logger disabled.");
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }
    module->active_buffer = module->ping_buffer;
    module->active_buffer_idx = 0;

    build_log_plan(module);

    module->observer = ObserverCreate("LOGGER_APP_STATE", module, on_app_state_change, NULL);
    if (module->observer) {
        Notifier *app_state_notifier = app_state_get_notifier();
        if (app_state_notifier) {
            app_state_notifier->subject.attach(app_state_notifier, module->observer);
        }
    }

    LOG_I(TAG, "Logger initialized. Logging to %s. Record size: %d bytes.", log_file_path, (int)module->dynamic_record_size);
    module->initialized = true;
    return HAL_ERR_NONE;
}

void logger_module_create_task(logger_module_t *module) {
    xTaskCreatePinnedToCore(logger_task, "LOGGER", 4096, module, TASK_PRIORITY_DEFAULT, &module->task_handle, 0);
}

static hal_err_t copy_config_file(const char *dest_path) {
    FILE *src = fopen("/sdcard/configuration.ini", "r");
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
    module->dynamic_record_size = 7;

    for (int i = 0; i < g_app_config.telemetry_params_count; ++i) {
        const char *name = g_app_config.telemetry_params[i];
        log_param_map_t *entry = &module->log_map[module->log_map_count];
        if (strcmp(name, "plane_speed") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_SPEED;
            entry->offset = offsetof(app_state_t, plane_speed);
            entry->size = sizeof(state->plane_speed);
        } else if (strcmp(name, "plane_fused_altitude") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_FUSED_ALTITUDE;
            entry->offset = offsetof(app_state_t, plane_fused_altitude);
            entry->size = sizeof(state->plane_fused_altitude);
        } else if (strcmp(name, "plane_baro_altitude") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_BARO_ALTITUDE;
            entry->offset = offsetof(app_state_t, plane_baro_altitude);
            entry->size = sizeof(state->plane_baro_altitude);
        } else if (strcmp(name, "plane_vspeed") == 0) {
            entry->field_mask = APP_STATE_FIELD_PLANE_VSPEED;
            entry->offset = offsetof(app_state_t, plane_vspeed);
            entry->size = sizeof(state->plane_vspeed);
        } else {
            continue;
        }
        module->dynamic_record_size += entry->size;
        module->log_map_count++;
    }
}
