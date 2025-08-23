#include "logger_module.h"

#include <string.h>

#include "app_state.h"
#include "log.h"
#include "target.h"

static const char *TAG = "LOGR";

// --- Producer: app_state observer callback ---
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
    // New record size is 15 bytes
    uint8_t record[15];
    // 1. Accel data (6 bytes)
    memcpy(&record[0], &state->accel_x, 2);
    memcpy(&record[2], &state->accel_y, 2);
    memcpy(&record[4], &state->accel_z, 2);
    // 2. Piezo mask (1 byte)
    memcpy(&record[6], &state->piezo_mask, 1);
    // 3. Telemetry data (6 bytes)
    memcpy(&record[7], &state->plane_speed, 2);
    memcpy(&record[9], &state->plane_fused_altitude, 4);
    // 4. Future use (2 bytes)
    uint16_t future = 0;
    memcpy(&record[13], &future, 2);

    if (module->active_buffer_idx + sizeof(record) > LOGGER_BUFFER_SIZE) {
        uint8_t *full_buffer = module->active_buffer;
        xQueueSend(module->buffer_queue, &full_buffer, portMAX_DELAY);
        module->active_buffer = (module->active_buffer == module->ping_buffer) ? module->pong_buffer : module->ping_buffer;
        module->active_buffer_idx = 0;
    }

    memcpy(module->active_buffer + module->active_buffer_idx, record, sizeof(record));
    module->active_buffer_idx += sizeof(record);

    if (module->active_buffer_idx >= LOGGER_BUFFER_SIZE) {
        uint8_t *full_buffer = module->active_buffer;
        xQueueSend(module->buffer_queue, &full_buffer, portMAX_DELAY);
        module->active_buffer = (module->active_buffer == module->ping_buffer) ? module->pong_buffer : module->ping_buffer;
        module->active_buffer_idx = 0;
    }
}

// --- Consumer: Logger FreeRTOS Task ---
static void logger_task(void *arg) {
    logger_module_t *module = (logger_module_t *)arg;
    uint8_t *buffer = NULL;

    // Safety check - if queue is NULL, task should not run
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

// --- Public API Implementation ---
hal_err_t logger_module_init(logger_module_t *module, hal_spi_bus_t sd_spi_bus) {
    memset(module, 0, sizeof(logger_module_t));

    hal_err_t err = sdcard_init(sd_spi_bus, SD_SPI_CS_GPIO);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize SD card HAL. Logger disabled.");
        module->sd_card_ok = false;
        return err;
    }

    err = sdcard_open_file("/log.bin", "w", &module->log_file);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to create log file. Logger disabled.");
        module->sd_card_ok = false;
        return err;
    }

    module->sd_card_ok = true;
    module->buffer_queue = xQueueCreate(2, sizeof(uint8_t *));
    if (module->buffer_queue == NULL) {
        LOG_E(TAG, "Failed to create buffer queue. Logger disabled.");
        module->sd_card_ok = false;
        return HAL_ERR_FAILED;
    }
    module->active_buffer = module->ping_buffer;
    module->active_buffer_idx = 0;

    module->observer = ObserverCreate("LOGGER_APP_STATE", module, on_app_state_change, NULL);
    if (module->observer) {
        Notifier *app_state_notifier = app_state_get_notifier();
        if (app_state_notifier) {
            app_state_notifier->subject.attach(app_state_notifier, module->observer);
        }
    }

    LOG_I(TAG, "Logger module initialized.");
    module->initialized = true;
    return HAL_ERR_NONE;
}

void logger_module_create_task(logger_module_t *module) {
    xTaskCreatePinnedToCore(logger_task, "LOGGER", 4096, module, TASK_PRIORITY_DEFAULT, &module->task_handle, 0);
}
