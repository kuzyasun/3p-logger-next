#pragma once

#include "app_state.h"
#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/sdcard.h"
#include "hal/spi.h"
#include "util/observer.h"

#define LOGGER_BUFFER_SIZE 8192

typedef struct {
    app_state_field_mask_e field_mask;
    size_t offset;
    size_t size;
} log_param_map_t;

typedef struct logger_module_s {
    uint8_t ping_buffer[LOGGER_BUFFER_SIZE];
    uint8_t pong_buffer[LOGGER_BUFFER_SIZE];
    uint8_t *active_buffer;
    volatile size_t active_buffer_idx;
    QueueHandle_t buffer_queue;
    TaskHandle_t task_handle;
    Observer *observer;
    sdcard_file_handle_t log_file;
    bool sd_card_ok;
    bool initialized;

    size_t dynamic_record_size;
    log_param_map_t log_map[MAX_LOG_TELEMETRY_PARAMS];
    int log_map_count;
} logger_module_t;

hal_err_t logger_module_init(logger_module_t *module, hal_spi_bus_t sd_spi_bus);
void logger_module_create_task(logger_module_t *module);
