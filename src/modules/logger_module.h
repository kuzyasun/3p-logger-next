#pragma once

#include "app_state.h"
#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/sdcard.h"
#include "util/observer.h"

#define LOGGER_BUFFER_SIZE 4096

typedef enum {
    LOG_DTYPE_UNKNOWN,
    LOG_DTYPE_INT16,
    LOG_DTYPE_INT32,
    LOG_DTYPE_FLOAT,
} log_param_data_type_t;

typedef struct {
    app_state_field_mask_e field_mask;
    size_t offset;
    size_t size;
    log_param_data_type_t type;
    char name[MAX_PARAM_NAME_LEN];
} log_param_map_t;

typedef struct {
    uint8_t *data;
    size_t size;
} logger_chunk_t;

typedef union {
    int16_t val_i16;
    int32_t val_i32;
    float val_f;
} log_snapshot_value_u;

typedef struct {
    uint64_t timestamp_ms;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    uint8_t piezo_mask;
    log_snapshot_value_u telemetry_values[MAX_LOG_TELEMETRY_PARAMS];
} log_data_snapshot_t;

typedef struct logger_module_s {
    uint8_t ping_buffer[LOGGER_BUFFER_SIZE];
    uint8_t pong_buffer[LOGGER_BUFFER_SIZE];
    uint8_t *active_buffer;
    volatile size_t active_buffer_idx;

    QueueHandle_t data_queue;
    QueueHandle_t buffer_queue;
    TaskHandle_t processing_task_handle;
    TaskHandle_t writer_task_handle;
    Observer *observer;
    sdcard_file_handle_t log_file;
    char csv_header[1024];

    bool sd_card_ok;
    bool initialized;

    size_t dynamic_record_size;
    log_param_map_t log_map[MAX_LOG_TELEMETRY_PARAMS];
    int log_map_count;
} logger_module_t;

hal_err_t logger_module_init(logger_module_t *module, bool is_sd_card_ok);
void logger_module_create_task(logger_module_t *module);
