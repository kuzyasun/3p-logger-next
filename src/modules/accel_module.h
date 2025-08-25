#pragma once

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/err.h"
#include "hal/gpio.h"
#include "hal/spi.h"
#include "lis3dh_reg.h"

// Forward declarations to break circular dependency
struct logger_module_s;
typedef struct logger_module_s logger_module_t;

// Configuration structure for the LIS3DH sensor
typedef struct {
    lis3dh_odr_t output_data_rate;
    lis3dh_fs_t full_scale;
    lis3dh_op_md_t op_mode;
    hal_spi_bus_t spi_bus;
    hal_gpio_t cs_pin;
} accel_config_t;

// Main module state structure
typedef struct accel_module_s {
    stmdev_ctx_t driver_ctx;
    hal_spi_device_handle_t spi_dev;
    TaskHandle_t task_handle;
    accel_config_t config;
    bool initialized;
    logger_module_t *logger;
} accel_module_t;

// Public API
hal_err_t accel_module_init(accel_module_t *module, const accel_config_t *config);
void accel_module_create_task(accel_module_t *module, logger_module_t *logger_instance);
