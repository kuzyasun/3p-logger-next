#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "modules/accel_module.h"
#include "protocols/protocol.h"

#define MAX_LOG_TELEMETRY_PARAMS 16
#define MAX_PARAM_NAME_LEN 32

// Expanded accelerometer config
typedef struct {
    lis3dh_odr_t output_data_rate;
    lis3dh_fs_t full_scale;
    lis3dh_op_md_t op_mode;
    bool bdu_enabled;
    hal_spi_bus_t spi_bus;
    hal_gpio_t cs_pin;
} extended_accel_config_t;

// Main configuration structure
typedef struct {
    extended_accel_config_t accel_config;
    uint8_t pz_dac1_threshold;
    uint8_t pz_dac2_threshold;
    bool uart1_enabled;
    protocol_e uart1_protocol;
    int uart1_baudrate;
    int telemetry_params_count;
    char telemetry_params[MAX_LOG_TELEMETRY_PARAMS][MAX_PARAM_NAME_LEN];
} app_config_t;

extern app_config_t g_app_config;

bool config_manager_load(void);
