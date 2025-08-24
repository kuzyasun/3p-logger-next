#include "config_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "log.h"
#include "target.h"

static const char *TAG = "CONFIG";

app_config_t g_app_config;

static void set_default_config(void);
static void parse_line(char *line);
static lis3dh_odr_t str_to_odr(const char *str);
static lis3dh_fs_t str_to_fs(const char *str);
static lis3dh_op_md_t str_to_opmode(const char *str);
static protocol_e str_to_protocol(const char *str);
static bool str_to_bool(const char *str);
static void trim(char *str);

bool config_manager_load(void) {
    set_default_config();

    LOG_I(TAG, "Loading configuration from %s", SD_MOUNT_PATH "/configuration.ini");
    FILE *f = fopen(SD_MOUNT_PATH "/configuration.ini", "r");
    if (!f) {
        LOG_W(TAG, "configuration.ini not found, using defaults");
        return false;
    }

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        parse_line(line);
    }
    fclose(f);
    return true;
}

static void set_default_config(void) {
    g_app_config.accel_config.output_data_rate = LIS3DH_ODR_400Hz;
    g_app_config.accel_config.full_scale = LIS3DH_16g;
    g_app_config.accel_config.op_mode = LIS3DH_HR_12bit;
    g_app_config.accel_config.bdu_enabled = true;
    g_app_config.accel_config.data_ready_interrupt_enabled = true;
    g_app_config.accel_config.spi_bus = ACCEL_SPI_HOST;
    g_app_config.accel_config.cs_pin = ACC_SPI_CS_GPIO;

    g_app_config.pz_dac1_threshold = 128;
    g_app_config.pz_dac2_threshold = 128;

    g_app_config.uart1_protocol = PROTOCOL_CRSF;
    g_app_config.uart1_baudrate = 420000;

    g_app_config.telemetry_params_count = 0;
}

static void parse_line(char *line) {
    char *key = strtok(line, "=");
    char *value = strtok(NULL, "\r\n");
    if (!key || !value) return;

    trim(key);
    trim(value);

    if (key[0] == '#') return;

    if (strcmp(key, "ACCEL_ODR") == 0) {
        g_app_config.accel_config.output_data_rate = str_to_odr(value);
    } else if (strcmp(key, "ACCEL_FS") == 0) {
        g_app_config.accel_config.full_scale = str_to_fs(value);
    } else if (strcmp(key, "ACCEL_OP_MODE") == 0) {
        g_app_config.accel_config.op_mode = str_to_opmode(value);
    } else if (strcmp(key, "ACCEL_BDU_ENABLED") == 0) {
        g_app_config.accel_config.bdu_enabled = str_to_bool(value);
    } else if (strcmp(key, "ACCEL_DATA_READY_INT") == 0) {
        g_app_config.accel_config.data_ready_interrupt_enabled = str_to_bool(value);
    } else if (strcmp(key, "ACCEL_SPI_BUS") == 0) {
        g_app_config.accel_config.spi_bus = (hal_spi_bus_t)atoi(value);
    } else if (strcmp(key, "ACCEL_CS_PIN") == 0) {
        g_app_config.accel_config.cs_pin = (hal_gpio_t)atoi(value);
    } else if (strcmp(key, "PZ_THRESHOLD_1") == 0) {
        g_app_config.pz_dac1_threshold = (uint8_t)atoi(value);
    } else if (strcmp(key, "PZ_THRESHOLD_2") == 0) {
        g_app_config.pz_dac2_threshold = (uint8_t)atoi(value);
    } else if (strcmp(key, "UART1_PROTOCOL") == 0) {
        g_app_config.uart1_protocol = str_to_protocol(value);
    } else if (strcmp(key, "UART1_BAUDRATE") == 0) {
        g_app_config.uart1_baudrate = atoi(value);
    } else if (strcmp(key, "LOG_TELEMETRY") == 0) {
        char *token = strtok(value, ",");
        int idx = 0;
        while (token && idx < MAX_LOG_TELEMETRY_PARAMS) {
            trim(token);
            strncpy(g_app_config.telemetry_params[idx], token, MAX_PARAM_NAME_LEN - 1);
            g_app_config.telemetry_params[idx][MAX_PARAM_NAME_LEN - 1] = '\0';
            idx++;
            token = strtok(NULL, ",");
        }
        g_app_config.telemetry_params_count = idx;
    }
}

// Simple string to enum conversions
static lis3dh_odr_t str_to_odr(const char *str) {
    if (!str) return LIS3DH_ODR_400Hz;
    if (strcmp(str, "PowerDown") == 0) return LIS3DH_POWER_DOWN;
    if (strcmp(str, "1Hz") == 0) return LIS3DH_ODR_1Hz;
    if (strcmp(str, "10Hz") == 0) return LIS3DH_ODR_10Hz;
    if (strcmp(str, "25Hz") == 0) return LIS3DH_ODR_25Hz;
    if (strcmp(str, "50Hz") == 0) return LIS3DH_ODR_50Hz;
    if (strcmp(str, "100Hz") == 0) return LIS3DH_ODR_100Hz;
    if (strcmp(str, "200Hz") == 0) return LIS3DH_ODR_200Hz;
    if (strcmp(str, "400Hz") == 0) return LIS3DH_ODR_400Hz;
    if (strcmp(str, "1.62kHz_LP") == 0) return LIS3DH_ODR_1kHz620_LP;
    if (strcmp(str, "5.376kHz_LP") == 0) return LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP;
    if (strcmp(str, "1.344kHz_Normal_HR") == 0) return LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP;
    return LIS3DH_ODR_400Hz;
}

static lis3dh_fs_t str_to_fs(const char *str) {
    if (!str) return LIS3DH_16g;
    if (strcmp(str, "2g") == 0) return LIS3DH_2g;
    if (strcmp(str, "4g") == 0) return LIS3DH_4g;
    if (strcmp(str, "8g") == 0) return LIS3DH_8g;
    if (strcmp(str, "16g") == 0) return LIS3DH_16g;
    return LIS3DH_16g;
}

static lis3dh_op_md_t str_to_opmode(const char *str) {
    if (!str) return LIS3DH_HR_12bit;
    if (strcmp(str, "LP_8bit") == 0) return LIS3DH_LP_8bit;
    if (strcmp(str, "NM_10bit") == 0) return LIS3DH_NM_10bit;
    if (strcmp(str, "HR_12bit") == 0) return LIS3DH_HR_12bit;
    return LIS3DH_HR_12bit;
}

static protocol_e str_to_protocol(const char *str) {
    if (!str) return PROTOCOL_CRSF;
    if (strcmp(str, "MSP") == 0) return PROTOCOL_MSP;
    if (strcmp(str, "MSP_V2") == 0) return PROTOCOL_MSP_V2;
    if (strcmp(str, "MAVLINK") == 0) return PROTOCOL_MAVLINK;
    if (strcmp(str, "NMEA") == 0) return PROTOCOL_NMEA;
    if (strcmp(str, "UBLOX") == 0) return PROTOCOL_UBLOX;
    if (strcmp(str, "CRSF") == 0) return PROTOCOL_CRSF;
    return PROTOCOL_CRSF;
}

static bool str_to_bool(const char *str) {
    if (!str) return false;
    return strcasecmp(str, "true") == 0 || strcmp(str, "1") == 0;
}

static void trim(char *str) {
    if (!str) return;
    char *start = str;
    while (*start == ' ' || *start == '\t') start++;
    if (start != str) memmove(str, start, strlen(start) + 1);
    char *end = str + strlen(str) - 1;
    while (end >= str && (*end == ' ' || *end == '\t')) {
        *end = '\0';
        end--;
    }
}
