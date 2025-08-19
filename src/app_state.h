#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_errors.h"
#include "util/observer.h"
#include "util/time-util.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_STATE_FIELD_NONE = 0,

    APP_STATE_FIELD_PLANE_LATITUDE = (1ULL << 0),
    APP_STATE_FIELD_PLANE_LONGITUDE = (1ULL << 1),
    APP_STATE_FIELD_PLANE_ALTITUDE = (1ULL << 2),
    APP_STATE_FIELD_PLANE_BARO_ALTITUDE = (1ULL << 3),
    APP_STATE_FIELD_PLANE_FUSED_ALTITUDE = (1ULL << 4),
    APP_STATE_FIELD_PLANE_SPEED = (1ULL << 5),
    APP_STATE_FIELD_PLANE_DISTANCE = (1ULL << 6),
    APP_STATE_FIELD_PLANE_STAR = (1ULL << 7),
    APP_STATE_FIELD_PLANE_FIX = (1ULL << 8),
    APP_STATE_FIELD_PLANE_PITCH = (1ULL << 9),
    APP_STATE_FIELD_PLANE_ROLL = (1ULL << 10),
    APP_STATE_FIELD_PLANE_HEADING = (1ULL << 11),
    APP_STATE_FIELD_PLANE_VBAT = (1ULL << 12),
    APP_STATE_FIELD_PLANE_BATTERY = (1ULL << 13),
    APP_STATE_FIELD_PLANE_RSSI = (1ULL << 14),
    APP_STATE_FIELD_PLANE_VSPEED = (1ULL << 15),
    APP_STATE_FIELD_PLANE_FLIGHT_MODE = (1ULL << 16),
    APP_STATE_FIELD_PLANE_ESC_RPM = (1ULL << 17),
    APP_STATE_FIELD_PLANE_ESC_VOLTAGE = (1ULL << 18),
    APP_STATE_FIELD_PLANE_ESC_CURRENT = (1ULL << 19),
    APP_STATE_FIELD_PLANE_ESC_TEMPERATURE = (1ULL << 20),
    APP_STATE_FIELD_PLANE_VTX_BAND = (1ULL << 21),
    APP_STATE_FIELD_PLANE_VTX_CHANNEL = (1ULL << 22),
    APP_STATE_FIELD_PLANE_VTX_POWER = (1ULL << 23),
    APP_STATE_FIELD_PLANE_RC_CHANNELS = (1ULL << 24),

} app_state_field_mask_e;

typedef enum {
    APP_MODE_IDLE,
    APP_MODE_LOGGING,
    APP_MODE_ERROR,
} app_mode_t;

typedef struct app_state_s {
    app_err_t system_error_code;
    // Plane telemetry
    int32_t plane_longitude;
    int32_t plane_latitude;
    int32_t plane_altitude;  // GPS altitude (m)
    // TODO convert to int16_t
    int32_t plane_baro_altitude;   // Baro altitude (m or dm->m converted)
    int32_t plane_fused_altitude;  // NEW: fused altitude (m)
    int16_t plane_speed;
    uint32_t plane_distance;
    uint8_t plane_star;
    uint8_t plane_fix;
    int16_t plane_pitch;
    int16_t plane_roll;
    int16_t plane_heading;
    uint16_t plane_vbat;
    uint16_t plane_battery;
    uint8_t plane_rssi;
    int16_t plane_vspeed;            // Vertical speed in m/s * 10
    char plane_flight_mode[24];      // Flight mode string
    int32_t plane_esc_rpm;           // ESC RPM
    uint16_t plane_esc_voltage;      // ESC voltage in mV
    uint16_t plane_esc_current;      // ESC current in mA
    uint8_t plane_esc_temperature;   // ESC temperature in °C
    uint8_t plane_vtx_band;          // VTX band
    uint8_t plane_vtx_channel;       // VTX channel
    uint8_t plane_vtx_power;         // VTX power level
    uint16_t plane_rc_channels[16];  // RC channel values (11-bit each)

    Notifier *changed_notifier;
    uint64_t changed_mask;
    bool update_in_progress;
    SemaphoreHandle_t state_mutex;
} app_state_t;

void app_state_init(void);
app_state_t *app_state_get_instance(void);
Notifier *app_state_get_notifier(void);

void app_state_begin_update(void);
void app_state_end_update(void);

void app_state_set_i32(app_state_field_mask_e field_mask, int32_t *field_ptr, int32_t value);
void app_state_set_i16(app_state_field_mask_e field_mask, int16_t *field_ptr, int16_t value);
void app_state_set_u16(app_state_field_mask_e field_mask, uint16_t *field_ptr, uint16_t value);
void app_state_set_u8(app_state_field_mask_e field_mask, uint8_t *field_ptr, uint8_t value);
void app_state_set_u32(app_state_field_mask_e field_mask, uint32_t *field_ptr, uint32_t value);
void app_state_set_i8(app_state_field_mask_e field_mask, int8_t *field_ptr, int8_t value);
void app_state_set_float(app_state_field_mask_e field_mask, float *field_ptr, float value);
void app_state_set_bool(app_state_field_mask_e field_mask, bool *field_ptr, bool value);

#ifdef __cplusplus
}
#endif
