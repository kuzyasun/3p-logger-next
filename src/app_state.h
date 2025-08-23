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

    // Missing fields that are used by protocols
    APP_STATE_FIELD_PLANE_ARMED = (1ULL << 25),
    APP_STATE_FIELD_PLANE_HOME_DIST = (1ULL << 26),
    APP_STATE_FIELD_PLANE_HOME_DIR = (1ULL << 27),

    // Additional GPS parameters
    APP_STATE_FIELD_PLANE_GPS_HDOP = (1ULL << 28),
    APP_STATE_FIELD_PLANE_GPS_VDOP = (1ULL << 29),
    APP_STATE_FIELD_PLANE_GPS_PDOP = (1ULL << 30),
    APP_STATE_FIELD_PLANE_GPS_FIX_MODE = (1ULL << 31),
    APP_STATE_FIELD_PLANE_GPS_SATS_IN_VIEW = (1ULL << 32),
    APP_STATE_FIELD_PLANE_GPS_GEO_SEP = (1ULL << 33),
    APP_STATE_FIELD_PLANE_GPS_MAG_VARIATION = (1ULL << 34),

    // Additional telemetry parameters
    APP_STATE_FIELD_PLANE_UPLINK_RSSI = (1ULL << 35),
    APP_STATE_FIELD_PLANE_UPLINK_LQ = (1ULL << 36),
    APP_STATE_FIELD_PLANE_UPLINK_SNR = (1ULL << 37),
    APP_STATE_FIELD_PLANE_DOWNLINK_RSSI = (1ULL << 38),
    APP_STATE_FIELD_PLANE_DOWNLINK_LQ = (1ULL << 39),
    APP_STATE_FIELD_PLANE_DOWNLINK_SNR = (1ULL << 40),
    APP_STATE_FIELD_PLANE_RF_POWER_LEVEL = (1ULL << 41),

    // Additional ESC telemetry (for multiple ESCs)
    APP_STATE_FIELD_PLANE_ESC2_RPM = (1ULL << 42),
    APP_STATE_FIELD_PLANE_ESC2_VOLTAGE = (1ULL << 43),
    APP_STATE_FIELD_PLANE_ESC2_CURRENT = (1ULL << 44),
    APP_STATE_FIELD_PLANE_ESC2_TEMPERATURE = (1ULL << 45),
    APP_STATE_FIELD_PLANE_ESC3_RPM = (1ULL << 46),
    APP_STATE_FIELD_PLANE_ESC3_VOLTAGE = (1ULL << 47),
    APP_STATE_FIELD_PLANE_ESC3_CURRENT = (1ULL << 48),
    APP_STATE_FIELD_PLANE_ESC3_TEMPERATURE = (1ULL << 49),
    APP_STATE_FIELD_PLANE_ESC4_RPM = (1ULL << 50),
    APP_STATE_FIELD_PLANE_ESC4_VOLTAGE = (1ULL << 51),
    APP_STATE_FIELD_PLANE_ESC4_CURRENT = (1ULL << 52),
    APP_STATE_FIELD_PLANE_ESC4_TEMPERATURE = (1ULL << 53),

    // Additional time/date parameters
    APP_STATE_FIELD_PLANE_GPS_TIME_HOURS = (1ULL << 54),
    APP_STATE_FIELD_PLANE_GPS_TIME_MINUTES = (1ULL << 55),
    APP_STATE_FIELD_PLANE_GPS_TIME_SECONDS = (1ULL << 56),
    APP_STATE_FIELD_PLANE_GPS_DATE_DAY = (1ULL << 57),
    APP_STATE_FIELD_PLANE_GPS_DATE_MONTH = (1ULL << 58),
    APP_STATE_FIELD_PLANE_GPS_DATE_YEAR = (1ULL << 59),
    APP_STATE_FIELD_CURRENT_MODE = (1ULL << 60),

} app_state_field_mask_e;

typedef enum {
    APP_MODE_IDLE = 0,
    APP_MODE_LOGGING = 1,
    APP_MODE_ERROR = 2,
} app_mode_t;

typedef struct app_state_s {
    app_err_t system_error_code;
    app_mode_t current_mode;  // Current application mode
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

    // Missing fields that are used by protocols
    uint8_t plane_armed;      // Armed status (0/1)
    int16_t plane_home_dist;  // Distance to home in meters
    int16_t plane_home_dir;   // Direction to home in degrees

    // Additional GPS parameters
    uint16_t plane_gps_hdop;          // Horizontal dilution of precision * 100
    uint16_t plane_gps_vdop;          // Vertical dilution of precision * 100
    uint16_t plane_gps_pdop;          // Position dilution of precision * 100
    uint8_t plane_gps_fix_mode;       // Fix mode (1=no fix, 2=2D, 3=3D)
    uint8_t plane_gps_sats_in_view;   // Number of satellites in view
    int16_t plane_gps_geo_sep;        // Geoid separation in cm
    int16_t plane_gps_mag_variation;  // Magnetic variation in degrees * 10

    // Additional telemetry parameters
    int8_t plane_uplink_rssi;      // Uplink RSSI in dBm
    uint8_t plane_uplink_lq;       // Uplink link quality in %
    int8_t plane_uplink_snr;       // Uplink SNR in dB
    int8_t plane_downlink_rssi;    // Downlink RSSI in dBm
    uint8_t plane_downlink_lq;     // Downlink link quality in %
    int8_t plane_downlink_snr;     // Downlink SNR in dB
    uint8_t plane_rf_power_level;  // RF power level

    // Additional ESC telemetry (for multiple ESCs)
    int32_t plane_esc2_rpm;          // ESC2 RPM
    uint16_t plane_esc2_voltage;     // ESC2 voltage in mV
    uint16_t plane_esc2_current;     // ESC2 current in mA
    uint8_t plane_esc2_temperature;  // ESC2 temperature in °C
    int32_t plane_esc3_rpm;          // ESC3 RPM
    uint16_t plane_esc3_voltage;     // ESC3 voltage in mV
    uint16_t plane_esc3_current;     // ESC3 current in mA
    uint8_t plane_esc3_temperature;  // ESC3 temperature in °C
    int32_t plane_esc4_rpm;          // ESC4 RPM
    uint16_t plane_esc4_voltage;     // ESC4 voltage in mV
    uint16_t plane_esc4_current;     // ESC4 current in mA
    uint8_t plane_esc4_temperature;  // ESC4 temperature in °C

    // Additional time/date parameters
    uint8_t plane_gps_time_hours;    // GPS time hours
    uint8_t plane_gps_time_minutes;  // GPS time minutes
    uint8_t plane_gps_time_seconds;  // GPS time seconds
    uint8_t plane_gps_date_day;      // GPS date day
    uint8_t plane_gps_date_month;    // GPS date month
    uint8_t plane_gps_date_year;     // GPS date year (2-digit)

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
