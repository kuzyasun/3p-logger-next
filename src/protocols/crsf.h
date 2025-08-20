#pragma once

#include "alt_fuser.h"
#include "io/io.h"
#include "protocols/protocol_parser.h"  // Include the common interface
#include "util/time-util.h"

// Frame IDs
#define GPS_ID 0x02
#define BATTERY_ID 0x08
#define ATTITUDE_ID 0x1E
#define FLIGHT_MODE_ID 0x21
#define LINK_STATS_ID 0x14
#define VARIO_ID 0x07
#define BARO_ALT_ID 0x09
#define RC_CHANNELS_ID 0x16
#define VTX_ID 0x0F
#define HEARTBEAT_ID 0x0B
#define DEVICE_INFO_ID 0x29
#define ESC_TELEMETRY_ID 0x28

typedef struct {
    int32_t rpm;
    float voltage_v;
    float current_a;
    uint8_t temperature_c;
} crsf_esc_t;

// Parsed CRSF telemetry state (subset used by the application)
typedef struct crsf_state_s {
    // GPS (ID 0x02)
    float gps_lat_deg;
    float gps_lon_deg;
    int16_t gps_alt_m;
    float baro_alt_m;
    uint8_t gps_sats;
    float gps_groundspeed_kmh;
    float gps_heading_deg;

    // Battery (ID 0x08)
    float bat_voltage_v;
    float bat_current_a;
    float bat_fuel_drawn_ah;
    uint8_t bat_remaining_pct;

    // Attitude (ID 0x1E)
    float att_pitch_deg;
    float att_roll_deg;
    float att_yaw_deg;

    // Flight mode (ID 0x21)
    char flight_mode[24];
    uint8_t flight_mode_len;

    int8_t uplink_rssi_dbm;
    uint8_t uplink_lq;
    int8_t uplink_snr_db;
    int8_t downlink_rssi_dbm;
    uint8_t downlink_lq;
    int8_t downlink_snr_db;
    uint8_t rf_power_level;
    float vspeed_ms;

    uint16_t rc_channels[16];  // 16 каналів, кожен з 11-бітним значенням
    uint8_t vtx_band;
    uint8_t vtx_channel;
    uint8_t vtx_power;
    uint8_t origin_addr;       // Адреса відправника (для Heartbeat)
    uint8_t destination_addr;  // Адреса отримувача

    crsf_esc_t esc_telemetry[4];

    AltFuser alt;
    float t_last_gps, t_last_baro;  // секунди (монотонний час)
    bool baro_seen_nonzero;         // бачили колись ненульове баро

    // Parser state
    uint8_t buf[64];
    int buf_pos;
    bool home_source;

    time_micros_t last_frame_recv;
} crsf_state_t;

typedef struct {
    protocol_parser_t parser;
    crsf_state_t state;
} crsf_parser_t;

void crsf_parser_init(crsf_parser_t *instance);
bool crsf_process_frame(crsf_state_t *state, const uint8_t *frame_data, size_t total_frame_len);