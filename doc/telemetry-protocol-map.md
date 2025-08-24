# Telemetry Protocol Map

This document maps each `app_state_t` field to the protocols that provide it. The data type column reflects the native type supplied by the protocol parser.

| Field | CRSF | MAVLINK | MSP | MSP_V2 | NMEA | UBLOX |
|-------|------|---------|-----|--------|------|-------|
| plane_longitude | int32_t | int32_t | int32_t | int32_t | int32_t | int32_t |
| plane_latitude | int32_t | int32_t | int32_t | int32_t | int32_t | int32_t |
| plane_altitude | int32_t | int32_t | int32_t | int32_t | int32_t | int32_t |
| plane_baro_altitude | int16_t | int16_t | int16_t | - | - | - |
| plane_fused_altitude | int32_t | - | - | - | - | - |
| plane_speed | - | int16_t | int16_t | int16_t | int16_t | int16_t |
| plane_distance | - | - | - | - | - | - |
| plane_star | uint8_t | - | uint8_t | uint8_t | uint8_t | - |
| plane_fix | uint8_t | - | uint8_t | uint8_t | uint8_t | - |
| plane_pitch | int16_t | - | int16_t | int16_t | - | - |
| plane_roll | int16_t | - | int16_t | int16_t | - | - |
| plane_heading | int16_t | int16_t | int16_t | int16_t | int16_t | int16_t |
| plane_vbat | uint16_t | - | uint16_t | uint16_t | - | - |
| plane_battery | uint16_t | - | uint16_t | uint16_t | - | - |
| plane_rssi | uint8_t | - | uint8_t | - | - | - |
| plane_vspeed | int16_t | int16_t | int16_t | - | - | - |
| plane_flight_mode | string | - | string | - | - | - |
| plane_esc_rpm | int32_t | - | - | - | - | - |
| plane_esc_voltage | uint16_t | - | - | - | - | - |
| plane_esc_current | uint16_t | - | uint16_t | - | - | - |
| plane_esc_temperature | uint8_t | - | - | - | - | - |
| plane_vtx_band | uint8_t | - | - | - | - | - |
| plane_vtx_channel | uint8_t | - | - | - | - | - |
| plane_vtx_power | uint8_t | - | - | - | - | - |
| plane_rc_channels | - | - | - | - | - | - |
| plane_armed | - | - | uint8_t | - | - | - |
| plane_home_dist | - | - | int16_t | - | - | - |
| plane_home_dir | - | - | int16_t | - | - | - |
| plane_gps_hdop | - | - | - | - | uint16_t | - |
| plane_gps_vdop | - | - | - | - | uint16_t | - |
| plane_gps_pdop | - | - | - | - | uint16_t | - |
| plane_gps_fix_mode | - | - | - | - | uint8_t | - |
| plane_gps_sats_in_view | - | - | - | - | uint8_t | - |
| plane_gps_geo_sep | - | - | - | - | int16_t | - |
| plane_gps_mag_variation | - | - | - | - | int16_t | - |
| plane_uplink_rssi | int8_t | - | - | - | - | - |
| plane_uplink_lq | uint8_t | - | - | - | - | - |
| plane_uplink_snr | int8_t | - | - | - | - | - |
| plane_downlink_rssi | int8_t | - | - | - | - | - |
| plane_downlink_lq | uint8_t | - | - | - | - | - |
| plane_downlink_snr | int8_t | - | - | - | - | - |
| plane_rf_power_level | uint8_t | - | - | - | - | - |
| plane_gps_time_hours | - | - | - | - | uint8_t | - |
| plane_gps_time_minutes | - | - | - | - | uint8_t | - |
| plane_gps_time_seconds | - | - | - | - | uint8_t | - |
| plane_gps_date_day | - | - | - | - | uint8_t | - |
| plane_gps_date_month | - | - | - | - | uint8_t | - |
| plane_gps_date_year | - | - | - | - | uint8_t | - |
