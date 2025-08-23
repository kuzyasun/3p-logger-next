#include "crsf.h"

#include <log.h>
#include <math.h>
#include <string.h>

#include "app_state.h"

// Для монотонного часу в секундах (ESP-IDF / FreeRTOS)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CRSF";

// Internal vtable functions
static void crsf_parser_internal_init(void *parser_state);
static void crsf_parser_internal_process_byte(void *parser_state, uint8_t byte);
static void crsf_parser_internal_destroy(void *parser_state);

// Internal helper function for CRSF frame processing
bool crsf_process_frame(crsf_state_t *state, const uint8_t *frame_data, size_t frame_length);

// CRC8-DVB-S2 implementation used by CRSF
static uint8_t crsf_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// Відновлення офсету GPS ALT з CRSF
static inline int16_t decode_gps_alt_crsf(int16_t raw) { return (raw > 100) ? (raw - 1000) : raw; }

// The only public function needed to create an instance
void crsf_parser_init(crsf_parser_t *instance) {
    if (!instance) {
        LOG_E(TAG, "Invalid instance pointer");
        return;
    }

    // Initialize the base parser structure
    instance->parser.state = &instance->state;
    instance->parser.vtable.init = crsf_parser_internal_init;
    instance->parser.vtable.process_byte = crsf_parser_internal_process_byte;
    instance->parser.vtable.destroy = crsf_parser_internal_destroy;
    instance->parser.is_initialized = false;

    // Initialize the internal state
    memset(&instance->state, 0, sizeof(crsf_state_t));
    instance->state.buf_pos = 0;
    instance->state.last_frame_recv = 0;

    // Tuned for mountain flying (робастність проти нулів GPS)
    AltFuserCfg cfg = {
        .gpsMinSats = 6,
        .gpsMaxHdop = 2.5f,
        .zeroDebounce = 4,    // приймати 0 лише після 4 послідовних тика
        .maxJumpM = 60.0f,    // відсікати разовий стрибок > 60 м
        .maxRateMps = 15.0f,  // відсікати |dz/dt| > 15 м/с
        .holdMaxS = 2.5f,     // тримати останнє добре до 2.5 с
        .wGps = 0.25f,        // баро — база, GPS лише LF-корекція
        .tauBaroS = 0.25f,    // легкий LP для баро
        .tauGpsS = 1.2f,      // важчий LP для GPS
        .fusedAlpha = 0.45f   // трохи швидше підтягування виходу
    };
    alt_fuser_init(&instance->state.alt, &cfg);

    LOG_I(TAG, "parser instance initialized");
}

// Internal vtable implementation: Initialize parser state
static void crsf_parser_internal_init(void *parser_state) {
    crsf_state_t *state = (crsf_state_t *)parser_state;
    if (!state) return;

    // Reset the parser state
    state->buf_pos = 0;
    state->last_frame_recv = 0;

    memset(state->buf, 0, sizeof(state->buf));

    LOG_I(TAG, "parser state initialized");
}

// Internal vtable implementation: Process a single byte
static void crsf_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    crsf_state_t *state = (crsf_state_t *)parser_state;
    if (!state) return;

    // If buffer is empty, we are waiting only for start byte
    if (state->buf_pos == 0) {
        bool is_valid_sync_byte = (byte == 0xEE || byte == 0xEA || byte == 0xC8);
        if (!is_valid_sync_byte) {
            return;  // Ignore all bytes until we find start byte
        }
    }

    // Add byte to buffer if there is space
    if (state->buf_pos < (int)sizeof(state->buf)) {
        state->buf[state->buf_pos++] = byte;
    } else {
        // Buffer overflow, reset and start over
        LOG_W(TAG, "buffer overflow, re-syncing");
        state->buf_pos = 0;
        return;
    }

    // Check if there is enough data to determine length
    if (state->buf_pos < 2) {
        return;  // Need at least 2 bytes (Sync + Length)
    }

    // --- LOGIC FOR PROCESSING FULL FRAME ---
    uint8_t frame_length_byte = state->buf[1];
    // Check for valid length to avoid large values
    if (frame_length_byte > sizeof(state->buf) - 2) {
        LOG_W(TAG, "Invalid frame length received: %d. Re-syncing.", frame_length_byte);
        state->buf_pos = 0;
        return;
    }

    uint8_t expected_frame_size = frame_length_byte + 2;  // Sync(1) + Length(1) + Payload(Length val)

    // If we have a full frame
    if (state->buf_pos >= expected_frame_size) {
        // Pass frame to processing (CRC check and parsing)
        if (crsf_process_frame(state, state->buf, expected_frame_size)) {
            // SUCCESS: Frame is valid.
            // If there is still data in the buffer, move it to the beginning
            if (state->buf_pos > expected_frame_size) {
                memmove(state->buf, &state->buf[expected_frame_size], state->buf_pos - expected_frame_size);
                state->buf_pos -= expected_frame_size;
            } else {
                state->buf_pos = 0;  // buffer processed completely
            }
        } else {
            // !!! Reset buffer completely to find next valid frame !!!
            LOG_W(TAG, "Failed to process frame, re-syncing parser.");
            state->buf_pos = 0;
        }
    }
}

// Internal vtable implementation: Destroy parser state
static void crsf_parser_internal_destroy(void *parser_state) {
    crsf_state_t *state = (crsf_state_t *)parser_state;
    if (!state) return;

    // Reset the state
    memset(state, 0, sizeof(crsf_state_t));
    LOG_D(TAG, "parser state destroyed");
}

// Internal helper function: Process a complete CRSF frame
bool crsf_process_frame(crsf_state_t *state, const uint8_t *frame_data, size_t frame_length) {
    if (!frame_data || frame_length < 3) {
        return false;
    }

    // Length from second byte of frame (includes type, data and CRC)
    const uint8_t crsf_len = frame_data[1];
    // Length of data for CRC calculation (Type + Payload)
    const uint8_t crc_payload_len = crsf_len - 1;
    // Calculate CRC for [Type] + [Payload]
    const uint8_t calculated_crc = crsf_crc8(&frame_data[2], crc_payload_len);
    // Get CRC from frame
    const uint8_t received_crc = frame_data[crsf_len + 1];  // frame_data[2 + crc_payload_len]

    if (calculated_crc != received_crc) {
        LOG_W(TAG, "CRC mismatch! Calculated: 0x%02X, Received: 0x%02X", calculated_crc, received_crc);
        return false;  // Drop frame if CRC does not match
    }

    const uint8_t crsf_id = frame_data[2];

    switch (crsf_id) {
        case GPS_ID: {
            if (frame_length < 18) return false;

            const int32_t gps_lat =
                (int32_t)(((uint32_t)frame_data[6] << 0) | ((uint32_t)frame_data[5] << 8) | ((uint32_t)frame_data[4] << 16) | ((uint32_t)frame_data[3] << 24));
            const int32_t gps_lon =
                (int32_t)(((uint32_t)frame_data[10] << 0) | ((uint32_t)frame_data[9] << 8) | ((uint32_t)frame_data[8] << 16) | ((uint32_t)frame_data[7] << 24));
            const uint16_t gps_groundspeed = (uint16_t)((frame_data[12] << 0) | (frame_data[11] << 8));
            const uint16_t gps_heading = (uint16_t)((frame_data[14] << 0) | (frame_data[13] << 8));
            int16_t gps_alt_raw = (int16_t)((frame_data[16] << 0) | (frame_data[15] << 8));
            int16_t gps_alt = decode_gps_alt_crsf(gps_alt_raw);
            const uint8_t sats = frame_data[17];

            state->gps_lat_deg = (float)gps_lat / 1e7f;
            state->gps_lon_deg = (float)gps_lon / 1e7f;
            state->gps_groundspeed_kmh = (float)gps_groundspeed * 0.1f;
            state->gps_heading_deg = (float)gps_heading * 0.01f;
            state->gps_alt_m = (float)gps_alt;
            state->gps_sats = sats;

            const float now_s = (float)(xTaskGetTickCount() * (portTICK_PERIOD_MS)) / 1000.0f;
            state->t_last_gps = now_s;

            // --- Fused altitude update ---
            const bool gpsValid = true;  // щойно прийшов GPS кадр
            // баро тільки якщо вже бачили ненульове і свіже <1с
            const bool barValid = state->baro_seen_nonzero && ((now_s - state->t_last_baro) < 1.0f);

            bool usedGps = false, usedBaro = false;
            float fused =
                alt_fuser_update(&state->alt, now_s, gpsValid, state->gps_alt_m, (int)state->gps_sats, NAN, barValid, state->baro_alt_m, &usedGps, &usedBaro);

            // FAILSAFE: якщо ф’юзер не прийняв жодного джерела — зафіксувати на GPS
            if (!usedGps && !usedBaro && state->gps_sats >= 4 && state->gps_alt_m != 0.0f) {
                fused = state->gps_alt_m;
                state->alt.fused = fused;
                state->alt.fusedInit = true;
                usedGps = true;
            }

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_i32(APP_STATE_FIELD_PLANE_LONGITUDE, &app_state->plane_longitude, gps_lon);
            app_state_set_i32(APP_STATE_FIELD_PLANE_LATITUDE, &app_state->plane_latitude, gps_lat);
            app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, gps_alt);
            app_state_set_i32(APP_STATE_FIELD_PLANE_FUSED_ALTITUDE, &app_state->plane_fused_altitude, (int32_t)lroundf(fused));
            app_state_set_u8(APP_STATE_FIELD_PLANE_STAR, &app_state->plane_star, sats);
            app_state_set_u8(APP_STATE_FIELD_PLANE_FIX, &app_state->plane_fix, (sats > 0) ? 1 : 0);
            app_state_end_update();

            LOG_D(TAG, "GPS: Lat=%.7f, Lon=%.7f, Alt=%dm, Sats=%d, Fused=%.1fm (gps:%d,baro:%d)", state->gps_lat_deg, state->gps_lon_deg, gps_alt, sats, fused,
                  usedGps, usedBaro);
            return true;
        }

        case BATTERY_ID: {
            if (frame_length < 12) return false;
            const uint16_t bat_mv_x10 = (uint16_t)((frame_data[4] << 0) | (frame_data[3] << 8));
            const uint16_t bat_ma_x10 = (uint16_t)((frame_data[6] << 0) | (frame_data[5] << 8));
            const int32_t ah_milli =
                (int32_t)(((uint32_t)frame_data[10] << 0) | ((uint32_t)frame_data[9] << 8) | ((uint32_t)frame_data[8] << 16) | ((uint32_t)frame_data[7] << 24));
            const uint8_t remain = frame_data[11];
            state->bat_voltage_v = (float)bat_mv_x10 * 0.1f;
            state->bat_current_a = (float)bat_ma_x10 * 0.1f;
            state->bat_fuel_drawn_ah = (float)ah_milli;
            state->bat_remaining_pct = remain;

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u16(APP_STATE_FIELD_PLANE_VBAT, &app_state->plane_vbat, bat_mv_x10);
            app_state_set_u16(APP_STATE_FIELD_PLANE_BATTERY, &app_state->plane_battery, (uint16_t)(ah_milli / 1000));
            app_state_end_update();

            LOG_D(TAG, "Battery: V=%.1fV, I=%.1fA, Ah=%.1f, Rem=%d%%", state->bat_voltage_v, state->bat_current_a, state->bat_fuel_drawn_ah, remain);
            return true;
        }

        case ATTITUDE_ID: {
            if (frame_length < 10) return false;
            const int16_t pitch_rad_1e4 = (int16_t)((frame_data[4] << 0) | (frame_data[3] << 8));
            const int16_t roll_rad_1e4 = (int16_t)((frame_data[6] << 0) | (frame_data[5] << 8));
            int16_t yaw_rad_1e4 = (int16_t)((frame_data[8] << 0) | (frame_data[7] << 8));
            const float rad2deg = 57.29577951308232f;
            const float pitch_deg = (float)pitch_rad_1e4 * rad2deg * 0.0001f;
            const float roll_deg = (float)roll_rad_1e4 * rad2deg * 0.0001f;
            float yaw_deg = (float)yaw_rad_1e4 * rad2deg * 0.0001f;
            while (yaw_deg < 0.0f) yaw_deg += 360.0f;
            while (yaw_deg >= 360.0f) yaw_deg -= 360.0f;

            state->att_pitch_deg = pitch_deg;
            state->att_roll_deg = roll_deg;
            state->att_yaw_deg = yaw_deg;

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_i16(APP_STATE_FIELD_PLANE_PITCH, &app_state->plane_pitch, (int16_t)pitch_deg);
            app_state_set_i16(APP_STATE_FIELD_PLANE_ROLL, &app_state->plane_roll, (int16_t)roll_deg);
            app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, (int16_t)yaw_deg);
            app_state_end_update();

            LOG_D(TAG, "Attitude: Pitch=%.1f, Roll=%.1f, Yaw=%.1f", pitch_deg, roll_deg, yaw_deg);
            return true;
        }

        case VARIO_ID: {
            if (frame_length < 6) return false;

            const int16_t vspeed_raw = (int16_t)((frame_data[4] << 0) | (frame_data[3] << 8));
            state->vspeed_ms = (float)vspeed_raw * 0.1f;

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_i16(APP_STATE_FIELD_PLANE_VSPEED, &app_state->plane_vspeed, vspeed_raw);
            app_state_end_update();

            LOG_D(TAG, "Vario: VSpeed=%.2fm/s", state->vspeed_ms);
            return true;
        }

        case BARO_ALT_ID: {
            if (frame_length < 6) return false;

            const int16_t baro_alt_raw = (int16_t)((frame_data[4] << 0) | (frame_data[3] << 8));
            state->baro_alt_m = (float)baro_alt_raw * 0.1f;

            const float now_s = (float)(xTaskGetTickCount() * (portTICK_PERIOD_MS)) / 1000.0f;
            state->t_last_baro = now_s;

            // Нульове баро не вважаємо валідним
            const bool barValid = (baro_alt_raw != 0);
            if (barValid) state->baro_seen_nonzero = true;

            // GPS валідний лише якщо кадр свіжий (<1с від останнього GPS)
            const bool gpsValid = ((now_s - state->t_last_gps) < 1.0f);

            bool usedGps = false, usedBaro = false;
            float fused =
                alt_fuser_update(&state->alt, now_s, gpsValid, state->gps_alt_m, (int)state->gps_sats, NAN, barValid, state->baro_alt_m, &usedGps, &usedBaro);

            // FAILSAFE: баро валідне, але ф’юзер не використав нічого → фіксуємо на баро
            if (!usedGps && !usedBaro && barValid) {
                fused = state->baro_alt_m;
                state->alt.fused = fused;
                state->alt.fusedInit = true;
                usedBaro = true;
            }

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_i32(APP_STATE_FIELD_PLANE_BARO_ALTITUDE, &app_state->plane_baro_altitude, (int32_t)baro_alt_raw);
            app_state_set_i32(APP_STATE_FIELD_PLANE_FUSED_ALTITUDE, &app_state->plane_fused_altitude, (int32_t)lroundf(fused));
            app_state_end_update();

            LOG_D(TAG, "Baro Altitude: %.1fm, Fused=%.1fm (gps:%d,baro:%d)", state->baro_alt_m, fused, usedGps, usedBaro);
            return true;
        }

        case FLIGHT_MODE_ID: {
            const uint8_t frame_len = frame_data[1];
            const uint8_t mode_len = (uint8_t)(frame_len - 3);
            const uint8_t copy_len = (mode_len < sizeof(state->flight_mode)) ? mode_len : (uint8_t)(sizeof(state->flight_mode) - 1);
            if (copy_len > 0) memcpy(state->flight_mode, &frame_data[3], copy_len);
            state->flight_mode[copy_len] = '\0';
            state->flight_mode_len = copy_len;

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            strncpy(app_state->plane_flight_mode, state->flight_mode, sizeof(app_state->plane_flight_mode) - 1);
            app_state->plane_flight_mode[sizeof(app_state->plane_flight_mode) - 1] = '\0';
            app_state_set_u32(APP_STATE_FIELD_PLANE_FLIGHT_MODE, (uint32_t *)&app_state->plane_flight_mode, 1);  // Trigger update
            app_state_end_update();

            LOG_D(TAG, "Flight Mode: %.*s", state->flight_mode_len, state->flight_mode);
            return true;
        }

        case LINK_STATS_ID: {
            if (frame_length < 12) return false;

            state->uplink_rssi_dbm = -(int8_t)frame_data[3];
            state->uplink_lq = frame_data[5];
            state->uplink_snr_db = (int8_t)frame_data[7];
            state->downlink_rssi_dbm = -(int8_t)frame_data[8];
            state->downlink_lq = frame_data[9];
            state->downlink_snr_db = (int8_t)frame_data[10];
            state->rf_power_level = frame_data[11];

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_PLANE_RSSI, &app_state->plane_rssi, state->downlink_lq);
            app_state_set_i8(APP_STATE_FIELD_PLANE_UPLINK_RSSI, &app_state->plane_uplink_rssi, state->uplink_rssi_dbm);
            app_state_set_u8(APP_STATE_FIELD_PLANE_UPLINK_LQ, &app_state->plane_uplink_lq, state->uplink_lq);
            app_state_set_i8(APP_STATE_FIELD_PLANE_UPLINK_SNR, &app_state->plane_uplink_snr, state->uplink_snr_db);
            app_state_set_i8(APP_STATE_FIELD_PLANE_DOWNLINK_RSSI, &app_state->plane_downlink_rssi, state->downlink_rssi_dbm);
            app_state_set_u8(APP_STATE_FIELD_PLANE_DOWNLINK_LQ, &app_state->plane_downlink_lq, state->downlink_lq);
            app_state_set_i8(APP_STATE_FIELD_PLANE_DOWNLINK_SNR, &app_state->plane_downlink_snr, state->downlink_snr_db);
            app_state_set_u8(APP_STATE_FIELD_PLANE_RF_POWER_LEVEL, &app_state->plane_rf_power_level, state->rf_power_level);
            app_state_end_update();

            LOG_D(TAG, "Link: UL_RSSI=%ddBm, UL_LQ=%d%%, DL_RSSI=%ddBm, DL_LQ=%d%%", state->uplink_rssi_dbm, state->uplink_lq, state->downlink_rssi_dbm,
                  state->downlink_lq);
            return true;
        }

        case RC_CHANNELS_ID: {
            if (frame_length < 25) return false;  // Type(1) + 16 channels * 11 bits = 22 bytes + CRC(1) = 24 bytes payload
            const uint8_t *ch_data = &frame_data[3];
            // Unpack 11-bit channels from byte array
            state->rc_channels[0] = ((ch_data[0] | ch_data[1] << 8) & 0x07FF);
            state->rc_channels[1] = ((ch_data[1] >> 3 | ch_data[2] << 5) & 0x07FF);
            state->rc_channels[2] = ((ch_data[2] >> 6 | ch_data[3] << 2 | ch_data[4] << 10) & 0x07FF);
            state->rc_channels[3] = ((ch_data[4] >> 1 | ch_data[5] << 7) & 0x07FF);
            state->rc_channels[4] = ((ch_data[5] >> 4 | ch_data[6] << 4) & 0x07FF);
            state->rc_channels[5] = ((ch_data[6] >> 7 | ch_data[7] << 1 | ch_data[8] << 9) & 0x07FF);
            state->rc_channels[6] = ((ch_data[8] >> 2 | ch_data[9] << 6) & 0x07FF);
            state->rc_channels[7] = ((ch_data[9] >> 5 | ch_data[10] << 3) & 0x07FF);
            state->rc_channels[8] = ((ch_data[11] | ch_data[12] << 8) & 0x07FF);
            state->rc_channels[9] = ((ch_data[12] >> 3 | ch_data[13] << 5) & 0x07FF);
            state->rc_channels[10] = ((ch_data[13] >> 6 | ch_data[14] << 2 | ch_data[15] << 10) & 0x07FF);
            state->rc_channels[11] = ((ch_data[15] >> 1 | ch_data[16] << 7) & 0x07FF);
            state->rc_channels[12] = ((ch_data[16] >> 4 | ch_data[17] << 4) & 0x07FF);
            state->rc_channels[13] = ((ch_data[17] >> 7 | ch_data[18] << 1 | ch_data[19] << 9) & 0x07FF);
            state->rc_channels[14] = ((ch_data[19] >> 2 | ch_data[20] << 6) & 0x07FF);
            state->rc_channels[15] = ((ch_data[20] >> 5 | ch_data[21] << 3) & 0x07FF);

            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            memcpy(app_state->plane_rc_channels, state->rc_channels, sizeof(state->rc_channels));
            app_state_set_u32(APP_STATE_FIELD_PLANE_RC_CHANNELS, (uint32_t *)&app_state->plane_rc_channels, 1);  // Trigger update
            app_state_end_update();

            LOG_D(TAG, "RC Channels: CH1=%u, CH2=%u, CH3=%u, CH4=%u ...", state->rc_channels[0], state->rc_channels[1], state->rc_channels[2],
                  state->rc_channels[3]);
            return true;
        }
        case VTX_ID: {
            if (frame_length < 5) return false;
            state->vtx_band = frame_data[3];
            state->vtx_channel = frame_data[4];
            // More fields can be present depending on VTX type
            if (frame_length >= 6) {
                state->vtx_power = frame_data[5];
            }

            // Update app_state with VTX information
            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_PLANE_VTX_BAND, &app_state->plane_vtx_band, state->vtx_band);
            app_state_set_u8(APP_STATE_FIELD_PLANE_VTX_CHANNEL, &app_state->plane_vtx_channel, state->vtx_channel);
            app_state_set_u8(APP_STATE_FIELD_PLANE_VTX_POWER, &app_state->plane_vtx_power, state->vtx_power);
            app_state_end_update();

            LOG_D(TAG, "VTX: Band=%d, Chan=%d, PwrLvl=%d", state->vtx_band, state->vtx_channel, state->vtx_power);
            return true;
        }
        case HEARTBEAT_ID: {
            if (frame_length < 5) return false;
            state->origin_addr = frame_data[3];
            state->destination_addr = frame_data[4];
            LOG_D(TAG, "Heartbeat from 0x%02X to 0x%02X", state->origin_addr, state->destination_addr);
            return true;
        }
        case DEVICE_INFO_ID: {
            // Frame is: Dest, Origin, Type, 'CRSF', 12-bytes name, 4-byte serial, version, params...
            if (frame_length < 24) return false;
            char device_name[13];
            memcpy(device_name, &frame_data[5], 12);
            device_name[12] = '\0';
            LOG_I(TAG, "Device Info: Name='%s' from address 0x%02X", device_name, frame_data[4]);
            return true;
        }

        case ESC_TELEMETRY_ID: {
            if (frame_length < 13) return false;  // ID(1) + Payload(11) + CRC(1)

            const uint8_t esc_index = frame_data[3];
            if (esc_index < 4) {  // Припускаємо, що у нас до 4 ESC
                // Дані упаковані як Big Endian
                const int32_t rpm = (int32_t)(((uint32_t)frame_data[7] << 0) | ((uint32_t)frame_data[6] << 8) | ((uint32_t)frame_data[5] << 16) |
                                              ((uint32_t)frame_data[4] << 24));
                const uint16_t voltage = (uint16_t)((frame_data[9] << 0) | (frame_data[8] << 8));
                const uint16_t current = (uint16_t)((frame_data[11] << 0) | (frame_data[10] << 8));
                const uint8_t temperature = frame_data[12];

                // Зберігаємо дані
                state->esc_telemetry[esc_index].rpm = rpm / 100;  // зазвичай передається як eRPM/100
                state->esc_telemetry[esc_index].voltage_v = (float)voltage * 0.1f;
                state->esc_telemetry[esc_index].current_a = (float)current * 0.1f;
                state->esc_telemetry[esc_index].temperature_c = temperature;

                // Update app_state with ESC telemetry
                app_state_t *app_state = app_state_get_instance();
                app_state_begin_update();

                if (esc_index == 0) {
                    app_state_set_i32(APP_STATE_FIELD_PLANE_ESC_RPM, &app_state->plane_esc_rpm, state->esc_telemetry[esc_index].rpm);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC_VOLTAGE, &app_state->plane_esc_voltage, voltage);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC_CURRENT, &app_state->plane_esc_current, current);
                    app_state_set_u8(APP_STATE_FIELD_PLANE_ESC_TEMPERATURE, &app_state->plane_esc_temperature, temperature);
                } else if (esc_index == 1) {
                    app_state_set_i32(APP_STATE_FIELD_PLANE_ESC2_RPM, &app_state->plane_esc2_rpm, state->esc_telemetry[esc_index].rpm);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC2_VOLTAGE, &app_state->plane_esc2_voltage, voltage);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC2_CURRENT, &app_state->plane_esc2_current, current);
                    app_state_set_u8(APP_STATE_FIELD_PLANE_ESC2_TEMPERATURE, &app_state->plane_esc2_temperature, temperature);
                } else if (esc_index == 2) {
                    app_state_set_i32(APP_STATE_FIELD_PLANE_ESC3_RPM, &app_state->plane_esc3_rpm, state->esc_telemetry[esc_index].rpm);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC3_VOLTAGE, &app_state->plane_esc3_voltage, voltage);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC3_CURRENT, &app_state->plane_esc3_current, current);
                    app_state_set_u8(APP_STATE_FIELD_PLANE_ESC3_TEMPERATURE, &app_state->plane_esc3_temperature, temperature);
                } else if (esc_index == 3) {
                    app_state_set_i32(APP_STATE_FIELD_PLANE_ESC4_RPM, &app_state->plane_esc4_rpm, state->esc_telemetry[esc_index].rpm);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC4_VOLTAGE, &app_state->plane_esc4_voltage, voltage);
                    app_state_set_u16(APP_STATE_FIELD_PLANE_ESC4_CURRENT, &app_state->plane_esc4_current, current);
                    app_state_set_u8(APP_STATE_FIELD_PLANE_ESC4_TEMPERATURE, &app_state->plane_esc4_temperature, temperature);
                }

                app_state_end_update();

                LOG_D(TAG, "ESC #%d: RPM=%d, V=%.1f, A=%.1f, Temp=%dC", esc_index, state->esc_telemetry[esc_index].rpm,
                      state->esc_telemetry[esc_index].voltage_v, state->esc_telemetry[esc_index].current_a, state->esc_telemetry[esc_index].temperature_c);
            }
            return true;
        }

        default:
            LOG_W(TAG, "Unhandled frame ID: 0x%02X", crsf_id);
            return false;
    }
}
