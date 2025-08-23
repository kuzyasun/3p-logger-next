#include "mavlink.h"

#include <log.h>
#include <stdlib.h>
#include <string.h>

#include "app_state.h"
#include "mavlink/common/mavlink.h"

static const char *TAG = "MAVLINK";

// Internal vtable functions
static void mavlink_parser_internal_init(void *parser_state);
static void mavlink_parser_internal_process_byte(void *parser_state, uint8_t byte);
static void mavlink_parser_internal_destroy(void *parser_state);

// Internal helper function for MAVLink frame processing
static bool mavlink_process_frame(mavlink_state_t *state, const uint8_t *frame_data, size_t frame_length);

// The only public function needed to create an instance
void mavlink_parser_init(mavlink_parser_t *instance) {
    if (!instance) {
        LOG_E(TAG, "Invalid instance pointer");
        return;
    }

    // Initialize the base parser structure
    instance->parser.state = &instance->state;
    instance->parser.vtable.init = mavlink_parser_internal_init;
    instance->parser.vtable.process_byte = mavlink_parser_internal_process_byte;
    instance->parser.vtable.destroy = mavlink_parser_internal_destroy;
    instance->parser.is_initialized = false;

    // Initialize the internal state
    memset(&instance->state, 0, sizeof(mavlink_state_t));
    instance->state.buf_pos = 0;
    instance->state.last_frame_recv = 0;

    LOG_I(TAG, "parser instance initialized");
}

// Internal vtable implementation: Initialize parser state
static void mavlink_parser_internal_init(void *parser_state) {
    mavlink_state_t *state = (mavlink_state_t *)parser_state;
    if (!state) return;

    // Reset the parser state
    state->buf_pos = 0;
    state->last_frame_recv = 0;

    memset(state->buf, 0, sizeof(state->buf));

    LOG_D(TAG, "parser state initialized");
}

// Internal vtable implementation: Process a single byte
static void mavlink_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    mavlink_state_t *state = (mavlink_state_t *)parser_state;
    if (!state) return;

    // Add byte to buffer
    if (state->buf_pos < (int)sizeof(state->buf)) {
        state->buf[state->buf_pos++] = byte;
    } else {
        // Buffer overflow, reset
        state->buf_pos = 0;
        LOG_W(TAG, "buffer overflow, resetting");
        return;
    }

    // Check for MAVLink frame start (0xFE for v1.0, 0xFD for v2.0)
    if (state->buf_pos == 1) {
        if (byte != 0xFE && byte != 0xFD) {
            // Not a valid MAVLink start byte, reset
            state->buf_pos = 0;
            return;
        }
    }

    // If we have enough bytes for a minimal MAVLink frame (start + length + ...)
    if (state->buf_pos >= 8) {
        uint8_t expected_length = state->buf[1];             // Length field
        uint8_t expected_frame_size = expected_length + 10;  // + header + checksum

        if (state->buf_pos >= expected_frame_size) {
            // We have a complete frame
            time_micros_t now = time_micros_now();
            state->last_frame_recv = now;

            // Process the complete frame
            if (mavlink_process_frame(state, state->buf, expected_frame_size)) {
                LOG_D(TAG, "frame processed successfully, length: %d", expected_frame_size);
            } else {
                LOG_W(TAG, "Failed to process frame");
            }

            // Remove the processed frame from buffer
            if (state->buf_pos > expected_frame_size) {
                memmove(state->buf, &state->buf[expected_frame_size], state->buf_pos - expected_frame_size);
                state->buf_pos -= expected_frame_size;
            } else {
                state->buf_pos = 0;
            }
        }
    }
}

// Internal vtable implementation: Destroy parser state
static void mavlink_parser_internal_destroy(void *parser_state) {
    mavlink_state_t *state = (mavlink_state_t *)parser_state;
    if (!state) return;

    // Reset the state
    memset(state, 0, sizeof(mavlink_state_t));
    LOG_D(TAG, "parser state destroyed");
}

// Internal helper function: Process a complete MAVLink frame
static bool mavlink_process_frame(mavlink_state_t *state, const uint8_t *frame_data, size_t frame_length) {
    if (!frame_data || frame_length < 8) {
        return false;
    }

    mavlink_message_t msg;
    mavlink_status_t status;
    bool msg_received = false;

    // Спробуємо розпарсити кожен байт кадру через бібліотеку MAVLink
    // Це надійний спосіб, що також перевіряє контрольну суму (CRC)
    for (size_t i = 0; i < frame_length; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, frame_data[i], &msg, &status)) {
            msg_received = true;
            break;  // Повідомлення успішно розпарсено
        }
    }

    if (!msg_received) {
        LOG_W(TAG, "Failed to parse MAVLink frame or CRC mismatch");
        return false;
    }

    app_state_t *app_state = app_state_get_instance();

    LOG_D(TAG, "Received MAVLink MSG ID: %d", msg.msgid);

    // Обробляємо повідомлення в залежності від його ID
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {  // ID: 33
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);

            // Розрахунок горизонтальної швидкості з векторів vx, vy
            float ground_speed_mps = sqrtf((float)pos.vx * (float)pos.vx + (float)pos.vy * (float)pos.vy) / 100.0f;

            app_state_begin_update();
            app_state_set_i32(APP_STATE_FIELD_PLANE_LATITUDE, &app_state->plane_latitude, pos.lat);
            app_state_set_i32(APP_STATE_FIELD_PLANE_LONGITUDE, &app_state->plane_longitude, pos.lon);
            app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, pos.alt / 1000);                     // в метри
            app_state_set_i32(APP_STATE_FIELD_PLANE_BARO_ALTITUDE, &app_state->plane_baro_altitude, pos.relative_alt / 1000);  // в метри
            app_state_set_i16(APP_STATE_FIELD_PLANE_SPEED, &app_state->plane_speed, (int16_t)(ground_speed_mps * 100));        // в см/с
            app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, pos.hdg / 100);                        // в градуси
            app_state_end_update();

            LOG_D(TAG, "GPS: Lat=%d, Lon=%d, Alt=%dm, Spd=%.2fm/s", pos.lat, pos.lon, pos.alt / 1000, ground_speed_mps);
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD: {  // ID: 74
            mavlink_vfr_hud_t hud;
            mavlink_msg_vfr_hud_decode(&msg, &hud);

            app_state_begin_update();
            // hud.groundspeed в м/с, hud.alt в метрах, hud.climb в м/с
            app_state_set_i16(APP_STATE_FIELD_PLANE_SPEED, &app_state->plane_speed, (int16_t)(hud.groundspeed * 100));  // в см/с
            app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, (int32_t)hud.alt);
            app_state_set_i16(APP_STATE_FIELD_PLANE_VSPEED, &app_state->plane_vspeed, (int16_t)(hud.climb * 100));  // в см/с
            app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, (int16_t)hud.heading);      // в градуси
            app_state_end_update();

            LOG_D(TAG, "HUD: Alt=%.1fm, Spd=%.1fm/s, VSpd=%.1fm/s", hud.alt, hud.groundspeed, hud.climb);
            break;
        }

        case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {  // ID: 35
            mavlink_rc_channels_raw_t rc;
            mavlink_msg_rc_channels_raw_decode(&msg, &rc);

            uint16_t rc_channels[8] = {rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw, rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw};

            // Залежно від вашої реалізації app_state, тут ви можете зберегти ці значення
            // memcpy(app_state->plane_rc_channels, rc_channels, sizeof(rc_channels));

            LOG_D(TAG, "RC Raw: CH1=%u, CH2=%u, CH3=%u, CH4=%u", rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw);
            break;
        }

        default:
            // Це повідомлення ми не обробляємо, але воно валідне
            return true;
    }

    return true;
}
