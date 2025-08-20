#include "ublox.h"

#include <log.h>
#include <string.h>

#include "app_state.h"

static const char *TAG = "UBLOX";

// Internal vtable functions
static void ublox_parser_internal_init(void *parser_state);
static void ublox_parser_internal_process_byte(void *parser_state, uint8_t byte);
static void ublox_parser_internal_destroy(void *parser_state);

// Internal helper function for UBLOX frame processing
static bool ublox_process_frame(ublox_state_t *state, uint8_t msg_class, uint8_t msg_id,
                                const uint8_t *frame_data, size_t frame_length);

// The only public function needed to create an instance
void ublox_parser_init(ublox_parser_t *instance, atp_t *atp_ctx) {
    if (!instance) {
        LOG_E(TAG, "Invalid instance pointer");
        return;
    }

    // Initialize the base parser structure
    instance->parser.state = &instance->state;
    instance->parser.vtable.init = ublox_parser_internal_init;
    instance->parser.vtable.process_byte = ublox_parser_internal_process_byte;
    instance->parser.vtable.destroy = ublox_parser_internal_destroy;
    instance->parser.is_initialized = false;

    // Initialize the internal state
    memset(&instance->state, 0, sizeof(ublox_state_t));
    instance->state.buf_pos = 0;
    instance->state.last_frame_recv = 0;
    instance->state.step = 0;
    instance->state.payload_length = 0;
    instance->state.msg_class = 0;
    instance->state.msg_id = 0;
    instance->state.ck_a = 0;
    instance->state.ck_b = 0;

    // Store the ATP context for app_state updates
    instance->atp_ctx = atp_ctx;

    LOG_I(TAG, "UBLOX parser instance initialized");
}

// Internal vtable implementation: Initialize parser state
static void ublox_parser_internal_init(void *parser_state) {
    ublox_state_t *state = (ublox_state_t *)parser_state;
    if (!state) return;

    // Reset the parser state
    state->buf_pos = 0;
    state->last_frame_recv = 0;
    state->step = 0;
    state->payload_length = 0;
    state->msg_class = 0;
    state->msg_id = 0;
    state->ck_a = 0;
    state->ck_b = 0;

    memset(state->buf, 0, sizeof(state->buf));

    LOG_D(TAG, "UBLOX parser state initialized");
}

// Internal vtable implementation: Process a single byte
static void ublox_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    ublox_state_t *state = (ublox_state_t *)parser_state;
    if (!state) return;

    switch (state->step) {
        case 0:  // Sync char 1
            if (byte == 0xB5) {
                state->step = 1;
            }
            break;
        case 1:  // Sync char 2
            if (byte == 0x62) {
                state->step = 2;
            } else {
                state->step = 0;
            }
            break;
        case 2:  // Class
            state->msg_class = byte;
            state->ck_a = byte;
            state->ck_b = state->ck_a;
            state->step = 3;
            break;
        case 3:  // ID
            state->msg_id = byte;
            state->ck_a += byte;
            state->ck_b += state->ck_a;
            state->step = 4;
            break;
        case 4:  // Length LSB
            state->payload_length = byte;
            state->ck_a += byte;
            state->ck_b += state->ck_a;
            state->step = 5;
            break;
        case 5:  // Length MSB
            state->payload_length |= ((uint16_t)byte << 8);
            state->ck_a += byte;
            state->ck_b += state->ck_a;
            state->buf_pos = 0;
            if (state->payload_length > UBLOX_BUFFER_SIZE) {
                LOG_W(TAG, "UBLOX payload too large: %u", (unsigned)state->payload_length);
                state->step = 0;
            } else {
                state->step = 6;
            }
            break;
        case 6:  // Payload
            state->buf[state->buf_pos++] = byte;
            state->ck_a += byte;
            state->ck_b += state->ck_a;
            if (state->buf_pos >= state->payload_length) {
                state->step = 7;
            }
            break;
        case 7:  // Checksum A
            if (byte == state->ck_a) {
                state->step = 8;
            } else {
                LOG_W(TAG, "UBLOX checksum A mismatch");
                state->step = 0;
            }
            break;
        case 8:  // Checksum B
            if (byte == state->ck_b) {
                time_micros_t now = time_micros_now();
                state->last_frame_recv = now;
                ublox_process_frame(state, state->msg_class, state->msg_id, state->buf,
                                    state->payload_length);
            } else {
                LOG_W(TAG, "UBLOX checksum B mismatch");
            }
            state->step = 0;
            break;
        default:
            state->step = 0;
            break;
    }
}

// Internal vtable implementation: Destroy parser state
static void ublox_parser_internal_destroy(void *parser_state) {
    ublox_state_t *state = (ublox_state_t *)parser_state;
    if (!state) return;

    // Reset the state
    memset(state, 0, sizeof(ublox_state_t));
    LOG_D(TAG, "UBLOX parser state destroyed");
}

// Internal helper function: Process a complete UBLOX frame
static bool ublox_process_frame(ublox_state_t *state, uint8_t msg_class, uint8_t msg_id,
                                const uint8_t *frame_data, size_t frame_length) {
    if (!frame_data || frame_length == 0) {
        return false;
    }

    // Handle only NAV-PVT messages for now
    if (msg_class == 0x01 && msg_id == 0x07 && frame_length >= 92) {
        int32_t lon = 0, lat = 0, h_msl = 0, gspeed = 0, heading = 0;
        memcpy(&lon, frame_data + 24, sizeof(int32_t));
        memcpy(&lat, frame_data + 28, sizeof(int32_t));
        memcpy(&h_msl, frame_data + 36, sizeof(int32_t));
        memcpy(&gspeed, frame_data + 60, sizeof(int32_t));
        memcpy(&heading, frame_data + 64, sizeof(int32_t));

        app_state_t *app_state = app_state_get_instance();
        app_state_begin_update();
        app_state_set_i32(APP_STATE_FIELD_PLANE_LONGITUDE, &app_state->plane_longitude, lon);
        app_state_set_i32(APP_STATE_FIELD_PLANE_LATITUDE, &app_state->plane_latitude, lat);
        app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, h_msl / 1000);
        app_state_set_i16(APP_STATE_FIELD_PLANE_SPEED, &app_state->plane_speed, (int16_t)(gspeed / 1000));
        app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, (int16_t)(heading / 100000));
        app_state_end_update();

        LOG_D(TAG,
              "UBLOX NAV-PVT: Lat=%d Lon=%d Alt=%dm Speed=%dm/s Heading=%d",
              lat, lon, h_msl / 1000, gspeed / 1000, heading / 100000);

        return true;
    }

    LOG_D(TAG, "UBLOX message class 0x%02X id 0x%02X not handled", msg_class, msg_id);
    return false;
}
