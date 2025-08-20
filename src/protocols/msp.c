#include "msp.h"

#include <log.h>
#include <string.h>

#include "app_state.h"

static const char *TAG = "MSP";

static void msp_parser_internal_init(void *parser_state);
static void msp_parser_internal_process_byte(void *parser_state, uint8_t byte);
static void msp_parser_internal_destroy(void *parser_state);
static void msp_process_packet(msp_state_t *state, uint8_t cmd, const uint8_t *payload, size_t len);

void msp_parser_init(msp_parser_t *instance, atp_t *atp_ctx) {
    if (!instance) {
        LOG_E(TAG, "Invalid instance pointer");
        return;
    }

    instance->parser.state = &instance->state;
    instance->parser.vtable.init = msp_parser_internal_init;
    instance->parser.vtable.process_byte = msp_parser_internal_process_byte;
    instance->parser.vtable.destroy = msp_parser_internal_destroy;
    instance->parser.is_initialized = false;

    memset(&instance->state, 0, sizeof(msp_state_t));
    instance->atp_ctx = atp_ctx;

    LOG_I(TAG, "MSP parser instance initialized");
}

static void msp_parser_internal_init(void *parser_state) {
    msp_state_t *state = (msp_state_t *)parser_state;
    if (!state) return;

    state->buf_pos = 0;
    state->step = 0;
    state->data_len = 0;
    state->cmd = 0;
    state->checksum = 0;
    state->last_frame_recv = 0;
    memset(state->buf, 0, sizeof(state->buf));

    LOG_D(TAG, "MSP parser state initialized");
}

static void msp_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    msp_state_t *state = (msp_state_t *)parser_state;
    if (!state) return;

    switch (state->step) {
        case 0:  // '$'
            if (byte == '$') state->step = 1;
            break;
        case 1:  // 'M'
            if (byte == 'M') state->step = 2; else state->step = 0;
            break;
        case 2:  // direction
            if (byte == '>' || byte == '<') state->step = 3; else state->step = 0;
            break;
        case 3:  // data length
            state->data_len = byte;
            state->checksum = byte;
            state->buf_pos = 0;
            state->step = 4;
            break;
        case 4:  // command
            state->cmd = byte;
            state->checksum ^= byte;
            state->step = (state->data_len > 0) ? 5 : 6;
            break;
        case 5:  // payload
            if (state->buf_pos < MSP_BUFFER_SIZE) {
                state->buf[state->buf_pos++] = byte;
            }
            state->checksum ^= byte;
            if (state->buf_pos >= state->data_len) {
                state->step = 6;
            }
            break;
        case 6:  // checksum
            if (byte == state->checksum) {
                time_micros_t now = time_micros_now();
                state->last_frame_recv = now;
                msp_process_packet(state, state->cmd, state->buf, state->data_len);
            } else {
                LOG_W(TAG, "MSP checksum mismatch");
            }
            state->step = 0;
            break;
        default:
            state->step = 0;
            break;
    }
}

static void msp_parser_internal_destroy(void *parser_state) {
    msp_state_t *state = (msp_state_t *)parser_state;
    if (!state) return;
    memset(state, 0, sizeof(msp_state_t));
    LOG_D(TAG, "MSP parser state destroyed");
}

#define MSP_RAW_GPS 106

static void msp_process_packet(msp_state_t *state, uint8_t cmd, const uint8_t *payload, size_t len) {
    if (cmd == MSP_RAW_GPS && len >= 16) {
        uint8_t fix = payload[0];
        uint8_t numsat = payload[1];
        int32_t lat = 0, lon = 0;
        uint16_t alt = 0, spd = 0, course = 0;
        memcpy(&lat, payload + 2, sizeof(int32_t));
        memcpy(&lon, payload + 6, sizeof(int32_t));
        memcpy(&alt, payload + 10, sizeof(uint16_t));
        memcpy(&spd, payload + 12, sizeof(uint16_t));
        memcpy(&course, payload + 14, sizeof(uint16_t));

        if (fix) {
            app_state_t *app_state = app_state_get_instance();
            app_state_begin_update();
            app_state_set_i32(APP_STATE_FIELD_PLANE_LONGITUDE, &app_state->plane_longitude, lon);
            app_state_set_i32(APP_STATE_FIELD_PLANE_LATITUDE, &app_state->plane_latitude, lat);
            app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, (int32_t)alt);
            app_state_set_i16(APP_STATE_FIELD_PLANE_SPEED, &app_state->plane_speed, (int16_t)(spd / 100));
            app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, (int16_t)(course / 10));
            app_state_set_u8(APP_STATE_FIELD_PLANE_STAR, &app_state->plane_star, numsat);
            app_state_set_u8(APP_STATE_FIELD_PLANE_FIX, &app_state->plane_fix, fix);
            app_state_end_update();

            LOG_D(TAG,
                  "MSP RAW_GPS: Lat=%d Lon=%d Alt=%dm Speed=%dm/s Course=%d",
                  lat, lon, alt, spd / 100, course / 10);
        }
    } else {
        LOG_D(TAG, "Unhandled MSP cmd %u", cmd);
    }
}
