#include "nmea.h"

#include <log.h>
#include <stdlib.h>
#include <string.h>

#include "app_state.h"

static gps_t gps;

static const char *TAG = "NMEA";

// Internal vtable functions
static void nmea_parser_internal_init(void *parser_state);
static void nmea_parser_internal_process_byte(void *parser_state, uint8_t byte);
static void nmea_parser_internal_destroy(void *parser_state);

// Internal helper function for NMEA sentence processing
static bool nmea_process_sentence(nmea_state_t *state, const uint8_t *sentence_data, size_t sentence_length);

// The only public function needed to create an instance
void nmea_parser_init(protocol_parser_t *parser) {
    if (!parser) {
        LOG_E(TAG, "Invalid parser pointer");
        return;
    }

    nmea_state_t *state = calloc(1, sizeof(nmea_state_t));
    if (!state) {
        LOG_E(TAG, "Failed to allocate memory for state");
        return;
    }

    parser->state = state;
    parser->vtable.init = nmea_parser_internal_init;
    parser->vtable.process_byte = nmea_parser_internal_process_byte;
    parser->vtable.destroy = nmea_parser_internal_destroy;
    parser->is_initialized = false;

    state->buf_pos = 0;
    state->last_frame_recv = 0;
    state->gps = &gps;

    LOG_I(TAG, "NMEA parser configured for dynamic allocation");
}

// Internal vtable implementation: Initialize parser state
static void nmea_parser_internal_init(void *parser_state) {
    nmea_state_t *state = (nmea_state_t *)parser_state;
    if (!state) return;

    // Reset the parser state
    state->buf_pos = 0;
    state->last_frame_recv = 0;

    memset(state->buf, 0, sizeof(state->buf));

    LOG_D(TAG, "parser state initialized");
}

// Internal vtable implementation: Process a single byte
static void nmea_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    nmea_state_t *state = (nmea_state_t *)parser_state;
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

    // Check for NMEA sentence end (CR+LF or just LF)
    if (byte == '\n' || byte == '\r') {
        if (state->buf_pos > 1) {
            // We have a complete sentence
            time_micros_t now = time_micros_now();
            state->last_frame_recv = now;

            // Process the complete sentence
            if (nmea_process_sentence(state, state->buf, state->buf_pos)) {
                LOG_D(TAG, "sentence processed successfully, length: %d", state->buf_pos);
            } else {
                LOG_W(TAG, "Failed to process sentence");
            }

            // Reset buffer for next sentence
            state->buf_pos = 0;
        }
    }
}

// Internal vtable implementation: Destroy parser state
static void nmea_parser_internal_destroy(void *parser_state) {
    if (!parser_state) return;
    free(parser_state);
    LOG_D(TAG, "parser state destroyed and memory freed");
}

// Internal helper function: Process a complete NMEA sentence
static bool nmea_process_sentence(nmea_state_t *state, const uint8_t *sentence_data, size_t sentence_length) {
    if (!sentence_data || sentence_length == 0) {
        return false;
    }

    // Process the sentence through the GPS module
    if (gps_process(state->gps, (uint8_t *)sentence_data, sentence_length)) {
        app_state_t *app_state = app_state_get_instance();

        // Check if we have a valid GPS fix
        if (state->gps->fix_mode >= 3) {
            int32_t new_lat = (int32_t)(state->gps->latitude * 10000000.0f);
            int32_t new_lon = (int32_t)(state->gps->longitude * 10000000.0f);

            // Check if position has changed
            if (app_state->plane_latitude != new_lat || app_state->plane_longitude != new_lon) {
                time_micros_t now = time_micros_now();

                // Update plane position
                app_state_begin_update();
                app_state_set_i32(APP_STATE_FIELD_PLANE_LONGITUDE, &app_state->plane_longitude, new_lon);
                app_state_set_i32(APP_STATE_FIELD_PLANE_LATITUDE, &app_state->plane_latitude, new_lat);
                app_state_set_i32(APP_STATE_FIELD_PLANE_ALTITUDE, &app_state->plane_altitude, (int32_t)(state->gps->altitude * 100));
                app_state_set_i16(APP_STATE_FIELD_PLANE_SPEED, &app_state->plane_speed, (int16_t)gps_to_speed(state->gps->speed, gps_speed_mps));
                app_state_set_i16(APP_STATE_FIELD_PLANE_HEADING, &app_state->plane_heading, (int16_t)state->gps->coarse);

                // Additional GPS parameters
                app_state_set_u8(APP_STATE_FIELD_PLANE_STAR, &app_state->plane_star, state->gps->sats_in_use);
                app_state_set_u8(APP_STATE_FIELD_PLANE_FIX, &app_state->plane_fix, state->gps->fix);
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_FIX_MODE, &app_state->plane_gps_fix_mode, state->gps->fix_mode);
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_SATS_IN_VIEW, &app_state->plane_gps_sats_in_view, state->gps->sats_in_view);
                app_state_set_u16(APP_STATE_FIELD_PLANE_GPS_HDOP, &app_state->plane_gps_hdop, (uint16_t)(state->gps->dop_h * 100));
                app_state_set_u16(APP_STATE_FIELD_PLANE_GPS_VDOP, &app_state->plane_gps_vdop, (uint16_t)(state->gps->dop_v * 100));
                app_state_set_u16(APP_STATE_FIELD_PLANE_GPS_PDOP, &app_state->plane_gps_pdop, (uint16_t)(state->gps->dop_p * 100));
                app_state_set_i16(APP_STATE_FIELD_PLANE_GPS_GEO_SEP, &app_state->plane_gps_geo_sep, (int16_t)(state->gps->geo_sep * 100));
                app_state_set_i16(APP_STATE_FIELD_PLANE_GPS_MAG_VARIATION, &app_state->plane_gps_mag_variation, (int16_t)(state->gps->variation * 10));

                // GPS time and date
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_TIME_HOURS, &app_state->plane_gps_time_hours, state->gps->hours);
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_TIME_MINUTES, &app_state->plane_gps_time_minutes, state->gps->minutes);
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_TIME_SECONDS, &app_state->plane_gps_time_seconds, state->gps->seconds);
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_DATE_DAY, &app_state->plane_gps_date_day, state->gps->date);
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_DATE_MONTH, &app_state->plane_gps_date_month, state->gps->month);
                app_state_set_u8(APP_STATE_FIELD_PLANE_GPS_DATE_YEAR, &app_state->plane_gps_date_year, state->gps->year);

                app_state_end_update();

                LOG_D(TAG, "Plane: Lat=%.6f, Lon=%.6f, Alt=%.1f, Speed=%.1f, Course=%.1f", state->gps->latitude, state->gps->longitude, state->gps->altitude,
                      state->gps->speed, state->gps->coarse);

                return true;
            }
        }
    }

    return false;
}