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
    if (!frame_data || frame_length < 10) {
        return false;
    }

    // Extract basic MAVLink frame information
    uint8_t start_byte = frame_data[0];
    uint8_t payload_length = frame_data[1];
    uint8_t sequence = frame_data[2];
    uint8_t system_id = frame_data[3];
    uint8_t component_id = frame_data[4];
    uint8_t message_id = frame_data[5];

    LOG_D(TAG, "frame: start=0x%02X, len=%d, seq=%d, sys=%d, comp=%d, msg=%d", start_byte, payload_length, sequence, system_id, component_id, message_id);

    return true;
}
