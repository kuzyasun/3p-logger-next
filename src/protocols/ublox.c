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
static bool ublox_process_frame(ublox_state_t *state, const uint8_t *frame_data, size_t frame_length);

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

    // Store the ATP context for app_state updates
    instance->atp_ctx = atp_ctx;

    LOG_I(TAG, "UBLOX parser instance initialized (stub implementation)");
}

// Internal vtable implementation: Initialize parser state
static void ublox_parser_internal_init(void *parser_state) {
    ublox_state_t *state = (ublox_state_t *)parser_state;
    if (!state) return;

    // Reset the parser state
    state->buf_pos = 0;
    state->last_frame_recv = 0;

    memset(state->buf, 0, sizeof(state->buf));

    LOG_D(TAG, "UBLOX parser state initialized");
}

// Internal vtable implementation: Process a single byte
static void ublox_parser_internal_process_byte(void *parser_state, uint8_t byte) {
    ublox_state_t *state = (ublox_state_t *)parser_state;
    if (!state) return;

    // Add byte to buffer
    if (state->buf_pos < (int)sizeof(state->buf)) {
        state->buf[state->buf_pos++] = byte;
    } else {
        // Buffer overflow, reset
        state->buf_pos = 0;
        LOG_W(TAG, "UBLOX buffer overflow, resetting");
        return;
    }

    // TODO: Implement UBLOX frame detection and processing
    // For now, just log that we received a byte
    LOG_D(TAG, "UBLOX received byte: 0x%02X (stub implementation)", byte);

    // Reset buffer after a few bytes for now (placeholder behavior)
    if (state->buf_pos >= 10) {
        state->buf_pos = 0;
        LOG_D(TAG, "UBLOX buffer reset (stub implementation)");
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
static bool ublox_process_frame(ublox_state_t *state, const uint8_t *frame_data, size_t frame_length) {
    if (!frame_data || frame_length == 0) {
        return false;
    }

    // TODO: Implement actual UBLOX frame processing
    // This is a stub implementation that just logs the frame
    LOG_D(TAG, "UBLOX frame received, length: %zu (stub implementation)", frame_length);

    // TODO: Parse UBLOX messages and update app_state
    // Example structure for future implementation:
    // - Check for UBLOX sync bytes (0xB5 0x62)
    // - Parse message class and ID
    // - Handle different message types (NAV-PVT, NAV-POSLLH, etc.)
    // - Update app_state with GPS data

    return false;  // Return false for now since this is a stub
}
