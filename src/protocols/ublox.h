#pragma once

#include "io/io.h"
#include "protocols/protocol_parser.h"  // Include the common interface
#include "util/time-util.h"

// Forward declaration
typedef struct atp_s atp_t;

#define UBLOX_BUFFER_SIZE 256

// The internal state of the UBLOX decoder
typedef struct {
    io_t *io;
    bool home_source;

    uint8_t buf[UBLOX_BUFFER_SIZE];
    int buf_pos;

    time_micros_t last_frame_recv;
} ublox_state_t;

// The public-facing struct for the entire UBLOX module
typedef struct {
    protocol_parser_t parser;  // The base parser "class"
    ublox_state_t state;       // The internal state
    atp_t *atp_ctx;            // Context for updating app_state
} ublox_parser_t;

// The only public function needed to create an instance
void ublox_parser_init(ublox_parser_t *instance, atp_t *atp_ctx);
