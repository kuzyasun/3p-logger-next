#pragma once

#include "gps/gps.h"
#include "io/io.h"
#include "protocols/protocol_parser.h"  // Include the common interface
#include "util/time-util.h"

// Forward declaration
typedef struct atp_s atp_t;

#define NMEA_FRAME_SIZE_MAX 267

// The internal state of the NMEA decoder
typedef struct {
    io_t *io;
    uint8_t buf[NMEA_FRAME_SIZE_MAX * 2];
    int buf_pos;
    bool home_source;

    gps_t *gps;

    time_micros_t last_frame_recv;
} nmea_state_t;

// The public-facing struct for the entire NMEA module
typedef struct {
    protocol_parser_t parser;  // The base parser "class"
    nmea_state_t state;        // The internal state
    atp_t *atp_ctx;            // Context for updating app_state
} nmea_parser_t;

// The only public function needed to create an instance
void nmea_parser_init(nmea_parser_t *instance, atp_t *atp_ctx);