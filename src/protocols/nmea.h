#pragma once

#include "gps/gps.h"
#include "io/io.h"
#include "protocols/protocol_parser.h"  // Include the common interface
#include "util/time-util.h"

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

// The only public function needed to create an instance
void nmea_parser_init(protocol_parser_t *parser);