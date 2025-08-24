#pragma once

#include "io/io.h"
#include "protocols/protocol_parser.h"  // Include the common interface
#include "util/time-util.h"

#define UBLOX_BUFFER_SIZE 256

// The internal state of the UBLOX decoder
typedef struct {
    io_t *io;
    bool home_source;

    uint8_t buf[UBLOX_BUFFER_SIZE];
    int buf_pos;

    time_micros_t last_frame_recv;
    uint8_t step;
    uint16_t payload_length;
    uint8_t msg_class;
    uint8_t msg_id;
    uint8_t ck_a;
    uint8_t ck_b;
} ublox_state_t;

// The only public function needed to create an instance
void ublox_parser_init(protocol_parser_t *parser);
