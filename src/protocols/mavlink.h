#pragma once

#include "io/io.h"
#include "protocols/protocol_parser.h"  // Include the common interface
#include "util/time-util.h"

#ifdef __cplusplus
extern "C" {
#endif

// Simple placeholder sizes for buffering MAVLink bytes
#define MAVLINK_BUFFER_SIZE 280

// The internal state of the MAVLink decoder
typedef struct {
    io_t *io;

    uint8_t buf[MAVLINK_BUFFER_SIZE * 2];
    int buf_pos;

    time_micros_t last_frame_recv;
} mavlink_state_t;

// The only public function needed to create an instance
void mavlink_parser_init(protocol_parser_t *parser);

#ifdef __cplusplus
}
#endif
