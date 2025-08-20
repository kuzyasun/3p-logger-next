#pragma once

#include "io/io.h"
#include "protocols/protocol_parser.h"
#include "util/time-util.h"

// Forward declaration
typedef struct atp_s atp_t;

#define MSP_BUFFER_SIZE 64

// Internal MSP parser state
typedef struct {
    io_t *io;
    uint8_t buf[MSP_BUFFER_SIZE];
    uint8_t buf_pos;
    uint8_t step;
    uint8_t data_len;
    uint8_t cmd;
    uint8_t checksum;
    time_micros_t last_frame_recv;
} msp_state_t;

// Public MSP parser structure
typedef struct {
    protocol_parser_t parser;
    msp_state_t state;
    atp_t *atp_ctx;
} msp_parser_t;

void msp_parser_init(msp_parser_t *instance, atp_t *atp_ctx);
