#pragma once

#include <hal/gpio_base.h>

#include "protocols/protocol.h"
#include "protocols/protocol_parser.h"
#include "serial.h"
#include "util/time-util.h"

// This struct was moved from tracker.h and renamed to break the dependency.
typedef struct io_uart_s {
    uint8_t com;
    bool io_runing;
    bool invalidate_input;
    bool invalidate_output;
    hal_gpio_t gpio_tx;
    hal_gpio_t gpio_rx;
    int baudrate;
    protocol_e protocol;
    protocol_io_type_e io_type;

    serial_port_t *serial_port;

} io_uart_t;

typedef struct io_manager_s {
    io_uart_t uart1;
    io_uart_t uart2;
} io_manager_t;

void io_manager_init(io_manager_t *iom);

void io_manager_task(void *arg);