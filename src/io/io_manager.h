#pragma once

#include <hal/gpio_base.h>
#include "hal_spi.h"

#include "protocols/protocol.h"
#include "protocols/protocol_parser.h"
#include "serial.h"
#include "util/time-util.h"

// Forward declaration to avoid circular dependency with app_logic.h
struct app_logic_s;

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

    // Active parser assigned to this UART instance
    protocol_parser_t *parser;

} io_uart_t;

typedef struct io_manager_s {
    io_uart_t uart1;
    io_uart_t uart2;
    hal_spi_bus_t accel_spi_bus;
    hal_spi_bus_t sdcard_spi_bus;
} io_manager_t;

hal_err_t io_manager_init(io_manager_t *iom);

// Configure and open a UART port with a specific protocol parser
void io_manager_configure_uart(io_manager_t *iom,
                               io_uart_t *uart,
                               struct app_logic_s *app_logic,
                               protocol_e protocol,
                               int baudrate,
                               hal_gpio_t rx_pin,
                               hal_gpio_t tx_pin);

void io_manager_task(void *arg);
