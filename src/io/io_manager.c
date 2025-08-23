#include "io_manager.h"

#include <errno.h>
#include <log.h>
#include <stdlib.h>
#include <string.h>
#include <target.h>

#include "app_logic.h"
#include "app_state.h"

#define ACCEL_SPI_HOST SPI2_HOST
#define SDCARD_SPI_HOST SPI3_HOST

static const char *TAG = "IOM";

/**
 * @brief Callback invoked by the serial driver for every received byte.
 */
static void uart_byte_received_callback(const serial_port_t *port, uint8_t b, void *user_data) {
    (void)port;
    io_uart_t *uart = (io_uart_t *)user_data;
    if (uart && uart->parser && uart->parser->vtable.process_byte) {
        uart->parser->vtable.process_byte(uart->parser->state, b);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

hal_err_t io_manager_init(io_manager_t *iom) {
    memset(iom, 0, sizeof(*iom));

    LOG_I(TAG, "Initializing SPI buses...");

    // Initialize SPI bus for Accelerometer
    hal_err_t err = hal_spi_bus_init(ACCEL_SPI_HOST, ACC_SPI_MISO_GPIO, ACC_SPI_MOSI_GPIO, ACC_SPI_CLK_GPIO);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize accelerometer SPI bus");
        return err;
    }
    iom->accel_spi_bus = ACCEL_SPI_HOST;

    // Initialize SPI bus for SD Card
    err = hal_spi_bus_init(SDCARD_SPI_HOST, SD_SPI_MISO_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_CLK_GPIO);
    if (err != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize SD card SPI bus");
        return err;
    }
    iom->sdcard_spi_bus = SDCARD_SPI_HOST;

    LOG_I(TAG, "IO Manager Initialized Successfully");
    iom->initialized = true;
    return HAL_ERR_NONE;
}

void io_manager_configure_uart(io_manager_t *iom, io_uart_t *uart, struct app_logic_s *app_logic, protocol_e protocol, int baudrate, hal_gpio_t rx_pin,
                               hal_gpio_t tx_pin) {
    (void)iom;

    // Select parser based on protocol
    switch (protocol) {
        case PROTOCOL_CRSF:
            uart->parser = &app_logic->crsf_parser.parser;
            break;
        case PROTOCOL_NMEA:
            uart->parser = &app_logic->nmea_parser.parser;
            break;
        case PROTOCOL_UBLOX:
            uart->parser = &app_logic->ublox_parser.parser;
            break;
        case PROTOCOL_MSP:
            uart->parser = &app_logic->msp_parser.parser;
            break;
        case PROTOCOL_MSP_V2:
            uart->parser = &app_logic->msp_v2_parser.parser;
            break;
        case PROTOCOL_MAVLINK:
            uart->parser = &app_logic->mavlink_parser.parser;
            break;
        default:
            uart->parser = NULL;
            LOG_W(TAG, "No parser assigned for protocol ID: %d", protocol);
            break;
    }

    if (uart->parser && !uart->parser->is_initialized && uart->parser->vtable.init) {
        uart->parser->vtable.init(uart->parser->state);
        uart->parser->is_initialized = true;
    }

    uart->protocol = protocol;
    uart->baudrate = baudrate;

    if (rx_pin == HAL_GPIO_NONE) rx_pin = SERIAL_UNUSED_GPIO;
    if (tx_pin == HAL_GPIO_NONE) tx_pin = SERIAL_UNUSED_GPIO;

    uart->gpio_rx = rx_pin;
    uart->gpio_tx = tx_pin;

    const bool rx_only = (uart->gpio_tx == SERIAL_UNUSED_GPIO) && (uart->gpio_rx != SERIAL_UNUSED_GPIO);
    const bool tx_only = (uart->gpio_rx == SERIAL_UNUSED_GPIO) && (uart->gpio_tx != SERIAL_UNUSED_GPIO);

    serial_port_config_t serial_config = {
        .baud_rate = uart->baudrate,
        .rx_pin = uart->gpio_rx,
        .tx_pin = uart->gpio_tx,
        .rx_buffer_size = tx_only ? 0 : 1024,  // TX-only -> no RX buffer
        .tx_buffer_size = rx_only ? 0 : 256,   // RX-only -> no TX buffer
        .inverted = false,
        .parity = SERIAL_PARITY_DISABLE,
        .stop_bits = SERIAL_STOP_BITS_1,
        .byte_callback = uart_byte_received_callback,
        .byte_callback_data = uart,
    };

    uart->serial_port = serial_port_open(&serial_config);

    if (uart->serial_port) {
        uart->io_runing = true;
        LOG_I(TAG, "Successfully configured UART with protocol %d on RX:%d/TX:%d at %d baud.", protocol, rx_pin, tx_pin, baudrate);
    } else {
        uart->io_runing = false;
        LOG_E(TAG, "Failed to open serial port for protocol %d.", protocol);
    }
}

void io_manager_task(void *arg) {
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
