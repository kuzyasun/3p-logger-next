#include "serial.h"

#include <assert.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_intr_alloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <hal/gpio.h>
#include <hal/mutex.h>
#include <string.h>
#include <target.h>

#include "util/macros.h"

// Define UART inverse constants if not already defined
#ifndef UART_INVERSE_TXD
#define UART_INVERSE_TXD (1 << 0)
#endif

#ifndef UART_INVERSE_RXD
#define UART_INVERSE_RXD (1 << 1)
#endif

typedef struct serial_port_s {
    const uart_port_t port_num;

    serial_port_config_t config;
    bool open;
    bool in_write;
    bool uses_driver;
    QueueHandle_t uart_queue;
    TaskHandle_t uart_task_handle;
    uint8_t buf[128];
    unsigned buf_pos;
    mutex_t mutex;
    bool half_duplex_tx_mode;  // Track half-duplex mode
} serial_port_t;

// We support 2 UART ports at maximum, ignoring UART0 since
// it's used for debugging.
static serial_port_t ports[] = {
    {.port_num = UART_NUM_1, .open = false, .in_write = false, .half_duplex_tx_mode = false},
#ifdef UART_NUM_2
    {.port_num = UART_NUM_2, .open = false, .in_write = false, .half_duplex_tx_mode = false},
#endif
};

static void serial_half_duplex_enable_rx(serial_port_t *port) {
    if (port->half_duplex_tx_mode) {
        // Configure GPIO for RX mode
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << port->config.rx_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = port->config.inverted ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY,
            .pull_down_en = port->config.inverted ? GPIO_PULLDOWN_ONLY : GPIO_PULLUP_ONLY,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);

        // Configure UART for RX mode
        uart_set_mode(port->port_num, UART_MODE_UART);

        // Clear any pending data
        uart_flush(port->port_num);

        port->half_duplex_tx_mode = false;
    }
}

static void serial_half_duplex_enable_tx(serial_port_t *port) {
    if (!port->half_duplex_tx_mode) {
        // Configure GPIO for TX mode
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << port->config.tx_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);

        // Set TX pin low initially
        gpio_set_level(port->config.tx_pin, 0);

        // Configure UART for TX mode
        uart_set_mode(port->port_num, UART_MODE_UART);

        port->half_duplex_tx_mode = true;
    }
}

static void serial_uart_event_task(void *pvParameters) {
    serial_port_t *port = (serial_port_t *)pvParameters;
    uart_event_t event;

    for (;;) {
        if (xQueueReceive(port->uart_queue, (void *)&event, port->config.rx_buffer_size)) {
            switch (event.type) {
                case UART_DATA:
                    if (event.size > 0) {
                        uint8_t *data = (uint8_t *)malloc(event.size);
                        if (data) {
                            int len = uart_read_bytes(port->port_num, data, event.size, 0);
                            if (len > 0) {
                                for (int i = 0; i < len; i++) {
                                    if (port->config.byte_callback) {
                                        port->config.byte_callback(port, data[i], port->config.byte_callback_data);
                                    } else {
                                        mutex_lock(&port->mutex);
                                        if (port->buf_pos < sizeof(port->buf)) {
                                            port->buf[port->buf_pos++] = data[i];
                                        }
                                        mutex_unlock(&port->mutex);
                                    }
                                }
                            }
                            free(data);
                        }
                    }
                    break;
                case UART_FIFO_OVF:
                    uart_flush(port->port_num);
                    break;
                case UART_BUFFER_FULL:
                    uart_flush(port->port_num);
                    break;
                case UART_BREAK:
                    break;
                case UART_PARITY_ERR:
                    break;
                case UART_FRAME_ERR:
                    break;
                default:
                    break;
            }
        }
    }
}

void serial_port_do_open(serial_port_t *port) {
    uart_parity_t parity;
    switch (port->config.parity) {
        case SERIAL_PARITY_DISABLE:
            parity = UART_PARITY_DISABLE;
            break;
        case SERIAL_PARITY_EVEN:
            parity = UART_PARITY_EVEN;
            break;
        case SERIAL_PARITY_ODD:
            parity = UART_PARITY_ODD;
            break;
        default:
            assert(0 && "invalid serial_parity_e");
    }

    uart_stop_bits_t stop_bits;
    switch (port->config.stop_bits) {
        case SERIAL_STOP_BITS_1:
            stop_bits = UART_STOP_BITS_1;
            break;
        case SERIAL_STOP_BITS_2:
            stop_bits = UART_STOP_BITS_2;
            break;
        default:
            assert(0 && "invalid serial_stop_bits_e");
    }

    uart_config_t uart_config = {
        .baud_rate = port->config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = parity,
        .stop_bits = stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(port->port_num, &uart_config));

    int tx_pin = port->config.tx_pin;
    int rx_pin = port->config.rx_pin;

    int rx_buffer_size = port->config.rx_buffer_size;
    // Must be 129 at least
    if (rx_buffer_size < 129) {
        rx_buffer_size = 129;
    }

    if (port->config.inverted) {
        ESP_ERROR_CHECK(uart_set_line_inverse(port->port_num, UART_INVERSE_TXD | UART_INVERSE_RXD));
    }

    // Check if we have a valid TX pin (not SERIAL_UNUSED_GPIO)
    bool has_valid_tx = (tx_pin != SERIAL_UNUSED_GPIO);
    bool has_valid_rx = (rx_pin != SERIAL_UNUSED_GPIO);

    if (has_valid_tx && has_valid_rx && tx_pin != rx_pin) {
        // Full duplex mode - both TX and RX pins are valid and different
        port->uses_driver = true;
        ESP_ERROR_CHECK(uart_driver_install(port->port_num, rx_buffer_size, port->config.tx_buffer_size, 10, &port->uart_queue, 0));
        ESP_ERROR_CHECK(uart_set_pin(port->port_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Create event task for UART events
        xTaskCreate(serial_uart_event_task, "uart_event_task", 2048, port, TASK_PRIORITY_UART_EVENT, &port->uart_task_handle);
    } else if (has_valid_rx && !has_valid_tx) {
        // Receive-only mode - only RX pin is valid
        // For ESP32, we need to use a valid GPIO for TX pin, but we can disable TX functionality
        port->uses_driver = true;
        ESP_ERROR_CHECK(uart_driver_install(port->port_num, rx_buffer_size, port->config.tx_buffer_size, 10, &port->uart_queue, 0));

        // Use a valid GPIO pin for TX (we'll use GPIO 21 as a dummy TX pin)
        // This is required because ESP32 uart_set_pin() doesn't accept -1 for TX
        int dummy_tx_pin = SERIAL_UNUSED_GPIO;
        ESP_ERROR_CHECK(uart_set_pin(port->port_num, dummy_tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Disable TX interrupts to prevent any TX-related issues
        ESP_ERROR_CHECK(uart_disable_tx_intr(port->port_num));

        // Create event task for UART events
        xTaskCreate(serial_uart_event_task, "uart_event_task", 2048, port, TASK_PRIORITY_UART_EVENT, &port->uart_task_handle);
    } else if (has_valid_tx && !has_valid_rx) {
        // Transmit-only mode - only TX pin is valid
        port->uses_driver = true;
        ESP_ERROR_CHECK(uart_driver_install(port->port_num, rx_buffer_size, port->config.tx_buffer_size, 10, &port->uart_queue, 0));
        ESP_ERROR_CHECK(uart_set_pin(port->port_num, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Create event task for UART events
        xTaskCreate(serial_uart_event_task, "uart_event_task", 2048, port, TASK_PRIORITY_UART_EVENT, &port->uart_task_handle);
    } else {
        // Half-duplex mode - same pin for TX and RX, or invalid configuration
        port->uses_driver = false;
        port->buf_pos = 0;
        port->half_duplex_tx_mode = false;

        // For half-duplex mode, we need to manually configure GPIO
        // and use the UART driver in a special way
        ESP_ERROR_CHECK(uart_driver_install(port->port_num, rx_buffer_size, port->config.tx_buffer_size, 10, &port->uart_queue, 0));

        // Start in RX mode
        serial_half_duplex_enable_rx(port);

        // Create event task for UART events
        xTaskCreate(serial_uart_event_task, "uart_event_task", 2048, port, TASK_PRIORITY_UART_EVENT, &port->uart_task_handle);
    }
    port->open = true;
    port->in_write = false;
}

serial_port_t *serial_port_open(const serial_port_config_t *config) {
    // Find a free UART
    serial_port_t *port = NULL;
    for (int ii = 0; ii < ARRAY_COUNT(ports); ii++) {
        if (!ports[ii].open) {
            port = &ports[ii];
            break;
        }
    }
    assert(port);
    mutex_open(&port->mutex);
    port->config = *config;
    serial_port_do_open(port);
    return port;
}

int serial_port_read(serial_port_t *port, void *buf, size_t size, time_ticks_t timeout) {
    if (port->uses_driver) {
        return uart_read_bytes(port->port_num, buf, size, timeout);
    }
    mutex_lock(&port->mutex);
    int cpy_size = MIN(size, port->buf_pos);
    if (cpy_size > 0) {
        memcpy(buf, port->buf, cpy_size);
        memmove(&port->buf[0], &port->buf[cpy_size], port->buf_pos - cpy_size);
        port->buf_pos -= cpy_size;
    }
    mutex_unlock(&port->mutex);
    return cpy_size;
}

bool serial_port_begin_write(serial_port_t *port) {
    if (!port->in_write) {
        if (serial_port_is_half_duplex(port)) {
            // Half duplex mode, switch to TX mode
            serial_half_duplex_enable_tx(port);
        }
        port->in_write = true;
        return true;
    }
    return false;
}

bool serial_port_end_write(serial_port_t *port) {
    if (port->in_write) {
        if (serial_port_is_half_duplex(port)) {
            // Wait for TX to complete
            uart_wait_tx_done(port->port_num, 100);
            // Switch back to RX mode
            serial_half_duplex_enable_rx(port);
        }
        port->in_write = false;
        return true;
    }
    return false;
}

int serial_port_write(serial_port_t *port, const void *buf, size_t size) {
    bool began_write = serial_port_begin_write(port);
    int n;
    if (port->uses_driver) {
        n = uart_write_bytes(port->port_num, buf, size);
    } else {
        // For half-duplex mode without driver, we need to manually send bytes
        // This is a simplified implementation - in practice, you might need
        // more sophisticated timing control
        const uint8_t *ptr = buf;
        for (unsigned ii = 0; ii < size; ii++) {
            // Send byte through GPIO (simplified)
            // In a real implementation, you'd need proper UART timing
            uart_write_bytes(port->port_num, &ptr[ii], 1);
        }
        n = size;
    }
    if (began_write) {
        serial_port_end_write(port);
    }
    return n;
}

bool serial_port_set_baudrate(serial_port_t *port, uint32_t baudrate) {
    ESP_ERROR_CHECK(uart_set_baudrate(port->port_num, baudrate));
    return true;
}

bool serial_port_set_inverted(serial_port_t *port, bool inverted) {
    uint32_t inverse_mask = inverted ? UART_INVERSE_TXD | UART_INVERSE_RXD : 0;
    ESP_ERROR_CHECK(uart_set_line_inverse(port->port_num, inverse_mask));
    return true;
}

void serial_port_close(serial_port_t *port) {
    assert(port->open);
    if (port->uses_driver) {
        ESP_ERROR_CHECK(uart_driver_delete(port->port_num));
    }

    // Delete the UART event task
    if (port->uart_task_handle) {
        vTaskDelete(port->uart_task_handle);
        port->uart_task_handle = NULL;
    }

    mutex_close(&port->mutex);
    port->open = false;
}

bool serial_port_is_half_duplex(const serial_port_t *port) { return port->config.tx_pin == port->config.rx_pin; }

serial_half_duplex_mode_e serial_port_half_duplex_mode(const serial_port_t *port) {
    if (serial_port_is_half_duplex(port)) {
        if (port->half_duplex_tx_mode) {
            return SERIAL_HALF_DUPLEX_MODE_TX;
        }
        return SERIAL_HALF_DUPLEX_MODE_RX;
    }
    return SERIAL_HALF_DUPLEX_MODE_NONE;
}

void serial_port_set_half_duplex_mode(serial_port_t *port, serial_half_duplex_mode_e mode) {
    assert(serial_port_is_half_duplex(port));
    {
        switch (mode) {
            case SERIAL_HALF_DUPLEX_MODE_NONE:
                break;
            case SERIAL_HALF_DUPLEX_MODE_RX:
                serial_half_duplex_enable_rx(port);
                break;
            case SERIAL_HALF_DUPLEX_MODE_TX:
                serial_half_duplex_enable_tx(port);
                break;
        }
    }
}

void serial_port_destroy(serial_port_t **port) {
    if (*port) {
        serial_port_close(*port);
        *port = NULL;
    }
}

io_flags_t serial_port_io_flags(serial_port_t *port) {
    if (serial_port_is_half_duplex(port)) {
        return IO_FLAG_HALF_DUPLEX;
    }
    return 0;
}
