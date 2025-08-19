#include "io_manager.h"

#include <errno.h>
#include <log.h>
#include <stdlib.h>
#include <string.h>
#include <target.h>

#include "app_state.h"

static const char *TAG = "IOM";

// ESP-NOW protocol packet structure (matching the one in wifi.c)
#define MAX_PROTOCOL_PACKET_SIZE 250
typedef struct {
    uint8_t data[MAX_PROTOCOL_PACKET_SIZE];
    size_t len;
    uint8_t sender_mac[6];
} protocol_packet_t;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void io_manager_init(io_manager_t *iom) {
    memset(iom, 0, sizeof(*iom));

    // Initialize UARTs, SPI buses, etc.

    LOG_I(TAG, "IO Manager Initialized");
}

void io_manager_task(void *arg) {
    io_manager_t *iom = (io_manager_t *)arg;
    app_state_t *state = app_state_get_instance();

    while (true) {
        // TODO update UARTS
        // TODO update SPI Buses

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
