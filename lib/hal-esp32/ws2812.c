#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <esp_rom_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <hal/ws2812.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"

// WS2812 timing parameters (in nanoseconds)
#define WS2812_T0H_NS 350
#define WS2812_T1H_NS 900
#define WS2812_T0L_NS 900
#define WS2812_T1L_NS 350
#define WS2812_RESET_US 50

// RMT configuration
// Use 20MHz so WS2812 timings (350ns, 900ns) are represented exactly in ticks

#define WS2812_RMT_MEM_BLOCK_SYMBOLS 64  // Number of symbols per block

// Static handles for RMT channel and encoder
static rmt_channel_handle_t ws2812_rmt_channel = NULL;
static rmt_encoder_handle_t ws2812_encoder = NULL;
static SemaphoreHandle_t ws2812_tx_done_sem = NULL;
static gpio_num_t ws2812_gpio = -1;

// RMT TX done callback
static bool IRAM_ATTR ws2812_tx_done_callback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (ws2812_tx_done_sem) {
        xSemaphoreGiveFromISR(ws2812_tx_done_sem, &xHigherPriorityTaskWoken);
    }
    return xHigherPriorityTaskWoken == pdTRUE;
}

hal_err_t hal_ws2812_open(hal_gpio_t gpio) {
    if (ws2812_rmt_channel) {
        return ESP_ERR_INVALID_STATE;
    }
    ws2812_gpio = (gpio_num_t)gpio;

    // 1. Try to create RMT TX channel (priority for DMA)
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = ws2812_gpio,
        .mem_block_symbols = WS2812_RMT_MEM_BLOCK_SYMBOLS,
        .resolution_hz = WS2812_RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,  // Better have a larger queue for DMA
        .flags.with_dma = true,  // Try to enable DMA
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &ws2812_rmt_channel);

    if (ret == ESP_ERR_NOT_SUPPORTED) {
        // DMA not supported, fall back to non-DMA mode
        ESP_LOGW("WS2812_HAL", "RMT DMA not supported on this chip. Falling back to non-DMA mode.");
        tx_chan_config.flags.with_dma = false;
        tx_chan_config.trans_queue_depth = 2;  // Can decrease queue
        ret = rmt_new_tx_channel(&tx_chan_config, &ws2812_rmt_channel);
    }

    // After all attempts, check for final errors
    ESP_ERROR_CHECK(ret);

    // 2. Create bytes encoder with WS2812 timing
    // TODO share frequency
    const int ticks_per_ns = (1000000000 / WS2812_RMT_RESOLUTION_HZ);  // 50ns per tick at 20MHz
    // Round up to ensure we never go under the WS2812 spec timings
    const int t0h_ticks = (WS2812_T0H_NS + ticks_per_ns - 1) / ticks_per_ns;
    const int t0l_ticks = (WS2812_T0L_NS + ticks_per_ns - 1) / ticks_per_ns;
    const int t1h_ticks = (WS2812_T1H_NS + ticks_per_ns - 1) / ticks_per_ns;
    const int t1l_ticks = (WS2812_T1L_NS + ticks_per_ns - 1) / ticks_per_ns;

    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 =
            {
                .duration0 = t0h_ticks,
                .level0 = 1,
                .duration1 = t0l_ticks,
                .level1 = 0,
            },
        .bit1 =
            {
                .duration0 = t1h_ticks,
                .level0 = 1,
                .duration1 = t1l_ticks,
                .level1 = 0,
            },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&encoder_config, &ws2812_encoder));

    // 3. Register TX done callback
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = ws2812_tx_done_callback,
    };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(ws2812_rmt_channel, &cbs, NULL));

    // 4. Enable the channel
    ESP_ERROR_CHECK(rmt_enable(ws2812_rmt_channel));

    return ESP_OK;
}

hal_err_t hal_ws2812_close(hal_gpio_t gpio) {
    if (ws2812_rmt_channel) {
        rmt_disable(ws2812_rmt_channel);
        rmt_del_channel(ws2812_rmt_channel);
        ws2812_rmt_channel = NULL;
    }
    if (ws2812_encoder) {
        rmt_del_encoder(ws2812_encoder);
        ws2812_encoder = NULL;
    }
    ws2812_gpio = -1;
    return ESP_OK;
}

hal_err_t hal_ws2812_set_colors(hal_gpio_t gpio, const hal_ws2812_color_t *colors, size_t count) {
    if (!ws2812_rmt_channel || !ws2812_encoder) {
        return ESP_ERR_INVALID_STATE;
    }
    if (ws2812_gpio != (gpio_num_t)gpio) {
        return ESP_ERR_INVALID_ARG;
    }
    // Prepare the semaphore
    if (!ws2812_tx_done_sem) {
        ws2812_tx_done_sem = xSemaphoreCreateBinary();
    }
    // Transmit the color data (convert to configured byte order expected by LED)
    // Each color is 3 bytes in the configured order
    size_t grb_len = count * 3;
    uint8_t *pix_bytes = (uint8_t *)malloc(grb_len);
    if (pix_bytes == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (size_t i = 0; i < count; i++) {
        const hal_ws2812_color_t *c = &colors[i];
        size_t base = i * 3;
#if HAL_WS2812_COLOR_ORDER == HAL_WS2812_ORDER_GRB
        pix_bytes[base + 0] = c->g;
        pix_bytes[base + 1] = c->r;
        pix_bytes[base + 2] = c->b;
#elif HAL_WS2812_COLOR_ORDER == HAL_WS2812_ORDER_RGB
        pix_bytes[base + 0] = c->r;
        pix_bytes[base + 1] = c->g;
        pix_bytes[base + 2] = c->b;
#elif HAL_WS2812_COLOR_ORDER == HAL_WS2812_ORDER_BRG
        pix_bytes[base + 0] = c->b;
        pix_bytes[base + 1] = c->r;
        pix_bytes[base + 2] = c->g;
#elif HAL_WS2812_COLOR_ORDER == HAL_WS2812_ORDER_GBR
        pix_bytes[base + 0] = c->g;
        pix_bytes[base + 1] = c->b;
        pix_bytes[base + 2] = c->r;
#elif HAL_WS2812_COLOR_ORDER == HAL_WS2812_ORDER_RBG
        pix_bytes[base + 0] = c->r;
        pix_bytes[base + 1] = c->b;
        pix_bytes[base + 2] = c->g;
#elif HAL_WS2812_COLOR_ORDER == HAL_WS2812_ORDER_BGR
        pix_bytes[base + 0] = c->b;
        pix_bytes[base + 1] = c->g;
        pix_bytes[base + 2] = c->r;
#else
        // Fallback to GRB
        pix_bytes[base + 0] = c->g;
        pix_bytes[base + 1] = c->r;
        pix_bytes[base + 2] = c->b;
#endif
    }
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };
    ESP_ERROR_CHECK(rmt_transmit(ws2812_rmt_channel, ws2812_encoder, pix_bytes, grb_len, &tx_config));
    // Wait for transmission to complete
    xSemaphoreTake(ws2812_tx_done_sem, pdMS_TO_TICKS(100));
    // Send reset pulse (low for >50us)
    gpio_set_level(ws2812_gpio, 0);
    esp_rom_delay_us(WS2812_RESET_US);
    free(pix_bytes);
    return ESP_OK;
}