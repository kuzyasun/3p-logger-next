#include "led_module.h"

static const char *TAG = "LED";

// Task to blink the board LED every second
void led_blink_task(void *pvParameters) {
    // Setup GPIO pin as output
    hal_gpio_setup(BOARDLED_PIN, HAL_GPIO_DIR_OUTPUT, HAL_GPIO_PULL_NONE);

    LOG_I(TAG, "LED blink task started on pin %d", BOARDLED_PIN);

    while (1) {
        // Turn LED on
        hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
        LOG_I(TAG, "LED ON");

        // Wait 500ms
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Turn LED off
        hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
        LOG_I(TAG, "LED OFF");

        // Wait 500ms (total 1 second cycle)
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void led_module_init(led_module_t *led_module) {
    if (!led_module) {
        return;
    }

    led_module->is_initialized = true;

    LOG_I(TAG, "LED module initialized");
}
