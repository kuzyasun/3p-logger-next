#include <log.h>
#include <target.h>

#include "esp_attr.h"
#include "esp_log_level.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio.h"
#include "io/io_manager.h"
#include "platform/system.h"

static const char *TAG = "Main";

static io_manager_t io_manager;
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

void app_main() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // esp_log_set_level_master(ESP_LOG_VERBOSE);
    // esp_log_level_set("*", ESP_LOG_DEBUG);
    // esp_log_level_set("rmt", ESP_LOG_DEBUG);

    esp_log_set_level_master(ESP_LOG_INFO);
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("rmt", ESP_LOG_INFO);

    // Chip info
    LOG_I(TAG, "\n3P Logger %s", FIRMWARE_VERSION);
    LOG_I(TAG, "ESP32 Model: %s", system_get_esp32_model());
    LOG_I(TAG, "ESP32 Chip Revision: %d", system_get_chip_revision());
    LOG_I(TAG, "ESP32 CPU Frequency: %lu MHz", system_get_cpu_frequency_mhz());
    LOG_I(TAG, "ESP32 CPU Cores: %lu", system_get_cpu_cores());

    io_manager_init(&io_manager);
    // Create LED blink task
    xTaskCreate(led_blink_task, "led_blink", 2048, NULL, 5, NULL);
    LOG_I(TAG, "LED blink task created");
}