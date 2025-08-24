#include "led_module.h"

#include "app_state.h"
#include "util/observer.h"

static const char *TAG = "LED";

// Task to control both board LED and external LED based on app mode
void led_blink_task(void *pvParameters) {
    led_module_t *led_module = (led_module_t *)pvParameters;

    // Setup both GPIO pins as output
    hal_gpio_setup(BOARDLED_PIN, HAL_GPIO_DIR_OUTPUT, HAL_GPIO_PULL_NONE);
    hal_gpio_setup(LED_OUT_GPIO, HAL_GPIO_DIR_OUTPUT, HAL_GPIO_PULL_NONE);

    LOG_I(TAG, "LED task started on pins %d (onboard) and %d (external)", BOARDLED_PIN, LED_OUT_GPIO);

    while (1) {
        // Turn both LEDs on
        hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_HIGH);
        hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_HIGH);

        // Wait time depends on current mode
        TickType_t on_delay;
        TickType_t off_delay;

        switch (led_module->current_mode) {
            case APP_MODE_LOGGING:
                // Fast blink for logging mode
                on_delay = pdMS_TO_TICKS(200);
                off_delay = pdMS_TO_TICKS(200);
                break;
            case APP_MODE_ERROR:
                // Very fast blink for error mode
                on_delay = pdMS_TO_TICKS(50);
                off_delay = pdMS_TO_TICKS(50);
                break;
            case APP_MODE_IDLE:
            default:
                // Normal blink for idle mode
                on_delay = pdMS_TO_TICKS(800);
                off_delay = pdMS_TO_TICKS(800);
                break;
        }

        vTaskDelay(on_delay);

        // Turn both LEDs off
        hal_gpio_set_level(BOARDLED_PIN, HAL_GPIO_LOW);
        hal_gpio_set_level(LED_OUT_GPIO, HAL_GPIO_LOW);

        vTaskDelay(off_delay);
    }
}

void led_module_on_app_state_change(Notifier *notifier, Observer *observer, void *data) {
    led_module_t *led_module = (led_module_t *)observer->context;
    uint64_t *changed_mask = (uint64_t *)data;

    // Check if the mode field changed
    if (*changed_mask & APP_STATE_FIELD_CURRENT_MODE) {
        app_state_t *app_state = app_state_get_instance();
        led_module->current_mode = app_state->current_mode;
    }
}

void led_module_init(led_module_t *led_module) {
    if (!led_module) {
        return;
    }

    led_module->is_initialized = true;
    led_module->current_mode = APP_MODE_IDLE;  // Default to idle mode

    // Subscribe to app state changes
    led_module->app_state_observer = ObserverCreate("LED_APP_STATE", led_module, led_module_on_app_state_change, NULL);
    if (led_module->app_state_observer) {
        Notifier *app_state_notifier = app_state_get_notifier();
        if (app_state_notifier) {
            app_state_notifier->subject.attach(app_state_notifier, led_module->app_state_observer);
            LOG_I(TAG, "LED module subscribed to app state changes");
        }
    }

    LOG_I(TAG, "LED module initialized");
}
