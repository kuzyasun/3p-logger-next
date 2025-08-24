#include "pz_module.h"

#include <driver/dac_oneshot.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "app_state.h"
#include "hal/gpio.h"
#include "log.h"
#include "target.h"

static const char *TAG = "PZ";
static QueueHandle_t pz_events_queue;
static dac_oneshot_handle_t dac1_handle;
static dac_oneshot_handle_t dac2_handle;

// ISR must be fast and placed in IRAM
static void IRAM_ATTR pz_gpio_isr_handler(void *arg) {
    uint8_t mask = 0;

    // Read the state of all 4 comparator pins
    if (gpio_get_level(COMP_1_OUT_GPIO)) mask |= (1 << 0);
    if (gpio_get_level(COMP_2_OUT_GPIO)) mask |= (1 << 1);
    if (gpio_get_level(COMP_3_OUT_GPIO)) mask |= (1 << 2);
    if (gpio_get_level(COMP_4_OUT_GPIO)) mask |= (1 << 3);

    // If any comparator triggered, send the mask to the queue
    if (mask > 0) {
        xQueueSendFromISR(pz_events_queue, &mask, NULL);
    }
}

static void pz_module_task(void *arg) {
    uint8_t received_mask;
    app_state_t *state = app_state_get_instance();

    while (1) {
        // Block and wait for a mask from the ISR
        if (xQueueReceive(pz_events_queue, &received_mask, portMAX_DELAY) == pdTRUE) {
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_PIEZO_MASK, &state->piezo_mask, received_mask);
            app_state_end_update();
        }
    }
}

app_err_t pz_module_init(pz_module_t *module) {
    // 1. Create the queue to pass data from ISR to task
    pz_events_queue = xQueueCreate(10, sizeof(uint8_t));
    if (!pz_events_queue) {
        LOG_E(TAG, "Failed to create event queue");
        return APP_ERR_GENERIC;
    }

    gpio_install_isr_service(0);

    // 2. Initialize DAC channels
    dac_oneshot_config_t dac1_cfg = {
        .chan_id = DAC_CHAN_0,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac1_cfg, &dac1_handle));

    dac_oneshot_config_t dac2_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&dac2_cfg, &dac2_handle));

    // 3. Configure comparator input GPIOs
    hal_gpio_setup(COMP_1_OUT_GPIO, HAL_GPIO_DIR_INPUT, HAL_GPIO_PULL_DOWN);
    hal_gpio_setup(COMP_2_OUT_GPIO, HAL_GPIO_DIR_INPUT, HAL_GPIO_PULL_DOWN);
    hal_gpio_setup(COMP_3_OUT_GPIO, HAL_GPIO_DIR_INPUT, HAL_GPIO_PULL_DOWN);
    hal_gpio_setup(COMP_4_OUT_GPIO, HAL_GPIO_DIR_INPUT, HAL_GPIO_PULL_DOWN);

    // 4. Attach ISR to each GPIO
    hal_gpio_set_isr(COMP_1_OUT_GPIO, HAL_GPIO_INTR_POSEDGE, pz_gpio_isr_handler, (void *)COMP_1_OUT_GPIO);
    hal_gpio_set_isr(COMP_2_OUT_GPIO, HAL_GPIO_INTR_POSEDGE, pz_gpio_isr_handler, (void *)COMP_2_OUT_GPIO);
    hal_gpio_set_isr(COMP_3_OUT_GPIO, HAL_GPIO_INTR_POSEDGE, pz_gpio_isr_handler, (void *)COMP_3_OUT_GPIO);
    hal_gpio_set_isr(COMP_4_OUT_GPIO, HAL_GPIO_INTR_POSEDGE, pz_gpio_isr_handler, (void *)COMP_4_OUT_GPIO);

    module->is_initialized = true;
    LOG_I(TAG, "Piezo module initialized.");
    return APP_OK;
}

app_err_t pz_module_set_threshold(uint8_t dac1_value, uint8_t dac2_value) {
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac1_handle, dac1_value));
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(dac2_handle, dac2_value));
    LOG_I(TAG, "DAC thresholds set to %d and %d", dac1_value, dac2_value);
    return APP_OK;
}

void pz_module_create_task(pz_module_t *module) {
    xTaskCreatePinnedToCore(pz_module_task, "PZ_TASK", 4096, module, TASK_PRIORITY_PZ_TASK, &module->task_handle, 0);
}
