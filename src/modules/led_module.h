#pragma once

#include <log.h>
#include <target.h>

#include "app_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio.h"
#include "util/observer.h"

typedef struct led_module_s {
    bool is_initialized;
    app_mode_t current_mode;
    Observer *app_state_observer;
} led_module_t;

void led_module_init(led_module_t *led_module);
void led_blink_task(void *pvParameters);
void led_module_on_app_state_change(Notifier *notifier, Observer *observer, void *data);
