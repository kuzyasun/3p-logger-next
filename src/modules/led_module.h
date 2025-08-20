#pragma once

#include <log.h>
#include <target.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio.h"

typedef struct led_module_s {
    bool is_initialized;
} led_module_t;

void led_module_init(led_module_t *led_module);
void led_blink_task(void *pvParameters);
