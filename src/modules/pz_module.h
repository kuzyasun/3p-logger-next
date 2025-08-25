#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_errors.h"
#include "logger_module.h"

// Main module state structure
typedef struct {
    bool is_initialized;
    TaskHandle_t task_handle;
    logger_module_t *logger;
} pz_module_t;

/**
 * @brief Initializes the Piezo module.
 * @param module Pointer to the pz_module_t instance.
 * @return APP_OK on success, or an error code on failure.
 */
app_err_t pz_module_init(pz_module_t *module);

/**
 * @brief Sets the reference voltage for the comparators via DACs.
 * @param dac1_value 8-bit value (0-255) for DAC channel 1.
 * @param dac2_value 8-bit value (0-255) for DAC channel 2.
 * @return APP_OK on success, or an error code on failure.
 */
app_err_t pz_module_set_threshold(uint8_t dac1_value, uint8_t dac2_value);

/**
 * @brief Creates the FreeRTOS task for the Piezo module.
 * @param module Pointer to the pz_module_t instance.
 */
void pz_module_create_task(pz_module_t *module, logger_module_t *logger_instance);
