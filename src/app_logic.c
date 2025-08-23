#include "app_logic.h"

#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <hal/gpio.h>
#include <log.h>
#include <stdlib.h>
#include <string.h>
#include <target.h>

#include "app_commands.h"
#include "app_state.h"
#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "io/io_manager.h"
#include "io/serial.h"
#include "modules/accel_module.h"
#include "modules/led_module.h"
#include "modules/logger_module.h"
#include "modules/pz_module.h"
#include "platform/system.h"
#include "protocols/crsf.h"
#include "protocols/mavlink.h"
#include "protocols/msp.h"
#include "protocols/msp_v2.h"
#include "protocols/nmea.h"
#include "protocols/ublox.h"

static const char *TAG = "APPL";

static QueueHandle_t g_command_queue;

void app_logic_init(app_logic_t *app) {
    LOG_I(TAG, "Initializing Application Logic...");

    config_manager_load();

    // Initialize IO Manager
    app->io_manager = calloc(1, sizeof(io_manager_t));
    if (app->io_manager == NULL) {
        LOG_E(TAG, "Failed to allocate memory for IO Manager. Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (io_manager_init(app->io_manager) != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize IO Manager. Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Initialize LED Module
    app->led_module = calloc(1, sizeof(led_module_t));
    if (app->led_module == NULL) {
        LOG_E(TAG, "Failed to allocate memory for LED Module. Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    led_module_init(app->led_module);

    // Initialize Accelerometer Module
    app->accel_module = calloc(1, sizeof(accel_module_t));
    if (app->accel_module == NULL) {
        LOG_E(TAG, "Failed to allocate memory for Accelerometer Module. Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Initialize Logger Module
    app->logger_module = calloc(1, sizeof(logger_module_t));
    if (app->logger_module == NULL) {
        LOG_E(TAG, "Failed to allocate memory for Logger Module. Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Initialize Piezo Module
    app->pz_module = calloc(1, sizeof(pz_module_t));
    if (app->pz_module == NULL) {
        LOG_E(TAG, "Failed to allocate memory for Piezo Module. Halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    g_command_queue = xQueueCreate(20, sizeof(app_command_t));

    // Initialize all protocol parsers
    nmea_parser_init(&app->nmea_parser);
    crsf_parser_init(&app->crsf_parser);
    ublox_parser_init(&app->ublox_parser);
    msp_parser_init(&app->msp_parser);
    msp_v2_parser_init(&app->msp_v2_parser);
    mavlink_parser_init(&app->mavlink_parser);

    // Configure UART1 for CRSF input
    LOG_I(TAG, "Configuring IO peripherals...");
    io_manager_configure_uart(app->io_manager, &app->io_manager->uart1, app, g_app_config.uart1_protocol, g_app_config.uart1_baudrate, UART1_RX_GPIO,
                              SERIAL_UNUSED_GPIO);

    LOG_I(TAG, "Application Logic Initialized.");
}

app_err_t app_logic_init_modules(app_logic_t *app) {
    LOG_I(TAG, "Initializing application modules...");
    app_state_t *state = app_state_get_instance();

    // Initialize Accelerometer
    accel_config_t accel_cfg = {
        .output_data_rate = g_app_config.accel_config.output_data_rate,
        .full_scale = g_app_config.accel_config.full_scale,
        .op_mode = g_app_config.accel_config.op_mode,
        .spi_bus = g_app_config.accel_config.spi_bus,
        .cs_pin = g_app_config.accel_config.cs_pin,
    };
    if (accel_module_init(app->accel_module, &accel_cfg) != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize accelerometer module. Entering error state.");
        app_state_begin_update();
        app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_ERROR);
        state->system_error_code = APP_ERR_ACCEL_INIT_FAILED;
        app_state_end_update();
        return APP_ERR_ACCEL_INIT_FAILED;
    }

    // Initialize Logger
    if (logger_module_init(app->logger_module, app->io_manager->sdcard_spi_bus) != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to initialize logger module. Entering error state.");
        app_state_begin_update();
        app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_ERROR);
        state->system_error_code = APP_ERR_LOGGER_INIT_FAILED;
        app_state_end_update();
        return APP_ERR_LOGGER_INIT_FAILED;
    }

    // Initialize Piezo
    if (pz_module_init(app->pz_module) != APP_OK) {
        LOG_E(TAG, "Failed to initialize piezo module. Entering error state.");
        app_state_begin_update();
        app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_ERROR);
        state->system_error_code = APP_ERR_GENERIC;
        app_state_end_update();
        return APP_ERR_GENERIC;
    }
    pz_module_set_threshold(g_app_config.pz_dac1_threshold, g_app_config.pz_dac2_threshold);

    LOG_I(TAG, "All application modules initialized successfully.");
    return APP_OK;
}

void app_logic_on_command(Notifier *notifier, Observer *observer, void *data) {
    app_command_t *cmd = (app_command_t *)data;
    LOG_D(TAG, "Command received from notifier, enqueuing command ID: %d", cmd->id);

    // Send the command to the queue to be processed by the app_logic_task.
    // Use a small timeout to prevent blocking, but drop the command if the queue is full.
    if (xQueueSend(g_command_queue, cmd, pdMS_TO_TICKS(10)) != pdPASS) {
        LOG_E(TAG, "Failed to enqueue command! Command queue is full.");
    }
}

void app_logic_on_event(Notifier *notifier, Observer *observer, void *data) {
    app_logic_t *app = (app_logic_t *)observer->context;
    app_event_t *event = (app_event_t *)data;
    app_command_t cmd;

    // Handle events from worker modules.
    switch (event->id) {
        default:
            // This is not a warning, as app_logic may not need to react to every event.
            // LOG_D(TAG, "Received unhandled event: %d", event->id);
            break;
    }
}

// Forward declaration of the private command handler
static app_err_t _app_logic_handle_command(app_logic_t *app, app_command_t *cmd);

void app_logic_task(void *arg) {
    app_logic_t *app = (app_logic_t *)arg;
    app_command_t received_cmd;

    LOG_I(TAG, "App Logic Task started.");

    while (1) {
        // Wait indefinitely for a command to arrive in the queue.
        if (xQueueReceive(g_command_queue, &received_cmd, portMAX_DELAY)) {
            LOG_D(TAG, "Processing command ID: %d", received_cmd.id);
            // Delegate the actual work to a handler function.
            _app_logic_handle_command(app, &received_cmd);
        }
    }
}

// Private function containing the command execution logic (the switch statement).
static app_err_t _app_logic_handle_command(app_logic_t *app, app_command_t *cmd) {
    app_err_t result = APP_OK;
    app_state_t *state = app_state_get_instance();
    switch (cmd->id) {
        case APP_CMD_SET_MODE_IDLE:
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_IDLE);
            app_state_end_update();
            LOG_I(TAG, "Mode changed to IDLE");
            break;
        case APP_CMD_SET_MODE_LOGGING:
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_LOGGING);
            app_state_end_update();
            LOG_I(TAG, "Mode changed to LOGGING");
            break;
        case APP_CMD_SET_MODE_ERROR:
            app_state_begin_update();
            app_state_set_u8(APP_STATE_FIELD_CURRENT_MODE, (uint8_t *)&state->current_mode, APP_MODE_ERROR);
            app_state_end_update();
            LOG_I(TAG, "Mode changed to ERROR");
            break;
        default:
            LOG_W(TAG, "Received unhandled application command: %d", cmd->id);
            result = APP_ERR_UNKNOWN_COMMAND;
            break;
    }

    return result;
}

// Test task to cycle through modes for demonstration
static void test_mode_cycle_task(void *pvParameters) {
    app_logic_t *app = (app_logic_t *)pvParameters;

    LOG_I(TAG, "Test mode cycle task started");

    // Wait a bit for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        // Cycle through modes: IDLE -> LOGGING -> ERROR -> IDLE
        LOG_I(TAG, "Setting mode to IDLE");
        app_logic_send_command(app, APP_CMD_SET_MODE_IDLE);
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5 seconds in IDLE

        LOG_I(TAG, "Setting mode to LOGGING");
        app_logic_send_command(app, APP_CMD_SET_MODE_LOGGING);
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5 seconds in LOGGING

        LOG_I(TAG, "Setting mode to ERROR");
        app_logic_send_command(app, APP_CMD_SET_MODE_ERROR);
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5 seconds in ERROR
    }
}

// ============================================================================
// Task Management Functions
// ============================================================================

esp_err_t app_logic_start_all_tasks(app_logic_t *app) {
    LOG_I(TAG, "Starting all application tasks...");

    uint32_t cpu_cores = system_get_cpu_cores();
    esp_err_t result = ESP_OK;

    // Start app_logic_task (this task)
    BaseType_t app_logic_ret = xTaskCreatePinnedToCore(app_logic_task, "APP_LOGIC", 4096, app, TASK_PRIORITY_APP_LOGIC, &app->app_logic_handle, 0);
    if (app_logic_ret != pdPASS) {
        LOG_E(TAG, "Failed to create app_logic_task");
        result = ESP_FAIL;
    } else {
        LOG_I(TAG, "app_logic_task created successfully");
    }

    // Start LED task
    if (app->led_module->is_initialized) {
        BaseType_t led_ret = xTaskCreatePinnedToCore(led_blink_task, "LED_BLINK", 4096, app->led_module, TASK_PRIORITY_DEFAULT, &app->led_task_handle, 0);
        if (led_ret != pdPASS) {
            LOG_E(TAG, "Failed to create LED task");
            result = ESP_FAIL;
        } else {
            LOG_I(TAG, "LED task created successfully");
        }
    } else {
        LOG_W(TAG, "Skipping LED task creation, module not initialized.");
    }

    // Start io_manager_task
    if (app->io_manager->initialized) {
        BaseType_t io_manager_ret = xTaskCreatePinnedToCore(io_manager_task, "IO_MANAGER", 4096, app->io_manager, TASK_PRIORITY_DEFAULT,
                                                            &app->io_manager_handle, (cpu_cores > 1) ? 1 : 0);
        if (io_manager_ret != pdPASS) {
            LOG_E(TAG, "Failed to create io_manager_task");
            result = ESP_FAIL;
        } else {
            LOG_I(TAG, "io_manager_task created successfully");
        }
    } else {
        LOG_W(TAG, "Skipping io_manager_task creation, module not initialized.");
    }

    if (app->accel_module->initialized) {
        accel_module_create_task(app->accel_module);
        app->accel_task_handle = app->accel_module->task_handle;
        if (app->accel_task_handle == NULL) {
            LOG_E(TAG, "Failed to create accel_module_task");
            result = ESP_FAIL;
        } else {
            LOG_I(TAG, "accel_module_task created successfully");
        }
    } else {
        LOG_W(TAG, "Skipping accel_module_task creation, module not initialized.");
    }

    if (app->logger_module->initialized && app->logger_module->sd_card_ok && app->accel_module->initialized) {
        logger_module_create_task(app->logger_module);
        app->logger_task_handle = app->logger_module->task_handle;
        if (app->logger_task_handle == NULL) {
            LOG_E(TAG, "Failed to create logger_task");
            result = ESP_FAIL;
        } else {
            LOG_I(TAG, "logger_task created successfully");
        }
    } else {
        LOG_W(TAG, "Skipping logger_task creation, module not initialized.");
    }

    if (app->pz_module->is_initialized) {
        pz_module_create_task(app->pz_module);
        app->pz_task_handle = app->pz_module->task_handle;
        if (app->pz_task_handle == NULL) {
            LOG_E(TAG, "Failed to create pz_module_task");
            result = ESP_FAIL;
        } else {
            LOG_I(TAG, "pz_module_task created successfully");
        }
    } else {
        LOG_W(TAG, "Skipping pz_module_task creation, module not initialized.");
    }

    // Start test mode cycle task
    // BaseType_t test_ret = xTaskCreatePinnedToCore(test_mode_cycle_task, "TEST_MODE", 4096, app, TASK_PRIORITY_DEFAULT, NULL, 0);
    // if (test_ret != pdPASS) {
    //     LOG_E(TAG, "Failed to create test mode cycle task");
    //     result = ESP_FAIL;
    // } else {
    //     LOG_I(TAG, "Test mode cycle task created successfully");
    // }

    return result;
}

esp_err_t app_logic_send_command(app_logic_t *app, app_command_id_e cmd_id) {
    if (!app) {
        return ESP_ERR_INVALID_ARG;
    }

    app_command_t cmd = {.id = cmd_id};

    if (xQueueSend(g_command_queue, &cmd, pdMS_TO_TICKS(10)) != pdPASS) {
        LOG_E(TAG, "Failed to send command %d to app logic queue", cmd_id);
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}
