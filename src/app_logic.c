#include "app_logic.h"

#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <log.h>
#include <stdlib.h>
#include <string.h>
#include <target.h>

#include "app_commands.h"
#include "app_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "io/io_manager.h"
#include "platform/system.h"

static const char *TAG = "APPL";

static QueueHandle_t g_command_queue;

void app_logic_init(app_logic_t *app, io_manager_t *io_manager, led_module_t *led_module) {
    app->io_manager = io_manager;
    app->led_module = led_module;

    g_command_queue = xQueueCreate(20, sizeof(app_command_t));

    LOG_I(TAG, "Application Logic Initialized.");
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
        default:
            LOG_W(TAG, "Received unhandled application command: %d", cmd->id);
            result = APP_ERR_UNKNOWN_COMMAND;
            break;
    }

    return result;
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

    // Start io_manager_task
    BaseType_t io_manager_ret =
        xTaskCreatePinnedToCore(io_manager_task, "IO_MANAGER", 4096, app->io_manager, TASK_PRIORITY_DEFAULT, &app->io_manager_handle, (cpu_cores > 1) ? 1 : 0);
    if (io_manager_ret != pdPASS) {
        LOG_E(TAG, "Failed to create io_manager_task");
        result = ESP_FAIL;
    } else {
        LOG_I(TAG, "io_manager_task created successfully");
    }

    // Start LED blink task
    BaseType_t led_ret = xTaskCreatePinnedToCore(led_blink_task, "LED_BLINK", 4096, app->led_module, TASK_PRIORITY_DEFAULT, &app->led_task_handle, 0);
    if (led_ret != pdPASS) {
        LOG_E(TAG, "Failed to create LED blink task");
        result = ESP_FAIL;
    } else {
        LOG_I(TAG, "LED blink task created successfully");
    }

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
