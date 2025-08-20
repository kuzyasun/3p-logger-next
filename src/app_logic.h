#pragma once

#include "app_commands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "io/io_manager.h"
#include "modules/led_module.h"
#include "protocols/crsf.h"
#include "protocols/msp.h"
#include "protocols/msp_v2.h"
#include "protocols/nmea.h"
#include "protocols/ublox.h"

typedef struct {
    TaskHandle_t app_logic_handle;
    TaskHandle_t io_manager_handle;
    TaskHandle_t led_task_handle;
} task_management_t;

typedef struct app_logic_s {
    io_manager_t *io_manager;
    led_module_t *led_module;

    TaskHandle_t app_logic_handle;
    TaskHandle_t io_manager_handle;
    TaskHandle_t led_task_handle;

    nmea_parser_t nmea_parser;
    crsf_parser_t crsf_parser;
    ublox_parser_t ublox_parser;
    msp_parser_t msp_parser;
    msp_v2_parser_t msp_v2_parser;

} app_logic_t;

void app_logic_init(app_logic_t *app, io_manager_t *io_manager, led_module_t *led_module);
void app_logic_task(void *arg);
void app_logic_on_command(Notifier *notifier, Observer *observer, void *data);
void app_logic_on_event(Notifier *notifier, Observer *observer, void *data);
esp_err_t app_logic_start_all_tasks(app_logic_t *app);
esp_err_t app_logic_send_command(app_logic_t *app, app_command_id_e cmd_id);
app_logic_t *get_global_app_logic_instance(void);