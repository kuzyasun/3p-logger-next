#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_state.h"

typedef enum {
    APP_CMD_NONE,
} app_command_id_e;

typedef struct {
    app_command_id_e id;
    union {
        // Use the new, clean typedefs
    } payload;

} app_command_t;

typedef enum {
    APP_EVENT_NONE,
} app_event_id_e;

typedef struct {
    app_event_id_e id;

    union {
        int error_code;
        bool success_status;
    } payload;
} app_event_t;
