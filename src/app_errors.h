#pragma once
#include <stdint.h>

typedef int32_t app_err_t;

#define APP_OK 0

#define APP_ERR_BASE 0x1000
#define APP_ERR_GENERIC (APP_ERR_BASE + 0)
#define APP_ERR_UNKNOWN_COMMAND (APP_ERR_BASE + 1)
#define APP_ERR_UART1_ERROR (APP_ERR_BASE + 2)
#define APP_ERR_SPI1_ERROR (APP_ERR_BASE + 3)
#define APP_ERR_SPI2_ERROR (APP_ERR_BASE + 4)
