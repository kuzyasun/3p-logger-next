#pragma once

#define ESP32

#define ACCEL_SPI_HOST SPI2_HOST
#define SDCARD_SPI_HOST SPI3_HOST

#define BUTTON_ENTER_GPIO 0

#define COMP_1_OUT_GPIO 1
#define COMP_2_OUT_GPIO 2
#define COMP_3_OUT_GPIO 4
#define COMP_4_OUT_GPIO 6

#define UART1_RX_GPIO 13
#define UART1_TX_GPIO 21

#define ACC_INT_GPIO 5
#define ACC_SPI_CLK_GPIO 7
#define ACC_SPI_MISO_GPIO 9
#define ACC_SPI_MOSI_GPIO 11
#define ACC_SPI_CS_GPIO 12

#define LED_OUT_GPIO 40

#define SD_SPI_CLK_GPIO 36
#define SD_SPI_CS_GPIO 34
#define SD_SPI_MISO_GPIO 3
#define SD_SPI_MOSI_GPIO 8

#define DAC_OUT_1_GPIO 17
#define DAC_OUT_2_GPIO 18

// Task Priority Definitions
typedef enum {
    TASK_PRIORITY_IDLE = 0,         // tskIDLE_PRIORITY equivalent
    TASK_PRIORITY_DEFAULT = 1,      // Default priority for most tasks
    TASK_PRIORITY_APP_LOGIC = 2,    // App logic task priority
    TASK_PRIORITY_UART_EVENT = 10,  // UART event task priority
} task_priority_t;
