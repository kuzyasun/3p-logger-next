#include <log.h>
#include <target.h>
#include <version.h>

#include "app_logic.h"
#include "esp_attr.h"
#include "esp_log_level.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio.h"
#include "io/io_manager.h"
#include "platform/system.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

static inline void reboot_to_download_mode(void) {
    // Примусити ROM-завантажувач на наступний рестарт
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
}

// SET BOOT MODE
static void usb_boot_cmd_task(void *arg) {
    const char *MAGIC = "BOOT\n";  // що надсилатиме PlatformIO
    char line[16];
    int idx = 0;

    while (1) {
        int c = fgetc(stdin);  // консоль уже на USB-CDC (CONFIG_ESP_CONSOLE_USB_CDC=y)
        if (c < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r') continue;
        if (idx < (int)sizeof(line) - 1) line[idx++] = (char)c;
        if (c == '\n') {
            line[idx] = 0;
            idx = 0;
            if (strcmp(line, MAGIC) == 0) {
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(50));
                reboot_to_download_mode();  // перехід у ROM USB bootloader
            }
        }
    }
}

void usb_boot_cmd_start(void) { xTaskCreatePinnedToCore(usb_boot_cmd_task, "usb_boot_cmd", 3072, NULL, 1, NULL, tskNO_AFFINITY); }

static const char *TAG = "Main";

static io_manager_t io_manager;
static app_logic_t app_logic;
static led_module_t led_module;

void app_main() {
    usb_boot_cmd_start();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // esp_log_set_level_master(ESP_LOG_VERBOSE);
    // esp_log_level_set("*", ESP_LOG_DEBUG);
    // esp_log_level_set("rmt", ESP_LOG_DEBUG);

    esp_log_set_level_master(ESP_LOG_INFO);
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("rmt", ESP_LOG_INFO);

    // Chip info
    LOG_I(TAG, "3P Logger %s", FIRMWARE_VERSION);
    LOG_I(TAG, "Build timestamp: %s", BUILD_TIMESTAMP);
    LOG_I(TAG, "Git hash: %s", GIT_HASH);
    LOG_I(TAG, "ESP32 Model: %s", system_get_esp32_model());
    LOG_I(TAG, "ESP32 Chip Revision: %d", system_get_chip_revision());
    LOG_I(TAG, "ESP32 CPU Frequency: %lu MHz", system_get_cpu_frequency_mhz());
    LOG_I(TAG, "ESP32 CPU Cores: %lu", system_get_cpu_cores());

    io_manager_init(&io_manager);
    led_module_init(&led_module);

    app_logic_init(&app_logic, &io_manager, &led_module);
    app_logic_start_all_tasks(&app_logic);
}