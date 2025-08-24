#include "hal/sdcard.h"

#include <driver/sdspi_host.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <stdio.h>
#include <unistd.h>

static sdmmc_card_t *g_card;

hal_err_t sdcard_init(hal_spi_bus_t bus, hal_gpio_t cs_pin, const char *mount_path, bool format_if_mount_failed) {
    // SPI bus is already initialized by the main application
    // No need to initialize it again here

    // Configure SD SPI device
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = bus;

    // Configure SDMMC host
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = bus;

    // Configure mount options
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_mount_failed,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    // Mount the SD card using the correct API
    esp_err_t ret = esp_vfs_fat_sdspi_mount(mount_path, &host, &slot_config, &mount_config, &g_card);
    if (ret != ESP_OK) {
        const char *error_msg = esp_err_to_name(ret);
        ESP_LOGE("SDCARD", "Failed to mount SD card: %s (0x%x)", error_msg, ret);
        return ret;
    }
    return HAL_ERR_NONE;
}

hal_err_t sdcard_open_file(const char *path, const char *mode, sdcard_file_handle_t *handle) {
    FILE *f = fopen(path, mode);
    if (!f) {
        return HAL_ERR_FAILED;
    }
    *handle = f;
    return HAL_ERR_NONE;
}

int sdcard_write(sdcard_file_handle_t handle, const void *data, size_t size) { return fwrite(data, 1, size, (FILE *)handle); }

hal_err_t sdcard_fsync(sdcard_file_handle_t handle) { return fsync(fileno((FILE *)handle)) == 0 ? HAL_ERR_NONE : HAL_ERR_FAILED; }

hal_err_t sdcard_close_file(sdcard_file_handle_t handle) { return fclose((FILE *)handle) == 0 ? HAL_ERR_NONE : HAL_ERR_FAILED; }

hal_err_t sdcard_deinit(const char *mount_path) {
    esp_vfs_fat_sdcard_unmount(mount_path, g_card);
    g_card = NULL;
    return HAL_ERR_NONE;
}
