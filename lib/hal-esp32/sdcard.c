#include "hal/sdcard.h"

#include <driver/sdspi_host.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <stdio.h>
#include <unistd.h>

#include "target.h"

static sdmmc_card_t *g_card;

hal_err_t sdcard_init(hal_spi_bus_t bus) {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = bus;

    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_SPI_CS_GPIO;
    slot_config.host_id = bus;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    esp_err_t ret = esp_vfs_fat_sdspi_mount("/", &host, &slot_config, &mount_config, &g_card);
    if (ret != ESP_OK) {
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

hal_err_t sdcard_deinit(void) {
    esp_vfs_fat_sdcard_unmount("/", g_card);
    g_card = NULL;
    return HAL_ERR_NONE;
}
