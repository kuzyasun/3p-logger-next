#pragma once

#include <stddef.h>

#include "hal/err.h"
#include "hal/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque file handle
typedef void *sdcard_file_handle_t;

// Public API
hal_err_t sdcard_init(hal_spi_bus_t bus, hal_gpio_t cs_pin);
hal_err_t sdcard_open_file(const char *path, const char *mode, sdcard_file_handle_t *handle);
int sdcard_write(sdcard_file_handle_t handle, const void *data, size_t size);
hal_err_t sdcard_fsync(sdcard_file_handle_t handle);
hal_err_t sdcard_close_file(sdcard_file_handle_t handle);
hal_err_t sdcard_deinit(void);

#ifdef __cplusplus
}
#endif
