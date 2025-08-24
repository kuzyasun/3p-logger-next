#include "accel_module.h"

#include <driver/spi_master.h>
#include <string.h>

#include "app_state.h"
#include "hal/gpio.h"
#include "log.h"
#include "target.h"

static const char *TAG = "ACCEL";

// --- Platform-specific functions to connect driver to HAL ---
static int32_t platform_spi_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    hal_spi_device_handle_t *dev = (hal_spi_device_handle_t *)handle;
    // For writing we cannot use the `cmd` field because the ESP-IDF driver
    // sends it before the data without holding CS.
    // So we form a buffer where the first byte is the register address.
    uint8_t tx_buffer[len + 1];
    tx_buffer[0] = reg;
    memcpy(&tx_buffer[1], bufp, len);
    hal_err_t result = hal_spi_device_transmit(dev, 0, 0, tx_buffer, len + 1, NULL, 0);
    LOG_D(TAG, "SPI Write: reg=0x%02X, len=%d, result=%d", reg, len, result);
    return result;
}

static int32_t platform_spi_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    hal_spi_device_handle_t *dev = (hal_spi_device_handle_t *)handle;

    if (len == 0) {
        return 0;  // Nothing to read
    }

    // SPI transaction for LIS3DH requires sending address, then reading data.
    // This means we need to send len + 1 bytes and receive len + 1 bytes.
    // The first received byte will be garbage.

    // Maximum buffer size that might be needed
    uint8_t tx_buffer[len + 1];
    uint8_t rx_buffer[len + 1];

    // 1. Form the first byte (command)
    tx_buffer[0] = reg | 0x80;  // Read flag
    if (len > 1) {
        tx_buffer[0] |= 0x40;  // Flag for auto-increment for multiple byte reads
    }

    // The rest of the buffer for transmission can be zeros (dummy bytes)
    memset(&tx_buffer[1], 0, len);

    // 2. Perform one transaction that sends the command and dummy bytes,
    //    and simultaneously reads the garbage and useful data.
    hal_err_t result = hal_spi_device_transmit(dev, 0, 0, tx_buffer, len + 1, rx_buffer, len + 1);

    if (result != HAL_ERR_NONE) {
        LOG_E(TAG, "hal_spi_device_transmit failed with error %d", result);
        return result;
    }

    // 3. Copy useful data (skipping the first garbage byte) to the output buffer.
    memcpy(bufp, &rx_buffer[1], len);

    LOG_D(TAG, "SPI Read: reg=0x%02X -> OK, data[0]=0x%02X", reg, bufp[0]);

    return result;
}

static void platform_delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// --- Module Task ---
static void accel_module_task(void *arg) {
    accel_module_t *module = (accel_module_t *)arg;
    app_state_t *state = app_state_get_instance();
    int16_t raw_accel[3];

    while (1) {
        uint8_t data_ready = 0;
        lis3dh_xl_data_ready_get(&module->driver_ctx, &data_ready);

        if (data_ready) {
            if (lis3dh_acceleration_raw_get(&module->driver_ctx, raw_accel) == 0) {
                app_state_begin_update();
                app_state_set_i16(APP_STATE_FIELD_ACCEL_X, &state->accel_x, raw_accel[0]);
                app_state_set_i16(APP_STATE_FIELD_ACCEL_Y, &state->accel_y, raw_accel[1]);
                app_state_set_i16(APP_STATE_FIELD_ACCEL_Z, &state->accel_z, raw_accel[2]);
                app_state_end_update();
                LOG_D(TAG, "Accel Raw X:%d Y:%d Z:%d", raw_accel[0], raw_accel[1], raw_accel[2]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- Public API Implementation ---
hal_err_t accel_module_init(accel_module_t *module, const accel_config_t *config) {
    if (!module || !config) {
        return HAL_ERR_FAILED;
    }

    module->config = *config;
    module->initialized = false;

    hal_spi_device_config_t spi_cfg = {
        .clock_speed_hz = 4 * 1000 * 1000,
        .spi_mode = 0,
        .cs = module->config.cs_pin,
        .command_bits = 0,
        .address_bits = 0,
    };

    LOG_I(TAG, "Adding SPI device: bus=%d, cs=%d, clock=%d Hz, mode=%d", module->config.spi_bus, module->config.cs_pin, spi_cfg.clock_speed_hz,
          spi_cfg.spi_mode);
    if (hal_spi_bus_add_device(module->config.spi_bus, &spi_cfg, &module->spi_dev) != HAL_ERR_NONE) {
        LOG_E(TAG, "Failed to add SPI device");
        return HAL_ERR_FAILED;
    }
    LOG_I(TAG, "SPI device added successfully");

    module->driver_ctx.write_reg = platform_spi_write;
    module->driver_ctx.read_reg = platform_spi_read;
    module->driver_ctx.mdelay = platform_delay;
    module->driver_ctx.handle = &module->spi_dev;

    LOG_I(TAG, "Reading WhoAmI register...");

    uint8_t whoamI = 0;
    int32_t result = lis3dh_device_id_get(&module->driver_ctx, &whoamI);
    LOG_I(TAG, "WhoAmI read result: %d, value: 0x%02X", result, whoamI);
    if (result != 0) {
        LOG_E(TAG, "Failed to read WhoAmI register, error: %d", result);
        return HAL_ERR_FAILED;
    }
    if (whoamI != LIS3DH_ID) {
        LOG_E(TAG, "Invalid WhoAmI: 0x%02X, expected 0x%02X", whoamI, LIS3DH_ID);
        return HAL_ERR_FAILED;
    }
    LOG_I(TAG, "LIS3DH WhoAmI check passed.");

    lis3dh_boot_set(&module->driver_ctx, PROPERTY_ENABLE);
    platform_delay(10);

    lis3dh_operating_mode_set(&module->driver_ctx, module->config.op_mode);
    lis3dh_full_scale_set(&module->driver_ctx, module->config.full_scale);
    lis3dh_data_rate_set(&module->driver_ctx, module->config.output_data_rate);
    lis3dh_block_data_update_set(&module->driver_ctx, PROPERTY_ENABLE);

    LOG_I(TAG, "LIS3DH initialized successfully.");
    module->initialized = true;
    return HAL_ERR_NONE;
}

void accel_module_create_task(accel_module_t *module) {
    if (!module) {
        return;
    }
    xTaskCreatePinnedToCore(accel_module_task, "ACCEL_TASK", 4096, module, TASK_PRIORITY_DEFAULT, &module->task_handle, 0);
}
