#pragma once

#include <esp_err.h>

typedef esp_err_t hal_err_t;

#define HAL_ERR_NONE ESP_OK
#define HAL_ERR_FAILED ESP_FAIL
#define HAL_ERR_ASSERT_OK(e) ESP_ERROR_CHECK(e)

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif