#ifndef NAU7802_MANAGER_H
#define NAU7802_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t nau7802_manager_init(void);

bool nau7802_manager_is_initialized(void);

void* nau7802_manager_get_device_handle(void);

SemaphoreHandle_t nau7802_manager_get_mutex(void);

#ifdef __cplusplus
}
#endif

#endif

