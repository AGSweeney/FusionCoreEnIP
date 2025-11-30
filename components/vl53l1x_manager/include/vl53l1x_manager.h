#ifndef VL53L1X_MANAGER_H
#define VL53L1X_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t vl53l1x_manager_init(void);

bool vl53l1x_manager_is_initialized(void);

esp_err_t vl53l1x_manager_get_config(system_vl53l1x_config_t *config);

esp_err_t vl53l1x_manager_set_config(const system_vl53l1x_config_t *config);

esp_err_t vl53l1x_manager_calibrate_offset(int16_t target_distance_mm);

esp_err_t vl53l1x_manager_calibrate_xtalk(uint16_t target_distance_mm);

#ifdef __cplusplus
}
#endif

#endif

