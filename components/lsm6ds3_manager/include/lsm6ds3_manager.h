#ifndef LSM6DS3_MANAGER_H
#define LSM6DS3_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lsm6ds3_manager_init(void);

bool lsm6ds3_manager_is_initialized(void);

esp_err_t lsm6ds3_manager_calibrate_gyro(uint32_t samples, uint32_t sample_delay_ms);

esp_err_t lsm6ds3_manager_calibrate_gyro_preview(uint32_t samples, uint32_t sample_delay_ms, float offset_mdps[3]);

esp_err_t lsm6ds3_manager_get_calibration_status(bool *accel_calibrated, bool *gyro_calibrated);

esp_err_t lsm6ds3_manager_get_gyro_offsets(float offset_mdps[3]);

esp_err_t lsm6ds3_manager_set_angle_zero(float roll, float pitch, float yaw);

esp_err_t lsm6ds3_manager_clear_angle_zero(void);

esp_err_t lsm6ds3_manager_get_angle_zero(float *roll, float *pitch, float *yaw, bool *roll_set, bool *pitch_set, bool *yaw_set);

esp_err_t lsm6ds3_manager_save_calibration(void);

esp_err_t lsm6ds3_manager_load_calibration(void);

esp_err_t lsm6ds3_manager_get_nvs_calibration(float accel_offset_mg[3], float gyro_offset_mdps[3], bool *accel_calibrated, bool *gyro_calibrated);

#ifdef __cplusplus
}
#endif

#endif

