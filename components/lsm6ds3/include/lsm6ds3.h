/**
 * @file lsm6ds3.h
 * @brief LSM6DS3 6-axis accelerometer and gyroscope driver
 * 
 * This driver provides an interface to control the LSM6DS3 6-axis motion sensor.
 * The LSM6DS3 combines a 3-axis accelerometer and 3-axis gyroscope on a single chip.
 * 
 * This driver uses register definitions from the STMicroelectronics
 * STMems_Standard_C_drivers library:
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers
 * 
 * The register driver files (driver/lsm6ds3_reg.c and driver/lsm6ds3_reg.h)
 * are from STMicroelectronics and are licensed under BSD 3-Clause License.
 * The full BSD 3-Clause license text is included in those files.
 * 
 * Features:
 * - 3-axis accelerometer (2g, 4g, 8g, 16g ranges)
 * - 3-axis gyroscope (125, 250, 500, 1000, 2000 DPS ranges)
 * - I2C and SPI interfaces
 * - Hardware offset compensation
 * - Software calibration support
 * - NVS calibration storage
 * - Block data update mode
 * 
 * @note Supports both I2C and SPI communication interfaces
 * @note Register driver files (lsm6ds3_reg.h/c) are from STMicroelectronics
 */

/*
 * MIT License
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * 
 * This driver incorporates code from STMicroelectronics:
 * Copyright (c) 2018-2025 STMicroelectronics
 * Licensed under BSD 3-Clause License
 * 
 * The register driver files (driver/lsm6ds3_reg.c and driver/lsm6ds3_reg.h)
 * contain the full BSD 3-Clause license text and copyright notice as required
 * by the BSD 3-Clause License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef LSM6DS3_H
#define LSM6DS3_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "lsm6ds3_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Communication interface type
 */
typedef enum {
    LSM6DS3_INTERFACE_I2C = 0,  /**< I2C interface */
    LSM6DS3_INTERFACE_SPI = 1   /**< SPI interface */
} lsm6ds3_interface_t;

/**
 * @brief Calibration data structure
 */
typedef struct {
    float accel_offset_mg[3];    /**< Accelerometer offset in mg [x, y, z] */
    float gyro_offset_mdps[3];   /**< Gyroscope offset in mdps [x, y, z] */
    bool accel_calibrated;       /**< Accelerometer calibration status */
    bool gyro_calibrated;        /**< Gyroscope calibration status */
} lsm6ds3_calibration_t;

/**
 * @brief LSM6DS3 device handle structure
 */
typedef struct {
    lsm6ds3_interface_t interface;  /**< Communication interface type */
    union {
        i2c_master_dev_handle_t i2c_dev;    /**< I2C device handle */
        spi_device_handle_t spi_device;      /**< SPI device handle */
    } bus_handle;                            /**< Bus handle union */
    lsm6ds3_ctx_t ctx;                      /**< Register context */
    lsm6ds3_calibration_t calibration;      /**< Calibration data */
} lsm6ds3_handle_t;

/**
 * @brief LSM6DS3 configuration structure
 */
typedef struct {
    lsm6ds3_interface_t interface;  /**< Communication interface type */
    union {
        struct {
            i2c_master_bus_handle_t bus_handle;  /**< I2C bus handle */
            uint8_t address;                     /**< I2C device address */
        } i2c;                                   /**< I2C configuration */
        struct {
            spi_device_handle_t device_handle;   /**< SPI device handle */
        } spi;                                   /**< SPI configuration */
    } bus;                                       /**< Bus configuration union */
} lsm6ds3_config_t;

/**
 * @brief Function prototypes
 * @{
 */

/**
 * @brief Initialize LSM6DS3 device
 * 
 * Initializes the LSM6DS3 sensor with the specified configuration.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_init(lsm6ds3_handle_t *handle, const lsm6ds3_config_t *config);
/**
 * @brief Deinitialize LSM6DS3 device
 * 
 * Releases resources associated with the LSM6DS3 device.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_deinit(lsm6ds3_handle_t *handle);

/**
 * @brief Reset LSM6DS3 device
 * 
 * Performs a software reset of the LSM6DS3 sensor.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_reset(lsm6ds3_handle_t *handle);

/**
 * @brief Read device ID
 * 
 * Reads the WHO_AM_I register to verify device communication.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param device_id Pointer to store device ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_get_device_id(lsm6ds3_handle_t *handle, uint8_t *device_id);

/**
 * @brief Set accelerometer output data rate
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param odr Output data rate (see lsm6ds3_reg.h for available rates)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_accel_odr(lsm6ds3_handle_t *handle, lsm6ds3_odr_xl_t odr);

/**
 * @brief Set accelerometer full-scale range
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param fs Full-scale range (2g, 4g, 8g, 16g)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_accel_full_scale(lsm6ds3_handle_t *handle, lsm6ds3_fs_xl_t fs);

/**
 * @brief Set gyroscope output data rate
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param odr Output data rate (see lsm6ds3_reg.h for available rates)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_gyro_odr(lsm6ds3_handle_t *handle, lsm6ds3_odr_g_t odr);

/**
 * @brief Set gyroscope full-scale range
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param fs Full-scale range (125, 250, 500, 1000, 2000 DPS)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_gyro_full_scale(lsm6ds3_handle_t *handle, lsm6ds3_fs_g_t fs);

/**
 * @brief Read accelerometer data
 * 
 * Reads accelerometer data in mg (milli-g).
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param accel_mg Array to store accelerometer data [x, y, z] in mg
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_read_accel(lsm6ds3_handle_t *handle, float accel_mg[3]);

/**
 * @brief Read gyroscope data
 * 
 * Reads gyroscope data in mdps (milli-degrees per second).
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param gyro_mdps Array to store gyroscope data [x, y, z] in mdps
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_read_gyro(lsm6ds3_handle_t *handle, float gyro_mdps[3]);

/**
 * @brief Enable/disable block data update mode
 * 
 * When enabled, output registers are not updated until both MSB and LSB are read.
 * This prevents reading data while registers are being updated.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param enable true to enable, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_enable_block_data_update(lsm6ds3_handle_t *handle, bool enable);

/**
 * @brief Calibrate accelerometer
 * 
 * Performs accelerometer calibration by collecting samples and calculating offsets.
 * The device should be stationary during calibration.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param samples Number of samples to collect
 * @param sample_delay_ms Delay between samples in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_calibrate_accel(lsm6ds3_handle_t *handle, uint32_t samples, uint32_t sample_delay_ms);

/**
 * @brief Calibrate gyroscope
 * 
 * Performs gyroscope calibration by collecting samples and calculating offsets.
 * The device should be stationary during calibration.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param samples Number of samples to collect
 * @param sample_delay_ms Delay between samples in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_calibrate_gyro(lsm6ds3_handle_t *handle, uint32_t samples, uint32_t sample_delay_ms);

/**
 * @brief Set accelerometer software offset
 * 
 * Sets the software offset for accelerometer calibration.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param offset_mg Offset values in mg [x, y, z]
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_accel_offset(lsm6ds3_handle_t *handle, const float offset_mg[3]);

/**
 * @brief Set accelerometer software offset for a single axis
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param axis Axis index (0=x, 1=y, 2=z)
 * @param offset_mg Offset value in mg
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_accel_offset_axis(lsm6ds3_handle_t *handle, uint8_t axis, float offset_mg);

/**
 * @brief Set gyroscope software offset
 * 
 * Sets the software offset for gyroscope calibration.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param offset_mdps Offset values in mdps [x, y, z]
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_gyro_offset(lsm6ds3_handle_t *handle, const float offset_mdps[3]);

/**
 * @brief Set gyroscope software offset for a single axis
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param axis Axis index (0=x, 1=y, 2=z)
 * @param offset_mdps Offset value in mdps
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_gyro_offset_axis(lsm6ds3_handle_t *handle, uint8_t axis, float offset_mdps);

/**
 * @brief Get accelerometer software offset
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param offset_mg Array to store offset values in mg [x, y, z]
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_get_accel_offset(lsm6ds3_handle_t *handle, float offset_mg[3]);

/**
 * @brief Get gyroscope software offset
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param offset_mdps Array to store offset values in mdps [x, y, z]
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_get_gyro_offset(lsm6ds3_handle_t *handle, float offset_mdps[3]);

/**
 * @brief Clear accelerometer calibration
 * 
 * Resets accelerometer calibration offsets to zero.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_clear_accel_calibration(lsm6ds3_handle_t *handle);

/**
 * @brief Clear gyroscope calibration
 * 
 * Resets gyroscope calibration offsets to zero.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_clear_gyro_calibration(lsm6ds3_handle_t *handle);

/**
 * @brief Set accelerometer hardware offset registers
 * 
 * Sets the hardware offset compensation registers in the sensor.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param offset_x X-axis offset (-128 to 127)
 * @param offset_y Y-axis offset (-128 to 127)
 * @param offset_z Z-axis offset (-128 to 127)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_accel_hw_offset(lsm6ds3_handle_t *handle, int8_t offset_x, int8_t offset_y, int8_t offset_z);

/**
 * @brief Set accelerometer hardware offset for a single axis
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param axis Axis index (0=x, 1=y, 2=z)
 * @param offset Offset value (-128 to 127)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_accel_hw_offset_axis(lsm6ds3_handle_t *handle, uint8_t axis, int8_t offset);

/**
 * @brief Set gyroscope hardware offset registers
 * 
 * Sets the hardware offset compensation registers in the sensor.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param offset_x X-axis offset (-128 to 127)
 * @param offset_y Y-axis offset (-128 to 127)
 * @param offset_z Z-axis offset (-128 to 127)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_set_gyro_hw_offset(lsm6ds3_handle_t *handle, int8_t offset_x, int8_t offset_y, int8_t offset_z);

/**
 * @brief Save calibration data to NVS
 * 
 * Saves the current calibration offsets to non-volatile storage.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param namespace NVS namespace to use for storage
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_save_calibration_to_nvs(lsm6ds3_handle_t *handle, const char *namespace);

/**
 * @brief Load calibration data from NVS
 * 
 * Loads calibration offsets from non-volatile storage.
 * 
 * @param handle Pointer to LSM6DS3 handle structure
 * @param namespace NVS namespace to use for loading
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lsm6ds3_load_calibration_from_nvs(lsm6ds3_handle_t *handle, const char *namespace);

/** @} */

#ifdef __cplusplus
}
#endif

#endif

