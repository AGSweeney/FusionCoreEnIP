/*******************************************************************************
 * Copyright (c) 2025, Adam G. Sweeney <agsweeney@gmail.com>
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/

#ifndef TERABEE_MINI_EVO_H
#define TERABEE_MINI_EVO_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Terabee Evo Mini I2C address
 * 
 * Default I2C address for Terabee Evo Mini sensor (7-bit address)
 * Can be changed using CHANGE_BASE_ADDR command (0xA2) to range 0x02-0x7F
 */
#define TERABEE_MINI_EVO_I2C_ADDRESS_DEFAULT 0x31

/**
 * @brief Terabee Evo Mini device handle
 */
typedef struct {
    i2c_master_bus_handle_t bus_handle;  /**< I2C bus handle */
    uint8_t i2c_address;                 /**< I2C device address */
    bool initialized;                     /**< Initialization status */
} terabee_mini_evo_handle_t;

/**
 * @brief Terabee Evo Mini measurement data
 */
typedef struct {
    uint16_t distance_mm;        /**< Distance measurement in millimeters */
    uint8_t status;              /**< Measurement status/quality */
    uint16_t signal_strength;    /**< Signal strength (if available) */
    bool valid;                  /**< Whether measurement is valid */
} terabee_mini_evo_data_t;

/**
 * @brief Initialize Terabee Evo Mini sensor
 * 
 * @param handle Pointer to device handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_init(terabee_mini_evo_handle_t *handle);

/**
 * @brief Deinitialize Terabee Evo Mini sensor
 * 
 * @param handle Pointer to device handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_deinit(terabee_mini_evo_handle_t *handle);

/**
 * @brief Read distance measurement from sensor
 * 
 * @param handle Pointer to device handle structure
 * @param data Pointer to data structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_read_distance(terabee_mini_evo_handle_t *handle, 
                                         terabee_mini_evo_data_t *data);

/**
 * @brief Check if sensor is ready for measurement
 * 
 * @param handle Pointer to device handle structure
 * @param ready Pointer to boolean to store ready status
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_is_ready(terabee_mini_evo_handle_t *handle, bool *ready);

/**
 * @brief Trigger a measurement (if sensor supports triggered mode)
 * 
 * @param handle Pointer to device handle structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_trigger_measurement(terabee_mini_evo_handle_t *handle);

/**
 * @brief Get sensor firmware version
 * 
 * @param handle Pointer to device handle structure
 * @param version Pointer to store version string (must be at least 16 bytes)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_get_version(terabee_mini_evo_handle_t *handle, char *version);

#ifdef __cplusplus
}
#endif

#endif /* TERABEE_MINI_EVO_H */

