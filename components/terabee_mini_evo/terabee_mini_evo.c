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

#include "terabee_mini_evo.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include <string.h>

static const char *TAG = "terabee_mini_evo";

// Terabee Evo Mini I2C Register Map
// TODO: Update with actual register addresses from datasheet
#define TERABEE_REG_DISTANCE_LOW     0x00  // Distance measurement low byte
#define TERABEE_REG_DISTANCE_HIGH    0x01  // Distance measurement high byte
#define TERABEE_REG_STATUS           0x02  // Status register
#define TERABEE_REG_SIGNAL_LOW       0x03  // Signal strength low byte (if available)
#define TERABEE_REG_SIGNAL_HIGH      0x04  // Signal strength high byte (if available)
#define TERABEE_REG_VERSION          0x10  // Firmware version (if available)
#define TERABEE_REG_COMMAND          0x20  // Command register (if available)

// Status bits
#define TERABEE_STATUS_VALID         (1 << 0)
#define TERABEE_STATUS_READY         (1 << 1)
#define TERABEE_STATUS_ERROR         (1 << 2)

// CRC8 Configuration
// Terabee Evo Mini uses CRC8 with polynomial 0x1D (SAE J1850 standard)
// Polynomial: x^8 + x^4 + x^3 + x^2 + 1 = 0x1D
#define TERABEE_CRC8_POLYNOMIAL      0x1D
#define TERABEE_CRC8_INITIAL_VALUE   0xFF

/**
 * @brief Calculate CRC8 checksum using Terabee's polynomial (0x1D, SAE J1850)
 * 
 * @param data Pointer to data buffer
 * @param length Number of bytes to calculate CRC over
 * @return Calculated CRC8 value
 */
static uint8_t terabee_crc8_calculate(const uint8_t *data, size_t length)
{
    uint8_t crc = TERABEE_CRC8_INITIAL_VALUE;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ TERABEE_CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

esp_err_t terabee_mini_evo_init(terabee_mini_evo_handle_t *handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Probe device
    esp_err_t ret = i2c_master_probe(handle->bus_handle, handle->i2c_address, 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Device not found at address 0x%02X", handle->i2c_address);
        return ret;
    }

    ESP_LOGI(TAG, "Terabee Evo Mini detected at address 0x%02X", handle->i2c_address);

    // TODO: Add initialization sequence (reset, configure, etc.)
    // This may include:
    // - Soft reset command
    // - Configuration register setup
    // - Timing/measurement mode setup
    // - Wait for sensor to be ready

    handle->initialized = true;
    ESP_LOGI(TAG, "Terabee Evo Mini initialized successfully");

    return ESP_OK;
}

esp_err_t terabee_mini_evo_deinit(terabee_mini_evo_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->initialized = false;
    ESP_LOGI(TAG, "Terabee Evo Mini deinitialized");

    return ESP_OK;
}

esp_err_t terabee_mini_evo_read_distance(terabee_mini_evo_handle_t *handle, 
                                         terabee_mini_evo_data_t *data)
{
    if (handle == NULL || data == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Create I2C device handle
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = handle->i2c_address,
        .scl_speed_hz = 400000,  // 400kHz as specified in Terabee Evo Mini datasheet
    };

    esp_err_t ret = i2c_master_bus_add_device(handle->bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    // Terabee Evo Mini protocol: Read 3 bytes (2 bytes distance + 1 byte CRC8)
    // Sensor is free-running, so we just read the current measurement
    uint8_t read_buffer[3] = {0};
    ret = i2c_master_receive(dev_handle, read_buffer, 3, 100);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read distance: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev_handle);
        return ret;
    }

    // Clean up device handle
    i2c_master_bus_rm_device(dev_handle);

    // Verify CRC8 checksum (3rd byte)
    // CRC is calculated over the first 2 bytes (distance data)
    uint8_t crc_received = read_buffer[2];
    uint8_t crc_calculated = terabee_crc8_calculate(read_buffer, 2);
    
    if (crc_received != crc_calculated) {
        ESP_LOGW(TAG, "CRC mismatch: received=0x%02X, calculated=0x%02X", 
                 crc_received, crc_calculated);
        // Still parse the data but mark as invalid
        data->valid = false;
    } else {
        data->valid = true;
    }

    // Parse distance (2 bytes: first byte is low, second byte is high)
    // Distance is in millimeters, 16-bit value
    data->distance_mm = (uint16_t)(read_buffer[0] | (read_buffer[1] << 8));
    
    // Status: Set based on CRC validity and distance range
    // Evo Mini range is typically 0-6000mm
    data->status = data->valid ? 0x01 : 0x00;
    
    // Additional validity check: distance should be within sensor range
    if (data->distance_mm == 0 || data->distance_mm > 6000) {
        data->valid = false;
        data->status = 0x00;
    }

    // TODO: Read signal strength if available
    data->signal_strength = 0;

    return ESP_OK;
}

esp_err_t terabee_mini_evo_is_ready(terabee_mini_evo_handle_t *handle, bool *ready)
{
    if (handle == NULL || ready == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->initialized) {
        *ready = false;
        return ESP_ERR_INVALID_STATE;
    }

    // Create I2C device handle
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = handle->i2c_address,
        .scl_speed_hz = 400000,  // 400kHz as specified in Terabee Evo Mini datasheet
    };

    esp_err_t ret = i2c_master_bus_add_device(handle->bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        *ready = false;
        return ret;
    }

    // Read status register
    uint8_t status = 0;
    ret = i2c_master_transmit_receive(dev_handle,
                                      (uint8_t[]){TERABEE_REG_STATUS}, 1,
                                      &status, 1, 100);

    i2c_master_bus_rm_device(dev_handle);

    if (ret != ESP_OK) {
        *ready = false;
        return ret;
    }

    *ready = (status & TERABEE_STATUS_READY) != 0;
    return ESP_OK;
}

esp_err_t terabee_mini_evo_trigger_measurement(terabee_mini_evo_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // TODO: Implement trigger command if sensor supports triggered mode
    // This would write to TERABEE_REG_COMMAND to start a measurement

    ESP_LOGW(TAG, "Trigger measurement not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t terabee_mini_evo_get_version(terabee_mini_evo_handle_t *handle, char *version)
{
    if (handle == NULL || version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // TODO: Implement version reading if sensor supports it
    // This would read from TERABEE_REG_VERSION or a version string register

    strncpy(version, "Unknown", 15);
    version[15] = '\0';
    
    ESP_LOGW(TAG, "Get version not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;
}

