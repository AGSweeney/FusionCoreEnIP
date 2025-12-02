/**
 * @file gp8403_dac.c
 * @brief DFRobot Gravity 2-Channel I2C DAC Module (0-10V) Driver Implementation
 * 
 * ESP-IDF port of the GP8403 DAC driver for ESP32-P4 platform.
 * 
 * This implementation is based on and references the DFRobot_GP8403 Arduino library:
 * https://github.com/DFRobot/DFRobot_GP8403
 * 
 * @note Product: DFRobot Gravity 2-Channel I2C DAC Module (0-10V)
 * @note SKU: DFR0971
 * @note Chip: GP8403
 * @note Product page: https://www.dfrobot.com/product-2613.html
 */

/*
 * Copyright (c) 2025, Adam G. Sweeney <agsweeney@gmail.com>
 * 
 * Portions of this code reference the DFRobot_GP8403 Arduino library:
 * Copyright (c) 2022 DFRobot (https://github.com/DFRobot/DFRobot_GP8403)
 * Written by tangjie (jie.tang@dfrobot.com)
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "gp8403_dac.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "gp8403_dac";

// GP8403 I2C register addresses
#define GP8403_REG_VSEL           0x01  // Voltage level register (0x00 = 5V, 0x11 = 10V)
#define GP8403_REG_CHANNEL0_DATA  0x02  // Channel 0 data register (12-bit)
#define GP8403_REG_CHANNEL1_DATA  0x04  // Channel 1 data register (12-bit)

// Voltage selection values
#define GP8403_VSEL_5V            0x00  // 5V output range
#define GP8403_VSEL_10V           0x11  // 10V output range

// I2C transaction timeout (ms)
#define GP8403_I2C_TIMEOUT_MS       100

/**
 * @brief Clamp voltage to valid range
 */
static inline float clamp_voltage(float voltage)
{
    if (voltage < GP8403_DAC_MIN_VOLTAGE) {
        return GP8403_DAC_MIN_VOLTAGE;
    }
    if (voltage > GP8403_DAC_MAX_VOLTAGE) {
        return GP8403_DAC_MAX_VOLTAGE;
    }
    return voltage;
}

/**
 * @brief Clamp DAC value to valid range
 */
static inline uint16_t clamp_dac_value(uint16_t value)
{
    if (value > GP8403_DAC_MAX_VALUE) {
        return GP8403_DAC_MAX_VALUE;
    }
    return value;
}

/**
 * @brief Write data to GP8403 register
 */
static esp_err_t gp8403_write_register(gp8403_dac_handle_t *handle, uint8_t reg, uint16_t data)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare data: GP8403 expects 12-bit format per datasheet and Arduino example
    // Format: [reg] [data_high_byte] [data_low_byte]
    // DATA High: bits 11-4 (8 bits) - shift value left by 4, take upper 8 bits
    // DATA Low: bits 3-0 in upper nibble (4 bits) - shift value left by 4, take lower byte
    // Per Arduino example: shift left 4, extract bytes, swap, then send high byte first
    uint8_t write_data[3];
    write_data[0] = reg;
    // Format per Arduino example: shift left 4, extract bytes, swap
    uint16_t shifted = data << 4;                    // Shift 12-bit value left by 4
    uint8_t hibyte = (shifted & 0xFF00) >> 8;       // Extract bits 11-4 (upper 8 bits)
    uint8_t lobyte = shifted & 0xFF;                // Extract bits 3-0 in upper nibble (lower byte)
    uint16_t swapped = (lobyte << 8) | hibyte;       // Swap bytes: low becomes high, high becomes low
    write_data[1] = (swapped >> 8) & 0xFF;           // High byte (was low byte)
    write_data[2] = swapped & 0xFF;                 // Low byte (was high byte)

    esp_err_t err = i2c_master_transmit(
        handle->dev_handle,
        write_data,
        sizeof(write_data),
        pdMS_TO_TICKS(GP8403_I2C_TIMEOUT_MS)
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s (reg=0x%02X, data=0x%03X)", 
                 esp_err_to_name(err), reg, data);
        return err;
    }

    // Small delay after write to allow GP8403 to process and release bus
    // This helps prevent bus contention with other I2C devices
    vTaskDelay(pdMS_TO_TICKS(1));

    return ESP_OK;
}

/**
 * @brief Set GP8403 voltage output range (5V or 10V)
 */
static esp_err_t gp8403_set_voltage_range(gp8403_dac_handle_t *handle, uint8_t vsel)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t write_data[2];
    write_data[0] = GP8403_REG_VSEL;
    write_data[1] = vsel;

    esp_err_t err = i2c_master_transmit(
        handle->dev_handle,
        write_data,
        sizeof(write_data),
        pdMS_TO_TICKS(GP8403_I2C_TIMEOUT_MS)
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set voltage range: %s (vsel=0x%02X)", 
                 esp_err_to_name(err), vsel);
        return err;
    }

    return ESP_OK;
}

bool gp8403_dac_init(const gp8403_dac_config_t *config, gp8403_dac_handle_t **handle)
{
    if (config == NULL || handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    if (config->bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C bus handle is NULL");
        return false;
    }

    // Validate I2C address (should be in range 0x58-0x5F for GP8403)
    uint8_t i2c_addr = config->i2c_addr;
    if (i2c_addr == 0) {
        i2c_addr = GP8403_DAC_DEFAULT_I2C_ADDR;
    }

    if (i2c_addr < 0x58 || i2c_addr > 0x5F) {
        ESP_LOGW(TAG, "I2C address 0x%02X may be out of valid range (0x58-0x5F)", i2c_addr);
    }

    // Allocate handle
    gp8403_dac_handle_t *dev_handle = calloc(1, sizeof(gp8403_dac_handle_t));
    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate device handle");
        return false;
    }

    // Configure I2C device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
    };

    esp_err_t err = i2c_master_bus_add_device(
        config->bus_handle,
        &dev_cfg,
        &dev_handle->dev_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        free(dev_handle);
        return false;
    }

    // Initialize handle
    dev_handle->i2c_addr = i2c_addr;
    dev_handle->initialized = true;
    dev_handle->channel0_voltage = 0.0f;
    dev_handle->channel1_voltage = 0.0f;

    // Configure for 10V output range - MUST be done before writing channel values
    esp_err_t vsel_err = gp8403_set_voltage_range(dev_handle, GP8403_VSEL_10V);
    if (vsel_err != ESP_OK) {
        ESP_LOGE(TAG, "CRITICAL: Failed to set 10V range - DAC may not output correctly! Error: %s", 
                 esp_err_to_name(vsel_err));
        // Don't continue if VSEL fails - device won't work correctly
        free(dev_handle);
        return false;
    }
    
    // Small delay after VSEL to ensure it's processed
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Initialize both channels to 0V
    gp8403_write_register(dev_handle, GP8403_REG_CHANNEL0_DATA, 0x000);
    gp8403_write_register(dev_handle, GP8403_REG_CHANNEL1_DATA, 0x000);

    *handle = dev_handle;

    return true;
}

void gp8403_dac_deinit(gp8403_dac_handle_t **handle)
{
    if (handle == NULL || *handle == NULL) {
        return;
    }

    gp8403_dac_handle_t *dev_handle = *handle;

    // Set both channels to 0V before deinitializing
    if (dev_handle->initialized) {
        gp8403_write_register(dev_handle, GP8403_REG_CHANNEL0_DATA, 0x000);
        gp8403_write_register(dev_handle, GP8403_REG_CHANNEL1_DATA, 0x000);
    }

    // Remove I2C device
    if (dev_handle->dev_handle != NULL) {
        i2c_master_bus_rm_device(dev_handle->dev_handle);
    }

    // Free handle
    free(dev_handle);
    *handle = NULL;
}

bool gp8403_dac_set_voltage(gp8403_dac_handle_t *handle, gp8403_channel_t channel, float voltage)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    if (channel >= GP8403_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return false;
    }

    // Clamp voltage to valid range
    float clamped_voltage = clamp_voltage(voltage);

    // Convert voltage to raw DAC value
    uint16_t dac_value = gp8403_dac_voltage_to_raw(clamped_voltage);

    // Select register based on channel
    uint8_t reg = (channel == GP8403_CHANNEL_0) ? GP8403_REG_CHANNEL0_DATA : GP8403_REG_CHANNEL1_DATA;

    // Write to DAC
    esp_err_t err = gp8403_write_register(handle, reg, dac_value);
    if (err != ESP_OK) {
        return false;
    }

    // Update cached voltage
    if (channel == GP8403_CHANNEL_0) {
        handle->channel0_voltage = clamped_voltage;
    } else {
        handle->channel1_voltage = clamped_voltage;
    }

    return true;
}

bool gp8403_dac_set_raw(gp8403_dac_handle_t *handle, gp8403_channel_t channel, uint16_t dac_value)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    if (channel >= GP8403_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid channel: %d", channel);
        return false;
    }

    // Clamp DAC value
    uint16_t clamped_value = clamp_dac_value(dac_value);

    // Select register based on channel
    uint8_t reg = (channel == GP8403_CHANNEL_0) ? GP8403_REG_CHANNEL0_DATA : GP8403_REG_CHANNEL1_DATA;

    // Write to DAC
    esp_err_t err = gp8403_write_register(handle, reg, clamped_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GP8403 channel %d write failed: %s (reg=0x%02X, value=0x%04X)", 
                 channel, esp_err_to_name(err), reg, clamped_value);
        return false;
    }

    // Update cached voltage
    // Calculate voltage from 12-bit value (0-4095 → 0-10V)
    float voltage = gp8403_dac_raw_to_voltage(clamped_value);
    if (channel == GP8403_CHANNEL_0) {
        handle->channel0_voltage = voltage;
    } else {
        handle->channel1_voltage = voltage;
    }

    return true;
}

float gp8403_dac_get_voltage(gp8403_dac_handle_t *handle, gp8403_channel_t channel)
{
    if (handle == NULL || !handle->initialized) {
        return 0.0f;
    }

    if (channel >= GP8403_CHANNEL_MAX) {
        return 0.0f;
    }

    return (channel == GP8403_CHANNEL_0) ? handle->channel0_voltage : handle->channel1_voltage;
}

bool gp8403_dac_set_both_channels(gp8403_dac_handle_t *handle, float voltage)
{
    return gp8403_dac_set_channels(handle, voltage, voltage);
}

bool gp8403_dac_set_channels(gp8403_dac_handle_t *handle, float voltage0, float voltage1)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    bool success = true;

    // Set channel 0
    if (!gp8403_dac_set_voltage(handle, GP8403_CHANNEL_0, voltage0)) {
        success = false;
    }

    // Set channel 1
    if (!gp8403_dac_set_voltage(handle, GP8403_CHANNEL_1, voltage1)) {
        success = false;
    }

    return success;
}

uint16_t gp8403_dac_voltage_to_raw(float voltage)
{
    float clamped = clamp_voltage(voltage);
    
    // Convert voltage (0-10V) to 12-bit value (0x000-0xFFF)
    // Formula: dac_value = (voltage / 10.0) × 4095
    float normalized = clamped / GP8403_DAC_MAX_VOLTAGE;
    uint16_t dac_value = (uint16_t)roundf(normalized * GP8403_DAC_MAX_VALUE);
    
    return clamp_dac_value(dac_value);
}

float gp8403_dac_raw_to_voltage(uint16_t dac_value)
{
    uint16_t clamped = clamp_dac_value(dac_value);
    
    // Convert 12-bit value (0x000-0xFFF) to voltage (0-10V)
    // Formula: voltage = (dac_value / 4095) × 10.0
    float normalized = (float)clamped / GP8403_DAC_MAX_VALUE;
    return normalized * GP8403_DAC_MAX_VOLTAGE;
}

bool gp8403_dac_is_initialized(gp8403_dac_handle_t *handle)
{
    return (handle != NULL && handle->initialized);
}

uint8_t gp8403_dac_get_i2c_addr(gp8403_dac_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    return handle->i2c_addr;
}

