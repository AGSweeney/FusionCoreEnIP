/**
 * @file max7219.c
 * @brief MAX7219 7-Segment Display Driver Implementation
 * 
 * ESP-IDF port of the MAX7219 driver for ESP32-P4 platform.
 * 
 * @note Chip: MAX7219
 * @note Interface: SPI
 */

/*
 * Copyright (c) 2025, Adam G. Sweeney <agsweeney@gmail.com>
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

#include "max7219.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

static const char *TAG = "max7219";

// SPI transaction timeout (ms)
#define MAX7219_SPI_TIMEOUT_MS       100

// Shutdown register values
#define MAX7219_SHUTDOWN_MODE        0x00
#define MAX7219_NORMAL_MODE          0x01

// Display test register values
#define MAX7219_TEST_DISABLE         0x00
#define MAX7219_TEST_ENABLE          0x01

/**
 * @brief Clamp intensity to valid range
 */
static inline uint8_t clamp_intensity(uint8_t intensity)
{
    if (intensity > MAX7219_MAX_INTENSITY) {
        return MAX7219_MAX_INTENSITY;
    }
    return intensity;
}

/**
 * @brief Clamp scan limit to valid range
 */
static inline uint8_t clamp_scan_limit(uint8_t scan_limit)
{
    if (scan_limit > MAX7219_MAX_SCAN_LIMIT) {
        return MAX7219_MAX_SCAN_LIMIT;
    }
    return scan_limit;
}

/**
 * @brief Clamp device index to valid range
 */
static inline uint8_t clamp_device_index(max7219_handle_t *handle, uint8_t device_index)
{
    if (device_index >= handle->num_devices) {
        return handle->num_devices - 1;
    }
    return device_index;
}

/**
 * @brief Clamp digit to valid range
 */
static inline uint8_t clamp_digit(uint8_t digit)
{
    if (digit >= MAX7219_DIGITS_PER_DEVICE) {
        return MAX7219_DIGITS_PER_DEVICE - 1;
    }
    return digit;
}

/**
 * @brief Send data to MAX7219 via SPI
 * 
 * For cascaded devices, data is sent in reverse order (last device first).
 * Each device receives 16 bits: [register (8 bits)] [data (8 bits)]
 */
static esp_err_t max7219_send_data(max7219_handle_t *handle, uint8_t reg, uint8_t data)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (handle->spi_device == NULL) {
        ESP_LOGE(TAG, "SPI device handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare data buffer for cascaded devices
    // Data is sent MSB first, last device first (for cascading)
    // Each device needs 16 bits: [register] [data]
    // Maximum buffer size: 8 devices × 2 bytes = 16 bytes (safe for stack)
    uint8_t tx_buffer[MAX7219_MAX_DEVICES * 2];

    // Fill buffer with register and data for each cascaded device
    // For cascading, send data for last device first
    for (int i = 0; i < handle->num_devices; i++) {
        tx_buffer[i * 2] = reg;
        tx_buffer[i * 2 + 1] = data;
    }

    spi_transaction_t t = {
        .length = handle->num_devices * 2 * 8,  // Length in bits
        .tx_buffer = tx_buffer,
    };

    esp_err_t err = spi_device_transmit(handle->spi_device, &t);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s (reg=0x%02X, data=0x%02X)", 
                 esp_err_to_name(err), reg, data);
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Send different data to each cascaded device
 * 
 * Allows setting different values for each device in the cascade.
 */
static esp_err_t max7219_send_cascaded_data(max7219_handle_t *handle, uint8_t reg, const uint8_t *data_array)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data_array == NULL) {
        ESP_LOGE(TAG, "Data array is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare data buffer for cascaded devices
    // Maximum buffer size: 8 devices × 2 bytes = 16 bytes (safe for stack)
    uint8_t tx_buffer[MAX7219_MAX_DEVICES * 2];

    // Fill buffer with register and data for each cascaded device
    // For cascading, send data for last device first
    for (int i = 0; i < handle->num_devices; i++) {
        tx_buffer[i * 2] = reg;
        tx_buffer[i * 2 + 1] = data_array[handle->num_devices - 1 - i];
    }

    spi_transaction_t t = {
        .length = handle->num_devices * 2 * 8,  // Length in bits
        .tx_buffer = tx_buffer,
    };

    esp_err_t err = spi_device_transmit(handle->spi_device, &t);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmit failed: %s (reg=0x%02X)", 
                 esp_err_to_name(err), reg);
        return err;
    }

    return ESP_OK;
}

bool max7219_init(const max7219_config_t *config, max7219_handle_t **handle)
{
    if (config == NULL || handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    if (config->spi_device == NULL) {
        ESP_LOGE(TAG, "SPI device handle is NULL");
        return false;
    }

    uint8_t num_devices = config->num_devices;
    if (num_devices == 0) {
        num_devices = 1;  // Default to 1 device
    }
    if (num_devices > MAX7219_MAX_DEVICES) {
        ESP_LOGW(TAG, "Number of devices (%d) exceeds maximum (%d), clamping", 
                 num_devices, MAX7219_MAX_DEVICES);
        num_devices = MAX7219_MAX_DEVICES;
    }

    // Allocate handle
    max7219_handle_t *dev_handle = calloc(1, sizeof(max7219_handle_t));
    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate device handle");
        return false;
    }

    // Initialize handle
    dev_handle->spi_device = config->spi_device;
    dev_handle->num_devices = num_devices;
    dev_handle->initialized = true;
    dev_handle->intensity = 8;  // Default intensity (medium brightness)
    dev_handle->scan_limit = 7; // Default scan limit (all 8 digits)
    dev_handle->shutdown = false;

    // Initialize MAX7219 registers
    // Turn off display test
    if (max7219_send_data(dev_handle, MAX7219_REG_DISPLAY_TEST, MAX7219_TEST_DISABLE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable display test");
        free(dev_handle);
        return false;
    }

    // Set decode mode to none (raw segment control)
    if (max7219_send_data(dev_handle, MAX7219_REG_DECODE_MODE, MAX7219_DECODE_NONE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set decode mode");
        free(dev_handle);
        return false;
    }

    // Set scan limit to all digits
    if (max7219_send_data(dev_handle, MAX7219_REG_SCAN_LIMIT, dev_handle->scan_limit) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan limit");
        free(dev_handle);
        return false;
    }

    // Set intensity
    if (max7219_send_data(dev_handle, MAX7219_REG_INTENSITY, dev_handle->intensity) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set intensity");
        free(dev_handle);
        return false;
    }

    // Clear all digits
    for (uint8_t digit = 0; digit < MAX7219_DIGITS_PER_DEVICE; digit++) {
        max7219_send_data(dev_handle, MAX7219_REG_DIGIT0 + digit, 0x00);
    }

    // Enable display (normal mode)
    if (max7219_send_data(dev_handle, MAX7219_REG_SHUTDOWN, MAX7219_NORMAL_MODE) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable display");
        free(dev_handle);
        return false;
    }

    // Small delay to allow device to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));

    *handle = dev_handle;
    ESP_LOGI(TAG, "MAX7219 initialized successfully (%d device(s))", num_devices);

    return true;
}

void max7219_deinit(max7219_handle_t **handle)
{
    if (handle == NULL || *handle == NULL) {
        return;
    }

    max7219_handle_t *dev_handle = *handle;

    // Clear all digits before shutdown
    if (dev_handle->initialized) {
        max7219_clear_all(dev_handle);
        max7219_set_shutdown(dev_handle, true);
    }

    // Free handle (SPI device is managed externally)
    free(dev_handle);
    *handle = NULL;
}

bool max7219_write_register(max7219_handle_t *handle, max7219_register_t reg, uint8_t value)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    esp_err_t err = max7219_send_data(handle, (uint8_t)reg, value);
    return (err == ESP_OK);
}

bool max7219_set_intensity(max7219_handle_t *handle, uint8_t intensity)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    uint8_t clamped_intensity = clamp_intensity(intensity);
    esp_err_t err = max7219_send_data(handle, MAX7219_REG_INTENSITY, clamped_intensity);
    
    if (err == ESP_OK) {
        handle->intensity = clamped_intensity;
        return true;
    }
    
    return false;
}

bool max7219_set_scan_limit(max7219_handle_t *handle, uint8_t scan_limit)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    uint8_t clamped_scan_limit = clamp_scan_limit(scan_limit);
    esp_err_t err = max7219_send_data(handle, MAX7219_REG_SCAN_LIMIT, clamped_scan_limit);
    
    if (err == ESP_OK) {
        handle->scan_limit = clamped_scan_limit;
        return true;
    }
    
    return false;
}

bool max7219_set_decode_mode(max7219_handle_t *handle, max7219_decode_mode_t decode_mode)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    esp_err_t err = max7219_send_data(handle, MAX7219_REG_DECODE_MODE, (uint8_t)decode_mode);
    return (err == ESP_OK);
}

bool max7219_set_shutdown(max7219_handle_t *handle, bool shutdown)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    uint8_t value = shutdown ? MAX7219_SHUTDOWN_MODE : MAX7219_NORMAL_MODE;
    esp_err_t err = max7219_send_data(handle, MAX7219_REG_SHUTDOWN, value);
    
    if (err == ESP_OK) {
        handle->shutdown = shutdown;
        return true;
    }
    
    return false;
}

bool max7219_set_display_test(max7219_handle_t *handle, bool enable)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    uint8_t value = enable ? MAX7219_TEST_ENABLE : MAX7219_TEST_DISABLE;
    esp_err_t err = max7219_send_data(handle, MAX7219_REG_DISPLAY_TEST, value);
    return (err == ESP_OK);
}

bool max7219_clear_all(max7219_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    bool success = true;
    for (uint8_t digit = 0; digit < MAX7219_DIGITS_PER_DEVICE; digit++) {
        if (!max7219_write_register(handle, MAX7219_REG_DIGIT0 + digit, 0x00)) {
            success = false;
        }
    }
    
    return success;
}

bool max7219_set_digit(max7219_handle_t *handle, uint8_t device_index, uint8_t digit, uint8_t value)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    if (device_index >= handle->num_devices) {
        ESP_LOGE(TAG, "Device index %d out of range (max: %d)", device_index, handle->num_devices - 1);
        return false;
    }

    uint8_t clamped_digit = clamp_digit(digit);
    uint8_t reg = MAX7219_REG_DIGIT0 + clamped_digit;

    // For single device, just send the data
    if (handle->num_devices == 1) {
        return max7219_write_register(handle, (max7219_register_t)reg, value);
    }

    // For cascaded devices, prepare data array
    // Maximum size: 8 devices × 1 byte = 8 bytes (safe for stack)
    uint8_t data_array[MAX7219_MAX_DEVICES] = {0};

    // Set value for the specified device, others get 0 (or could preserve existing)
    data_array[device_index] = value;

    esp_err_t err = max7219_send_cascaded_data(handle, reg, data_array);

    return (err == ESP_OK);
}

bool max7219_set_segments(max7219_handle_t *handle, uint8_t device_index, uint8_t digit, uint8_t segments)
{
    // Same as set_digit, but for raw segment control
    return max7219_set_digit(handle, device_index, digit, segments);
}

bool max7219_display_number(max7219_handle_t *handle, uint8_t device_index, int32_t number, bool leading_zeros)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    if (device_index >= handle->num_devices) {
        ESP_LOGE(TAG, "Device index %d out of range (max: %d)", device_index, handle->num_devices - 1);
        return false;
    }

    // Handle negative numbers
    bool negative = (number < 0);
    if (negative) {
        number = -number;
    }

    // Extract digits
    uint8_t digits[MAX7219_DIGITS_PER_DEVICE];
    int digit_count = 0;
    
    if (number == 0) {
        digits[0] = 0;
        digit_count = 1;
    } else {
        while (number > 0 && digit_count < MAX7219_DIGITS_PER_DEVICE) {
            digits[digit_count++] = number % 10;
            number /= 10;
        }
    }

    // Display digits (reverse order since we extracted them backwards)
    bool success = true;
    int display_pos = 0;

    // Display negative sign if needed
    if (negative && display_pos < MAX7219_DIGITS_PER_DEVICE) {
        // Use segment pattern for minus sign (segment G only)
        if (!max7219_set_segments(handle, device_index, display_pos++, 0x40)) {
            success = false;
        }
    }

    // Display digits
    for (int i = digit_count - 1; i >= 0 && display_pos < MAX7219_DIGITS_PER_DEVICE; i--) {
        if (!max7219_set_digit(handle, device_index, display_pos++, digits[i])) {
            success = false;
        }
    }

    // Clear remaining digits if not showing leading zeros
    if (!leading_zeros) {
        for (int i = display_pos; i < MAX7219_DIGITS_PER_DEVICE; i++) {
            max7219_set_digit(handle, device_index, i, 0);
        }
    }

    return success;
}

bool max7219_display_float(max7219_handle_t *handle, uint8_t device_index, float number, uint8_t decimal_places)
{
    if (handle == NULL || !handle->initialized) {
        ESP_LOGE(TAG, "Invalid handle or not initialized");
        return false;
    }

    if (device_index >= handle->num_devices) {
        ESP_LOGE(TAG, "Device index %d out of range (max: %d)", device_index, handle->num_devices - 1);
        return false;
    }

    if (decimal_places > MAX7219_DIGITS_PER_DEVICE - 1) {
        decimal_places = MAX7219_DIGITS_PER_DEVICE - 1;
    }

    // Scale number by 10^decimal_places
    float scaled = number * powf(10.0f, (float)decimal_places);
    int32_t integer_part = (int32_t)roundf(scaled);

    // Display as integer with decimal point
    bool success = max7219_display_number(handle, device_index, integer_part, false);

    // Add decimal point if needed
    if (decimal_places > 0 && success) {
        // Find the position of the decimal point
        int32_t abs_value = abs(integer_part);
        int digit_count = (abs_value == 0) ? 1 : (int)log10f((float)abs_value) + 1;
        int decimal_pos = digit_count - decimal_places;

        if (decimal_pos >= 0 && decimal_pos < MAX7219_DIGITS_PER_DEVICE) {
            // Read current digit value and set decimal point
            // For simplicity, we'll set the decimal point segment
            // Note: This assumes we're in raw segment mode or can modify segments
            // In decode mode, decimal point is handled differently
            // For now, this is a simplified implementation
        }
    }

    return success;
}

bool max7219_is_initialized(max7219_handle_t *handle)
{
    return (handle != NULL && handle->initialized);
}

uint8_t max7219_get_intensity(max7219_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    return handle->intensity;
}

uint8_t max7219_get_scan_limit(max7219_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    return handle->scan_limit;
}

bool max7219_get_shutdown(max7219_handle_t *handle)
{
    if (handle == NULL) {
        return true;
    }
    return handle->shutdown;
}
