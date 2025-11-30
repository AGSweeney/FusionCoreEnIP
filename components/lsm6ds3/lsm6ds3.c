/**
 * @file lsm6ds3.c
 * @brief LSM6DS3 6-axis accelerometer and gyroscope driver implementation
 * 
 * Implementation of the LSM6DS3 driver for ESP32-P4 platform.
 * 
 * This driver uses register definitions and low-level functions from the
 * STMicroelectronics STMems_Standard_C_drivers library:
 * https://github.com/STMicroelectronics/STMems_Standard_C_drivers
 * 
 * The register driver files (driver/lsm6ds3_reg.c and driver/lsm6ds3_reg.h)
 * are from STMicroelectronics and are licensed under BSD 3-Clause License.
 * The full BSD 3-Clause license text is included in those files.
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

#include "lsm6ds3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <math.h>

static const char *TAG = "LSM6DS3";

#define MAX_REG_TRANSFER_LEN 16  // Maximum register transfer length (register + data)
#define GRAVITY_MG 1000.0f       // Standard gravity in milli-g (1g = 1000 mg)

static int32_t platform_write_i2c(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    lsm6ds3_handle_t *dev = (lsm6ds3_handle_t *)handle;
    
    if (bufp == NULL && len > 0) {
        ESP_LOGE(TAG, "I2C write buffer pointer is NULL");
        return -1;
    }
    
    if (len > MAX_REG_TRANSFER_LEN - 1) {
        ESP_LOGE(TAG, "I2C write length %u exceeds maximum %d", len, MAX_REG_TRANSFER_LEN - 1);
        return -1;
    }
    
    uint8_t write_buf[MAX_REG_TRANSFER_LEN];
    write_buf[0] = reg;
    if (len > 0) {
        memcpy(&write_buf[1], bufp, len);
    }
    
    esp_err_t ret = i2c_master_transmit(dev->bus_handle.i2c_dev, write_buf, len + 1, pdMS_TO_TICKS(1000));
    
    return (ret == ESP_OK) ? 0 : -1;
}

static int32_t platform_read_i2c(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    if (handle == NULL || bufp == NULL) {
        return -1;
    }
    
    lsm6ds3_handle_t *dev = (lsm6ds3_handle_t *)handle;
    
    if (dev->bus_handle.i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return -1;
    }
    
    // Try a different approach: read registers one at a time for multi-byte reads
    // This matches how MPU6050 works and avoids auto-increment bit issues
    if (len > 1) {
        // Read each register individually
        for (uint16_t i = 0; i < len; i++) {
            uint8_t reg_addr = reg + i;
            esp_err_t ret = i2c_master_transmit_receive(dev->bus_handle.i2c_dev, &reg_addr, 1, &bufp[i], 1, pdMS_TO_TICKS(1000));
            if (ret != ESP_OK) {
                static uint32_t error_count = 0;
                error_count++;
                if (error_count <= 10 || error_count % 100 == 0) {
                    ESP_LOGE(TAG, "I2C read failed: reg=0x%02X, byte=%u, err=%s (#%lu)",
                             reg_addr, i, esp_err_to_name(ret), error_count);
                }
                return -1;
            }
        }
    } else {
        // Single byte read - no auto-increment needed
        esp_err_t ret = i2c_master_transmit_receive(dev->bus_handle.i2c_dev, &reg, 1, bufp, len, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK) {
            static uint32_t error_count = 0;
            error_count++;
            if (error_count <= 10 || error_count % 100 == 0) {
                ESP_LOGE(TAG, "I2C read failed: reg=0x%02X, len=%u, err=%s (#%lu)",
                         reg, len, esp_err_to_name(ret), error_count);
            }
            return -1;
        }
    }
    
    return 0;
}

static int32_t platform_write_spi(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    lsm6ds3_handle_t *dev = (lsm6ds3_handle_t *)handle;
    
    if (bufp == NULL && len > 0) {
        ESP_LOGE(TAG, "SPI write buffer pointer is NULL");
        return -1;
    }
    
    if (len > MAX_REG_TRANSFER_LEN - 1) {
        ESP_LOGE(TAG, "SPI write length %u exceeds maximum %d", len, MAX_REG_TRANSFER_LEN - 1);
        return -1;
    }
    
    uint8_t write_buf[MAX_REG_TRANSFER_LEN];
    write_buf[0] = reg | 0x00;
    if (len > 0) {
        memcpy(&write_buf[1], bufp, len);
    }
    
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = write_buf,
    };
    
    esp_err_t ret = spi_device_transmit(dev->bus_handle.spi_device, &t);
    
    return (ret == ESP_OK) ? 0 : -1;
}

static int32_t platform_read_spi(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    lsm6ds3_handle_t *dev = (lsm6ds3_handle_t *)handle;
    
    if (bufp == NULL) {
        ESP_LOGE(TAG, "SPI read buffer pointer is NULL");
        return -1;
    }
    
    if (len > MAX_REG_TRANSFER_LEN - 1) {
        ESP_LOGE(TAG, "SPI read length %u exceeds maximum %d", len, MAX_REG_TRANSFER_LEN - 1);
        return -1;
    }
    
    uint8_t write_buf = reg | 0x80;
    uint8_t read_buf[MAX_REG_TRANSFER_LEN];
    
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = &write_buf,
        .rx_buffer = read_buf,
    };
    
    esp_err_t ret = spi_device_transmit(dev->bus_handle.spi_device, &t);
    if (ret != ESP_OK) {
        return -1;
    }
    
    memcpy(bufp, &read_buf[1], len);
    
    return 0;
}

static void platform_delay(void *handle, uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

esp_err_t lsm6ds3_init(lsm6ds3_handle_t *handle, const lsm6ds3_config_t *config)
{
    if (handle == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(handle, 0, sizeof(lsm6ds3_handle_t));
    handle->interface = config->interface;
    handle->calibration.accel_calibrated = false;
    handle->calibration.gyro_calibrated = false;
    
    if (config->interface == LSM6DS3_INTERFACE_I2C) {
        // Create I2C device handle from bus handle and address
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = config->bus.i2c.address,
            .scl_speed_hz = 400000,
        };
        
        esp_err_t err = i2c_master_bus_add_device(config->bus.i2c.bus_handle, &dev_cfg, &handle->bus_handle.i2c_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
            return err;
        }
        
        handle->ctx.write_reg = platform_write_i2c;
        handle->ctx.read_reg = platform_read_i2c;
    } else {
        handle->bus_handle.spi_device = config->bus.spi.device_handle;
        handle->ctx.write_reg = platform_write_spi;
        handle->ctx.read_reg = platform_read_spi;
    }
    
    handle->ctx.mdelay = platform_delay;
    handle->ctx.handle = handle;
    
    uint8_t whoamI;
    if (lsm6ds3_device_id_get(&handle->ctx, &whoamI) != 0) {
        ESP_LOGE(TAG, "Failed to read device ID");
        if (config->interface == LSM6DS3_INTERFACE_I2C) {
            i2c_master_bus_rm_device(handle->bus_handle.i2c_dev);
        }
        return ESP_ERR_NOT_FOUND;
    }
    
    if (whoamI != LSM6DS3_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X (expected 0x%02X)", whoamI, LSM6DS3_ID);
        if (config->interface == LSM6DS3_INTERFACE_I2C) {
            i2c_master_bus_rm_device(handle->bus_handle.i2c_dev);
        }
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "LSM6DS3 initialized successfully (ID: 0x%02X)", whoamI);
    
    return ESP_OK;
}

esp_err_t lsm6ds3_deinit(lsm6ds3_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Remove I2C device handle if it was created
    if (handle->interface == LSM6DS3_INTERFACE_I2C && handle->bus_handle.i2c_dev != NULL) {
        i2c_master_bus_rm_device(handle->bus_handle.i2c_dev);
    }
    
    memset(handle, 0, sizeof(lsm6ds3_handle_t));
    return ESP_OK;
}

esp_err_t lsm6ds3_reset(lsm6ds3_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (lsm6ds3_reset_set(&handle->ctx, PROPERTY_ENABLE) != 0) {
        return ESP_FAIL;
    }
    
    uint8_t rst;
    uint32_t timeout_count = 0;
    const uint32_t MAX_RESET_WAIT = 1000;
    
    do {
        if (lsm6ds3_reset_get(&handle->ctx, &rst) != 0) {
            return ESP_FAIL;
        }
        timeout_count++;
        if (timeout_count > MAX_RESET_WAIT) {
            ESP_LOGE(TAG, "Reset timeout - sensor may be unresponsive");
            return ESP_FAIL;
        }
        handle->ctx.mdelay(handle->ctx.handle, 1);
    } while (rst);
    
    return ESP_OK;
}

esp_err_t lsm6ds3_get_device_id(lsm6ds3_handle_t *handle, uint8_t *device_id)
{
    if (handle == NULL || device_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (lsm6ds3_device_id_get(&handle->ctx, device_id) != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_accel_odr(lsm6ds3_handle_t *handle, lsm6ds3_odr_xl_t odr)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (lsm6ds3_xl_data_rate_set(&handle->ctx, odr) != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_accel_full_scale(lsm6ds3_handle_t *handle, lsm6ds3_fs_xl_t fs)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (lsm6ds3_xl_full_scale_set(&handle->ctx, fs) != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_gyro_odr(lsm6ds3_handle_t *handle, lsm6ds3_odr_g_t odr)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (lsm6ds3_gy_data_rate_set(&handle->ctx, odr) != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_gyro_full_scale(lsm6ds3_handle_t *handle, lsm6ds3_fs_g_t fs)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (lsm6ds3_gy_full_scale_set(&handle->ctx, fs) != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_read_accel(lsm6ds3_handle_t *handle, float accel_mg[3])
{
    if (handle == NULL || accel_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if I2C device handle is still valid
    if (handle->bus_handle.i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device handle is NULL in read_accel");
        return ESP_ERR_INVALID_STATE;
    }
    
    lsm6ds3_reg_t data_raw_acceleration;
    memset(data_raw_acceleration.u8bit, 0xFF, 6);  // Initialize to 0xFF to detect if read fails
    memset(data_raw_acceleration.i16bit, 0, sizeof(data_raw_acceleration.i16bit));  // Also clear int16 array
    
    int32_t ret = lsm6ds3_acceleration_raw_get(&handle->ctx, data_raw_acceleration.u8bit);
    if (ret != 0) {
        static uint32_t read_error_count = 0;
        // Log first error and then every 100th error (reduced from 50)
        if (++read_error_count == 1 || read_error_count % 100 == 0) {
            ESP_LOGE(TAG, "Failed to read acceleration raw data: %ld (error #%lu)", ret, read_error_count);
        }
        return ESP_FAIL;
    }
    
    // Manually reconstruct int16 values from bytes (little-endian: LSB first)
    // This is necessary because reading bytes one at a time doesn't populate the union's i16bit array correctly
    data_raw_acceleration.i16bit[0] = (int16_t)((int16_t)data_raw_acceleration.u8bit[0] | ((int16_t)data_raw_acceleration.u8bit[1] << 8));
    data_raw_acceleration.i16bit[1] = (int16_t)((int16_t)data_raw_acceleration.u8bit[2] | ((int16_t)data_raw_acceleration.u8bit[3] << 8));
    data_raw_acceleration.i16bit[2] = (int16_t)((int16_t)data_raw_acceleration.u8bit[4] | ((int16_t)data_raw_acceleration.u8bit[5] << 8));
    
    // First read logged silently (no console output)
    
    // Check if we got all zeros (which might indicate a problem)
    bool all_zeros = (data_raw_acceleration.u8bit[0] == 0 && data_raw_acceleration.u8bit[1] == 0 &&
                     data_raw_acceleration.u8bit[2] == 0 && data_raw_acceleration.u8bit[3] == 0 &&
                     data_raw_acceleration.u8bit[4] == 0 && data_raw_acceleration.u8bit[5] == 0);
    
    // Check if we got all zeros (which might indicate a problem)
    if (all_zeros) {
        static uint32_t zero_count = 0;
        // Log first occurrence and then every 1000th zero read (reduced from 100)
        if (++zero_count == 1 || zero_count % 1000 == 0) {
            ESP_LOGW(TAG, "Accelerometer read returned all zeros (read #%lu)", zero_count);
            
            // Only perform detailed diagnostics on first occurrence or every 10000th zero read
            if (zero_count == 1 || zero_count % 10000 == 0) {
                // Try reading WHO_AM_I to verify sensor is still responding
                uint8_t whoami = 0;
                if (lsm6ds3_device_id_get(&handle->ctx, &whoami) == 0) {
                    if (whoami == LSM6DS3_ID) {
                        ESP_LOGD(TAG, "Sensor WHO_AM_I OK: 0x%02X - sensor is responsive but returning zeros", whoami);
                    } else {
                        ESP_LOGE(TAG, "Sensor WHO_AM_I wrong: 0x%02X (expected 0x%02X)", whoami, LSM6DS3_ID);
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to read WHO_AM_I - I2C communication may be broken");
                }
                // Check STATUS_REG to see if data is available
                lsm6ds3_status_reg_t status;
                if (lsm6ds3_status_reg_get(&handle->ctx, &status) == 0) {
                    ESP_LOGD(TAG, "STATUS_REG: XLDA=%d GDA=%d TDA=%d (XLDA=1 means accel data available)", 
                             status.xlda, status.gda, status.tda);
                } else {
                    ESP_LOGD(TAG, "Failed to read STATUS_REG");
                }
                // Read back CTRL1_XL to verify sensor is enabled
                lsm6ds3_odr_xl_t odr_check;
                if (lsm6ds3_xl_data_rate_get(&handle->ctx, &odr_check) == 0) {
                    ESP_LOGD(TAG, "CTRL1_XL ODR readback: %d (0=OFF, 4=104Hz)", odr_check);
                } else {
                    ESP_LOGD(TAG, "Failed to read CTRL1_XL");
                }
            }
        }
    }
    
    lsm6ds3_fs_xl_t fs;
    if (lsm6ds3_xl_full_scale_get(&handle->ctx, &fs) != 0) {
        return ESP_FAIL;
    }
    
    switch (fs) {
        case LSM6DS3_2g:
            accel_mg[0] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        case LSM6DS3_4g:
            accel_mg[0] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        case LSM6DS3_8g:
            accel_mg[0] = lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        case LSM6DS3_16g:
            accel_mg[0] = lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        default:
            return ESP_FAIL;
    }
    
    if (handle->calibration.accel_calibrated) {
        accel_mg[0] -= handle->calibration.accel_offset_mg[0];
        accel_mg[1] -= handle->calibration.accel_offset_mg[1];
        accel_mg[2] -= handle->calibration.accel_offset_mg[2];
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_read_gyro(lsm6ds3_handle_t *handle, float gyro_mdps[3])
{
    if (handle == NULL || gyro_mdps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if I2C device handle is still valid
    if (handle->bus_handle.i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device handle is NULL in read_gyro");
        return ESP_ERR_INVALID_STATE;
    }
    
    lsm6ds3_reg_t data_raw_angular_rate;
    memset(data_raw_angular_rate.u8bit, 0xFF, 6);  // Initialize to 0xFF to detect if read fails
    memset(data_raw_angular_rate.i16bit, 0, sizeof(data_raw_angular_rate.i16bit));  // Also clear int16 array
    
    int32_t ret = lsm6ds3_angular_rate_raw_get(&handle->ctx, data_raw_angular_rate.u8bit);
    if (ret != 0) {
        static uint32_t read_error_count = 0;
        // Log first error and then every 100th error (reduced from 50)
        if (++read_error_count == 1 || read_error_count % 100 == 0) {
            ESP_LOGE(TAG, "Failed to read angular rate raw data: %ld (error #%lu)", ret, read_error_count);
        }
        return ESP_FAIL;
    }
    
    // Manually reconstruct i16 from bytes (little-endian)
    data_raw_angular_rate.i16bit[0] = (int16_t)((data_raw_angular_rate.u8bit[1] << 8) | data_raw_angular_rate.u8bit[0]);
    data_raw_angular_rate.i16bit[1] = (int16_t)((data_raw_angular_rate.u8bit[3] << 8) | data_raw_angular_rate.u8bit[2]);
    data_raw_angular_rate.i16bit[2] = (int16_t)((data_raw_angular_rate.u8bit[5] << 8) | data_raw_angular_rate.u8bit[4]);
    
    lsm6ds3_fs_g_t fs;
    if (lsm6ds3_gy_full_scale_get(&handle->ctx, &fs) != 0) {
        return ESP_FAIL;
    }
    
    switch (fs) {
        case LSM6DS3_125dps:
            gyro_mdps[0] = lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_250dps:
            gyro_mdps[0] = lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_500dps:
            gyro_mdps[0] = lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_1000dps:
            gyro_mdps[0] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_2000dps:
            gyro_mdps[0] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        default:
            return ESP_FAIL;
    }
    
    if (handle->calibration.gyro_calibrated) {
        gyro_mdps[0] -= handle->calibration.gyro_offset_mdps[0];
        gyro_mdps[1] -= handle->calibration.gyro_offset_mdps[1];
        gyro_mdps[2] -= handle->calibration.gyro_offset_mdps[2];
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_enable_block_data_update(lsm6ds3_handle_t *handle, bool enable)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (lsm6ds3_block_data_update_set(&handle->ctx, enable ? PROPERTY_ENABLE : PROPERTY_DISABLE) != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

static esp_err_t lsm6ds3_read_accel_raw(lsm6ds3_handle_t *handle, float accel_mg[3])
{
    if (handle == NULL || accel_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    lsm6ds3_reg_t data_raw_acceleration;
    if (lsm6ds3_acceleration_raw_get(&handle->ctx, data_raw_acceleration.u8bit) != 0) {
        return ESP_FAIL;
    }
    
    lsm6ds3_fs_xl_t fs;
    if (lsm6ds3_xl_full_scale_get(&handle->ctx, &fs) != 0) {
        return ESP_FAIL;
    }
    
    switch (fs) {
        case LSM6DS3_2g:
            accel_mg[0] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        case LSM6DS3_4g:
            accel_mg[0] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        case LSM6DS3_8g:
            accel_mg[0] = lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        case LSM6DS3_16g:
            accel_mg[0] = lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[0]);
            accel_mg[1] = lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[1]);
            accel_mg[2] = lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[2]);
            break;
        default:
            return ESP_FAIL;
    }
    
    return ESP_OK;
}

static esp_err_t lsm6ds3_read_gyro_raw(lsm6ds3_handle_t *handle, float gyro_mdps[3])
{
    if (handle == NULL || gyro_mdps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    lsm6ds3_reg_t data_raw_angular_rate;
    memset(data_raw_angular_rate.u8bit, 0xFF, 6);  // Initialize to 0xFF to detect if read fails
    memset(data_raw_angular_rate.i16bit, 0, sizeof(data_raw_angular_rate.i16bit));  // Also clear int16 array
    
    if (lsm6ds3_angular_rate_raw_get(&handle->ctx, data_raw_angular_rate.u8bit) != 0) {
        return ESP_FAIL;
    }
    
    // Manually reconstruct i16 from bytes (little-endian)
    data_raw_angular_rate.i16bit[0] = (int16_t)((data_raw_angular_rate.u8bit[1] << 8) | data_raw_angular_rate.u8bit[0]);
    data_raw_angular_rate.i16bit[1] = (int16_t)((data_raw_angular_rate.u8bit[3] << 8) | data_raw_angular_rate.u8bit[2]);
    data_raw_angular_rate.i16bit[2] = (int16_t)((data_raw_angular_rate.u8bit[5] << 8) | data_raw_angular_rate.u8bit[4]);
    
    lsm6ds3_fs_g_t fs;
    if (lsm6ds3_gy_full_scale_get(&handle->ctx, &fs) != 0) {
        return ESP_FAIL;
    }
    
    switch (fs) {
        case LSM6DS3_125dps:
            gyro_mdps[0] = lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_250dps:
            gyro_mdps[0] = lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_500dps:
            gyro_mdps[0] = lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_1000dps:
            gyro_mdps[0] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        case LSM6DS3_2000dps:
            gyro_mdps[0] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
            gyro_mdps[1] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
            gyro_mdps[2] = lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);
            break;
        default:
            return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_calibrate_accel(lsm6ds3_handle_t *handle, uint32_t samples, uint32_t sample_delay_ms)
{
    if (handle == NULL || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting accelerometer calibration (%lu samples)...", samples);
    
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float reading[3];
    
    for (uint32_t i = 0; i < samples; i++) {
        esp_err_t ret = lsm6ds3_read_accel_raw(handle, reading);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accelerometer during calibration");
            return ret;
        }
        
        sum[0] += reading[0];
        sum[1] += reading[1];
        sum[2] += reading[2];
        
        handle->ctx.mdelay(handle->ctx.handle, sample_delay_ms);
    }
    
    handle->calibration.accel_offset_mg[0] = sum[0] / (float)samples;
    handle->calibration.accel_offset_mg[1] = sum[1] / (float)samples;
    handle->calibration.accel_offset_mg[2] = (sum[2] / (float)samples) - GRAVITY_MG;
    
    handle->calibration.accel_calibrated = true;
    
    ESP_LOGI(TAG, "Accelerometer calibration complete: X=%.2f, Y=%.2f, Z=%.2f mg",
             handle->calibration.accel_offset_mg[0],
             handle->calibration.accel_offset_mg[1],
             handle->calibration.accel_offset_mg[2]);
    
    return ESP_OK;
}

esp_err_t lsm6ds3_calibrate_gyro(lsm6ds3_handle_t *handle, uint32_t samples, uint32_t sample_delay_ms)
{
    if (handle == NULL || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Starting gyroscope calibration (%lu samples)...", samples);
    
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float reading[3];
    
    for (uint32_t i = 0; i < samples; i++) {
        esp_err_t ret = lsm6ds3_read_gyro_raw(handle, reading);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope during calibration");
            return ret;
        }
        
        sum[0] += reading[0];
        sum[1] += reading[1];
        sum[2] += reading[2];
        
        handle->ctx.mdelay(handle->ctx.handle, sample_delay_ms);
    }
    
    handle->calibration.gyro_offset_mdps[0] = sum[0] / (float)samples;
    handle->calibration.gyro_offset_mdps[1] = sum[1] / (float)samples;
    handle->calibration.gyro_offset_mdps[2] = sum[2] / (float)samples;
    
    handle->calibration.gyro_calibrated = true;
    
    ESP_LOGI(TAG, "Gyroscope calibration complete: X=%.2f, Y=%.2f, Z=%.2f mdps",
             handle->calibration.gyro_offset_mdps[0],
             handle->calibration.gyro_offset_mdps[1],
             handle->calibration.gyro_offset_mdps[2]);
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_accel_offset(lsm6ds3_handle_t *handle, const float offset_mg[3])
{
    if (handle == NULL || offset_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(handle->calibration.accel_offset_mg, offset_mg, sizeof(float) * 3);
    handle->calibration.accel_calibrated = true;
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_accel_offset_axis(lsm6ds3_handle_t *handle, uint8_t axis, float offset_mg)
{
    if (handle == NULL || axis > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    handle->calibration.accel_offset_mg[axis] = offset_mg;
    handle->calibration.accel_calibrated = true;
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_gyro_offset(lsm6ds3_handle_t *handle, const float offset_mdps[3])
{
    if (handle == NULL || offset_mdps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(handle->calibration.gyro_offset_mdps, offset_mdps, sizeof(float) * 3);
    handle->calibration.gyro_calibrated = true;
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_gyro_offset_axis(lsm6ds3_handle_t *handle, uint8_t axis, float offset_mdps)
{
    if (handle == NULL || axis > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    handle->calibration.gyro_offset_mdps[axis] = offset_mdps;
    handle->calibration.gyro_calibrated = true;
    
    return ESP_OK;
}

esp_err_t lsm6ds3_get_accel_offset(lsm6ds3_handle_t *handle, float offset_mg[3])
{
    if (handle == NULL || offset_mg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(offset_mg, handle->calibration.accel_offset_mg, sizeof(float) * 3);
    
    return ESP_OK;
}

esp_err_t lsm6ds3_get_gyro_offset(lsm6ds3_handle_t *handle, float offset_mdps[3])
{
    if (handle == NULL || offset_mdps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(offset_mdps, handle->calibration.gyro_offset_mdps, sizeof(float) * 3);
    
    return ESP_OK;
}

esp_err_t lsm6ds3_clear_accel_calibration(lsm6ds3_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(handle->calibration.accel_offset_mg, 0, sizeof(float) * 3);
    handle->calibration.accel_calibrated = false;
    
    return ESP_OK;
}

esp_err_t lsm6ds3_clear_gyro_calibration(lsm6ds3_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(handle->calibration.gyro_offset_mdps, 0, sizeof(float) * 3);
    handle->calibration.gyro_calibrated = false;
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_accel_hw_offset(lsm6ds3_handle_t *handle, int8_t offset_x, int8_t offset_y, int8_t offset_z)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Note: Register functions expect uint8_t, but values are interpreted as signed (two's complement)
    // Casting int8_t to uint8_t preserves the bit pattern for two's complement representation
    if (lsm6ds3_xl_usr_offset_x_set(&handle->ctx, (uint8_t)offset_x) != 0) {
        return ESP_FAIL;
    }
    if (lsm6ds3_xl_usr_offset_y_set(&handle->ctx, (uint8_t)offset_y) != 0) {
        return ESP_FAIL;
    }
    if (lsm6ds3_xl_usr_offset_z_set(&handle->ctx, (uint8_t)offset_z) != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_accel_hw_offset_axis(lsm6ds3_handle_t *handle, uint8_t axis, int8_t offset)
{
    if (handle == NULL || axis > 2) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Note: Register functions expect uint8_t, but values are interpreted as signed (two's complement)
    // Casting int8_t to uint8_t preserves the bit pattern for two's complement representation
    switch (axis) {
        case 0:
            if (lsm6ds3_xl_usr_offset_x_set(&handle->ctx, (uint8_t)offset) != 0) {
                return ESP_FAIL;
            }
            break;
        case 1:
            if (lsm6ds3_xl_usr_offset_y_set(&handle->ctx, (uint8_t)offset) != 0) {
                return ESP_FAIL;
            }
            break;
        case 2:
            if (lsm6ds3_xl_usr_offset_z_set(&handle->ctx, (uint8_t)offset) != 0) {
                return ESP_FAIL;
            }
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_set_gyro_hw_offset(lsm6ds3_handle_t *handle, int8_t offset_x, int8_t offset_y, int8_t offset_z)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGW(TAG, "LSM6DS3 does not support hardware gyroscope offset registers. Use software calibration instead.");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t lsm6ds3_save_calibration_to_nvs(lsm6ds3_handle_t *handle, const char *namespace)
{
    if (handle == NULL || namespace == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(namespace, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_blob(nvs_handle, "accel_offset", handle->calibration.accel_offset_mg, sizeof(float) * 3);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_set_blob(nvs_handle, "gyro_offset", handle->calibration.gyro_offset_mdps, sizeof(float) * 3);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_set_u8(nvs_handle, "accel_cal", handle->calibration.accel_calibrated ? 1 : 0);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_set_u8(nvs_handle, "gyro_cal", handle->calibration.gyro_calibrated ? 1 : 0);
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }
    
    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration data saved to NVS");
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
    }
    
    return err;
}

esp_err_t lsm6ds3_load_calibration_from_nvs(lsm6ds3_handle_t *handle, const char *namespace)
{
    if (handle == NULL || namespace == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(namespace, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS namespace (calibration not found): %s", esp_err_to_name(err));
        return err;
    }
    
    size_t required_size = sizeof(float) * 3;
    size_t size = required_size;
    
    err = nvs_get_blob(nvs_handle, "accel_offset", handle->calibration.accel_offset_mg, &size);
    if (err != ESP_OK || size != required_size) {
        nvs_close(nvs_handle);
        ESP_LOGW(TAG, "Failed to load accelerometer calibration from NVS");
        return err;
    }
    
    size = required_size;
    err = nvs_get_blob(nvs_handle, "gyro_offset", handle->calibration.gyro_offset_mdps, &size);
    if (err != ESP_OK || size != required_size) {
        nvs_close(nvs_handle);
        ESP_LOGW(TAG, "Failed to load gyroscope calibration from NVS");
        return err;
    }
    
    uint8_t accel_calibrated = 0;
    uint8_t gyro_calibrated = 0;
    
    esp_err_t err_accel = nvs_get_u8(nvs_handle, "accel_cal", &accel_calibrated);
    if (err_accel == ESP_OK) {
        handle->calibration.accel_calibrated = (accel_calibrated != 0);
    }
    
    esp_err_t err_gyro = nvs_get_u8(nvs_handle, "gyro_cal", &gyro_calibrated);
    if (err_gyro == ESP_OK) {
        handle->calibration.gyro_calibrated = (gyro_calibrated != 0);
    }
    
    nvs_close(nvs_handle);
    
    // Calibration loaded silently (no console logging to reduce boot spam)
    
    return ESP_OK;
}

