#include "lsm6ds3_manager.h"
#include "i2c_bus_manager.h"
#include "system_config.h"
#include "fusion_core_assembly.h"
#include "lsm6ds3.h"
#include "lsm6ds3_fusion.h"
#include "ota_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "lsm6ds3_manager";

#define LSM6DS3_I2C_ADDRESS_1 0x6A
#define LSM6DS3_I2C_ADDRESS_2 0x6B
#define LSM6DS3_UPDATE_INTERVAL_MS 20

static bool s_initialized = false;
static lsm6ds3_handle_t s_lsm6ds3_device;
static lsm6ds3_complementary_filter_t s_complementary_filter;
/* Madgwick filter not currently used - kept for future implementation */
/* static lsm6ds3_madgwick_filter_t s_madgwick_filter; */
static lsm6ds3_angle_zero_t s_angle_zero;
static TaskHandle_t s_task_handle = NULL;
static uint8_t s_i2c_address = 0;
static SemaphoreHandle_t s_calibration_mutex = NULL;

static void lsm6ds3_update_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t update_interval = pdMS_TO_TICKS(LSM6DS3_UPDATE_INTERVAL_MS);

    ESP_LOGI(TAG, "LSM6DS3 update task started");

    while (1) {
        // Check if OTA is in progress - skip I2C operations to avoid errors
        ota_status_info_t ota_status;
        if (ota_manager_get_status(&ota_status) && 
            ota_status.status == OTA_STATUS_IN_PROGRESS) {
            // OTA in progress - skip I2C operations to avoid errors
            vTaskDelay(update_interval);
            continue;
        }
        
        if (s_initialized) {
            float accel_mg[3] = {0};
            float gyro_mdps[3] = {0};
            
            if (lsm6ds3_read_accel(&s_lsm6ds3_device, accel_mg) == ESP_OK &&
                lsm6ds3_read_gyro(&s_lsm6ds3_device, gyro_mdps) == ESP_OK) {
                
                float gyro_dps[3];
                gyro_dps[0] = gyro_mdps[0] / 1000.0f;
                gyro_dps[1] = gyro_mdps[1] / 1000.0f;
                gyro_dps[2] = gyro_mdps[2] / 1000.0f;
                
                float accel_g[3];
                accel_g[0] = accel_mg[0] / 1000.0f;
                accel_g[1] = accel_mg[1] / 1000.0f;
                accel_g[2] = accel_mg[2] / 1000.0f;
                
                // Calculate time delta from period_ms (typically 20ms for 50Hz update rate)
                float dt = (float)LSM6DS3_UPDATE_INTERVAL_MS / 1000.0f;
                
                // Update filter with new sensor data
                esp_err_t fusion_err = lsm6ds3_complementary_update(&s_complementary_filter, accel_g, gyro_dps, dt);
                
                if (fusion_err == ESP_OK) {
                    float roll = 0.0f, pitch = 0.0f, ground_angle = 0.0f;
                    
                    // Get fused roll and pitch angles
                    lsm6ds3_complementary_get_angles(&s_complementary_filter, &roll, &pitch);
                    
                    // Apply angle zero offsets to roll and pitch (if set)
                    lsm6ds3_apply_angle_zero(&s_angle_zero, &roll, &pitch, NULL);
                    
                    // Calculate ground angle from vertical using fused roll and pitch (after zero offsets)
                    ground_angle = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
                    
                    // Apply sign convention: negative for angles > 90°
                    if (ground_angle > 90.0f) {
                        ground_angle = -ground_angle;
                    }
                    
                    // Clamp angles to ±180 degrees (typical IMU range for roll/pitch/ground angle)
                    const float max_angle = 180.0f;
                    const float min_angle = -180.0f;
                    
                    if (roll > max_angle) roll = max_angle;
                    if (roll < min_angle) roll = min_angle;
                    if (pitch > max_angle) pitch = max_angle;
                    if (pitch < min_angle) pitch = min_angle;
                    if (ground_angle > max_angle) ground_angle = max_angle;
                    if (ground_angle < min_angle) ground_angle = min_angle;
                    
                    // Scale angles by 100 for fixed-point storage (like NAU7802 weight)
                    // int16_t range: -32,768 to 32,767, so ±180° (18000 scaled) fits comfortably
                    int16_t roll_int = (int16_t)(roll * 100.0f + (roll >= 0.0f ? 0.5f : -0.5f));
                    int16_t pitch_int = (int16_t)(pitch * 100.0f + (pitch >= 0.0f ? 0.5f : -0.5f));
                    int16_t ground_angle_int = (int16_t)(ground_angle * 100.0f + (ground_angle >= 0.0f ? 0.5f : -0.5f));
                    
                    SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
                    if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        memcpy(&INPUT_ASSEMBLY_100[LSM6DS3_BYTE_START], &roll_int, sizeof(int16_t));
                        memcpy(&INPUT_ASSEMBLY_100[LSM6DS3_BYTE_START + 2], &pitch_int, sizeof(int16_t));
                        memcpy(&INPUT_ASSEMBLY_100[LSM6DS3_BYTE_START + 4], &ground_angle_int, sizeof(int16_t));
                        xSemaphoreGive(assembly_mutex);
                    }
                }
            }
        }
        vTaskDelay(update_interval);
    }
}

esp_err_t lsm6ds3_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "LSM6DS3 manager already initialized");
        return ESP_OK;
    }

    if (!system_lsm6ds3_enabled_load()) {
        ESP_LOGI(TAG, "LSM6DS3 is disabled in configuration");
        return ESP_OK;
    }

    uint8_t address = 0;
    i2c_master_bus_handle_t bus_handle = NULL;
    
    i2c_master_bus_handle_t primary_bus = i2c_bus_manager_get_primary_bus();
    i2c_master_bus_handle_t secondary_bus = i2c_bus_manager_get_secondary_bus();
    
    if (primary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(primary_bus, LSM6DS3_I2C_ADDRESS_1, 100);
        if (ret == ESP_OK) {
            address = LSM6DS3_I2C_ADDRESS_1;
            bus_handle = primary_bus;
        } else {
            ret = i2c_master_probe(primary_bus, LSM6DS3_I2C_ADDRESS_2, 100);
            if (ret == ESP_OK) {
                address = LSM6DS3_I2C_ADDRESS_2;
                bus_handle = primary_bus;
            }
        }
    }
    
    if (address == 0 && secondary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(secondary_bus, LSM6DS3_I2C_ADDRESS_1, 100);
        if (ret == ESP_OK) {
            address = LSM6DS3_I2C_ADDRESS_1;
            bus_handle = secondary_bus;
        } else {
            ret = i2c_master_probe(secondary_bus, LSM6DS3_I2C_ADDRESS_2, 100);
            if (ret == ESP_OK) {
                address = LSM6DS3_I2C_ADDRESS_2;
                bus_handle = secondary_bus;
            }
        }
    }
    
    if (address == 0 || bus_handle == NULL) {
        ESP_LOGI(TAG, "LSM6DS3 not detected at addresses 0x%02X or 0x%02X on either bus", 
                 LSM6DS3_I2C_ADDRESS_1, LSM6DS3_I2C_ADDRESS_2);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "LSM6DS3 detected at address 0x%02X on %s bus", 
             address, (bus_handle == primary_bus) ? "primary" : "secondary");

    s_i2c_address = address;

    memset(&s_lsm6ds3_device, 0, sizeof(s_lsm6ds3_device));
    s_lsm6ds3_device.interface = LSM6DS3_INTERFACE_I2C;
    
    lsm6ds3_config_t lsm6ds3_config = {
        .interface = LSM6DS3_INTERFACE_I2C,
        .bus.i2c.bus_handle = bus_handle,
        .bus.i2c.address = address,
    };
    
    esp_err_t err = lsm6ds3_init(&s_lsm6ds3_device, &lsm6ds3_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LSM6DS3: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t device_id = 0;
    err = lsm6ds3_get_device_id(&s_lsm6ds3_device, &device_id);
    if (err != ESP_OK || device_id != 0x69) {
        ESP_LOGE(TAG, "LSM6DS3 device ID check failed (got 0x%02X, expected 0x69)", device_id);
        return ESP_ERR_NOT_FOUND;
    }

    err = lsm6ds3_set_accel_odr(&s_lsm6ds3_device, LSM6DS3_XL_ODR_104Hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer ODR: %s", esp_err_to_name(err));
        return err;
    }

    err = lsm6ds3_set_accel_full_scale(&s_lsm6ds3_device, LSM6DS3_4g);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer full-scale: %s", esp_err_to_name(err));
        return err;
    }

    err = lsm6ds3_set_gyro_odr(&s_lsm6ds3_device, LSM6DS3_GY_ODR_104Hz);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope ODR: %s", esp_err_to_name(err));
        return err;
    }

    err = lsm6ds3_set_gyro_full_scale(&s_lsm6ds3_device, LSM6DS3_500dps);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyroscope full-scale: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    lsm6ds3_complementary_init(&s_complementary_filter, 0.96f, 104.0f);
    memset(&s_angle_zero, 0, sizeof(s_angle_zero));
    
    esp_err_t angle_zero_err = lsm6ds3_load_angle_zero_from_nvs(&s_angle_zero, "lsm6ds3");
    if (angle_zero_err == ESP_OK) {
        if (s_angle_zero.roll_zero_set || s_angle_zero.pitch_zero_set || s_angle_zero.yaw_zero_set) {
            ESP_LOGI(TAG, "Loaded angle zero offsets from NVS: roll=%.2f%s, pitch=%.2f%s, yaw=%.2f%s",
                     s_angle_zero.roll_zero, s_angle_zero.roll_zero_set ? "" : " (not set)",
                     s_angle_zero.pitch_zero, s_angle_zero.pitch_zero_set ? "" : " (not set)",
                     s_angle_zero.yaw_zero, s_angle_zero.yaw_zero_set ? "" : " (not set)");
        }
    }

    s_calibration_mutex = xSemaphoreCreateMutex();
    if (s_calibration_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create calibration mutex");
        return ESP_ERR_NO_MEM;
    }

    float accel_offset_mg[3] = {0};
    float gyro_offset_mdps[3] = {0};
    bool accel_calibrated = false;
    bool gyro_calibrated = false;
    
    esp_err_t nvs_err = lsm6ds3_manager_get_nvs_calibration(accel_offset_mg, gyro_offset_mdps, &accel_calibrated, &gyro_calibrated);
    if (nvs_err == ESP_OK && gyro_calibrated) {
        s_lsm6ds3_device.calibration.gyro_calibrated = true;
        s_lsm6ds3_device.calibration.gyro_offset_mdps[0] = gyro_offset_mdps[0];
        s_lsm6ds3_device.calibration.gyro_offset_mdps[1] = gyro_offset_mdps[1];
        s_lsm6ds3_device.calibration.gyro_offset_mdps[2] = gyro_offset_mdps[2];
        ESP_LOGI(TAG, "Loaded gyroscope calibration from NVS: X=%.2f, Y=%.2f, Z=%.2f mdps",
                 gyro_offset_mdps[0], gyro_offset_mdps[1], gyro_offset_mdps[2]);
    } else {
        s_lsm6ds3_device.calibration.gyro_calibrated = false;
        s_lsm6ds3_device.calibration.gyro_offset_mdps[0] = 0.0f;
        s_lsm6ds3_device.calibration.gyro_offset_mdps[1] = 0.0f;
        s_lsm6ds3_device.calibration.gyro_offset_mdps[2] = 0.0f;
        ESP_LOGI(TAG, "No gyroscope calibration found in NVS, using zero offsets");
    }
    
    s_lsm6ds3_device.calibration.accel_calibrated = false;

    s_initialized = true;
    ESP_LOGI(TAG, "LSM6DS3 manager initialized successfully at address 0x%02X", address);

    xTaskCreate(lsm6ds3_update_task, "lsm6ds3_task", 4096, NULL, 5, &s_task_handle);
    if (s_task_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create LSM6DS3 update task");
    }

    return ESP_OK;
}

bool lsm6ds3_manager_is_initialized(void)
{
    return s_initialized;
}

esp_err_t lsm6ds3_manager_calibrate_gyro(uint32_t samples, uint32_t sample_delay_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_calibration_mutex != NULL) {
        if (xSemaphoreTake(s_calibration_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }
    
    esp_err_t err = lsm6ds3_calibrate_gyro(&s_lsm6ds3_device, samples, sample_delay_ms);
    
    if (err == ESP_OK) {
        lsm6ds3_save_calibration_to_nvs(&s_lsm6ds3_device, "lsm6ds3");
        lsm6ds3_complementary_init(&s_complementary_filter, 0.96f, 104.0f);
    }
    
    if (s_calibration_mutex != NULL) {
        xSemaphoreGive(s_calibration_mutex);
    }
    
    return err;
}

esp_err_t lsm6ds3_manager_calibrate_gyro_preview(uint32_t samples, uint32_t sample_delay_ms, float offset_mdps[3])
{
    if (!s_initialized || offset_mdps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_calibration_mutex != NULL) {
        if (xSemaphoreTake(s_calibration_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }
    
    float saved_offsets[3];
    bool saved_calibrated = s_lsm6ds3_device.calibration.gyro_calibrated;
    memcpy(saved_offsets, s_lsm6ds3_device.calibration.gyro_offset_mdps, sizeof(float) * 3);
    
    esp_err_t err = lsm6ds3_calibrate_gyro(&s_lsm6ds3_device, samples, sample_delay_ms);
    
    if (err == ESP_OK) {
        memcpy(offset_mdps, s_lsm6ds3_device.calibration.gyro_offset_mdps, sizeof(float) * 3);
        memcpy(s_lsm6ds3_device.calibration.gyro_offset_mdps, saved_offsets, sizeof(float) * 3);
        s_lsm6ds3_device.calibration.gyro_calibrated = saved_calibrated;
    }
    
    if (s_calibration_mutex != NULL) {
        xSemaphoreGive(s_calibration_mutex);
    }
    
    return err;
}

esp_err_t lsm6ds3_manager_get_calibration_status(bool *accel_calibrated, bool *gyro_calibrated)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (accel_calibrated != NULL) {
        *accel_calibrated = false;
    }
    
    if (gyro_calibrated != NULL) {
        *gyro_calibrated = s_lsm6ds3_device.calibration.gyro_calibrated;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_manager_get_gyro_offsets(float offset_mdps[3])
{
    if (!s_initialized || offset_mdps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return lsm6ds3_get_gyro_offset(&s_lsm6ds3_device, offset_mdps);
}

esp_err_t lsm6ds3_manager_set_angle_zero(float roll, float pitch, float yaw)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err = lsm6ds3_set_angle_zero(&s_angle_zero, roll, pitch, yaw);
    
    if (err == ESP_OK) {
        esp_err_t nvs_err = lsm6ds3_save_angle_zero_to_nvs(&s_angle_zero, "lsm6ds3");
        if (nvs_err == ESP_OK) {
            ESP_LOGI(TAG, "Angle zero offsets saved to NVS: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
        } else {
            ESP_LOGE(TAG, "Failed to save angle zero offsets to NVS: %s", esp_err_to_name(nvs_err));
            return nvs_err;
        }
    }
    
    return err;
}

esp_err_t lsm6ds3_manager_clear_angle_zero(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err = lsm6ds3_clear_angle_zero(&s_angle_zero);
    
    if (err == ESP_OK) {
        esp_err_t nvs_err = lsm6ds3_save_angle_zero_to_nvs(&s_angle_zero, "lsm6ds3");
        if (nvs_err == ESP_OK) {
            ESP_LOGI(TAG, "Angle zero offsets cleared and saved to NVS");
        } else {
            ESP_LOGE(TAG, "Failed to save cleared angle zero offsets to NVS: %s", esp_err_to_name(nvs_err));
            return nvs_err;
        }
    }
    
    return err;
}

esp_err_t lsm6ds3_manager_get_angle_zero(float *roll, float *pitch, float *yaw, bool *roll_set, bool *pitch_set, bool *yaw_set)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (roll != NULL) {
        *roll = s_angle_zero.roll_zero;
    }
    if (pitch != NULL) {
        *pitch = s_angle_zero.pitch_zero;
    }
    if (yaw != NULL) {
        *yaw = s_angle_zero.yaw_zero;
    }
    if (roll_set != NULL) {
        *roll_set = s_angle_zero.roll_zero_set;
    }
    if (pitch_set != NULL) {
        *pitch_set = s_angle_zero.pitch_zero_set;
    }
    if (yaw_set != NULL) {
        *yaw_set = s_angle_zero.yaw_zero_set;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_manager_save_calibration(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t err1 = lsm6ds3_save_calibration_to_nvs(&s_lsm6ds3_device, "lsm6ds3");
    if (err1 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save calibration to NVS: %s", esp_err_to_name(err1));
        return err1;
    }
    
    esp_err_t err2 = lsm6ds3_save_angle_zero_to_nvs(&s_angle_zero, "lsm6ds3");
    if (err2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save angle zero offsets to NVS: %s", esp_err_to_name(err2));
        return err2;
    }
    
    ESP_LOGI(TAG, "Calibration and angle zero offsets saved to NVS");
    return ESP_OK;
}

esp_err_t lsm6ds3_manager_load_calibration(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return ESP_OK;
}

esp_err_t lsm6ds3_manager_get_nvs_calibration(float accel_offset_mg[3], float gyro_offset_mdps[3], bool *accel_calibrated, bool *gyro_calibrated)
{
    if (accel_offset_mg == NULL || gyro_offset_mdps == NULL || accel_calibrated == NULL || gyro_calibrated == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(accel_offset_mg, 0, sizeof(float) * 3);
    memset(gyro_offset_mdps, 0, sizeof(float) * 3);
    *accel_calibrated = false;
    *gyro_calibrated = false;
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("lsm6ds3", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        return err;
    }
    
    size_t required_size = sizeof(float) * 3;
    size_t size = required_size;
    err = nvs_get_blob(nvs_handle, "gyro_offset", gyro_offset_mdps, &size);
    if (err == ESP_OK && size == required_size) {
        uint8_t gyro_cal = 0;
        if (nvs_get_u8(nvs_handle, "gyro_cal", &gyro_cal) == ESP_OK) {
            *gyro_calibrated = (gyro_cal != 0);
        }
    }
    
    nvs_close(nvs_handle);
    
    return ESP_OK;
}

