#include "vl53l1x_manager.h"
#include "i2c_bus_manager.h"
#include "system_config.h"
#include "fusion_core_assembly.h"
#include "vl53l1x.h"
#include "vl53l1x_types.h"
#include "i2c_handler.h"
#include "VL53L1X_api.h"
#include "ota_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "vl53l1x_manager";

static bool s_initialized = false;
static vl53l1x_handle_t s_vl53l1x_handle;
static vl53l1x_device_handle_t s_vl53l1x_device;
static vl53l1x_i2c_handle_t s_i2c_handle;
static TaskHandle_t s_task_handle = NULL;
static system_vl53l1x_config_t s_config;
static SemaphoreHandle_t s_config_mutex = NULL;

#define VL53L1X_I2C_ADDRESS 0x29
#define VL53L1X_UPDATE_INTERVAL_MS 20

static esp_err_t vl53l1x_manager_apply_config(void)
{
    if (!s_initialized || s_vl53l1x_device.dev == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint16_t dev = s_vl53l1x_device.dev;
    VL53L1X_ERROR err;
    
    err = VL53L1X_StopRanging(dev);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to stop ranging: %d", err);
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    err = VL53L1X_SetDistanceMode(dev, s_config.distance_mode);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set distance mode: %d", err);
    }
    
    err = VL53L1X_SetTimingBudgetInMs(dev, s_config.timing_budget_ms);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set timing budget: %d", err);
    }
    
    err = VL53L1X_SetInterMeasurementInMs(dev, s_config.inter_measurement_ms);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set inter-measurement period: %d", err);
    }
    
    err = VL53L1X_SetROI(dev, s_config.roi_x_size, s_config.roi_y_size);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set ROI: %d", err);
    }
    
    err = VL53L1X_SetROICenter(dev, s_config.roi_center_spad);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set ROI center: %d", err);
    }
    
    err = VL53L1X_SetOffset(dev, s_config.offset_mm);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set offset: %d", err);
    }
    
    err = VL53L1X_SetXtalk(dev, s_config.xtalk_cps);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set xtalk: %d", err);
    }
    
    err = VL53L1X_SetSignalThreshold(dev, s_config.signal_threshold_kcps);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set signal threshold: %d", err);
    }
    
    err = VL53L1X_SetSigmaThreshold(dev, s_config.sigma_threshold_mm);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set sigma threshold: %d", err);
    }
    
    err = VL53L1X_SetDistanceThreshold(dev, s_config.threshold_low_mm, 
                                       s_config.threshold_high_mm, 
                                       s_config.threshold_window, 1);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set distance threshold: %d", err);
    }
    
    err = VL53L1X_SetInterruptPolarity(dev, s_config.interrupt_polarity);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to set interrupt polarity: %d", err);
    }
    
    if (s_config.i2c_address != VL53L1X_I2C_ADDRESS) {
        err = VL53L1X_SetI2CAddress(dev, s_config.i2c_address);
        if (err != VL53L1X_ERROR_NONE) {
            ESP_LOGW(TAG, "Failed to set I2C address: %d", err);
        } else {
            ESP_LOGI(TAG, "VL53L1X I2C address changed to 0x%02X", s_config.i2c_address);
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    err = VL53L1X_StartRanging(dev);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "Failed to restart ranging: %d", err);
    }
    
    return ESP_OK;
}

static void vl53l1x_update_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t update_interval = pdMS_TO_TICKS(VL53L1X_UPDATE_INTERVAL_MS);
    uint32_t consecutive_errors = 0;
    const uint32_t max_consecutive_errors = 3;  // Back off after 3 errors
    const TickType_t error_backoff_delay = pdMS_TO_TICKS(200);  // Longer backoff
    const uint32_t max_total_errors_before_disable = 10;  // Disable after 10 consecutive errors (~1 second max)

    ESP_LOGI(TAG, "VL53L1X update task started");
    
    // Wait for I2C bus to stabilize after reboot (especially after OTA)
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        // Check if OTA is in progress - skip I2C operations to avoid errors
        ota_status_info_t ota_status;
        if (ota_manager_get_status(&ota_status) && 
            ota_status.status == OTA_STATUS_IN_PROGRESS) {
            // OTA in progress - skip I2C operations to avoid errors
            consecutive_errors = 0; // Reset error counter
            vTaskDelay(update_interval);
            continue;
        }
        
        if (s_initialized) {
            uint8_t is_data_ready = 0;
            VL53L1X_ERROR err = VL53L1X_CheckForDataReady(s_vl53l1x_device.dev, &is_data_ready);
            
            if (err == VL53L1X_ERROR_NONE && is_data_ready) {
                consecutive_errors = 0; // Reset error counter on success
                
                uint16_t distance = vl53l1x_get_mm(&s_vl53l1x_device);
                
                uint16_t ambient = 0;
                uint16_t signal_rate = 0;
                uint16_t num_spads = 16;
                
                VL53L1X_GetAmbientRate(s_vl53l1x_device.dev, &ambient);
                VL53L1X_GetSignalRate(s_vl53l1x_device.dev, &signal_rate);
                
                uint16_t sig_per_spad = (num_spads > 0) ? (signal_rate / num_spads) : 0;
                
                uint8_t status = 0x01;
                
                SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
                if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    memcpy(&INPUT_ASSEMBLY_100[VL53L1X_BYTE_START], &distance, sizeof(uint16_t));
                    INPUT_ASSEMBLY_100[VL53L1X_BYTE_START + 2] = status;
                    memcpy(&INPUT_ASSEMBLY_100[VL53L1X_BYTE_START + 3], &ambient, sizeof(uint16_t));
                    memcpy(&INPUT_ASSEMBLY_100[VL53L1X_BYTE_START + 5], &sig_per_spad, sizeof(uint16_t));
                    memcpy(&INPUT_ASSEMBLY_100[VL53L1X_BYTE_START + 7], &num_spads, sizeof(uint16_t));
                    xSemaphoreGive(assembly_mutex);
                }
            } else if (err != VL53L1X_ERROR_NONE) {
                // I2C error detected (timeout, invalid state, etc.)
                consecutive_errors++;
                
                if (consecutive_errors == 1) {
                    // Log first error for debugging
                    ESP_LOGW(TAG, "VL53L1X CheckForDataReady error: %d (attempt %lu)", err, consecutive_errors);
                } else if (consecutive_errors == max_consecutive_errors) {
                    // Log when we hit max errors and back off
                    ESP_LOGW(TAG, "VL53L1X repeated I2C errors (%lu), backing off", consecutive_errors);
                } else if (consecutive_errors >= max_total_errors_before_disable) {
                    // After 50+ consecutive errors (~1 second), device is likely disconnected/faulty
                    ESP_LOGE(TAG, "VL53L1X device not responding after %lu attempts - disabling manager", consecutive_errors);
                    ESP_LOGE(TAG, "To re-enable, disable and re-enable VL53L1X in configuration, then reboot");
                    
                    // Mark as not initialized to stop further attempts
                    s_initialized = false;
                    
                    // Exit the task
                    vTaskDelete(NULL);
                    return;
                }
                
                // Back off if we get too many consecutive errors (I2C bus may not be ready)
                if (consecutive_errors >= max_consecutive_errors && consecutive_errors < max_total_errors_before_disable) {
                    vTaskDelay(error_backoff_delay);
                    // Don't reset counter - we want to track total consecutive errors
                }
            }
        }
        vTaskDelay(update_interval);
    }
}

esp_err_t vl53l1x_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "VL53L1X manager already initialized");
        return ESP_OK;
    }

    if (!system_vl53l1x_enabled_load()) {
        ESP_LOGI(TAG, "VL53L1X is disabled in configuration");
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus_handle = NULL;
    uint8_t bus_num = 0;
    int sda_gpio = 0;
    int scl_gpio = 0;
    
    i2c_master_bus_handle_t primary_bus = i2c_bus_manager_get_primary_bus();
    i2c_master_bus_handle_t secondary_bus = i2c_bus_manager_get_secondary_bus();
    
    if (primary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(primary_bus, VL53L1X_I2C_ADDRESS, 100);
        if (ret == ESP_OK) {
            bus_handle = primary_bus;
            bus_num = I2C_BUS_PRIMARY_NUM;
            sda_gpio = I2C_PRIMARY_SDA_GPIO;
            scl_gpio = I2C_PRIMARY_SCL_GPIO;
        }
    }
    
    if (bus_handle == NULL && secondary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(secondary_bus, VL53L1X_I2C_ADDRESS, 100);
        if (ret == ESP_OK) {
            bus_handle = secondary_bus;
            bus_num = I2C_BUS_SECONDARY_NUM;
            sda_gpio = I2C_SECONDARY_SDA_GPIO;
            scl_gpio = I2C_SECONDARY_SCL_GPIO;
        }
    }
    
    if (bus_handle == NULL) {
        ESP_LOGI(TAG, "VL53L1X not detected at address 0x%02X on either bus", VL53L1X_I2C_ADDRESS);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "VL53L1X detected at address 0x%02X on %s bus", 
             VL53L1X_I2C_ADDRESS, (bus_num == I2C_BUS_PRIMARY_NUM) ? "primary" : "secondary");

    // VL53L1X may still be in boot sequence even if it responds to I2C probe
    // Give it additional time to fully initialize its internal state machine
    ESP_LOGI(TAG, "Waiting for VL53L1X to complete internal boot sequence...");
    vTaskDelay(pdMS_TO_TICKS(200));

    s_config_mutex = xSemaphoreCreateMutex();
    if (s_config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create config mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(&s_i2c_handle, 0, sizeof(s_i2c_handle));
    s_i2c_handle.i2c_port = bus_num;
    s_i2c_handle.sda_gpio = sda_gpio;
    s_i2c_handle.scl_gpio = scl_gpio;
    s_i2c_handle.bus_handle = bus_handle;
    s_i2c_handle.initialized = true;

    s_vl53l1x_handle.initialized = false;
    s_vl53l1x_handle.i2c_handle = &s_i2c_handle;

    if (!vl53l1x_init(&s_vl53l1x_handle)) {
        ESP_LOGE(TAG, "Failed to initialize VL53L1X driver");
        return ESP_FAIL;
    }

    memset(&s_vl53l1x_device, 0, sizeof(s_vl53l1x_device));
    s_vl53l1x_device.vl53l1x_handle = &s_vl53l1x_handle;
    s_vl53l1x_device.i2c_address = VL53L1X_I2C_ADDRESS;
    s_vl53l1x_device.scl_speed_hz = 400000;
    s_vl53l1x_device.xshut_gpio = GPIO_NUM_NC;
    s_vl53l1x_device.interrupt_gpio = GPIO_NUM_NC;

    if (!vl53l1x_add_device(&s_vl53l1x_device)) {
        ESP_LOGE(TAG, "Failed to add VL53L1X device");
        return ESP_FAIL;
    }
    
    system_vl53l1x_config_load(&s_config);
    vTaskDelay(pdMS_TO_TICKS(100));
    vl53l1x_manager_apply_config();
    
    // Test if device is actually responding after initialization
    uint8_t test_ready = 0;
    VL53L1X_ERROR test_err = VL53L1X_CheckForDataReady(s_vl53l1x_device.dev, &test_ready);
    if (test_err != VL53L1X_ERROR_NONE) {
        ESP_LOGW(TAG, "VL53L1X device not responding after initialization (error: %d)", test_err);
        ESP_LOGW(TAG, "Device may need more time to boot or may be faulty");
        // Continue anyway - the update task will handle errors and auto-disable if persistent
    }

    s_initialized = true;
    ESP_LOGI(TAG, "VL53L1X manager initialized successfully");

    xTaskCreate(vl53l1x_update_task, "vl53l1x_task", 4096, NULL, 5, &s_task_handle);
    if (s_task_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create VL53L1X update task");
    }

    return ESP_OK;
}

bool vl53l1x_manager_is_initialized(void)
{
    return s_initialized;
}

esp_err_t vl53l1x_manager_get_config(system_vl53l1x_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(config, &s_config, sizeof(system_vl53l1x_config_t));
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }
    
    memcpy(config, &s_config, sizeof(system_vl53l1x_config_t));
    return ESP_OK;
}

esp_err_t vl53l1x_manager_set_config(const system_vl53l1x_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    bool mutex_taken = false;
    if (s_config_mutex != NULL) {
        if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        mutex_taken = true;
    }
    
    memcpy(&s_config, config, sizeof(system_vl53l1x_config_t));
    
    bool initialized = s_initialized;
    
    if (mutex_taken) {
        xSemaphoreGive(s_config_mutex);
    }
    
    if (initialized) {
        esp_err_t err = vl53l1x_manager_apply_config();
        if (err != ESP_OK) {
            return err;
        }
    }
    
    if (!system_vl53l1x_config_save(config)) {
        ESP_LOGE(TAG, "Failed to save VL53L1X configuration to NVS");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "VL53L1X configuration saved to NVS successfully");
    return ESP_OK;
}

esp_err_t vl53l1x_manager_calibrate_offset(int16_t target_distance_mm)
{
    if (!s_initialized || s_vl53l1x_device.dev == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint16_t dev = s_vl53l1x_device.dev;
    uint16_t measured_distance = 0;
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    VL53L1X_ERROR err = VL53L1X_GetDistance(dev, &measured_distance);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to get distance for offset calibration: %d", err);
        return ESP_FAIL;
    }
    
    int16_t offset = (int16_t)measured_distance - target_distance_mm;
    s_config.offset_mm = offset;
    
    err = VL53L1X_SetOffset(dev, offset);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to set offset: %d", err);
        return ESP_FAIL;
    }
    
    if (!system_vl53l1x_config_save(&s_config)) {
        ESP_LOGW(TAG, "Failed to save offset to NVS");
    }
    
    ESP_LOGI(TAG, "Offset calibration complete: measured=%d mm, target=%d mm, offset=%d mm", 
             measured_distance, target_distance_mm, offset);
    
    return ESP_OK;
}

esp_err_t vl53l1x_manager_calibrate_xtalk(uint16_t target_distance_mm)
{
    if (!s_initialized || s_vl53l1x_device.dev == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint16_t dev = s_vl53l1x_device.dev;
    uint16_t measured_distance = 0;
    uint16_t signal_rate = 0;
    uint16_t ambient_rate = 0;
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    VL53L1X_ERROR err = VL53L1X_GetDistance(dev, &measured_distance);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to get distance for xtalk calibration: %d", err);
        return ESP_FAIL;
    }
    
    err = VL53L1X_GetSignalRate(dev, &signal_rate);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to get signal rate: %d", err);
        return ESP_FAIL;
    }
    
    err = VL53L1X_GetAmbientRate(dev, &ambient_rate);
    if (err != VL53L1X_ERROR_NONE) {
        ESP_LOGE(TAG, "Failed to get ambient rate: %d", err);
        return ESP_FAIL;
    }
    
    if (target_distance_mm > 0 && measured_distance > target_distance_mm) {
        uint16_t distance_error = measured_distance - target_distance_mm;
        uint16_t xtalk_estimate = (signal_rate * distance_error) / target_distance_mm;
        s_config.xtalk_cps = xtalk_estimate;
        
        err = VL53L1X_SetXtalk(dev, s_config.xtalk_cps);
        if (err != VL53L1X_ERROR_NONE) {
            ESP_LOGE(TAG, "Failed to set xtalk: %d", err);
            return ESP_FAIL;
        }
        
        if (!system_vl53l1x_config_save(&s_config)) {
            ESP_LOGW(TAG, "Failed to save xtalk to NVS");
        }
        
        ESP_LOGI(TAG, "Xtalk calibration complete: measured=%d mm, target=%d mm, xtalk=%d cps", 
                 measured_distance, target_distance_mm, s_config.xtalk_cps);
    } else {
        ESP_LOGW(TAG, "Xtalk calibration: invalid target distance or measurement");
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

