#include "nau7802_manager.h"
#include "i2c_bus_manager.h"
#include "system_config.h"
#include "fusion_core_assembly.h"
#include "nau7802.h"
#include "ota_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "nau7802_manager";

#define NAU7802_I2C_ADDRESS 0x2A
#define NAU7802_UPDATE_INTERVAL_MS 20
#define NAU7802_CONFIG_RELOAD_INTERVAL_MS 5000

static bool s_initialized = false;
static nau7802_t s_nau7802_device;
static SemaphoreHandle_t s_nau7802_mutex = NULL;
static SemaphoreHandle_t s_state_mutex = NULL;
static TaskHandle_t s_task_handle = NULL;

static void nau7802_scale_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t update_interval = pdMS_TO_TICKS(NAU7802_UPDATE_INTERVAL_MS);
    const TickType_t config_reload_interval = pdMS_TO_TICKS(NAU7802_CONFIG_RELOAD_INTERVAL_MS);
    const TickType_t connection_check_interval = pdMS_TO_TICKS(5000); // Check connection every 5 seconds
    TickType_t last_config_reload = xTaskGetTickCount();
    TickType_t last_connection_check = 0;
    bool cached_connected = true; // Assume connected initially
    
    uint8_t average_samples = system_nau7802_average_load();
    
    ESP_LOGI(TAG, "NAU7802 scale task started, average samples: %d", average_samples);
    
    while (1) {
        // Check if OTA is in progress - skip I2C operations to avoid errors
        ota_status_info_t ota_status;
        if (ota_manager_get_status(&ota_status) && 
            ota_status.status == OTA_STATUS_IN_PROGRESS) {
            // OTA in progress - skip I2C operations to avoid errors
            vTaskDelay(update_interval);
            continue;
        }
        
        TickType_t now = xTaskGetTickCount();
        if (now - last_config_reload >= config_reload_interval) {
            average_samples = system_nau7802_average_load();
            last_config_reload = now;
            ESP_LOGD(TAG, "NAU7802 config reloaded: average=%d", average_samples);
        }
        
        bool initialized = false;
        if (s_state_mutex != NULL) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            initialized = s_initialized;
            xSemaphoreGive(s_state_mutex);
        } else {
            initialized = s_initialized;
        }
        
        if (initialized) {
            // Check OTA status again before I2C operations (in case it changed during config reload)
            ota_status_info_t ota_status_check;
            if (ota_manager_get_status(&ota_status_check) && 
                ota_status_check.status == OTA_STATUS_IN_PROGRESS) {
                // OTA in progress - skip I2C operations to avoid errors
                vTaskDelay(update_interval);
                continue;
            }
            
            bool connected = cached_connected;
            // Only check connection status periodically (every 5 seconds) to avoid I2C bus spam
            if (now - last_connection_check >= connection_check_interval) {
                // Check OTA status before connection check I2C operation
                ota_status_info_t ota_status_conn;
                if (ota_manager_get_status(&ota_status_conn) && 
                    ota_status_conn.status == OTA_STATUS_IN_PROGRESS) {
                    // OTA in progress - skip I2C operations to avoid errors
                    vTaskDelay(update_interval);
                    continue;
                }
                
                if (s_nau7802_mutex != NULL && xSemaphoreTake(s_nau7802_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    connected = nau7802_is_connected(&s_nau7802_device);
                    if (connected != cached_connected) {
                        ESP_LOGI(TAG, "NAU7802 connection status changed: %s", connected ? "connected" : "disconnected");
                    }
                    cached_connected = connected;
                    last_connection_check = now;
                    xSemaphoreGive(s_nau7802_mutex);
                }
            }
            
            if (connected) {
                // Check OTA status one more time right before main I2C operations
                ota_status_info_t ota_status_final;
                if (ota_manager_get_status(&ota_status_final) && 
                    ota_status_final.status == OTA_STATUS_IN_PROGRESS) {
                    // OTA in progress - skip I2C operations to avoid errors
                    vTaskDelay(update_interval);
                    continue;
                }
                
                bool available = false;
                int32_t raw_reading = 0;
                float weight_grams = 0.0f;
                
                if (s_nau7802_mutex != NULL && xSemaphoreTake(s_nau7802_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    available = nau7802_available(&s_nau7802_device);
                    
                    if (available) {
                        if (average_samples > 1) {
                            raw_reading = nau7802_get_average(&s_nau7802_device, average_samples, 1000);
                        } else {
                            raw_reading = nau7802_get_reading(&s_nau7802_device);
                        }
                    }
                    
                    weight_grams = nau7802_get_weight(&s_nau7802_device, true, average_samples, 1000);
                    
                    xSemaphoreGive(s_nau7802_mutex);
                }
                
                uint8_t unit = system_nau7802_unit_load();
                float weight_converted = weight_grams;
                if (unit == 1) {
                    weight_converted = weight_grams / 453.592f;
                } else if (unit == 2) {
                    weight_converted = weight_grams / 1000.0f;
                }
                
                const float max_weight = 21474836.47f;
                const float min_weight = -21474836.48f;
                if (weight_converted > max_weight) {
                    weight_converted = max_weight;
                } else if (weight_converted < min_weight) {
                    weight_converted = min_weight;
                }
                
                int32_t weight_scaled = (int32_t)(weight_converted * 100.0f + 0.5f);
                
                uint8_t status_byte = 0;
                if (available) status_byte |= 0x01;
                if (connected) status_byte |= 0x02;
                if (initialized) status_byte |= 0x04;
                
                SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
                if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    memcpy(&INPUT_ASSEMBLY_100[NAU7802_BYTE_START], &weight_scaled, sizeof(int32_t));
                    memcpy(&INPUT_ASSEMBLY_100[NAU7802_BYTE_START + 4], &raw_reading, sizeof(int32_t));
                    INPUT_ASSEMBLY_100[NAU7802_BYTE_START + 8] = unit;
                    INPUT_ASSEMBLY_100[NAU7802_BYTE_START + 9] = status_byte;
                    xSemaphoreGive(assembly_mutex);
                }
            }
        }
        
        vTaskDelay(update_interval);
    }
}

esp_err_t nau7802_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "NAU7802 manager already initialized");
        return ESP_OK;
    }

    if (!system_nau7802_enabled_load()) {
        ESP_LOGI(TAG, "NAU7802 is disabled in configuration");
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus_handle = NULL;
    
    i2c_master_bus_handle_t primary_bus = i2c_bus_manager_get_primary_bus();
    i2c_master_bus_handle_t secondary_bus = i2c_bus_manager_get_secondary_bus();
    
    if (primary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(primary_bus, NAU7802_I2C_ADDRESS, 100);
        if (ret == ESP_OK) {
            bus_handle = primary_bus;
        }
    }
    
    if (bus_handle == NULL && secondary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(secondary_bus, NAU7802_I2C_ADDRESS, 100);
        if (ret == ESP_OK) {
            bus_handle = secondary_bus;
        }
    }
    
    if (bus_handle == NULL) {
        ESP_LOGI(TAG, "NAU7802 not detected at address 0x%02X on either bus", NAU7802_I2C_ADDRESS);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "NAU7802 detected at address 0x%02X on %s bus", 
             NAU7802_I2C_ADDRESS, (bus_handle == primary_bus) ? "primary" : "secondary");

    s_nau7802_mutex = xSemaphoreCreateMutex();
    s_state_mutex = xSemaphoreCreateMutex();

    esp_err_t err = nau7802_init(&s_nau7802_device, bus_handle, NAU7802_I2C_ADDRESS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NAU7802 init() failed: %s", esp_err_to_name(err));
        return err;
    }

    if (!nau7802_is_connected(&s_nau7802_device)) {
        ESP_LOGW(TAG, "NAU7802 not connected on I2C bus");
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t ldo_value = system_nau7802_ldo_load();
    uint8_t gain = system_nau7802_gain_load();
    uint8_t sample_rate = system_nau7802_sample_rate_load();
    uint8_t channel = system_nau7802_channel_load();
    
    err = nau7802_begin(&s_nau7802_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NAU7802 begin() failed: %s", esp_err_to_name(err));
        return err;
    }

    if (ldo_value != 4) {
        esp_err_t ldo_err = nau7802_set_ldo(&s_nau7802_device, ldo_value);
        if (ldo_err == ESP_OK) {
            ESP_LOGI(TAG, "NAU7802 LDO set to %d", ldo_value);
            vTaskDelay(pdMS_TO_TICKS(250));
        } else {
            ESP_LOGW(TAG, "Failed to set NAU7802 LDO: %s", esp_err_to_name(ldo_err));
        }
    }
    
    if (gain != 7) {
        esp_err_t gain_err = nau7802_set_gain(&s_nau7802_device, (nau7802_gain_t)gain);
        if (gain_err == ESP_OK) {
            ESP_LOGI(TAG, "NAU7802 gain set to %d (x%d)", gain, 1 << gain);
        } else {
            ESP_LOGW(TAG, "Failed to set NAU7802 gain: %s", esp_err_to_name(gain_err));
        }
    }
    
    if (sample_rate != 3) {
        esp_err_t rate_err = nau7802_set_sample_rate(&s_nau7802_device, (nau7802_sps_t)sample_rate);
        if (rate_err == ESP_OK) {
            const char *sps_str = (sample_rate == 0) ? "10" : (sample_rate == 1) ? "20" : 
                                  (sample_rate == 2) ? "40" : (sample_rate == 3) ? "80" : "320";
            ESP_LOGI(TAG, "NAU7802 sample rate set to %d (%s SPS)", sample_rate, sps_str);
        } else {
            ESP_LOGW(TAG, "Failed to set NAU7802 sample rate: %s", esp_err_to_name(rate_err));
        }
    }
    
    if (channel != 0) {
        esp_err_t chan_err = nau7802_set_channel(&s_nau7802_device, (nau7802_channel_t)channel);
        if (chan_err == ESP_OK) {
            ESP_LOGI(TAG, "NAU7802 channel set to %d (Channel %d)", channel, channel + 1);
        } else {
            ESP_LOGW(TAG, "Failed to set NAU7802 channel: %s", esp_err_to_name(chan_err));
        }
    }
    
    ESP_LOGI(TAG, "Performing NAU7802 AFE calibration with current settings");
    err = nau7802_calibrate_af(&s_nau7802_device);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NAU7802 AFE calibration failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NAU7802 AFE calibration completed successfully");
    }
    
    float cal_factor = system_nau7802_calibration_factor_load();
    float zero_offset = system_nau7802_zero_offset_load();
    if (cal_factor > 0.0f) {
        nau7802_set_calibration_factor(&s_nau7802_device, cal_factor);
    }
    if (zero_offset != 0.0f) {
        nau7802_set_zero_offset(&s_nau7802_device, zero_offset);
    }

    if (s_task_handle != NULL) {
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
    }
    
    xTaskCreate(nau7802_scale_task, "nau7802_task", 4096, NULL, 5, &s_task_handle);
    if (s_task_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create NAU7802 task");
    } else {
        ESP_LOGI(TAG, "NAU7802 scale reading task started");
    }

    if (s_state_mutex != NULL) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_initialized = true;
        xSemaphoreGive(s_state_mutex);
    } else {
        s_initialized = true;
    }
    
    ESP_LOGI(TAG, "NAU7802 manager initialized successfully");
    return ESP_OK;
}

bool nau7802_manager_is_initialized(void)
{
    bool initialized = false;
    if (s_state_mutex != NULL) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        initialized = s_initialized;
        xSemaphoreGive(s_state_mutex);
    } else {
        initialized = s_initialized;
    }
    return initialized;
}

void* nau7802_manager_get_device_handle(void)
{
    if (!s_initialized) {
        return NULL;
    }
    return (void*)&s_nau7802_device;
}

SemaphoreHandle_t nau7802_manager_get_mutex(void)
{
    return s_nau7802_mutex;
}

