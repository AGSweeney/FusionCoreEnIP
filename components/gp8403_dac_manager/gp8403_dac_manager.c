#include "gp8403_dac_manager.h"
#include "i2c_bus_manager.h"
#include "system_config.h"
#include "fusion_core_assembly.h"
#include "gp8403_dac.h"
#include "ota_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "gp8403_dac_manager";

#define GP8403_DAC_MAX_DEVICES 4
#define GP8403_DAC_BASE_ADDRESS 0x58
#define GP8403_DAC_UPDATE_INTERVAL_MS 20

typedef struct {
    gp8403_dac_handle_t *handle;
    uint8_t address;
    bool initialized;
} gp8403_device_info_t;

static bool s_initialized = false;
static gp8403_device_info_t s_devices[GP8403_DAC_MAX_DEVICES];
static TaskHandle_t s_task_handle = NULL;

static void gp8403_dac_update_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t update_interval = pdMS_TO_TICKS(GP8403_DAC_UPDATE_INTERVAL_MS);

    ESP_LOGI(TAG, "GP8403 DAC update task started");

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
            uint16_t channel_values[GP8403_DAC_MAX_DEVICES * 2] = {0};
            SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
            if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                for (int i = 0; i < GP8403_DAC_MAX_DEVICES; i++) {
                    if (s_devices[i].initialized && s_devices[i].handle != NULL) {
                        uint8_t byte_offset = GP8403_DAC_BYTE_START + ((s_devices[i].address - GP8403_DAC_BASE_ADDRESS) * 4);
                        memcpy(&channel_values[i * 2], &OUTPUT_ASSEMBLY_150[byte_offset], sizeof(uint16_t));
                        memcpy(&channel_values[i * 2 + 1], &OUTPUT_ASSEMBLY_150[byte_offset + 2], sizeof(uint16_t));
                    }
                }
                xSemaphoreGive(assembly_mutex);
            }
            
            for (int i = 0; i < GP8403_DAC_MAX_DEVICES; i++) {
                if (s_devices[i].initialized && s_devices[i].handle != NULL) {
                    uint16_t channel0_raw = channel_values[i * 2];
                    uint16_t channel1_raw = channel_values[i * 2 + 1];
                    
                    if (channel0_raw > 0xFFF) channel0_raw = 0xFFF;
                    if (channel1_raw > 0xFFF) channel1_raw = 0xFFF;
                    
                    gp8403_dac_set_raw(s_devices[i].handle, GP8403_CHANNEL_0, channel0_raw);
                    gp8403_dac_set_raw(s_devices[i].handle, GP8403_CHANNEL_1, channel1_raw);
                }
            }
        }
        vTaskDelay(update_interval);
    }
}

esp_err_t gp8403_dac_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "GP8403 DAC manager already initialized");
        return ESP_OK;
    }

    if (!system_gp8403_dac_enabled_load()) {
        ESP_LOGI(TAG, "GP8403 DAC is disabled in configuration");
        return ESP_OK;
    }

    i2c_master_bus_handle_t primary_bus = i2c_bus_manager_get_primary_bus();
    i2c_master_bus_handle_t secondary_bus = i2c_bus_manager_get_secondary_bus();
    
    if (primary_bus == NULL && secondary_bus == NULL) {
        ESP_LOGE(TAG, "No I2C buses available");
        return ESP_ERR_INVALID_STATE;
    }

    memset(s_devices, 0, sizeof(s_devices));

    int device_count = 0;
    for (uint8_t addr = GP8403_DAC_BASE_ADDRESS; addr < GP8403_DAC_BASE_ADDRESS + GP8403_DAC_MAX_DEVICES; addr++) {
        i2c_master_bus_handle_t bus_handle = NULL;
        esp_err_t probe_result = ESP_FAIL;
        
        if (primary_bus != NULL) {
            probe_result = i2c_master_probe(primary_bus, addr, 100);
            if (probe_result == ESP_OK) {
                bus_handle = primary_bus;
            }
        }
        
        if (bus_handle == NULL && secondary_bus != NULL) {
            probe_result = i2c_master_probe(secondary_bus, addr, 100);
            if (probe_result == ESP_OK) {
                bus_handle = secondary_bus;
            }
        }
        
        if (bus_handle != NULL) {
            gp8403_dac_config_t config = {
                .bus_handle = bus_handle,
                .i2c_addr = addr,
            };
            
            gp8403_dac_handle_t *handle = NULL;
            if (gp8403_dac_init(&config, &handle)) {
                s_devices[device_count].handle = handle;
                s_devices[device_count].address = addr;
                s_devices[device_count].initialized = true;
                device_count++;
                ESP_LOGI(TAG, "GP8403 DAC initialized at address 0x%02X on %s bus", 
                         addr, (bus_handle == primary_bus) ? "primary" : "secondary");
            } else {
                ESP_LOGW(TAG, "Failed to initialize GP8403 DAC at address 0x%02X", addr);
            }
        }
    }

    if (device_count == 0) {
        ESP_LOGI(TAG, "No GP8403 DAC devices detected");
        return ESP_OK;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "GP8403 DAC manager initialized with %d device(s)", device_count);

    xTaskCreatePinnedToCore(gp8403_dac_update_task, "gp8403_dac_task", 4096, NULL, 5, &s_task_handle, 1);
    if (s_task_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create GP8403 DAC update task");
    }

    return ESP_OK;
}

bool gp8403_dac_manager_is_initialized(void)
{
    return s_initialized;
}

