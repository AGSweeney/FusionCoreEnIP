#include "mcp230xx_manager.h"
#include "i2c_bus_manager.h"
#include "system_config.h"
#include "mcp_config.h"
#include "fusion_core_assembly.h"
#include "mcp23008.h"
#include "mcp23008_init.h"
#include "mcp23017.h"
#include "mcp23017_init.h"
#include "ota_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "mcp230xx_manager";

#define MCP230XX_UPDATE_INTERVAL_MS 20

typedef struct {
    uint8_t i2c_address;
    uint8_t device_type;
    bool initialized;
    union {
        mcp23008_t mcp23008_dev;
        mcp23017_t mcp23017_dev;
    } device;
} mcp230xx_device_t;

static bool s_initialized = false;
static mcp230xx_device_t s_devices[MCP_MAX_DEVICES];
static uint8_t s_device_count = 0;
static TaskHandle_t s_task_handle = NULL;

static int compare_devices(const void *a, const void *b)
{
    const mcp230xx_device_t *dev_a = (const mcp230xx_device_t *)a;
    const mcp230xx_device_t *dev_b = (const mcp230xx_device_t *)b;
    return (int)dev_a->i2c_address - (int)dev_b->i2c_address;
}

static void mcp230xx_update_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t update_interval = pdMS_TO_TICKS(MCP230XX_UPDATE_INTERVAL_MS);
    static uint32_t error_counts[MCP_MAX_DEVICES] = {0};
    const uint32_t MAX_CONSECUTIVE_ERRORS = 10; // Skip device after 10 consecutive errors

    ESP_LOGI(TAG, "MCP230XX update task started");

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
            uint8_t local_output_assembly[16] = {0};
            SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
            if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                memcpy(local_output_assembly, OUTPUT_ASSEMBLY_150, 16);
                xSemaphoreGive(assembly_mutex);
            } else {
                vTaskDelay(update_interval);
                continue;
            }
            
            // Static arrays to track last written values
            static uint16_t last_mcp23017_values[MCP_MAX_DEVICES] = {0};
            static uint8_t last_mcp23008_values[MCP_MAX_DEVICES] = {0};
            static uint32_t last_readback_time[MCP_MAX_DEVICES] = {0};
            const uint32_t READBACK_INTERVAL_MS = 100; // Read GPIO state every 100ms for feedback
            
            for (int i = 0; i < s_device_count; i++) {
                if (!s_devices[i].initialized) {
                    continue;
                }

                if (error_counts[i] >= MAX_CONSECUTIVE_ERRORS) {
                    continue;
                }

                uint8_t output_byte_offset = (s_devices[i].i2c_address - 0x20) * 2;
                uint8_t input_byte_offset = 40 + output_byte_offset;
                bool operation_success = false;
                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                bool needs_readback = (current_time - last_readback_time[i]) >= READBACK_INTERVAL_MS;

                if (s_devices[i].device_type == 0) {
                    uint16_t gpio_value = ((uint16_t)local_output_assembly[output_byte_offset + 1] << 8) | local_output_assembly[output_byte_offset];
                    
                    // Only write if value changed
                    if (gpio_value != last_mcp23017_values[i]) {
                        if (mcp23017_write_gpio(&s_devices[i].device.mcp23017_dev, gpio_value) == ESP_OK) {
                            last_mcp23017_values[i] = gpio_value;
                            operation_success = true;
                        }
                    } else {
                        operation_success = true; // No write needed, but operation is "successful"
                    }
                    
                    // Read back GPIO state periodically for feedback (even if we didn't write)
                    if (needs_readback && operation_success) {
                        uint16_t readback = 0;
                        if (mcp23017_read_gpio(&s_devices[i].device.mcp23017_dev, &readback) == ESP_OK) {
                            if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                                INPUT_ASSEMBLY_100[input_byte_offset] = (uint8_t)(readback & 0xFF);
                                INPUT_ASSEMBLY_100[input_byte_offset + 1] = (uint8_t)(readback >> 8);
                                xSemaphoreGive(assembly_mutex);
                            }
                            last_readback_time[i] = current_time;
                        }
                    }
                } else {
                    uint8_t gpio_value = local_output_assembly[output_byte_offset];
                    
                    // Only write if value changed
                    if (gpio_value != last_mcp23008_values[i]) {
                        if (mcp23008_write_gpio(&s_devices[i].device.mcp23008_dev, gpio_value) == ESP_OK) {
                            last_mcp23008_values[i] = gpio_value;
                            operation_success = true;
                        }
                    } else {
                        operation_success = true; // No write needed, but operation is "successful"
                    }
                    
                    // Read back GPIO state periodically for feedback (even if we didn't write)
                    if (needs_readback && operation_success) {
                        uint8_t readback = 0;
                        if (mcp23008_read_gpio(&s_devices[i].device.mcp23008_dev, &readback) == ESP_OK) {
                            if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                                INPUT_ASSEMBLY_100[input_byte_offset] = readback;
                                xSemaphoreGive(assembly_mutex);
                            }
                            last_readback_time[i] = current_time;
                        }
                    }
                }

                if (operation_success) {
                    if (error_counts[i] > 0) {
                        error_counts[i] = 0;
                    }
                } else {
                    error_counts[i]++;
                    if (error_counts[i] == MAX_CONSECUTIVE_ERRORS) {
                        ESP_LOGW(TAG, "MCP230XX device at 0x%02X failed %d times, skipping further operations", 
                                s_devices[i].i2c_address, MAX_CONSECUTIVE_ERRORS);
                    }
                }
            }
        }
        vTaskDelay(update_interval);
    }
}

esp_err_t mcp230xx_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "MCP230XX manager already initialized");
        return ESP_OK;
    }

    i2c_master_bus_handle_t primary_bus = i2c_bus_manager_get_primary_bus();
    i2c_master_bus_handle_t secondary_bus = i2c_bus_manager_get_secondary_bus();
    
    if (primary_bus == NULL && secondary_bus == NULL) {
        ESP_LOGE(TAG, "No I2C buses available");
        return ESP_ERR_INVALID_STATE;
    }

    mcp_detected_devices_t detected_devices = {0};
    
    for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
        i2c_master_bus_handle_t bus = NULL;
        esp_err_t probe_result = ESP_FAIL;
        
        if (primary_bus != NULL) {
            probe_result = i2c_master_probe(primary_bus, addr, 100);
            if (probe_result == ESP_OK) {
                bus = primary_bus;
            }
        }
        
        if (bus == NULL && secondary_bus != NULL) {
            probe_result = i2c_master_probe(secondary_bus, addr, 100);
            if (probe_result == ESP_OK) {
                bus = secondary_bus;
            }
        }
        
        if (bus != NULL && detected_devices.device_count < MCP_MAX_DEVICES) {
            detected_devices.devices[detected_devices.device_count].i2c_address = addr;
            detected_devices.devices[detected_devices.device_count].detected = true;
            detected_devices.devices[detected_devices.device_count].device_type = 0xFF;
            detected_devices.device_count++;
        }
    }
    
    mcp_config_set_detected_devices(&detected_devices);

    if (!system_mcp_enabled_load()) {
        ESP_LOGI(TAG, "MCP230XX is disabled in configuration");
        return ESP_OK;
    }

    mcp_config_t config;
    if (!mcp_config_load_all(&config)) {
        ESP_LOGI(TAG, "No MCP230XX devices configured");
        return ESP_OK;
    }

    memset(s_devices, 0, sizeof(s_devices));
    s_device_count = 0;

    for (int i = 0; i < config.device_count && s_device_count < MCP_MAX_DEVICES; i++) {
        if (!config.devices[i].enabled) {
            continue;
        }

        uint8_t addr = config.devices[i].i2c_address;
        if (addr < 0x20 || addr > 0x27) {
            ESP_LOGW(TAG, "Invalid MCP230XX address: 0x%02X", addr);
            continue;
        }

        i2c_master_bus_handle_t bus = NULL;
        esp_err_t probe_result = ESP_FAIL;
        
        if (primary_bus != NULL) {
            probe_result = i2c_master_probe(primary_bus, addr, 100);
            if (probe_result == ESP_OK) {
                bus = primary_bus;
            }
        }
        
        if (bus == NULL && secondary_bus != NULL) {
            probe_result = i2c_master_probe(secondary_bus, addr, 100);
            if (probe_result == ESP_OK) {
                bus = secondary_bus;
            }
        }
        
        if (bus == NULL) {
            ESP_LOGD(TAG, "MCP230XX device not detected at address 0x%02X", addr);
            continue;
        }

        mcp230xx_device_t *dev = &s_devices[s_device_count];
        dev->i2c_address = addr;
        dev->device_type = config.devices[i].device_type;
        dev->initialized = false;

        if (dev->device_type == 0) {
            if (mcp23017_init_with_bus(bus, addr, &dev->device.mcp23017_dev)) {
                dev->initialized = true;
                ESP_LOGI(TAG, "MCP23017 initialized at address 0x%02X", addr);
            }
        } else {
            if (mcp23008_init_with_bus(bus, addr, &dev->device.mcp23008_dev)) {
                dev->initialized = true;
                ESP_LOGI(TAG, "MCP23008 initialized at address 0x%02X", addr);
            }
        }

        if (dev->initialized) {
            s_device_count++;
        }
    }

    for (int j = 0; j < s_device_count; j++) {
        if (!s_devices[j].initialized) {
            continue;
        }
        
        bool found_in_detected = false;
        for (int i = 0; i < detected_devices.device_count; i++) {
            if (detected_devices.devices[i].i2c_address == s_devices[j].i2c_address) {
                detected_devices.devices[i].device_type = s_devices[j].device_type;
                detected_devices.devices[i].detected = true;
                found_in_detected = true;
                break;
            }
        }
        
        if (!found_in_detected && detected_devices.device_count < MCP_MAX_DEVICES) {
            detected_devices.devices[detected_devices.device_count].i2c_address = s_devices[j].i2c_address;
            detected_devices.devices[detected_devices.device_count].detected = true;
            detected_devices.devices[detected_devices.device_count].device_type = s_devices[j].device_type;
            detected_devices.device_count++;
        }
    }
    
    mcp_config_set_detected_devices(&detected_devices);
    
    if (s_device_count == 0) {
        ESP_LOGI(TAG, "No MCP230XX devices initialized");
        return ESP_OK;
    }

    qsort(s_devices, s_device_count, sizeof(mcp230xx_device_t), compare_devices);

    s_initialized = true;
    ESP_LOGI(TAG, "MCP230XX manager initialized with %d device(s)", s_device_count);

    xTaskCreate(mcp230xx_update_task, "mcp230xx_task", 4096, NULL, 5, &s_task_handle);
    if (s_task_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create MCP230XX update task");
    }

    return ESP_OK;
}

bool mcp230xx_manager_is_initialized(void)
{
    return s_initialized;
}

