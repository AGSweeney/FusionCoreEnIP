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

#include "terabee_mini_evo_manager.h"
#include "terabee_mini_evo.h"
#include "i2c_bus_manager.h"
#include "system_config.h"
#include "fusion_core_assembly.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "terabee_mini_evo_manager";

static bool s_initialized = false;
static terabee_mini_evo_handle_t s_terabee_handle;
static TaskHandle_t s_task_handle = NULL;
static system_terabee_mini_evo_config_t s_config;
static SemaphoreHandle_t s_config_mutex = NULL;

#define TERABEE_MINI_EVO_I2C_ADDRESS 0x31  // Default 7-bit I2C address (can be changed via CHANGE_BASE_ADDR command)
#define TERABEE_MINI_EVO_UPDATE_INTERVAL_MS 20  // ~50 Hz update rate

// Assembly data offset - TODO: Define in fusion_core_assembly.h
// This should be after VL53L1X data (16 bytes) and other sensors
#define TERABEE_MINI_EVO_BYTE_START 16  // Adjust based on assembly layout
#define TERABEE_MINI_EVO_BYTE_END (TERABEE_MINI_EVO_BYTE_START + 8)  // 8 bytes for data

static void terabee_mini_evo_update_task(void *pvParameters)
{
    TickType_t update_interval = pdMS_TO_TICKS(TERABEE_MINI_EVO_UPDATE_INTERVAL_MS);
    uint32_t consecutive_errors = 0;
    const uint32_t MAX_CONSECUTIVE_ERRORS = 10;

    ESP_LOGI(TAG, "Terabee Evo Mini update task started");

    while (1) {
        if (!s_initialized) {
            vTaskDelay(update_interval);
            continue;
        }

        terabee_mini_evo_data_t data = {0};
        esp_err_t ret = terabee_mini_evo_read_distance(&s_terabee_handle, &data);

        if (ret == ESP_OK && data.valid) {
            consecutive_errors = 0;  // Reset error counter on success

            // Write to input assembly
            SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
            if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                // Distance (2 bytes)
                memcpy(&INPUT_ASSEMBLY_100[TERABEE_MINI_EVO_BYTE_START], &data.distance_mm, sizeof(uint16_t));
                // Status (1 byte)
                INPUT_ASSEMBLY_100[TERABEE_MINI_EVO_BYTE_START + 2] = data.status;
                // Signal strength (2 bytes, if available)
                memcpy(&INPUT_ASSEMBLY_100[TERABEE_MINI_EVO_BYTE_START + 3], &data.signal_strength, sizeof(uint16_t));
                // Reserved (3 bytes for future use)
                memset(&INPUT_ASSEMBLY_100[TERABEE_MINI_EVO_BYTE_START + 5], 0, 3);

                xSemaphoreGive(assembly_mutex);
            }
        } else {
            consecutive_errors++;
            if (consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
                ESP_LOGW(TAG, "Too many consecutive errors (%lu), disabling sensor", consecutive_errors);
                s_initialized = false;
                // Optionally disable in config
                // system_terabee_mini_evo_enabled_save(false);
            } else if (consecutive_errors == 1) {
                ESP_LOGW(TAG, "Read error: %s", esp_err_to_name(ret));
            }
        }

        vTaskDelay(update_interval);
    }
}

esp_err_t terabee_mini_evo_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Terabee Evo Mini manager already initialized");
        return ESP_OK;
    }

    if (!system_terabee_mini_evo_enabled_load()) {
        ESP_LOGI(TAG, "Terabee Evo Mini is disabled in configuration");
        return ESP_OK;
    }

    i2c_master_bus_handle_t bus_handle = NULL;
    uint8_t bus_num = 0;
    int sda_gpio = 0;
    int scl_gpio = 0;

    i2c_master_bus_handle_t primary_bus = i2c_bus_manager_get_primary_bus();
    i2c_master_bus_handle_t secondary_bus = i2c_bus_manager_get_secondary_bus();

    // Try primary bus first
    if (primary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(primary_bus, TERABEE_MINI_EVO_I2C_ADDRESS, 100);
        if (ret == ESP_OK) {
            bus_handle = primary_bus;
            bus_num = I2C_BUS_PRIMARY_NUM;
            sda_gpio = I2C_PRIMARY_SDA_GPIO;
            scl_gpio = I2C_PRIMARY_SCL_GPIO;
        }
    }

    // Try secondary bus if not found on primary
    if (bus_handle == NULL && secondary_bus != NULL) {
        esp_err_t ret = i2c_master_probe(secondary_bus, TERABEE_MINI_EVO_I2C_ADDRESS, 100);
        if (ret == ESP_OK) {
            bus_handle = secondary_bus;
            bus_num = I2C_BUS_SECONDARY_NUM;
            sda_gpio = I2C_SECONDARY_SDA_GPIO;
            scl_gpio = I2C_SECONDARY_SCL_GPIO;
        }
    }

    if (bus_handle == NULL) {
        ESP_LOGI(TAG, "Terabee Evo Mini not detected at address 0x%02X on either bus", 
                 TERABEE_MINI_EVO_I2C_ADDRESS);
        return ESP_OK;  // Not an error - sensor may not be connected
    }

    ESP_LOGI(TAG, "Terabee Evo Mini detected at address 0x%02X on %s bus",
             TERABEE_MINI_EVO_I2C_ADDRESS,
             (bus_num == I2C_BUS_PRIMARY_NUM) ? "primary" : "secondary");

    // Give sensor time to initialize
    vTaskDelay(pdMS_TO_TICKS(200));

    s_config_mutex = xSemaphoreCreateMutex();
    if (s_config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create config mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize handle
    memset(&s_terabee_handle, 0, sizeof(s_terabee_handle));
    s_terabee_handle.bus_handle = bus_handle;
    s_terabee_handle.i2c_address = TERABEE_MINI_EVO_I2C_ADDRESS;
    s_terabee_handle.initialized = false;

    // Initialize sensor
    esp_err_t ret = terabee_mini_evo_init(&s_terabee_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Terabee Evo Mini: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_config_mutex);
        return ret;
    }

    // Load configuration
    system_terabee_mini_evo_config_load(&s_config);
    vTaskDelay(pdMS_TO_TICKS(100));

    // TODO: Apply configuration (if sensor supports configuration)

    s_initialized = true;
    ESP_LOGI(TAG, "Terabee Evo Mini manager initialized successfully");

    // Start update task
    xTaskCreate(terabee_mini_evo_update_task, "terabee_task", 4096, NULL, 5, &s_task_handle);
    if (s_task_handle == NULL) {
        ESP_LOGW(TAG, "Failed to create update task");
        s_initialized = false;
        terabee_mini_evo_deinit(&s_terabee_handle);
        vSemaphoreDelete(s_config_mutex);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool terabee_mini_evo_manager_is_initialized(void)
{
    return s_initialized;
}

esp_err_t terabee_mini_evo_manager_get_config(system_terabee_mini_evo_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        memcpy(config, &s_config, sizeof(system_terabee_mini_evo_config_t));
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t terabee_mini_evo_manager_set_config(const system_terabee_mini_evo_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        memcpy(&s_config, config, sizeof(system_terabee_mini_evo_config_t));
        xSemaphoreGive(s_config_mutex);

        // Save to NVS
        system_terabee_mini_evo_config_save(&s_config);

        // TODO: Apply configuration to sensor if initialized

        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

