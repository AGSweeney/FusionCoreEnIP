#include "vl53l1x_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "vl53l1x_config";
static const char *NVS_NAMESPACE = "vl53l1x";
static const char *NVS_KEY = "config";

void vl53l1x_config_get_defaults(vl53l1x_config_t *config)
{
    if (config == NULL) {
        return;
    }
    
    memset(config, 0, sizeof(vl53l1x_config_t));
    
    config->distance_mode = 2;              // LONG mode
    config->timing_budget_ms = 100;         // 100ms
    config->inter_measurement_ms = 100;     // 100ms
    config->roi_x_size = 16;                // Full width
    config->roi_y_size = 16;                // Full height
    config->roi_center_spad = 199;          // Center
    config->offset_mm = 0;                  // No offset
    config->xtalk_cps = 0;                  // No xtalk compensation
    config->signal_threshold_kcps = 1024;   // Default threshold
    config->sigma_threshold_mm = 15;        // Default sigma
    config->threshold_low_mm = 0;           // Disabled
    config->threshold_high_mm = 0;          // Disabled
    config->threshold_window = 0;           // Disabled
    config->interrupt_polarity = 1;         // Active high
    config->i2c_address = 0x29;            // Default address
}

bool vl53l1x_config_validate(const vl53l1x_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    
    // Validate distance_mode
    if (config->distance_mode != 1 && config->distance_mode != 2) {
        ESP_LOGE(TAG, "Invalid distance_mode: %d (must be 1 or 2)", config->distance_mode);
        return false;
    }
    
    // Validate timing_budget_ms
    const uint16_t valid_budgets[] = {15, 20, 33, 50, 100, 200, 500};
    bool budget_valid = false;
    for (int i = 0; i < sizeof(valid_budgets)/sizeof(valid_budgets[0]); i++) {
        if (config->timing_budget_ms == valid_budgets[i]) {
            budget_valid = true;
            break;
        }
    }
    if (!budget_valid) {
        ESP_LOGE(TAG, "Invalid timing_budget_ms: %d", config->timing_budget_ms);
        return false;
    }
    
    // Validate inter_measurement_ms
    if (config->inter_measurement_ms < config->timing_budget_ms) {
        ESP_LOGE(TAG, "inter_measurement_ms (%lu) must be >= timing_budget_ms (%d)",
                 config->inter_measurement_ms, config->timing_budget_ms);
        return false;
    }
    
    // Validate ROI sizes
    if (config->roi_x_size < 4 || config->roi_x_size > 16) {
        ESP_LOGE(TAG, "Invalid roi_x_size: %d (must be 4-16)", config->roi_x_size);
        return false;
    }
    if (config->roi_y_size < 4 || config->roi_y_size > 16) {
        ESP_LOGE(TAG, "Invalid roi_y_size: %d (must be 4-16)", config->roi_y_size);
        return false;
    }
    
    // Validate ROI center
    if (config->roi_center_spad > 199) {
        ESP_LOGE(TAG, "Invalid roi_center_spad: %d (must be 0-199)", config->roi_center_spad);
        return false;
    }
    
    // Validate offset
    if (config->offset_mm < -128 || config->offset_mm > 127) {
        ESP_LOGE(TAG, "Invalid offset_mm: %d (must be -128 to +127)", config->offset_mm);
        return false;
    }
    
    // Validate threshold high >= low
    if (config->threshold_high_mm > 0 && config->threshold_low_mm > 0) {
        if (config->threshold_high_mm < config->threshold_low_mm) {
            ESP_LOGE(TAG, "threshold_high_mm (%d) must be >= threshold_low_mm (%d)",
                     config->threshold_high_mm, config->threshold_low_mm);
            return false;
        }
    }
    
    // Validate threshold_window
    if (config->threshold_window > 3) {
        ESP_LOGE(TAG, "Invalid threshold_window: %d (must be 0-3)", config->threshold_window);
        return false;
    }
    
    // Validate interrupt_polarity
    if (config->interrupt_polarity > 1) {
        ESP_LOGE(TAG, "Invalid interrupt_polarity: %d (must be 0 or 1)", config->interrupt_polarity);
        return false;
    }
    
    // Validate i2c_address
    if (config->i2c_address < 0x29 || config->i2c_address > 0x7F) {
        ESP_LOGE(TAG, "Invalid i2c_address: 0x%02X (must be 0x29-0x7F)", config->i2c_address);
        return false;
    }
    
    return true;
}

bool vl53l1x_config_load(vl53l1x_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No saved configuration found, using defaults");
            vl53l1x_config_get_defaults(config);
            return false; // Return false to indicate no saved config
        }
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return false;
    }
    
    size_t required_size = sizeof(vl53l1x_config_t);
    err = nvs_get_blob(handle, NVS_KEY, config, &required_size);
    nvs_close(handle);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved configuration found, using defaults");
        vl53l1x_config_get_defaults(config);
        return false; // Return false to indicate no saved config
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load configuration: %s", esp_err_to_name(err));
        return false;
    }
    
    if (required_size != sizeof(vl53l1x_config_t)) {
        ESP_LOGW(TAG, "Configuration size mismatch (expected %zu, got %zu), using defaults",
                 sizeof(vl53l1x_config_t), required_size);
        vl53l1x_config_get_defaults(config);
        return false;
    }
    
    // Validate loaded configuration
    if (!vl53l1x_config_validate(config)) {
        ESP_LOGW(TAG, "Loaded configuration is invalid, using defaults");
        vl53l1x_config_get_defaults(config);
        return false;
    }
    
    ESP_LOGI(TAG, "Configuration loaded successfully from NVS");
    return true;
}

bool vl53l1x_config_save(const vl53l1x_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    
    // Validate before saving
    if (!vl53l1x_config_validate(config)) {
        ESP_LOGE(TAG, "Cannot save invalid configuration");
        return false;
    }
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return false;
    }
    
    err = nvs_set_blob(handle, NVS_KEY, config, sizeof(vl53l1x_config_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }
    
    err = nvs_commit(handle);
    nvs_close(handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit configuration: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG, "Configuration saved successfully to NVS");
    return true;
}

