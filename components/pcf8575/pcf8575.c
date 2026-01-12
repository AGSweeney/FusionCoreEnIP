#include "pcf8575.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include <string.h>

static const char *TAG = "pcf8575";

#define PCF8575_I2C_TIMEOUT_MS 100
#define PCF8575_MAX_RETRIES 3
#define PCF8575_MAX_PIN 15

/**
 * @brief Write data to PCF8575
 * 
 * PCF8575 uses a simple write protocol: 2 bytes (low byte, high byte)
 * Low byte = P0-P7, High byte = P8-P15
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param port_value 16-bit port value to write
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t pcf8575_write_internal(pcf8575_t *dev, uint16_t port_value)
{
    if (dev == NULL || dev->i2c_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2];
    data[0] = (uint8_t)(port_value & 0xFF);        // P0-P7 (low byte)
    data[1] = (uint8_t)((port_value >> 8) & 0xFF); // P8-P15 (high byte)

    esp_err_t err;
    int retry_count = 0;

    do {
        err = i2c_master_transmit(dev->i2c_dev, data, sizeof(data), pdMS_TO_TICKS(PCF8575_I2C_TIMEOUT_MS));

        // Retry on timeout or bus errors
        if ((err == ESP_ERR_TIMEOUT || err == ESP_FAIL) && retry_count < PCF8575_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(1 << retry_count)); // Exponential backoff: 1ms, 2ms, 4ms
            retry_count++;
        } else {
            break;
        }
    } while (retry_count < PCF8575_MAX_RETRIES);

    if (err == ESP_OK) {
        dev->port_state = port_value;  // Update cached state
    }

    return err;
}

/**
 * @brief Read data from PCF8575
 * 
 * PCF8575 uses a simple read protocol: 2 bytes (low byte, high byte)
 * Low byte = P0-P7, High byte = P8-P15
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param port_value Pointer to store 16-bit port value
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t pcf8575_read_internal(pcf8575_t *dev, uint16_t *port_value)
{
    if (dev == NULL || dev->i2c_dev == NULL || port_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2];
    esp_err_t err;
    int retry_count = 0;

    do {
        err = i2c_master_receive(dev->i2c_dev, data, sizeof(data), pdMS_TO_TICKS(PCF8575_I2C_TIMEOUT_MS));

        // Retry on timeout or bus errors
        if ((err == ESP_ERR_TIMEOUT || err == ESP_FAIL) && retry_count < PCF8575_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(1 << retry_count)); // Exponential backoff: 1ms, 2ms, 4ms
            retry_count++;
        } else {
            break;
        }
    } while (retry_count < PCF8575_MAX_RETRIES);

    if (err == ESP_OK) {
        *port_value = ((uint16_t)data[1] << 8) | data[0];  // High byte = P8-P15, Low byte = P0-P7
        dev->port_state = *port_value;  // Update cached state
    }

    return err;
}

bool pcf8575_init(pcf8575_t *dev, i2c_master_dev_handle_t i2c_dev, const pcf8575_config_t *config)
{
    if (dev == NULL || i2c_dev == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return false;
    }

    memset(dev, 0, sizeof(pcf8575_t));
    dev->i2c_dev = i2c_dev;

    // Default configuration: all pins as inputs (0xFFFF)
    uint16_t initial_state = 0xFFFF;
    if (config != NULL) {
        initial_state = config->initial_port_state;
    }

    // Write initial port state
    esp_err_t err = pcf8575_write_internal(dev, initial_state);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write initial port state: %s", esp_err_to_name(err));
        return false;
    }

    dev->initialized = true;
    ESP_LOGI(TAG, "PCF8575 initialized with initial state: 0x%04X", initial_state);

    return true;
}

esp_err_t pcf8575_deinit(pcf8575_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    dev->initialized = false;
    dev->i2c_dev = NULL;
    dev->port_state = 0;

    return ESP_OK;
}

esp_err_t pcf8575_read_port(pcf8575_t *dev, uint16_t *port_value)
{
    if (dev == NULL || port_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return pcf8575_read_internal(dev, port_value);
}

esp_err_t pcf8575_write_port(pcf8575_t *dev, uint16_t port_value)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    return pcf8575_write_internal(dev, port_value);
}

esp_err_t pcf8575_read_pin(pcf8575_t *dev, uint8_t pin, bool *level)
{
    if (dev == NULL || level == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pin > PCF8575_MAX_PIN) {
        ESP_LOGE(TAG, "Invalid pin number: %d (max: %d)", pin, PCF8575_MAX_PIN);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t port_value;
    esp_err_t err = pcf8575_read_internal(dev, &port_value);
    if (err != ESP_OK) {
        return err;
    }

    *level = (port_value & (1U << pin)) != 0;
    return ESP_OK;
}

esp_err_t pcf8575_write_pin(pcf8575_t *dev, uint8_t pin, bool level)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pin > PCF8575_MAX_PIN) {
        ESP_LOGE(TAG, "Invalid pin number: %d (max: %d)", pin, PCF8575_MAX_PIN);
        return ESP_ERR_INVALID_ARG;
    }

    // Read current state first (read-modify-write)
    uint16_t port_value = dev->port_state;
    
    // Update the pin
    if (level) {
        port_value |= (1U << pin);  // Set bit (makes pin input with pull-up)
    } else {
        port_value &= ~(1U << pin); // Clear bit (makes pin output low)
    }

    return pcf8575_write_internal(dev, port_value);
}

esp_err_t pcf8575_update_pins(pcf8575_t *dev, uint16_t mask, uint16_t value)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Read current state first (read-modify-write)
    uint16_t port_value = dev->port_state;

    // Update only pins specified in mask
    port_value = (port_value & ~mask) | (value & mask);

    return pcf8575_write_internal(dev, port_value);
}

esp_err_t pcf8575_set_pin_input(pcf8575_t *dev, uint8_t pin)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pin > PCF8575_MAX_PIN) {
        ESP_LOGE(TAG, "Invalid pin number: %d (max: %d)", pin, PCF8575_MAX_PIN);
        return ESP_ERR_INVALID_ARG;
    }

    // Set pin to 1 (input mode with pull-up)
    return pcf8575_write_pin(dev, pin, true);
}

esp_err_t pcf8575_set_pin_output(pcf8575_t *dev, uint8_t pin, bool level)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pin > PCF8575_MAX_PIN) {
        ESP_LOGE(TAG, "Invalid pin number: %d (max: %d)", pin, PCF8575_MAX_PIN);
        return ESP_ERR_INVALID_ARG;
    }

    // Set pin output level
    // Note: level=true makes it input with pull-up (quasi-bidirectional)
    //       level=false makes it output low
    return pcf8575_write_pin(dev, pin, level);
}

esp_err_t pcf8575_set_pins_input(pcf8575_t *dev, uint16_t mask)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Set pins to 1 (input mode with pull-up)
    return pcf8575_update_pins(dev, mask, 0xFFFF);
}

esp_err_t pcf8575_set_pins_output(pcf8575_t *dev, uint16_t mask, uint16_t initial_value)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Set pins as outputs with initial values
    // Pins in mask will be set to values in initial_value
    // Pins not in mask remain unchanged
    uint16_t port_value = dev->port_state;
    port_value = (port_value & ~mask) | (initial_value & mask);

    return pcf8575_write_internal(dev, port_value);
}

esp_err_t pcf8575_toggle_pin(pcf8575_t *dev, uint8_t pin)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        ESP_LOGE(TAG, "Device not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (pin > PCF8575_MAX_PIN) {
        ESP_LOGE(TAG, "Invalid pin number: %d (max: %d)", pin, PCF8575_MAX_PIN);
        return ESP_ERR_INVALID_ARG;
    }

    // Read current state
    uint16_t port_value = dev->port_state;
    bool current_level = (port_value & (1U << pin)) != 0;

    // Toggle: if input (1), make output low (0); if output low (0), make input (1)
    return pcf8575_write_pin(dev, pin, !current_level);
}

uint16_t pcf8575_get_cached_state(pcf8575_t *dev)
{
    if (dev == NULL) {
        return 0;
    }

    return dev->port_state;
}

bool pcf8575_is_initialized(pcf8575_t *dev)
{
    if (dev == NULL) {
        return false;
    }

    return dev->initialized;
}
