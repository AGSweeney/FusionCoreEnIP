#include "mcp23008_init.h"
#include "mcp23008.h"
#include "esp_log.h"

static const char *TAG = "mcp23008_init";

bool mcp23008_init_with_bus(i2c_master_bus_handle_t bus_handle,
                            uint8_t i2c_address,
                            mcp23008_t *dev)
{
    if (dev == NULL || bus_handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    // Probe for device
    esp_err_t err = i2c_master_probe(bus_handle, i2c_address, 1000);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "MCP23008 not found at address 0x%02X: %s", 
                 i2c_address, esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "MCP23008 found at address 0x%02X", i2c_address);

    // Add device to bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = 400000,  // 400kHz
    };

    i2c_master_dev_handle_t dev_handle;
    err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MCP23008 device: %s", esp_err_to_name(err));
        return false;
    }

    mcp23008_config_t cfg = {
        .iodir = 0x00,      // All outputs
        .ipol = 0x00,       // Normal polarity
        .gpinten = 0x00,    // No interrupts
        .defval = 0x00,
        .intcon = 0x00,
        .iocon = 0x00,      // Default IOCON settings
        .gppu = 0x00,       // No pull-ups
    };

    if (!mcp23008_init(dev, dev_handle, &cfg)) {
        ESP_LOGE(TAG, "Failed to initialize MCP23008 registers");
        return false;
    }

    ESP_LOGI(TAG, "MCP23008 initialized successfully at address 0x%02X", i2c_address);
    return true;
}

