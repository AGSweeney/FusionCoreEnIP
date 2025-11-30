#include "mcp23017_init.h"
#include "mcp23017.h"
#include "esp_log.h"
#include "i2c_config.h"
#include "driver/i2c_master.h"

static const char *TAG = "mcp23017_init";

// Store bus handles for each I2C port (ESP32-P4 has 3 I2C ports)
#define MCP23017_MAX_I2C_PORTS 3
static i2c_master_bus_handle_t s_bus_handles[MCP23017_MAX_I2C_PORTS] = {NULL};

bool mcp23017_init_default(int scl_gpio, int sda_gpio, 
                           i2c_port_num_t i2c_port, 
                           mcp23017_t *dev)
{
    if (dev == NULL) {
        ESP_LOGE(TAG, "Invalid device pointer");
        return false;
    }

    // Check if bus is already initialized for this port (may have been initialized by scan)
    if (s_bus_handles[i2c_port] == NULL) {
        i2c_master_bus_config_t bus_config = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = i2c_port,
            .scl_io_num = scl_gpio,
            .sda_io_num = sda_gpio,
            .glitch_ignore_cnt = 7,
            .flags = {
                .enable_internal_pullup = i2c_config_get_internal_pullup_enabled(),  // System-wide I2C config
            }
        };

        esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handles[i2c_port]);
        if (err != ESP_OK) {
            // Bus might already be initialized by another component or hardware issue
            // This is expected behavior - silently fail without logging errors
            // The audio codec or other components may have initialized the bus
            ESP_LOGD(TAG, "I2C bus initialization failed (error: %s) - bus may be managed elsewhere", esp_err_to_name(err));
            return false;
        }
        ESP_LOGI(TAG, "I2C bus initialized on port %d", i2c_port);
    } else {
        ESP_LOGI(TAG, "Reusing existing I2C bus handle for port %d", i2c_port);
    }

    // First try default address, then scan if not found
    uint8_t device_address = MCP23017_I2C_ADDR_DEFAULT;
    esp_err_t err = i2c_master_probe(s_bus_handles[i2c_port], device_address, 1000);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MCP23017 not found at default address 0x%02X, scanning...", 
                 MCP23017_I2C_ADDR_DEFAULT);
        
        // Scan for MCP23017 at any address
        if (!mcp23017_find_device(s_bus_handles[i2c_port], scl_gpio, sda_gpio, 
                                  i2c_port, &device_address)) {
            ESP_LOGW(TAG, "MCP23017 not found on I2C bus (checked 0x20-0x27)");
            return false;
        }
    } else {
        ESP_LOGI(TAG, "MCP23017 found at default address 0x%02X", device_address);
    }

    // Add device to bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = 400000,  // 400kHz
    };

    i2c_master_dev_handle_t dev_handle;
    err = i2c_master_bus_add_device(s_bus_handles[i2c_port], &dev_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MCP23017 device: %s", esp_err_to_name(err));
        return false;
    }

    mcp23017_config_t cfg = {
        .iodir_a = 0x00,      // All outputs on Port A
        .iodir_b = 0x00,      // All outputs on Port B
        .ipol_a = 0x00,       // Normal polarity
        .ipol_b = 0x00,       // Normal polarity
        .gpinten_a = 0x00,   // No interrupts
        .gpinten_b = 0x00,   // No interrupts
        .defval_a = 0x00,
        .defval_b = 0x00,
        .intcon_a = 0x00,
        .intcon_b = 0x00,
        .iocon = 0x00,        // Default IOCON settings
        .gppu_a = 0x00,       // No pull-ups
        .gppu_b = 0x00,       // No pull-ups
    };

    if (!mcp23017_init(dev, dev_handle, &cfg)) {
        ESP_LOGE(TAG, "Failed to initialize MCP23017 registers");
        return false;
    }

    ESP_LOGI(TAG, "MCP23017 initialized successfully at address 0x%02X", device_address);
    return true;
}

i2c_master_bus_handle_t mcp23017_get_bus_handle(i2c_port_num_t i2c_port)
{
    if (i2c_port >= MCP23017_MAX_I2C_PORTS) {
        return NULL;
    }
    return s_bus_handles[i2c_port];
}

bool mcp23017_init_with_bus(i2c_master_bus_handle_t bus_handle,
                            uint8_t i2c_address,
                            mcp23017_t *dev)
{
    if (dev == NULL || bus_handle == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    // Probe for device
    esp_err_t err = i2c_master_probe(bus_handle, i2c_address, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCP23017 not found at address 0x%02X: %s", 
                 i2c_address, esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "MCP23017 found at address 0x%02X", i2c_address);

    // Add device to bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = 400000,  // 400kHz
    };

    i2c_master_dev_handle_t dev_handle;
    err = i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MCP23017 device: %s", esp_err_to_name(err));
        return false;
    }

    mcp23017_config_t cfg = {
        .iodir_a = 0x00,      // All outputs on Port A
        .iodir_b = 0x00,      // All outputs on Port B
        .ipol_a = 0x00,
        .ipol_b = 0x00,
        .gpinten_a = 0x00,
        .gpinten_b = 0x00,
        .defval_a = 0x00,
        .defval_b = 0x00,
        .intcon_a = 0x00,
        .intcon_b = 0x00,
        .iocon = 0x00,
        .gppu_a = 0x00,
        .gppu_b = 0x00,
    };

    if (!mcp23017_init(dev, dev_handle, &cfg)) {
        ESP_LOGE(TAG, "Failed to initialize MCP23017 registers");
        return false;
    }

    ESP_LOGI(TAG, "MCP23017 initialized successfully at address 0x%02X", i2c_address);
    return true;
}

int mcp23017_scan_i2c_bus(i2c_master_bus_handle_t bus_handle,
                          int scl_gpio,
                          int sda_gpio,
                          i2c_port_num_t i2c_port,
                          uint8_t *addresses,
                          int max_addresses)
{
    if (addresses == NULL || max_addresses <= 0) {
        ESP_LOGE(TAG, "Invalid parameters for I2C scan");
        return -1;
    }

    // Use provided bus handle, or get existing one, or initialize new one
    if (bus_handle == NULL) {
        // Check if we already have a bus handle for this port
        if (s_bus_handles[i2c_port] != NULL) {
            bus_handle = s_bus_handles[i2c_port];
            ESP_LOGI(TAG, "Reusing existing I2C bus handle for port %d", i2c_port);
        } else {
            // Try to initialize new bus
            i2c_master_bus_config_t bus_config = {
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .i2c_port = i2c_port,
                .scl_io_num = scl_gpio,
                .sda_io_num = sda_gpio,
                .glitch_ignore_cnt = 7,
                .flags = {
                    .enable_internal_pullup = i2c_config_get_internal_pullup_enabled(),  // System-wide I2C config
                }
            };

            esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handles[i2c_port]);
            if (err != ESP_OK) {
                // Bus might already be initialized by another component or hardware issue
                // Common error codes: ESP_ERR_INVALID_STATE, ESP_FAIL, ESP_ERR_NOT_FOUND
                // Silently return 0 - this is expected if bus is managed elsewhere
                // Only log at debug level to avoid cluttering logs
                ESP_LOGD(TAG, "I2C bus initialization skipped (error: %s) - bus may be managed by another component", esp_err_to_name(err));
                return 0; // Return 0 devices found rather than error
            }
            bus_handle = s_bus_handles[i2c_port];
            ESP_LOGI(TAG, "I2C bus initialized for scanning on port %d", i2c_port);
        }
    }

    int found_count = 0;
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    // Scan addresses 0x08 to 0x77 (valid 7-bit I2C addresses)
    // Skip reserved addresses: 0x00-0x07 and 0x78-0x7F
    for (uint8_t addr = 0x08; addr < 0x78 && found_count < max_addresses; addr++) {
        esp_err_t err = i2c_master_probe(bus_handle, addr, 100); // 100ms timeout for scanning
        if (err == ESP_OK) {
            addresses[found_count] = addr;
            found_count++;
            
            // Identify known devices for better logging
            const char* device_name = "";
            if (addr == 0x18) {
                device_name = " (Audio Codec)";
            } else if (addr >= 0x20 && addr <= 0x27) {
                device_name = " (Possible MCP23017/MCP23008)";
            } else if (addr == 0x29) {
                device_name = " (VL53L1X)";
            }
            
            ESP_LOGI(TAG, "  Found device at address 0x%02X%s", addr, device_name);
        }
    }

    if (found_count == 0) {
        ESP_LOGW(TAG, "No I2C devices found on bus");
    } else {
        ESP_LOGI(TAG, "Found %d device(s) on I2C bus", found_count);
    }

    return found_count;
}

bool mcp23017_find_device(i2c_master_bus_handle_t bus_handle,
                          int scl_gpio,
                          int sda_gpio,
                          i2c_port_num_t i2c_port,
                          uint8_t *found_address)
{
    if (found_address == NULL) {
        ESP_LOGE(TAG, "Invalid found_address pointer");
        return false;
    }

    *found_address = 0;

    // Initialize bus if needed
    if (bus_handle == NULL) {
        if (s_bus_handles[i2c_port] == NULL) {
            i2c_master_bus_config_t bus_config = {
                .clk_source = I2C_CLK_SRC_DEFAULT,
                .i2c_port = i2c_port,
                .scl_io_num = scl_gpio,
                .sda_io_num = sda_gpio,
                .glitch_ignore_cnt = 7,
                .flags = {
                    .enable_internal_pullup = i2c_config_get_internal_pullup_enabled(),  // System-wide I2C config
                }
            };

            esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handles[i2c_port]);
            if (err != ESP_OK) {
                // Bus initialization failed - expected if bus is managed elsewhere
                ESP_LOGD(TAG, "I2C bus initialization failed (error: %s) - bus may be managed elsewhere", esp_err_to_name(err));
                return false;
            }
        }
        bus_handle = s_bus_handles[i2c_port];
    }

    // MCP23017 can be at addresses 0x20-0x27 (A0, A1, A2 pins)
    // Scan all possible addresses
    for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
        esp_err_t err = i2c_master_probe(bus_handle, addr, 1000);
        if (err == ESP_OK) {
            // Try to verify it's an MCP23017 by reading a register
            // We'll do a simple probe - if it responds, assume it's valid
            *found_address = addr;
            ESP_LOGI(TAG, "MCP23017 found at address 0x%02X", addr);
            return true;
        }
    }

    ESP_LOGW(TAG, "No MCP23017 device found on I2C bus (checked addresses 0x20-0x27)");
    return false;
}

