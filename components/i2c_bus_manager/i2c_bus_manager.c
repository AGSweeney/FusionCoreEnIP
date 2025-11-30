#include "i2c_bus_manager.h"
#include "i2c_config.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include <string.h>

static const char *TAG = "i2c_bus_manager";

static i2c_master_bus_handle_t s_primary_bus = NULL;
static i2c_master_bus_handle_t s_secondary_bus = NULL;

static i2c_device_counts_t s_device_counts = {0};

static i2c_device_type_t identify_device_type(uint8_t address)
{
    switch (address) {
        case 0x18:
            return DEVICE_TYPE_ES8311;
        case 0x29:
            return DEVICE_TYPE_VL53L1X;
        case 0x2A:
            return DEVICE_TYPE_NAU7802;
        case 0x6A:
        case 0x6B:
            return DEVICE_TYPE_LSM6DS3;
        case 0x20:
        case 0x21:
        case 0x22:
        case 0x23:
        case 0x24:
        case 0x25:
        case 0x26:
        case 0x27:
            return DEVICE_TYPE_MCP230XX;
        case 0x58:
        case 0x59:
        case 0x5A:
        case 0x5B:
        case 0x5C:
        case 0x5D:
        case 0x5E:
        case 0x5F:
            return DEVICE_TYPE_GP8403;
        default:
            return DEVICE_TYPE_UNKNOWN;
    }
}

static const char* device_type_to_string(i2c_device_type_t type)
{
    switch (type) {
        case DEVICE_TYPE_ES8311: return "ES8311";
        case DEVICE_TYPE_VL53L1X: return "VL53L1X";
        case DEVICE_TYPE_NAU7802: return "NAU7802";
        case DEVICE_TYPE_LSM6DS3: return "LSM6DS3";
        case DEVICE_TYPE_MCP230XX: return "MCP230XX";
        case DEVICE_TYPE_GP8403: return "GP8403";
        default: return "Unknown";
    }
}

static void scan_bus(i2c_master_bus_handle_t bus_handle, const char* bus_name)
{
    if (bus_handle == NULL) {
        return;
    }

    ESP_LOGI(TAG, "Scanning %s I2C bus...", bus_name);
    int found_count = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        esp_err_t ret = i2c_master_probe(bus_handle, addr, 100);
        if (ret == ESP_OK) {
            i2c_device_type_t type = identify_device_type(addr);
            const char* type_str = device_type_to_string(type);

            if (type == DEVICE_TYPE_ES8311) {
                ESP_LOGI(TAG, "  Found device at 0x%02X: %s (recognized, skipping)", addr, type_str);
            } else if (type != DEVICE_TYPE_UNKNOWN) {
                ESP_LOGI(TAG, "  Found device at 0x%02X: %s", addr, type_str);
                found_count++;

                switch (type) {
                    case DEVICE_TYPE_VL53L1X:
                        s_device_counts.vl53l1x_count++;
                        break;
                    case DEVICE_TYPE_LSM6DS3:
                        s_device_counts.lsm6ds3_count++;
                        break;
                    case DEVICE_TYPE_NAU7802:
                        s_device_counts.nau7802_count++;
                        break;
                    case DEVICE_TYPE_MCP230XX:
                        s_device_counts.mcp230xx_count++;
                        break;
                    case DEVICE_TYPE_GP8403:
                        if (addr >= 0x58 && addr <= 0x5B) {
                            s_device_counts.gp8403_count++;
                        }
                        break;
                    default:
                        break;
                }
            } else {
                ESP_LOGI(TAG, "  Found device at 0x%02X: Unknown device", addr);
            }
        }
    }

    ESP_LOGI(TAG, "%s scan complete: %d supported device(s) found", bus_name, found_count);
}

esp_err_t i2c_bus_manager_init(void)
{
    esp_err_t ret;

    memset(&s_device_counts, 0, sizeof(s_device_counts));

    bool primary_pullup = system_i2c_primary_pullup_load();
    bool secondary_pullup = system_i2c_secondary_pullup_load();

    i2c_master_bus_config_t primary_bus_config = {
        .i2c_port = I2C_BUS_PRIMARY_NUM,
        .sda_io_num = I2C_PRIMARY_SDA_GPIO,
        .scl_io_num = I2C_PRIMARY_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = primary_pullup,
        },
    };

    ret = i2c_new_master_bus(&primary_bus_config, &s_primary_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize primary I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Primary I2C bus initialized (SDA: GPIO%d, SCL: GPIO%d, pullup: %s)", 
             I2C_PRIMARY_SDA_GPIO, I2C_PRIMARY_SCL_GPIO, primary_pullup ? "enabled" : "disabled");

    i2c_master_bus_config_t secondary_bus_config = {
        .i2c_port = I2C_BUS_SECONDARY_NUM,
        .sda_io_num = I2C_SECONDARY_SDA_GPIO,
        .scl_io_num = I2C_SECONDARY_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = secondary_pullup,
        },
    };

    ret = i2c_new_master_bus(&secondary_bus_config, &s_secondary_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize secondary I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Secondary I2C bus initialized (SDA: GPIO%d, SCL: GPIO%d, pullup: %s)", 
             I2C_SECONDARY_SDA_GPIO, I2C_SECONDARY_SCL_GPIO, secondary_pullup ? "enabled" : "disabled");

    scan_bus(s_primary_bus, "Primary");
    scan_bus(s_secondary_bus, "Secondary");

    ESP_LOGI(TAG, "Device counts - VL53L1X: %d, LSM6DS3: %d, NAU7802: %d, MCP230XX: %d, GP8403: %d",
             s_device_counts.vl53l1x_count,
             s_device_counts.lsm6ds3_count,
             s_device_counts.nau7802_count,
             s_device_counts.mcp230xx_count,
             s_device_counts.gp8403_count);

    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_manager_get_primary_bus(void)
{
    return s_primary_bus;
}

i2c_master_bus_handle_t i2c_bus_manager_get_secondary_bus(void)
{
    return s_secondary_bus;
}

i2c_master_bus_handle_t i2c_bus_manager_get_bus(uint8_t bus_num)
{
    if (bus_num == I2C_BUS_PRIMARY_NUM) {
        return s_primary_bus;
    } else if (bus_num == I2C_BUS_SECONDARY_NUM) {
        return s_secondary_bus;
    }
    return NULL;
}

bool i2c_bus_manager_is_device_detected(uint8_t address, i2c_device_type_t type)
{
    i2c_master_bus_handle_t bus = NULL;
    
    if (address == 0x2A) {
        bus = s_primary_bus;
    } else {
        bus = s_secondary_bus;
    }

    if (bus == NULL) {
        return false;
    }

    esp_err_t ret = i2c_master_probe(bus, address, 100);
    if (ret == ESP_OK) {
        i2c_device_type_t detected_type = identify_device_type(address);
        return (detected_type == type);
    }

    return false;
}

void i2c_bus_manager_get_device_counts(i2c_device_counts_t *counts)
{
    if (counts != NULL) {
        memcpy(counts, &s_device_counts, sizeof(i2c_device_counts_t));
    }
}

