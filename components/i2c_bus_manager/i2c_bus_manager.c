#include "i2c_bus_manager.h"
#include "i2c_config.h"
#include "system_config.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
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

/**
 * @brief Perform I2C bus recovery by cycling SCL to clear stuck devices
 * 
 * This function manually toggles the SCL line to allow devices that are
 * stuck in a transaction to complete and release the bus. This is critical
 * after software resets where I2C devices remain powered but the ESP32 resets.
 * 
 * @param sda_gpio SDA GPIO number
 * @param scl_gpio SCL GPIO number
 * @param bus_name Name for logging
 */
static void i2c_bus_recovery(int sda_gpio, int scl_gpio, const char* bus_name)
{
    ESP_LOGI(TAG, "Performing I2C bus recovery for %s bus (SDA: GPIO%d, SCL: GPIO%d)", 
             bus_name, sda_gpio, scl_gpio);
    
    // Configure GPIO as open-drain outputs
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    // Reset and configure SCL
    io_conf.pin_bit_mask = (1ULL << scl_gpio);
    gpio_reset_pin(scl_gpio);
    gpio_config(&io_conf);
    
    // Reset and configure SDA
    io_conf.pin_bit_mask = (1ULL << sda_gpio);
    gpio_reset_pin(sda_gpio);
    gpio_config(&io_conf);
    
    // Set both lines high initially
    gpio_set_level(scl_gpio, 1);
    gpio_set_level(sda_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    // Check if SDA is stuck low (device holding bus)
    int sda_level = gpio_get_level(sda_gpio);
    if (sda_level == 0) {
        ESP_LOGW(TAG, "%s bus: SDA stuck low, cycling SCL to recover", bus_name);
        
        // Send up to 9 clock pulses to allow device to complete transaction
        for (int i = 0; i < 9; i++) {
            gpio_set_level(scl_gpio, 0);
            esp_rom_delay_us(5);  // 100kHz clock period
            gpio_set_level(scl_gpio, 1);
            esp_rom_delay_us(5);
            
            // Check if SDA is released
            if (gpio_get_level(sda_gpio) == 1) {
                ESP_LOGI(TAG, "%s bus: SDA released after %d clock pulses", bus_name, i + 1);
                break;
            }
        }
        
        // Final check
        if (gpio_get_level(sda_gpio) == 0) {
            ESP_LOGW(TAG, "%s bus: SDA still stuck low after recovery attempt", bus_name);
        }
    }
    
    // Send STOP condition to reset any partial transactions
    gpio_set_level(sda_gpio, 0);
    esp_rom_delay_us(5);
    gpio_set_level(scl_gpio, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sda_gpio, 1);
    esp_rom_delay_us(10);
    
    // If SDA is STILL stuck after recovery, it means devices are powered on
    // but not yet ready (still booting). Log warning but continue - devices
    // should stabilize shortly.
    sda_level = gpio_get_level(sda_gpio);
    if (sda_level == 0) {
        ESP_LOGW(TAG, "%s bus: SDA still low after recovery - devices may still be booting", bus_name);
        ESP_LOGW(TAG, "%s bus: This is normal after power-on, will retry during device init", bus_name);
    } else {
        ESP_LOGI(TAG, "%s bus: Recovery successful, bus is clean", bus_name);
    }
    
    ESP_LOGI(TAG, "%s bus recovery complete", bus_name);
}

esp_err_t i2c_bus_manager_init(void)
{
    esp_err_t ret;

    memset(&s_device_counts, 0, sizeof(s_device_counts));

    ESP_LOGI(TAG, "Waiting for I2C devices to power up and stabilize...");
    // CRITICAL: Wait for I2C devices to complete their own boot sequence
    // VL53L1X takes up to 400ms to boot and holds SDA low during boot!
    // LSM6DS3 takes ~50ms, NAU7802 takes ~20ms
    // Must wait for slowest device before attempting bus operations
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Perform bus recovery BEFORE initializing I2C driver
    // This clears any stuck devices from interrupted transactions during software reset
    i2c_bus_recovery(I2C_PRIMARY_SDA_GPIO, I2C_PRIMARY_SCL_GPIO, "Primary");
    i2c_bus_recovery(I2C_SECONDARY_SDA_GPIO, I2C_SECONDARY_SCL_GPIO, "Secondary");
    
    // Small delay to let devices stabilize after recovery
    vTaskDelay(pdMS_TO_TICKS(10));

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

