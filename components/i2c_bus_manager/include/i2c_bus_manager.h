#ifndef I2C_BUS_MANAGER_H
#define I2C_BUS_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_BUS_PRIMARY_NUM 0
#define I2C_BUS_SECONDARY_NUM 1

#define I2C_PRIMARY_SDA_GPIO 7
#define I2C_PRIMARY_SCL_GPIO 8
#define I2C_SECONDARY_SDA_GPIO 33
#define I2C_SECONDARY_SCL_GPIO 32

#define I2C_FREQ_HZ 400000

typedef enum {
    DEVICE_TYPE_UNKNOWN = 0,
    DEVICE_TYPE_ES8311 = 1,
    DEVICE_TYPE_VL53L1X = 2,
    DEVICE_TYPE_NAU7802 = 3,
    DEVICE_TYPE_LSM6DS3 = 4,
    DEVICE_TYPE_MCP230XX = 5,
    DEVICE_TYPE_GP8403 = 6,
} i2c_device_type_t;

typedef struct {
    uint8_t address;
    i2c_device_type_t type;
    bool detected;
} i2c_device_info_t;

typedef struct {
    uint8_t vl53l1x_count;
    uint8_t lsm6ds3_count;
    uint8_t nau7802_count;
    uint8_t mcp230xx_count;
    uint8_t gp8403_count;
} i2c_device_counts_t;

esp_err_t i2c_bus_manager_init(void);

i2c_master_bus_handle_t i2c_bus_manager_get_primary_bus(void);
i2c_master_bus_handle_t i2c_bus_manager_get_secondary_bus(void);

i2c_master_bus_handle_t i2c_bus_manager_get_bus(uint8_t bus_num);

bool i2c_bus_manager_is_device_detected(uint8_t address, i2c_device_type_t type);

void i2c_bus_manager_get_device_counts(i2c_device_counts_t *counts);

/**
 * @brief Enable or disable the secondary I2C bus at runtime
 * 
 * @param enabled True to enable, false to disable
 * @return esp_err_t ESP_OK on success
 * 
 * Note: This will initialize or deinitialize the secondary bus.
 * If disabling, all devices on the secondary bus will become unavailable.
 */
esp_err_t i2c_bus_manager_set_secondary_enabled(bool enabled);

/**
 * @brief Check if secondary I2C bus is currently enabled
 * 
 * @return true if enabled, false if disabled
 */
bool i2c_bus_manager_is_secondary_enabled(void);

#ifdef __cplusplus
}
#endif

#endif

