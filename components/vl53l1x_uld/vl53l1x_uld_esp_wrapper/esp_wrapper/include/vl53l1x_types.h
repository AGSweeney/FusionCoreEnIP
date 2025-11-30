#ifndef VL53L1X_TYPES_H
#define VL53L1X_TYPES_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"

typedef i2c_master_dev_handle_t dev_handle_t;

typedef struct
{
    i2c_port_num_t i2c_port;
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
    bool initialized;
    i2c_master_bus_handle_t bus_handle;
} vl53l1x_i2c_handle_t;

typedef struct
{
    bool initialized;
    vl53l1x_i2c_handle_t *i2c_handle;
} vl53l1x_handle_t;

typedef struct
{
    vl53l1x_handle_t *vl53l1x_handle;
    gpio_num_t xshut_gpio;
    uint8_t i2c_address;
    gpio_num_t interrupt_gpio;
    uint32_t scl_speed_hz;

    dev_handle_t dev_handle;
    uint16_t dev;
    // range mode
    // capture mode
} vl53l1x_device_handle_t;

static const uint8_t VL53L1X_DEFAULT_I2C_ADDRESS = 0x29;

static const vl53l1x_handle_t VL53L1X_INIT = {
    .initialized = false,
    .i2c_handle = NULL,
};

static const vl53l1x_device_handle_t VL53L1X_DEVICE_INIT = {
    .vl53l1x_handle = NULL,
    .xshut_gpio = GPIO_NUM_NC,
    .i2c_address = VL53L1X_DEFAULT_I2C_ADDRESS,
    .interrupt_gpio = GPIO_NUM_NC,
    .scl_speed_hz = 400000,
    .dev_handle = NULL,
    .dev = 0,
};

static const vl53l1x_i2c_handle_t VL53L1X_I2C_INIT = {
    .i2c_port = I2C_NUM_0,
    .sda_gpio = GPIO_NUM_NC,
    .scl_gpio = GPIO_NUM_NC,
    .initialized = false,
    .bus_handle = NULL,
};
#endif