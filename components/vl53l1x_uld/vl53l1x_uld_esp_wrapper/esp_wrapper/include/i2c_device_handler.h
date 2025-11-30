#ifndef I2C_DEVICE_HANDLER_H
#define I2C_DEVICE_HANDLER_H

#include "i2c_handler.h"

typedef struct
{
    uint16_t dev;
    dev_handle_t dev_handle;
} device_key_t;

typedef struct
{
    device_key_t *keys;
    uint8_t count;
} device_map_t;

static const device_map_t DEVICE_MAP_INIT = {
    .keys = NULL,
    .count = 0};


void deinit_device_map();
void add_device_key(const uint16_t dev, const dev_handle_t handle);
int8_t remove_device_key(const uint16_t address);
void change_device_address(const uint16_t dev, const uint8_t new_address);
dev_handle_t get_device_handle(const uint16_t dev);

uint16_t create_dev(const i2c_port_num_t port, const uint8_t address);
uint8_t get_address(const uint16_t dev);
i2c_port_num_t get_port(const uint16_t dev);
#endif