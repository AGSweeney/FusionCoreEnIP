#ifndef I2C_HANDLER_H
#define I2C_HANDLER_H

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "vl53l1x_types.h"

bool i2c_master_init(vl53l1x_i2c_handle_t *i2c_handle);
bool i2c_add_device(vl53l1x_device_handle_t *device);
bool i2c_update_address(const uint16_t dev, const uint8_t new_address);

uint8_t i2c_find_first_addr(vl53l1x_i2c_handle_t *i2c_handle);

bool i2c_write(const dev_handle_t *device, uint16_t reg, uint8_t value);
bool i2c_read(const dev_handle_t *device, uint16_t reg, uint8_t *buf, size_t len);

int8_t i2c_write_multi(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t i2c_read_multi(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t i2c_write_dword(uint16_t dev, uint16_t index, uint32_t data);
int8_t i2c_write_word(uint16_t dev, uint16_t index, uint16_t data);
int8_t i2c_write_byte(uint16_t dev, uint16_t index, uint8_t data);
int8_t i2c_read_byte(uint16_t dev, uint16_t index, uint8_t *pdata);
int8_t i2c_read_word(uint16_t dev, uint16_t index, uint16_t *pdata);
int8_t i2c_read_dword(uint16_t dev, uint16_t index, uint32_t *pdata);
#endif