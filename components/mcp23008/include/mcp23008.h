#ifndef MCP23008_H
#define MCP23008_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCP23008_I2C_ADDR_DEFAULT 0x20

#define MCP23008_REG_IODIR   0x00
#define MCP23008_REG_IPOL    0x01
#define MCP23008_REG_GPINTEN 0x02
#define MCP23008_REG_DEFVAL  0x03
#define MCP23008_REG_INTCON  0x04
#define MCP23008_REG_IOCON   0x05
#define MCP23008_REG_GPPU    0x06
#define MCP23008_REG_INTF    0x07
#define MCP23008_REG_INTCAP  0x08
#define MCP23008_REG_GPIO    0x09
#define MCP23008_REG_OLAT    0x0A

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} mcp23008_t;

typedef struct {
    uint8_t iodir;
    uint8_t ipol;
    uint8_t gpinten;
    uint8_t defval;
    uint8_t intcon;
    uint8_t iocon;
    uint8_t gppu;
} mcp23008_config_t;

bool mcp23008_init(mcp23008_t *dev,
                   i2c_master_dev_handle_t i2c_dev,
                   const mcp23008_config_t *cfg);

esp_err_t mcp23008_write_register(mcp23008_t *dev, uint8_t reg, uint8_t value);
esp_err_t mcp23008_read_register(mcp23008_t *dev, uint8_t reg, uint8_t *value);

esp_err_t mcp23008_write_gpio(mcp23008_t *dev, uint8_t value);
esp_err_t mcp23008_read_gpio(mcp23008_t *dev, uint8_t *value);
esp_err_t mcp23008_set_direction(mcp23008_t *dev, uint8_t mask);
esp_err_t mcp23008_set_polarity(mcp23008_t *dev, uint8_t mask);
esp_err_t mcp23008_set_pullups(mcp23008_t *dev, uint8_t mask);
esp_err_t mcp23008_update_gpio_mask(mcp23008_t *dev, uint8_t mask, uint8_t value);
esp_err_t mcp23008_write_pin(mcp23008_t *dev, uint8_t pin, bool level);
esp_err_t mcp23008_read_pin(mcp23008_t *dev, uint8_t pin, bool *level);

#ifdef __cplusplus
}
#endif

#endif

