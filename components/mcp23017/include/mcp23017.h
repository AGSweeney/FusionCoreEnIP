#ifndef MCP23017_H
#define MCP23017_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCP23017_I2C_ADDR_DEFAULT 0x20

/* Register addresses (BANK=0) */
#define MCP23017_REG_IODIRA   0x00
#define MCP23017_REG_IODIRB   0x01
#define MCP23017_REG_IPOLA    0x02
#define MCP23017_REG_IPOLB    0x03
#define MCP23017_REG_GPINTENA 0x04
#define MCP23017_REG_GPINTENB 0x05
#define MCP23017_REG_DEFVALA  0x06
#define MCP23017_REG_DEFVALB  0x07
#define MCP23017_REG_INTCONA  0x08
#define MCP23017_REG_INTCONB  0x09
#define MCP23017_REG_IOCON    0x0A
#define MCP23017_REG_GPPUA    0x0C
#define MCP23017_REG_GPPUB    0x0D
#define MCP23017_REG_INTFA    0x0E
#define MCP23017_REG_INTFB    0x0F
#define MCP23017_REG_INTCAPA  0x10
#define MCP23017_REG_INTCAPB  0x11
#define MCP23017_REG_GPIOA    0x12
#define MCP23017_REG_GPIOB    0x13
#define MCP23017_REG_OLATA    0x14
#define MCP23017_REG_OLATB    0x15

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} mcp23017_t;

typedef struct {
    uint8_t iodir_a;
    uint8_t iodir_b;
    uint8_t ipol_a;
    uint8_t ipol_b;
    uint8_t gpinten_a;
    uint8_t gpinten_b;
    uint8_t defval_a;
    uint8_t defval_b;
    uint8_t intcon_a;
    uint8_t intcon_b;
    uint8_t iocon;
    uint8_t gppu_a;
    uint8_t gppu_b;
} mcp23017_config_t;

bool mcp23017_init(mcp23017_t *dev,
                   i2c_master_dev_handle_t i2c_dev,
                   const mcp23017_config_t *cfg);

esp_err_t mcp23017_write_register(mcp23017_t *dev, uint8_t reg, uint8_t value);
esp_err_t mcp23017_read_register(mcp23017_t *dev, uint8_t reg, uint8_t *value);

esp_err_t mcp23017_write_gpio(mcp23017_t *dev, uint16_t value);
esp_err_t mcp23017_read_gpio(mcp23017_t *dev, uint16_t *value);
esp_err_t mcp23017_set_direction(mcp23017_t *dev, uint16_t mask);
esp_err_t mcp23017_set_polarity(mcp23017_t *dev, uint16_t mask);
esp_err_t mcp23017_set_pullups(mcp23017_t *dev, uint16_t mask);
esp_err_t mcp23017_update_gpio_mask(mcp23017_t *dev, uint16_t mask, uint16_t value);
esp_err_t mcp23017_write_pin(mcp23017_t *dev, uint8_t pin, bool level);
esp_err_t mcp23017_read_pin(mcp23017_t *dev, uint8_t pin, bool *level);

#ifdef __cplusplus
}
#endif

#endif

