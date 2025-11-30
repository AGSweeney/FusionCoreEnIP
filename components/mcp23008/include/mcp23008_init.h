#ifndef MCP23008_INIT_H
#define MCP23008_INIT_H

#include "mcp23008.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MCP23008 on default I2C address (0x20)
 * 
 * @param bus_handle Existing I2C bus handle (must be initialized)
 * @param i2c_address I2C address (default: MCP23008_I2C_ADDR_DEFAULT = 0x20)
 * @param dev Pointer to mcp23008_t structure to initialize
 * @return true if initialization successful, false otherwise
 */
bool mcp23008_init_with_bus(i2c_master_bus_handle_t bus_handle,
                            uint8_t i2c_address,
                            mcp23008_t *dev);

#ifdef __cplusplus
}
#endif

#endif // MCP23008_INIT_H

