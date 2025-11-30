#ifndef MCP23017_INIT_H
#define MCP23017_INIT_H

#include "mcp23017.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize MCP23017 on default I2C address (0x20)
 * 
 * This function initializes the I2C bus (if not already initialized) and
 * sets up the MCP23017 with default configuration:
 * - Port A (GPA0-7): All inputs
 * - Port B (GPB0-7): All outputs
 * - No pull-ups, no interrupts, normal polarity
 * 
 * @param scl_gpio GPIO number for I2C SCL line
 * @param sda_gpio GPIO number for I2C SDA line
 * @param i2c_port I2C port number (default: I2C_NUM_0)
 * @param dev Pointer to mcp23017_t structure to initialize
 * @return true if initialization successful, false otherwise
 */
bool mcp23017_init_default(int scl_gpio, int sda_gpio, 
                           i2c_port_num_t i2c_port, 
                           mcp23017_t *dev);

/**
 * @brief Get the I2C bus handle (for sharing with other devices)
 * 
 * @param i2c_port I2C port number
 * @return i2c_master_bus_handle_t or NULL if not initialized
 */
i2c_master_bus_handle_t mcp23017_get_bus_handle(i2c_port_num_t i2c_port);

/**
 * @brief Initialize MCP23017 using an existing I2C bus handle
 * 
 * Use this if you already have an I2C bus initialized (e.g., shared with VL53L1X)
 * 
 * @param bus_handle Existing I2C bus handle
 * @param i2c_address I2C address (default: MCP23017_I2C_ADDR_DEFAULT = 0x20)
 * @param dev Pointer to mcp23017_t structure to initialize
 * @return true if initialization successful, false otherwise
 */
bool mcp23017_init_with_bus(i2c_master_bus_handle_t bus_handle,
                            uint8_t i2c_address,
                            mcp23017_t *dev);

/**
 * @brief Scan I2C bus for devices
 * 
 * Scans the I2C bus and returns a list of found device addresses.
 * 
 * @param bus_handle I2C bus handle (can be NULL, will initialize if needed)
 * @param scl_gpio GPIO number for I2C SCL line (used if bus_handle is NULL)
 * @param sda_gpio GPIO number for I2C SDA line (used if bus_handle is NULL)
 * @param i2c_port I2C port number (used if bus_handle is NULL)
 * @param addresses Output array to store found addresses
 * @param max_addresses Maximum number of addresses to store
 * @return Number of devices found, or -1 on error
 */
int mcp23017_scan_i2c_bus(i2c_master_bus_handle_t bus_handle,
                          int scl_gpio,
                          int sda_gpio,
                          i2c_port_num_t i2c_port,
                          uint8_t *addresses,
                          int max_addresses);

/**
 * @brief Find MCP23017 device on I2C bus
 * 
 * Scans for MCP23017 devices at addresses 0x20-0x27 (all possible address combinations)
 * 
 * @param bus_handle I2C bus handle (can be NULL, will initialize if needed)
 * @param scl_gpio GPIO number for I2C SCL line (used if bus_handle is NULL)
 * @param sda_gpio GPIO number for I2C SDA line (used if bus_handle is NULL)
 * @param i2c_port I2C port number (used if bus_handle is NULL)
 * @param found_address Output parameter for found address (0 if not found)
 * @return true if MCP23017 found, false otherwise
 */
bool mcp23017_find_device(i2c_master_bus_handle_t bus_handle,
                          int scl_gpio,
                          int sda_gpio,
                          i2c_port_num_t i2c_port,
                          uint8_t *found_address);

#ifdef __cplusplus
}
#endif

#endif // MCP23017_INIT_H

