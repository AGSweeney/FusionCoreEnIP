#ifndef PCF8575_H
#define PCF8575_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file pcf8575.h
 * @brief PCF8575 16-bit I2C I/O Expander Driver
 * 
 * This driver provides full-featured support for the PCF8575 16-bit I/O expander.
 * The PCF8575 features quasi-bidirectional I/O pins that can be configured as
 * inputs or outputs.
 * 
 * **Status: Scaffolded - NOT included in build**
 * 
 * @copyright Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 */

#define PCF8575_I2C_ADDR_BASE 0x20  /**< Base I2C address (A0=A1=A2=0) */
#define PCF8575_I2C_ADDR_DEFAULT 0x20  /**< Default I2C address */

/**
 * @brief PCF8575 device handle
 */
typedef struct {
    i2c_master_dev_handle_t i2c_dev;  /**< I2C device handle */
    uint16_t port_state;              /**< Cached port state (for read-modify-write) */
    bool initialized;                 /**< Initialization flag */
} pcf8575_t;

/**
 * @brief PCF8575 configuration structure
 * 
 * The PCF8575 uses quasi-bidirectional I/O:
 * - Writing 1 to a pin makes it an input (high-impedance with weak pull-up)
 * - Writing 0 to a pin makes it an output (drives low)
 * - Reading always reads the current pin state
 * 
 * Initial port state: All pins set to 1 (inputs) by default
 */
typedef struct {
    uint16_t initial_port_state;  /**< Initial port state (0xFFFF = all inputs, 0x0000 = all outputs low) */
    int interrupt_gpio;            /**< GPIO pin for INT interrupt signal (-1 if not used) */
    bool interrupt_active_low;    /**< Interrupt polarity (true = active low, false = active high) */
} pcf8575_config_t;

/**
 * @brief Initialize PCF8575 device
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param i2c_dev I2C master device handle (from i2c_master_bus_add_device)
 * @param config Configuration structure (can be NULL for defaults)
 * @return true on success, false on failure
 */
bool pcf8575_init(pcf8575_t *dev, i2c_master_dev_handle_t i2c_dev, const pcf8575_config_t *config);

/**
 * @brief Deinitialize PCF8575 device
 * 
 * @param dev Pointer to PCF8575 device handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_deinit(pcf8575_t *dev);

/**
 * @brief Read all 16 I/O pins
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param port_value Pointer to store 16-bit port value (P0-P15)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_read_port(pcf8575_t *dev, uint16_t *port_value);

/**
 * @brief Write all 16 I/O pins
 * 
 * Note: Writing 1 to a pin makes it an input (high-impedance with pull-up)
 *       Writing 0 to a pin makes it an output (drives low)
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param port_value 16-bit port value to write (P0-P15)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_write_port(pcf8575_t *dev, uint16_t port_value);

/**
 * @brief Read a single pin
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param pin Pin number (0-15)
 * @param level Pointer to store pin level (true = high, false = low)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_read_pin(pcf8575_t *dev, uint8_t pin, bool *level);

/**
 * @brief Write a single pin
 * 
 * Note: Writing 1 makes the pin an input (high-impedance with pull-up)
 *       Writing 0 makes the pin an output (drives low)
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param pin Pin number (0-15)
 * @param level Pin level (true = input/high, false = output/low)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_write_pin(pcf8575_t *dev, uint8_t pin, bool level);

/**
 * @brief Update multiple pins using a mask
 * 
 * Updates only the pins specified in the mask, leaving others unchanged.
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param mask Bitmask of pins to update (1 = update, 0 = leave unchanged)
 * @param value New values for pins specified in mask
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_update_pins(pcf8575_t *dev, uint16_t mask, uint16_t value);

/**
 * @brief Set pin as input
 * 
 * Sets pin to input mode (high-impedance with weak pull-up) by writing 1.
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param pin Pin number (0-15)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_set_pin_input(pcf8575_t *dev, uint8_t pin);

/**
 * @brief Set pin as output
 * 
 * Sets pin to output mode. The pin will drive low (0) or high (1) based on value.
 * Note: To drive high, you must write 1, which makes it an input with pull-up.
 * For true output high, use external pull-up or use the quasi-bidirectional nature.
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param pin Pin number (0-15)
 * @param level Output level (true = high via pull-up, false = low output)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_set_pin_output(pcf8575_t *dev, uint8_t pin, bool level);

/**
 * @brief Configure multiple pins as inputs
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param mask Bitmask of pins to configure as inputs (1 = input, 0 = unchanged)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_set_pins_input(pcf8575_t *dev, uint16_t mask);

/**
 * @brief Configure multiple pins as outputs
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param mask Bitmask of pins to configure as outputs (1 = output, 0 = unchanged)
 * @param initial_value Initial output values for pins in mask
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_set_pins_output(pcf8575_t *dev, uint16_t mask, uint16_t initial_value);

/**
 * @brief Toggle a pin
 * 
 * Toggles the state of a pin. If input, becomes output low. If output low, becomes input.
 * 
 * @param dev Pointer to PCF8575 device handle
 * @param pin Pin number (0-15)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t pcf8575_toggle_pin(pcf8575_t *dev, uint8_t pin);

/**
 * @brief Get cached port state
 * 
 * Returns the last known port state (from last read or write operation).
 * 
 * @param dev Pointer to PCF8575 device handle
 * @return uint16_t Cached port state
 */
uint16_t pcf8575_get_cached_state(pcf8575_t *dev);

/**
 * @brief Check if device is initialized
 * 
 * @param dev Pointer to PCF8575 device handle
 * @return true if initialized, false otherwise
 */
bool pcf8575_is_initialized(pcf8575_t *dev);

#ifdef __cplusplus
}
#endif

#endif // PCF8575_H
