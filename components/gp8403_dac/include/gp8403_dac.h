/**
 * @file gp8403_dac.h
 * @brief DFRobot Gravity 2-Channel I2C DAC Module (0-10V) Driver
 * 
 * This driver provides an interface to control the GP8403-based 2-channel
 * 0-10V DAC module from DFRobot (SKU: DFR0971).
 * 
 * This implementation is based on and references the DFRobot_GP8403 Arduino library:
 * https://github.com/DFRobot/DFRobot_GP8403
 * 
 * Features:
 * - 2 independent channels (VOUT0, VOUT1)
 * - 0-10V output range per channel
 * - 12-bit resolution (0x000 - 0xFFF)
 * - 8 configurable I2C addresses via DIP switch
 * - Output voltage error < 0.5%
 * - Linearity error < 0.1%
 * - Output short-circuit protection
 * 
 * @note Product page: https://www.dfrobot.com/product-2613.html
 * @note Chip: GP8403
 * @note Based on DFRobot_GP8403 library by tangjie (jie.tang@dfrobot.com)
 */

#ifndef GP8403_DAC_H
#define GP8403_DAC_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GP8403 DAC channel selection
 */
typedef enum {
    GP8403_CHANNEL_0 = 0,  /**< Channel 0 (VOUT0) */
    GP8403_CHANNEL_1 = 1,  /**< Channel 1 (VOUT1) */
    GP8403_CHANNEL_MAX     /**< Maximum channel number */
} gp8403_channel_t;

/**
 * @brief GP8403 DAC configuration structure
 */
typedef struct {
    i2c_master_bus_handle_t bus_handle;  /**< I2C master bus handle */
    uint8_t i2c_addr;                     /**< I2C device address (default: 0x58) */
} gp8403_dac_config_t;

/**
 * @brief GP8403 DAC device handle
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle;  /**< I2C device handle */
    uint8_t i2c_addr;                     /**< I2C address */
    bool initialized;                     /**< Initialization status */
    float channel0_voltage;               /**< Last set voltage for channel 0 */
    float channel1_voltage;               /**< Last set voltage for channel 1 */
} gp8403_dac_handle_t;

/**
 * @brief Default I2C address for GP8403 DAC
 * 
 * Base address is 0x58. Address can be changed via DIP switch:
 * - Address range: 0x58 - 0x5F (8 addresses)
 * - Allows cascading up to 16 devices (2 channels × 8 addresses)
 */
#define GP8403_DAC_DEFAULT_I2C_ADDR    0x58

/**
 * @brief Maximum output voltage (V)
 */
#define GP8403_DAC_MAX_VOLTAGE         10.0f

/**
 * @brief Minimum output voltage (V)
 */
#define GP8403_DAC_MIN_VOLTAGE         0.0f

/**
 * @brief DAC resolution (12-bit)
 */
#define GP8403_DAC_RESOLUTION          4096  // 2^12

/**
 * @brief Maximum DAC value (12-bit)
 */
#define GP8403_DAC_MAX_VALUE           0xFFF

/**
 * @brief Initialize GP8403 DAC device
 * 
 * Initializes the GP8403 DAC module and creates an I2C device handle.
 * 
 * @param config Configuration structure with I2C bus handle and address
 * @param handle Pointer to store the device handle
 * @return true if initialization successful, false otherwise
 * 
 * @note The I2C bus must be initialized before calling this function
 * @note Default I2C address is 0x58 (can be changed via DIP switch)
 */
bool gp8403_dac_init(const gp8403_dac_config_t *config, gp8403_dac_handle_t **handle);

/**
 * @brief Deinitialize GP8403 DAC device
 * 
 * Releases the I2C device handle and frees resources.
 * 
 * @param handle Device handle (will be set to NULL after deinit)
 */
void gp8403_dac_deinit(gp8403_dac_handle_t **handle);

/**
 * @brief Set output voltage for a channel
 * 
 * Sets the output voltage for the specified channel. The voltage is
 * converted to a 12-bit DAC value internally.
 * 
 * @param handle Device handle
 * @param channel Channel number (GP8403_CHANNEL_0 or GP8403_CHANNEL_1)
 * @param voltage Output voltage in volts (0.0 to 10.0)
 * @return true if set successfully, false on error
 * 
 * @note Voltage values outside 0-10V range will be clamped
 * @note Output voltage error: < 0.5%
 * @note Linearity error: < 0.1%
 */
bool gp8403_dac_set_voltage(gp8403_dac_handle_t *handle, gp8403_channel_t channel, float voltage);

/**
 * @brief Set output voltage using raw DAC value
 * 
 * Sets the output voltage using a raw 12-bit DAC value (0x000 - 0xFFF).
 * 
 * @param handle Device handle
 * @param channel Channel number (GP8403_CHANNEL_0 or GP8403_CHANNEL_1)
 * @param dac_value Raw DAC value (0x000 - 0xFFF)
 * @return true if set successfully, false on error
 * 
 * @note DAC value is clamped to 0x000 - 0xFFF range
 * @note Formula: Voltage = (dac_value / 0xFFF) × 10.0V
 */
bool gp8403_dac_set_raw(gp8403_dac_handle_t *handle, gp8403_channel_t channel, uint16_t dac_value);

/**
 * @brief Get last set voltage for a channel
 * 
 * Returns the last voltage value that was successfully set for the channel.
 * This does not read back from the device, but returns the cached value.
 * 
 * @param handle Device handle
 * @param channel Channel number (GP8403_CHANNEL_0 or GP8403_CHANNEL_1)
 * @return Last set voltage in volts, or 0.0 if not set or invalid channel
 */
float gp8403_dac_get_voltage(gp8403_dac_handle_t *handle, gp8403_channel_t channel);

/**
 * @brief Set both channels to the same voltage
 * 
 * Convenience function to set both channels to the same voltage value.
 * 
 * @param handle Device handle
 * @param voltage Output voltage in volts (0.0 to 10.0)
 * @return true if both channels set successfully, false on error
 */
bool gp8403_dac_set_both_channels(gp8403_dac_handle_t *handle, float voltage);

/**
 * @brief Set both channels independently
 * 
 * Sets both channels to different voltage values in a single operation.
 * 
 * @param handle Device handle
 * @param voltage0 Voltage for channel 0 (0.0 to 10.0)
 * @param voltage1 Voltage for channel 1 (0.0 to 10.0)
 * @return true if both channels set successfully, false on error
 */
bool gp8403_dac_set_channels(gp8403_dac_handle_t *handle, float voltage0, float voltage1);

/**
 * @brief Convert voltage to raw DAC value
 * 
 * Converts a voltage value (0-10V) to a 12-bit DAC value (0x000-0xFFF).
 * 
 * @param voltage Voltage in volts (0.0 to 10.0)
 * @return Raw DAC value (0x000 - 0xFFF)
 */
uint16_t gp8403_dac_voltage_to_raw(float voltage);

/**
 * @brief Convert raw DAC value to voltage
 * 
 * Converts a 12-bit DAC value (0x000-0xFFF) to voltage (0-10V).
 * 
 * @param dac_value Raw DAC value (0x000 - 0xFFF)
 * @return Voltage in volts (0.0 to 10.0)
 */
float gp8403_dac_raw_to_voltage(uint16_t dac_value);

/**
 * @brief Check if device is initialized
 * 
 * @param handle Device handle
 * @return true if initialized, false otherwise
 */
bool gp8403_dac_is_initialized(gp8403_dac_handle_t *handle);

/**
 * @brief Get I2C address of the device
 * 
 * @param handle Device handle
 * @return I2C address, or 0 if handle is invalid
 */
uint8_t gp8403_dac_get_i2c_addr(gp8403_dac_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // GP8403_DAC_H

