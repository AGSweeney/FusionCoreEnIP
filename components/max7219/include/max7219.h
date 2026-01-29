/**
 * @file max7219.h
 * @brief MAX7219 7-Segment Display Driver
 * 
 * This driver provides an interface to control the MAX7219 serial LED display driver
 * for driving 7-segment displays, LED matrices, and other LED displays.
 * 
 * Features:
 * - 8-digit 7-segment display support
 * - Cascadable (multiple chips daisy-chained)
 * - Brightness control (16 levels)
 * - Decode mode control (BCD or raw)
 * - Scan limit control (1-8 digits)
 * - Shutdown mode control
 * - Display test mode
 * - SPI communication interface
 * 
 * @note Chip: MAX7219
 * @note Interface: SPI
 */

#ifndef MAX7219_H
#define MAX7219_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MAX7219 register addresses
 */
typedef enum {
    MAX7219_REG_NOOP = 0x00,           /**< No operation */
    MAX7219_REG_DIGIT0 = 0x01,        /**< Digit 0 */
    MAX7219_REG_DIGIT1 = 0x02,        /**< Digit 1 */
    MAX7219_REG_DIGIT2 = 0x03,        /**< Digit 2 */
    MAX7219_REG_DIGIT3 = 0x04,        /**< Digit 3 */
    MAX7219_REG_DIGIT4 = 0x05,        /**< Digit 4 */
    MAX7219_REG_DIGIT5 = 0x06,        /**< Digit 5 */
    MAX7219_REG_DIGIT6 = 0x07,        /**< Digit 6 */
    MAX7219_REG_DIGIT7 = 0x08,        /**< Digit 7 */
    MAX7219_REG_DECODE_MODE = 0x09,   /**< Decode mode */
    MAX7219_REG_INTENSITY = 0x0A,     /**< Intensity (brightness) */
    MAX7219_REG_SCAN_LIMIT = 0x0B,    /**< Scan limit */
    MAX7219_REG_SHUTDOWN = 0x0C,      /**< Shutdown mode */
    MAX7219_REG_DISPLAY_TEST = 0x0F   /**< Display test */
} max7219_register_t;

/**
 * @brief MAX7219 decode mode
 */
typedef enum {
    MAX7219_DECODE_NONE = 0x00,       /**< No decode (raw segment control) */
    MAX7219_DECODE_DIGIT0 = 0x01,     /**< Decode digit 0 only */
    MAX7219_DECODE_DIGIT0_3 = 0x0F,   /**< Decode digits 0-3 */
    MAX7219_DECODE_ALL = 0xFF          /**< Decode all digits */
} max7219_decode_mode_t;

/**
 * @brief MAX7219 configuration structure
 */
typedef struct {
    spi_device_handle_t spi_device;   /**< SPI device handle */
    uint8_t num_devices;              /**< Number of cascaded devices (default: 1) */
} max7219_config_t;

/**
 * @brief MAX7219 device handle
 */
typedef struct {
    spi_device_handle_t spi_device;   /**< SPI device handle */
    uint8_t num_devices;              /**< Number of cascaded devices */
    bool initialized;                 /**< Initialization status */
    uint8_t intensity;                /**< Current intensity (0-15) */
    uint8_t scan_limit;               /**< Current scan limit (0-7) */
    bool shutdown;                    /**< Shutdown state */
} max7219_handle_t;

/**
 * @brief Maximum number of cascaded devices
 */
#define MAX7219_MAX_DEVICES           8

/**
 * @brief Maximum intensity level
 */
#define MAX7219_MAX_INTENSITY         15

/**
 * @brief Maximum scan limit (0-7 for 1-8 digits)
 */
#define MAX7219_MAX_SCAN_LIMIT        7

/**
 * @brief Number of digits per device
 */
#define MAX7219_DIGITS_PER_DEVICE     8

/**
 * @brief Initialize MAX7219 device
 * 
 * Initializes the MAX7219 display driver and configures default settings.
 * 
 * @param config Configuration structure with SPI device handle and number of cascaded devices
 * @param handle Pointer to store the device handle
 * @return true if initialization successful, false otherwise
 * 
 * @note The SPI device must be initialized before calling this function
 * @note Default settings: intensity=8, scan_limit=7 (all digits), decode_mode=none, shutdown=false
 */
bool max7219_init(const max7219_config_t *config, max7219_handle_t **handle);

/**
 * @brief Deinitialize MAX7219 device
 * 
 * Releases the SPI device handle and frees resources.
 * 
 * @param handle Device handle (will be set to NULL after deinit)
 */
void max7219_deinit(max7219_handle_t **handle);

/**
 * @brief Write to MAX7219 register
 * 
 * Writes a value to a register on all cascaded devices.
 * 
 * @param handle Device handle
 * @param reg Register address
 * @param value Register value
 * @return true if write successful, false otherwise
 */
bool max7219_write_register(max7219_handle_t *handle, max7219_register_t reg, uint8_t value);

/**
 * @brief Set display intensity (brightness)
 * 
 * Sets the display intensity level (0-15).
 * 
 * @param handle Device handle
 * @param intensity Intensity level (0-15, where 0=min, 15=max)
 * @return true if set successfully, false on error
 */
bool max7219_set_intensity(max7219_handle_t *handle, uint8_t intensity);

/**
 * @brief Set scan limit (number of digits to display)
 * 
 * Sets the number of digits to scan (1-8).
 * 
 * @param handle Device handle
 * @param scan_limit Scan limit (0-7, where 0=1 digit, 7=8 digits)
 * @return true if set successfully, false on error
 */
bool max7219_set_scan_limit(max7219_handle_t *handle, uint8_t scan_limit);

/**
 * @brief Set decode mode
 * 
 * Sets the decode mode for BCD decoding or raw segment control.
 * 
 * @param handle Device handle
 * @param decode_mode Decode mode (see max7219_decode_mode_t)
 * @return true if set successfully, false on error
 */
bool max7219_set_decode_mode(max7219_handle_t *handle, max7219_decode_mode_t decode_mode);

/**
 * @brief Set shutdown mode
 * 
 * Controls the shutdown state of the display.
 * 
 * @param handle Device handle
 * @param shutdown true to shutdown (display off), false to enable (display on)
 * @return true if set successfully, false on error
 */
bool max7219_set_shutdown(max7219_handle_t *handle, bool shutdown);

/**
 * @brief Enable display test mode
 * 
 * Turns on all segments for testing purposes.
 * 
 * @param handle Device handle
 * @param enable true to enable test mode, false to disable
 * @return true if set successfully, false on error
 */
bool max7219_set_display_test(max7219_handle_t *handle, bool enable);

/**
 * @brief Clear all digits
 * 
 * Clears all digits on all cascaded devices.
 * 
 * @param handle Device handle
 * @return true if cleared successfully, false on error
 */
bool max7219_clear_all(max7219_handle_t *handle);

/**
 * @brief Set digit value (BCD decode mode)
 * 
 * Sets a digit value when in BCD decode mode.
 * 
 * @param handle Device handle
 * @param device_index Device index in cascade (0 = first device)
 * @param digit Digit number (0-7)
 * @param value Value to display (0-9, or special codes for BCD decode)
 * @return true if set successfully, false on error
 */
bool max7219_set_digit(max7219_handle_t *handle, uint8_t device_index, uint8_t digit, uint8_t value);

/**
 * @brief Set digit segments (raw mode)
 * 
 * Sets raw segment data for a digit when not in decode mode.
 * 
 * @param handle Device handle
 * @param device_index Device index in cascade (0 = first device)
 * @param digit Digit number (0-7)
 * @param segments Segment bitmask (bit 0=DP, bit 1=G, bit 2=F, bit 3=E, bit 4=D, bit 5=C, bit 6=B, bit 7=A)
 * @return true if set successfully, false on error
 */
bool max7219_set_segments(max7219_handle_t *handle, uint8_t device_index, uint8_t digit, uint8_t segments);

/**
 * @brief Display a number
 * 
 * Displays a number across multiple digits with optional leading zeros.
 * 
 * @param handle Device handle
 * @param device_index Device index in cascade (0 = first device)
 * @param number Number to display
 * @param leading_zeros true to show leading zeros, false to blank leading digits
 * @return true if displayed successfully, false on error
 */
bool max7219_display_number(max7219_handle_t *handle, uint8_t device_index, int32_t number, bool leading_zeros);

/**
 * @brief Display a floating point number
 * 
 * Displays a floating point number with specified decimal places.
 * 
 * @param handle Device handle
 * @param device_index Device index in cascade (0 = first device)
 * @param number Number to display
 * @param decimal_places Number of decimal places (0-7)
 * @return true if displayed successfully, false on error
 */
bool max7219_display_float(max7219_handle_t *handle, uint8_t device_index, float number, uint8_t decimal_places);

/**
 * @brief Check if device is initialized
 * 
 * @param handle Device handle
 * @return true if initialized, false otherwise
 */
bool max7219_is_initialized(max7219_handle_t *handle);

/**
 * @brief Get current intensity level
 * 
 * @param handle Device handle
 * @return Current intensity level (0-15), or 0 if handle is invalid
 */
uint8_t max7219_get_intensity(max7219_handle_t *handle);

/**
 * @brief Get current scan limit
 * 
 * @param handle Device handle
 * @return Current scan limit (0-7), or 0 if handle is invalid
 */
uint8_t max7219_get_scan_limit(max7219_handle_t *handle);

/**
 * @brief Get shutdown state
 * 
 * @param handle Device handle
 * @return true if shutdown, false if enabled
 */
bool max7219_get_shutdown(max7219_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // MAX7219_H
