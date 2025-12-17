/**
 * @file abz_encoder_manager.h
 * @brief ABZ Encoder Manager - GPIO Interrupt Handler and Assembly Integration
 * 
 * This component manages ABZ encoder GPIO interrupts, processes quadrature
 * signals, calculates velocity, and updates EtherNet/IP assembly data.
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * To enable it:
 * 1. Add component to build system
 * 2. Call abz_encoder_manager_init() in main.c
 * 3. Configure GPIO pins via Kconfig or NVS
 * 
 * @copyright Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ABZ_ENCODER_MANAGER_H
#define ABZ_ENCODER_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"
#include "abz_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encoder configuration structure
 */
typedef struct {
    int gpio_a;                      /**< GPIO pin for A channel */
    int gpio_b;                      /**< GPIO pin for B channel */
    int gpio_z;                      /**< GPIO pin for Z channel (index) */
    abz_encoder_resolution_t resolution; /**< Resolution mode (1x or 4x) */
    uint32_t velocity_interval_ms;   /**< Velocity calculation interval (ms) */
    bool index_reset_position;       /**< Reset position on index pulse */
    bool enabled;                   /**< Encoder enabled flag */
} abz_encoder_config_t;

/**
 * @brief Initialize encoder manager
 * 
 * This function:
 * - Loads configuration from NVS
 * - Configures GPIO interrupts for A, B, Z channels
 * - Creates task for processing encoder events
 * - Initializes encoder driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_manager_init(void);

/**
 * @brief Check if encoder manager is initialized
 * 
 * @return true if initialized
 */
bool abz_encoder_manager_is_initialized(void);

/**
 * @brief Get encoder configuration
 * 
 * @param config Pointer to configuration structure to fill
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_manager_get_config(abz_encoder_config_t *config);

/**
 * @brief Set encoder configuration
 * 
 * @param config Pointer to configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_manager_set_config(const abz_encoder_config_t *config);

/**
 * @brief Get current encoder position
 * 
 * @return int32_t Current position count
 */
int32_t abz_encoder_manager_get_position(void);

/**
 * @brief Reset encoder position to zero
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_manager_reset_position(void);

/**
 * @brief Get current encoder velocity
 * 
 * @return int16_t Velocity in counts per second
 */
int16_t abz_encoder_manager_get_velocity(void);

/**
 * @brief Get index pulse detected flag
 * 
 * @return true if index pulse was detected
 */
bool abz_encoder_manager_get_index_detected(void);

/**
 * @brief Get encoder direction
 * 
 * @return abz_encoder_direction_t Last movement direction
 */
abz_encoder_direction_t abz_encoder_manager_get_direction(void);

#ifdef __cplusplus
}
#endif

#endif // ABZ_ENCODER_MANAGER_H
