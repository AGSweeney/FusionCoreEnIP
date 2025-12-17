/**
 * @file abz_encoder.h
 * @brief ABZ Rotary Encoder Driver - Quadrature Decoding Library
 * 
 * This component provides quadrature decoding for ABZ rotary encoders.
 * It implements a state machine to decode A/B channel quadrature signals
 * and track position, direction, and index pulse detection.
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * To enable it, add the component to CMakeLists.txt and integrate into main.c
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

#ifndef ABZ_ENCODER_H
#define ABZ_ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Encoder resolution mode
 */
typedef enum {
    ABZ_ENCODER_RESOLUTION_1X = 1,  /**< 1x resolution - count on A channel edges only */
    ABZ_ENCODER_RESOLUTION_4X = 4    /**< 4x resolution - count on all A/B edges */
} abz_encoder_resolution_t;

/**
 * @brief Encoder direction
 */
typedef enum {
    ABZ_ENCODER_DIRECTION_FORWARD = 1,   /**< Forward/clockwise rotation */
    ABZ_ENCODER_DIRECTION_REVERSE = -1   /**< Reverse/counter-clockwise rotation */
} abz_encoder_direction_t;

/**
 * @brief Encoder handle structure
 */
typedef struct {
    int32_t position;              /**< Current position count (signed 32-bit) */
    abz_encoder_resolution_t resolution; /**< Resolution mode (1x or 4x) */
    uint8_t state;                 /**< Current quadrature state (0-3) */
    bool index_detected;           /**< Index pulse detected flag */
    abz_encoder_direction_t last_direction; /**< Last movement direction */
    bool initialized;              /**< Initialization flag */
} abz_encoder_t;

/**
 * @brief Initialize encoder handle
 * 
 * @param encoder Pointer to encoder handle structure
 * @param resolution Resolution mode (1x or 4x)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_init(abz_encoder_t *encoder, abz_encoder_resolution_t resolution);

/**
 * @brief Process quadrature state change
 * 
 * This function should be called from the interrupt handler or task
 * when A or B channel changes state. It decodes the quadrature signal
 * and updates the position counter.
 * 
 * @param encoder Pointer to encoder handle
 * @param a_state Current A channel state (0 or 1)
 * @param b_state Current B channel state (0 or 1)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_process_quadrature(abz_encoder_t *encoder, bool a_state, bool b_state);

/**
 * @brief Process index pulse
 * 
 * Call this when Z channel index pulse is detected (typically rising edge).
 * 
 * @param encoder Pointer to encoder handle
 * @param reset_position If true, reset position counter to 0
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_process_index(abz_encoder_t *encoder, bool reset_position);

/**
 * @brief Get current position
 * 
 * @param encoder Pointer to encoder handle
 * @return int32_t Current position count
 */
int32_t abz_encoder_get_position(const abz_encoder_t *encoder);

/**
 * @brief Set position (for calibration or reset)
 * 
 * @param encoder Pointer to encoder handle
 * @param position New position value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_set_position(abz_encoder_t *encoder, int32_t position);

/**
 * @brief Reset position to zero
 * 
 * @param encoder Pointer to encoder handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_reset_position(abz_encoder_t *encoder);

/**
 * @brief Get index pulse detected flag
 * 
 * @param encoder Pointer to encoder handle
 * @return true if index pulse was detected
 */
bool abz_encoder_get_index_detected(const abz_encoder_t *encoder);

/**
 * @brief Clear index pulse detected flag
 * 
 * @param encoder Pointer to encoder handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t abz_encoder_clear_index(abz_encoder_t *encoder);

/**
 * @brief Get last movement direction
 * 
 * @param encoder Pointer to encoder handle
 * @return abz_encoder_direction_t Last direction (FORWARD or REVERSE)
 */
abz_encoder_direction_t abz_encoder_get_direction(const abz_encoder_t *encoder);

/**
 * @brief Calculate velocity from position delta
 * 
 * Helper function to calculate velocity in counts per second.
 * 
 * @param position_delta Change in position
 * @param time_delta_ms Time delta in milliseconds
 * @return int16_t Velocity in counts per second
 */
int16_t abz_encoder_calculate_velocity(int32_t position_delta, uint32_t time_delta_ms);

#ifdef __cplusplus
}
#endif

#endif // ABZ_ENCODER_H
