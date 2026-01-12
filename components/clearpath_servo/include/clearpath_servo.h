/**
 * @file clearpath_servo.h
 * @brief ClearPath Servo Driver - Step and Direction Control
 * 
 * This component provides step and direction control for Teknic ClearPath servos,
 * similar to the ClearCore library API. It handles step pulse generation,
 * position tracking, velocity control, and acceleration/deceleration limits.
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * To enable it, add the component to CMakeLists.txt and integrate into main.c
 * 
 * Attribution:
 * ------------
 * This implementation is inspired by and follows the API design patterns of the
 * Teknic ClearCore library (https://github.com/Teknic-Inc/ClearCore-library).
 * The API function names and concepts (Move, MoveVelocity, VelMax, AccelMax,
 * StepsComplete, HLFB) are based on the ClearCore library API for compatibility
 * and familiarity.
 * 
 * References:
 * - Teknic ClearCore Library: https://github.com/Teknic-Inc/ClearCore-library
 * - ClearCore Library Documentation: https://teknic-inc.github.io/ClearCore-library/
 * - ClearPath Servo User Manual: https://www.teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf
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

#ifndef CLEARPATH_SERVO_H
#define CLEARPATH_SERVO_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Move target type
 */
typedef enum {
    CLEARPATH_SERVO_MOVE_TARGET_RELATIVE = 0,  /**< Relative move from current position */
    CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE = 1   /**< Absolute move to target position */
} clearpath_servo_move_target_t;

/**
 * @brief HLFB (High Level Feedback) interpretation mode
 */
typedef enum {
    CLEARPATH_SERVO_HLFB_MODE_ENABLED = 0,      /**< HLFB indicates enabled state */
    CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE = 1, /**< HLFB indicates move complete */
    CLEARPATH_SERVO_HLFB_MODE_IN_POSITION = 2,  /**< HLFB indicates in position */
    CLEARPATH_SERVO_HLFB_MODE_FAULT = 3,        /**< HLFB indicates fault condition */
    CLEARPATH_SERVO_HLFB_MODE_CUSTOM = 4        /**< Custom HLFB interpretation */
} clearpath_servo_hlfb_mode_t;

/**
 * @brief Homing sensor type
 */
typedef enum {
    CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN = 0,   /**< NO: Sensor triggers when closed (HIGH) */
    CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_CLOSED = 1  /**< NC: Sensor triggers when opened (LOW) */
} clearpath_servo_homing_sensor_type_t;

/**
 * @brief Motion profile type
 */
typedef enum {
    CLEARPATH_SERVO_PROFILE_TRAPEZOIDAL = 0,  /**< Trapezoidal: constant acceleration/deceleration */
    CLEARPATH_SERVO_PROFILE_S_CURVE = 1       /**< S-curve: jerk-limited acceleration/deceleration */
} clearpath_servo_profile_type_t;

/**
 * @brief Servo state
 */
typedef enum {
    CLEARPATH_SERVO_STATE_IDLE = 0,
    CLEARPATH_SERVO_STATE_MOVING,
    CLEARPATH_SERVO_STATE_VELOCITY_MODE,
    CLEARPATH_SERVO_STATE_HOMING,
    CLEARPATH_SERVO_STATE_COORDINATED  /**< Coordinated motion (linear or circular interpolation) */
} clearpath_servo_state_t;

/**
 * @brief Servo configuration structure
 */
typedef struct {
    int gpio_step;          /**< GPIO pin for step signal */
    int gpio_dir;           /**< GPIO pin for direction signal */
    int gpio_enable;        /**< GPIO pin for enable signal (-1 if not used) */
    int gpio_hlfb;          /**< GPIO pin for HLFB feedback (-1 if not used) */
    int gpio_homing_sensor; /**< GPIO pin for homing sensor (-1 if not used) */
    uint32_t vel_max;       /**< Maximum velocity in steps per second */
    uint32_t accel_max;     /**< Maximum acceleration in steps per second squared */
    uint32_t jerk_max;      /**< Maximum jerk in steps per second cubed (for S-curve profile, 0 = use default) */
    clearpath_servo_profile_type_t profile_type; /**< Motion profile type (trapezoidal or S-curve) */
    clearpath_servo_hlfb_mode_t hlfb_mode; /**< HLFB interpretation mode */
    bool hlfb_active_high;   /**< HLFB active high (true) or active low (false) */
    clearpath_servo_homing_sensor_type_t homing_sensor_type; /**< Homing sensor type (NO/NC) */
    uint32_t homing_velocity; /**< Homing velocity in steps per second (0 = use vel_max) */
} clearpath_servo_config_t;

/**
 * @brief Servo handle structure (opaque)
 */
typedef struct clearpath_servo_handle_s clearpath_servo_handle_t;

/**
 * @brief Initialize servo with configuration
 * 
 * @param config Pointer to servo configuration
 * @param handle_out Pointer to receive servo handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_init(const clearpath_servo_config_t *config, clearpath_servo_handle_t **handle_out);

/**
 * @brief Deinitialize servo and free resources
 * 
 * @param handle Servo handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_deinit(clearpath_servo_handle_t *handle);

/**
 * @brief Move to absolute or relative position
 * 
 * @param handle Servo handle
 * @param steps Number of steps (positive = forward, negative = reverse)
 * @param target_type Absolute or relative move
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_move(clearpath_servo_handle_t *handle, int32_t steps, clearpath_servo_move_target_t target_type);

/**
 * @brief Start continuous velocity move
 * 
 * @param handle Servo handle
 * @param velocity Velocity in steps per second (positive = forward, negative = reverse)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_move_velocity(clearpath_servo_handle_t *handle, int32_t velocity);

/**
 * @brief Stop motion immediately
 * 
 * @param handle Servo handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_stop(clearpath_servo_handle_t *handle);

/**
 * @brief Set maximum velocity
 * 
 * @param handle Servo handle
 * @param vel_max Maximum velocity in steps per second
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_vel_max(clearpath_servo_handle_t *handle, uint32_t vel_max);

/**
 * @brief Set maximum acceleration
 * 
 * @param handle Servo handle
 * @param accel_max Maximum acceleration in steps per second squared
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_accel_max(clearpath_servo_handle_t *handle, uint32_t accel_max);

/**
 * @brief Set maximum jerk (for S-curve profile)
 * 
 * @param handle Servo handle
 * @param jerk_max Maximum jerk in steps per second cubed
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_jerk_max(clearpath_servo_handle_t *handle, uint32_t jerk_max);

/**
 * @brief Set motion profile type
 * 
 * @param handle Servo handle
 * @param profile_type Profile type (trapezoidal or S-curve)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_profile_type(clearpath_servo_handle_t *handle, clearpath_servo_profile_type_t profile_type);

/**
 * @brief Get current position
 * 
 * @param handle Servo handle
 * @return int32_t Current position in steps
 */
int32_t clearpath_servo_get_position(clearpath_servo_handle_t *handle);

/**
 * @brief Set position (for calibration or reset)
 * 
 * @param handle Servo handle
 * @param position New position value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_position(clearpath_servo_handle_t *handle, int32_t position);

/**
 * @brief Atomically increment/decrement position by delta
 * 
 * Internal function for step generation to avoid race conditions.
 * More efficient than get_position + set_position for step updates.
 * 
 * @param handle Servo handle
 * @param delta Position delta (+1 or -1 typically)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_increment_position(clearpath_servo_handle_t *handle, int32_t delta);

/**
 * @brief Generate a step pulse using RMT hardware peripheral
 * 
 * Internal function for step generation. Generates a precise step pulse
 * using the RMT peripheral for accurate timing (typically 10 microseconds).
 * 
 * @param handle Servo handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_generate_step_pulse(clearpath_servo_handle_t *handle);

/**
 * @brief Reset position to zero
 * 
 * @param handle Servo handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_reset_position(clearpath_servo_handle_t *handle);

/**
 * @brief Check if move is complete (from step count)
 * 
 * @param handle Servo handle
 * @return true if move is complete
 */
bool clearpath_servo_steps_complete(clearpath_servo_handle_t *handle);

/**
 * @brief Get current velocity
 * 
 * @param handle Servo handle
 * @return int32_t Current velocity in steps per second
 */
int32_t clearpath_servo_get_velocity(clearpath_servo_handle_t *handle);

/**
 * @brief Get HLFB GPIO status
 * 
 * @param handle Servo handle
 * @return bool HLFB GPIO level (true = high, false = low)
 */
bool clearpath_servo_get_hlfb_status(clearpath_servo_handle_t *handle);

/**
 * @brief Get configured HLFB mode
 * 
 * @param handle Servo handle
 * @return clearpath_servo_hlfb_mode_t HLFB interpretation mode
 */
clearpath_servo_hlfb_mode_t clearpath_servo_get_hlfb_mode(clearpath_servo_handle_t *handle);

/**
 * @brief Check if servo is enabled (from HLFB)
 * 
 * @param handle Servo handle
 * @return true if enabled (based on HLFB mode)
 */
bool clearpath_servo_is_enabled(clearpath_servo_handle_t *handle);

/**
 * @brief Check if move is complete (from HLFB)
 * 
 * @param handle Servo handle
 * @return true if move is complete (based on HLFB mode)
 */
bool clearpath_servo_is_move_complete(clearpath_servo_handle_t *handle);

/**
 * @brief Check for fault condition (from HLFB)
 * 
 * @param handle Servo handle
 * @return true if fault detected (based on HLFB mode)
 */
bool clearpath_servo_is_fault(clearpath_servo_handle_t *handle);

/**
 * @brief Enable or disable servo (via Enable GPIO)
 * 
 * @param handle Servo handle
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_enable(clearpath_servo_handle_t *handle, bool enable);

/**
 * @brief Start homing move
 * 
 * Moves the servo at homing velocity until the homing sensor is triggered.
 * When sensor triggers, motion stops and position is set to 0.
 * 
 * @param handle Servo handle
 * @param direction Direction to move (positive = forward, negative = reverse)
 * @param timeout_ms Maximum time to wait for sensor trigger (0 = no timeout)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if homing sensor not configured,
 *                   ESP_ERR_TIMEOUT if timeout reached before sensor trigger
 */
esp_err_t clearpath_servo_home(clearpath_servo_handle_t *handle, int32_t direction, uint32_t timeout_ms);

/**
 * @brief Check if homing is complete
 * 
 * @param handle Servo handle
 * @return true if homing is complete (sensor triggered and position set to 0)
 */
bool clearpath_servo_is_homing_complete(clearpath_servo_handle_t *handle);

/**
 * @brief Get homing sensor status
 * 
 * @param handle Servo handle
 * @return true if sensor is triggered (based on sensor type: NO=HIGH, NC=LOW)
 */
bool clearpath_servo_get_homing_sensor_status(clearpath_servo_handle_t *handle);

/**
 * @brief Check if homing timeout has been reached
 * 
 * @param handle Servo handle
 * @return true if timeout reached, false otherwise
 */
bool clearpath_servo_is_homing_timeout(clearpath_servo_handle_t *handle);

/**
 * @brief Get target position
 * 
 * @param handle Servo handle
 * @return int32_t Target position in steps
 */
int32_t clearpath_servo_get_target_position(clearpath_servo_handle_t *handle);

/**
 * @brief Get target velocity
 * 
 * @param handle Servo handle
 * @return int32_t Target velocity in steps per second
 */
int32_t clearpath_servo_get_target_velocity(clearpath_servo_handle_t *handle);

/**
 * @brief Get maximum velocity
 * 
 * @param handle Servo handle
 * @return uint32_t Maximum velocity in steps per second
 */
uint32_t clearpath_servo_get_vel_max(clearpath_servo_handle_t *handle);

/**
 * @brief Get maximum acceleration
 * 
 * @param handle Servo handle
 * @return uint32_t Maximum acceleration in steps per second squared
 */
uint32_t clearpath_servo_get_accel_max(clearpath_servo_handle_t *handle);

/**
 * @brief Get maximum jerk
 * 
 * @param handle Servo handle
 * @return uint32_t Maximum jerk in steps per second cubed
 */
uint32_t clearpath_servo_get_jerk_max(clearpath_servo_handle_t *handle);

/**
 * @brief Get motion profile type
 * 
 * @param handle Servo handle
 * @return clearpath_servo_profile_type_t Current profile type
 */
clearpath_servo_profile_type_t clearpath_servo_get_profile_type(clearpath_servo_handle_t *handle);

/**
 * @brief Get servo state
 * 
 * @param handle Servo handle
 * @return clearpath_servo_state_t Current servo state (IDLE, MOVING, VELOCITY_MODE)
 */
clearpath_servo_state_t clearpath_servo_get_state(clearpath_servo_handle_t *handle);

/**
 * @brief Set current velocity (for motion profile tracking)
 * 
 * Internal function for step generation task to update velocity during motion.
 * 
 * @param handle Servo handle
 * @param velocity Current velocity in steps per second
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_current_velocity(clearpath_servo_handle_t *handle, int32_t velocity);

/**
 * @brief Set servo state (internal function for manager)
 * 
 * Internal function for manager to set servo state for coordinated motion.
 * 
 * @param handle Servo handle
 * @param state New state
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_state(clearpath_servo_handle_t *handle, clearpath_servo_state_t state);

/**
 * @brief Set current acceleration (for S-curve profile tracking)
 * 
 * Internal function for step generation task to update acceleration during S-curve motion.
 * 
 * @param handle Servo handle
 * @param acceleration Current acceleration in steps per second squared
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_set_current_acceleration(clearpath_servo_handle_t *handle, int32_t acceleration);

/**
 * @brief Get current acceleration (for S-curve profile tracking)
 * 
 * Internal function for step generation task to get current acceleration.
 * 
 * @param handle Servo handle
 * @return int32_t Current acceleration in steps per second squared
 */
int32_t clearpath_servo_get_current_acceleration(clearpath_servo_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // CLEARPATH_SERVO_H
