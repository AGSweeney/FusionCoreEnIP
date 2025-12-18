/**
 * @file clearpath_servo_manager.h
 * @brief ClearPath Servo Manager - Multi-Servo Management and Assembly Integration
 * 
 * This component manages multiple ClearPath servos, generates step pulses,
 * monitors HLFB feedback, and integrates with EtherNet/IP assembly data.
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * To enable it:
 * 1. Add component to build system
 * 2. Call clearpath_servo_manager_init() in main.c
 * 3. Configure GPIO pins via system_config or NVS
 * 
 * Attribution:
 * ------------
 * Step generation algorithms and servo control concepts are based on patterns
 * from the Teknic ClearCore library. The multi-servo management approach and
 * step pulse generation follow similar principles to ClearCore's MotorManager
 * and StepGenerator classes.
 * 
 * References:
 * - Teknic ClearCore Library: https://github.com/Teknic-Inc/ClearCore-library
 * - ClearCore Motor Control: https://teknic-inc.github.io/ClearCore-library/_motor_driver_main.html
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

#ifndef CLEARPATH_SERVO_MANAGER_H
#define CLEARPATH_SERVO_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "clearpath_servo.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum number of servos supported
 */
#define CLEARPATH_SERVO_MANAGER_MAX_SERVOS 4

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
    clearpath_servo_hlfb_mode_t hlfb_mode; /**< HLFB interpretation mode */
    bool hlfb_active_high;   /**< HLFB active high (true) or active low (false) */
    clearpath_servo_homing_sensor_type_t homing_sensor_type; /**< Homing sensor type (NO/NC) */
    uint32_t homing_velocity; /**< Homing velocity in steps per second (0 = use vel_max) */
    bool enabled;           /**< Servo enabled flag */
} clearpath_servo_manager_config_t;

/**
 * @brief Initialize servo manager
 * 
 * This function:
 * - Loads configuration from NVS/system_config
 * - Initializes servo driver instances
 * - Creates background task for step generation
 * - Configures GPIO pins
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_init(void);

/**
 * @brief Check if servo manager is initialized
 * 
 * @return true if initialized
 */
bool clearpath_servo_manager_is_initialized(void);

/**
 * @brief Get servo configuration
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @param config Pointer to configuration structure to fill
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_get_config(uint8_t servo_index, clearpath_servo_manager_config_t *config);

/**
 * @brief Set servo configuration
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @param config Pointer to configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_set_config(uint8_t servo_index, const clearpath_servo_manager_config_t *config);

/**
 * @brief Get current servo position
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @return int32_t Current position in steps
 */
int32_t clearpath_servo_manager_get_position(uint8_t servo_index);

/**
 * @brief Get current servo velocity
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @return int32_t Current velocity in steps per second
 */
int32_t clearpath_servo_manager_get_velocity(uint8_t servo_index);

/**
 * @brief Get servo status flags
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @return uint8_t Status flags (bit 0=move complete, bit 1=enabled, bit 2=fault, bit 3=in position)
 */
uint8_t clearpath_servo_manager_get_status(uint8_t servo_index);

/**
 * @brief Get number of initialized servos
 * 
 * @return uint8_t Number of servos (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS)
 */
uint8_t clearpath_servo_manager_get_count(void);

/**
 * @brief Start homing move for a servo
 * 
 * Moves the servo at homing velocity until the homing sensor is triggered.
 * When sensor triggers, motion stops and position is set to 0.
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @param direction Direction to move (positive = forward, negative = reverse)
 * @param timeout_ms Maximum time to wait for sensor trigger (0 = no timeout)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE if homing sensor not configured
 */
esp_err_t clearpath_servo_manager_home(uint8_t servo_index, int32_t direction, uint32_t timeout_ms);

/**
 * @brief Check if homing is complete for a servo
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @return true if homing is complete (sensor triggered and position set to 0)
 */
bool clearpath_servo_manager_is_homing_complete(uint8_t servo_index);

/**
 * @brief Get homing sensor status for a servo
 * 
 * @param servo_index Servo index (0 to CLEARPATH_SERVO_MANAGER_MAX_SERVOS-1)
 * @return true if sensor is triggered (based on sensor type: NO=HIGH, NC=LOW)
 */
bool clearpath_servo_manager_get_homing_sensor_status(uint8_t servo_index);

#ifdef __cplusplus
}
#endif

#endif // CLEARPATH_SERVO_MANAGER_H
