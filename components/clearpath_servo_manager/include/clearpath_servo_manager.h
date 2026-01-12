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
 * @brief Unit type for motion control
 */
typedef enum {
    CLEARPATH_SERVO_UNIT_MM = 0,      /**< Millimeters */
    CLEARPATH_SERVO_UNIT_INCH = 1     /**< Inches */
} clearpath_servo_unit_t;

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
    uint32_t steps_per_unit;  /**< Steps per unit (e.g., 800 steps/mm or 400 steps/inch) */
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

/**
 * @brief Set unit type for all axes
 * 
 * Sets the unit type (mm or inches) for all coordinated motion operations.
 * All axes must use the same unit type - mixing units is not supported.
 * 
 * @param unit_type Unit type (CLEARPATH_SERVO_UNIT_MM or CLEARPATH_SERVO_UNIT_INCH)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_set_unit_type(clearpath_servo_unit_t unit_type);

/**
 * @brief Get unit type
 * 
 * @return clearpath_servo_unit_t Current unit type
 */
clearpath_servo_unit_t clearpath_servo_manager_get_unit_type(void);

/**
 * @brief Start coordinated linear move
 * 
 * Moves multiple axes in a straight line to target positions.
 * All axes start and stop together, maintaining proportional motion.
 * 
 * Positions and feedrate are in units (mm or inches) as configured.
 * Feedrate is in units/minute along the path (not per axis). The path length
 * is calculated as sqrt(sum of squares of axis distances). Each axis moves
 * at the velocity needed to complete its distance in the same time, ensuring
 * synchronized arrival at target positions.
 * 
 * @param axis_count Number of axes (2-4)
 * @param axis_indices Array of servo indices
 * @param target_positions Array of target positions in units (one per axis)
 * @param feedrate Feedrate in units/minute along the path
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_move_linear(
    uint8_t axis_count,
    const uint8_t *axis_indices,
    const float *target_positions,
    float feedrate
);

/**
 * @brief Start circular interpolation (arc move)
 * 
 * Moves X and Y axes in a circular arc.
 * 
 * Positions, radius, and feedrate are in units (mm or inches) as configured.
 * Feedrate is in units/minute along the arc path. The arc length is calculated
 * as radius * angle_radians. Both axes move at velocities needed to maintain
 * the specified feedrate along the arc, ensuring synchronized motion.
 * 
 * @param axis_x_index X axis servo index (0-3, where servo index = motor axis number)
 * @param axis_y_index Y axis servo index (0-3, where servo index = motor axis number)
 * @param center_x Center X position in units
 * @param center_y Center Y position in units
 * @param radius Radius in units
 * @param start_angle_deg Start angle in degrees (0-360)
 * @param end_angle_deg End angle in degrees (0-360)
 * @param clockwise True for clockwise, false for counter-clockwise
 * @param feedrate Feedrate in units/minute along the arc path
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_move_arc(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    float center_x,
    float center_y,
    float radius,
    int32_t start_angle_deg,
    int32_t end_angle_deg,
    bool clockwise,
    float feedrate
);

/**
 * @brief Start circular interpolation (arc move) using end point and radius
 * 
 * Moves X and Y axes in a circular arc from current position to end point.
 * Similar to G-code G02/G03 with R parameter (radius mode).
 * 
 * The center is calculated from the current position, end point, and radius.
 * There are two possible arcs (major and minor) - the direction parameter
 * determines which arc to take.
 * 
 * Positions, radius, and feedrate are in units (mm or inches) as configured.
 * Feedrate is in units/minute along the arc path.
 * 
 * @param axis_x_index X axis servo index (0-3, where servo index = motor axis number)
 * @param axis_y_index Y axis servo index (0-3, where servo index = motor axis number)
 * @param end_x End X position in units (absolute if absolute=true, relative if absolute=false)
 * @param end_y End Y position in units (absolute if absolute=true, relative if absolute=false)
 * @param absolute If true, end_x/end_y are absolute positions; if false, they are relative to current position
 * @param radius Radius in units (must be >= half the distance from start to end)
 * @param clockwise True for clockwise (G02), false for counter-clockwise (G03)
 * @param feedrate Feedrate in units/minute along the arc path
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if radius is too small
 */
esp_err_t clearpath_servo_manager_move_arc_to_point(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    float end_x,
    float end_y,
    bool absolute,
    float radius,
    bool clockwise,
    float feedrate
);

/**
 * @brief Start helical arc move (XYZ circular interpolation)
 * 
 * Moves X and Y axes in a circular arc while simultaneously moving Z axis linearly.
 * This creates a helical (spiral) motion path. Similar to G-code G02/G03 with Z axis.
 * 
 * X and Y follow a circular path using Bresenham circle algorithm.
 * Z moves linearly, synchronized with the arc progress using Bresenham line algorithm.
 * 
 * Positions, radius, and feedrate are in units (mm or inches) as configured.
 * Feedrate is in units/minute along the helical path (3D path length).
 * 
 * @param axis_x_index X axis servo index (0-3, where servo index = motor axis number)
 * @param axis_y_index Y axis servo index (0-3, where servo index = motor axis number)
 * @param axis_z_index Z axis servo index (0-3, where servo index = motor axis number)
 * @param center_x Center X position in units
 * @param center_y Center Y position in units
 * @param radius Radius in units
 * @param start_angle_deg Start angle in degrees (can be > 360 for multi-turn helixes)
 * @param end_angle_deg End angle in degrees (can be > 360 for multi-turn helixes)
 * @param z_start Z start position in units
 * @param z_end Z end position in units
 * @param clockwise True for clockwise, false for counter-clockwise
 * @param feedrate Feedrate in units/minute along the helical path
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_move_arc_helical(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    uint8_t axis_z_index,
    float center_x,
    float center_y,
    float radius,
    int32_t start_angle_deg,
    int32_t end_angle_deg,
    float z_start,
    float z_end,
    bool clockwise,
    float feedrate
);

/**
 * @brief Start helical arc move using end point and radius
 * 
 * Moves X and Y axes in a circular arc while simultaneously moving Z axis linearly,
 * from current position to end point. Similar to G-code G02/G03 with R parameter and Z axis.
 * 
 * The center is automatically calculated from current position, end point, and radius.
 * There are two possible arcs (major and minor) - the direction parameter determines which.
 * 
 * X and Y follow a circular path using Bresenham circle algorithm.
 * Z moves linearly, synchronized with the arc progress using Bresenham line algorithm.
 * 
 * Positions, radius, and feedrate are in units (mm or inches) as configured.
 * Feedrate is in units/minute along the helical path (3D path length).
 * 
 * @param axis_x_index X axis servo index (0-3, where servo index = motor axis number)
 * @param axis_y_index Y axis servo index (0-3, where servo index = motor axis number)
 * @param axis_z_index Z axis servo index (0-3, where servo index = motor axis number)
 * @param end_x End X position in units (absolute if absolute=true, relative if absolute=false)
 * @param end_y End Y position in units (absolute if absolute=true, relative if absolute=false)
 * @param end_z End Z position in units (absolute if absolute=true, relative if absolute=false)
 * @param absolute If true, end_x/end_y/end_z are absolute positions; if false, they are relative to current position
 * @param radius Radius in units (must be >= half the distance from start to end in XY plane)
 * @param clockwise True for clockwise (G02), false for counter-clockwise (G03)
 * @param feedrate Feedrate in units/minute along the helical path
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if radius is too small
 */
esp_err_t clearpath_servo_manager_move_arc_helical_to_point(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    uint8_t axis_z_index,
    float end_x,
    float end_y,
    float end_z,
    bool absolute,
    float radius,
    bool clockwise,
    float feedrate
);

/**
 * @brief Check if coordinated move is complete
 * 
 * @return true if all axes have reached their targets
 */
bool clearpath_servo_manager_is_coordinated_move_complete(void);

/**
 * @brief Stop coordinated move
 * 
 * Stops all axes in the coordinated motion group immediately.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_stop_coordinated_move(void);

#ifdef __cplusplus
}
#endif

#endif // CLEARPATH_SERVO_MANAGER_H
