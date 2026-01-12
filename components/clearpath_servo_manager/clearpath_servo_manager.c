/**
 * @file clearpath_servo_manager.c
 * @brief ClearPath Servo Manager Implementation
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * 
 * Architecture:
 * ------------
 * 
 * Step Generation Flow:
 *   Output Assembly 150 → Manager Task → Step Generation → GPIO → ClearPath Servo
 *   ClearPath Servo → HLFB GPIO → Manager Task → Status → Input Assembly 100
 * 
 * 1. Background task reads commands from Output Assembly 150 (bytes 32-51)
 * 2. Task generates step pulses based on velocity/acceleration profiles
 * 3. Position tracking updates based on step count
 * 4. HLFB monitoring updates status flags
 * 5. Status/feedback written to Input Assembly 100 (bytes 61-71)
 * 
 * Step Generation:
 * - Background task generates step pulses using RMT hardware peripheral
 * - RMT provides precise timing (typically 10 microsecond pulse width)
 * - Trapezoidal velocity profile (accel → constant → decel)
 * - Updates position counter on each step
 * - Checks move completion based on position and HLFB
 * - RMT hardware enables higher step rates than software GPIO toggling
 *   (practical rates depend on task frequency and RMT configuration)
 * 
 * Assembly Data Layout:
 * - Output Assembly 150, Bytes 32-51: Servo commands (5 bytes per servo, 20 bytes total for 4 servos)
 *   - Byte 0: Command type (0=stop, 1=move_abs, 2=move_rel, 3=move_velocity, 4=enable, 5=disable)
 *   - Bytes 1-4: int32_t value (position for move_abs/move_rel, velocity for move_velocity)
 * - Input Assembly 100, Bytes 61-71: Servo feedback (11 bytes total)
 *   - Bytes 61-64: Servo 0 position (int32_t)
 *   - Bytes 65-66: Servo 0 velocity (int16_t)
 *   - Bytes 67: Servo 0 status (uint8_t)
 *   - Bytes 68-71: Servo 1 position (int32_t, truncated) + status
 * 
 * Attribution:
 * ------------
 * Step generation algorithms, trapezoidal velocity profiles, and servo control
 * concepts are based on patterns from the Teknic ClearCore library. The high-frequency
 * step generation task and velocity/acceleration management follow similar principles
 * to ClearCore's StepGenerator implementation.
 * 
 * References:
 * - Teknic ClearCore Library: https://github.com/Teknic-Inc/ClearCore-library
 * - ClearCore Step Generation: https://teknic-inc.github.io/ClearCore-library/_move_gen.html
 * - ClearPath Servo User Manual: https://www.teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf
 */

#include "clearpath_servo_manager.h"
#include "clearpath_servo.h"
#include "fusion_core_assembly.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "clearpath_servo_manager";

#define STEP_GENERATION_TASK_FREQ_HZ 5000  // 5 kHz step generation frequency
#define STEP_GENERATION_TASK_INTERVAL_US (1000000 / STEP_GENERATION_TASK_FREQ_HZ)
#define ASSEMBLY_UPDATE_INTERVAL_MS 10  // Update assembly data every 10ms
// Note: Step pulse width is now controlled by RMT hardware (typically 10 microseconds)

// Servo instance structure
typedef struct {
    clearpath_servo_handle_t *handle;
    clearpath_servo_manager_config_t config;
    bool initialized;
    int32_t last_position;
    uint64_t last_step_time_us;
} servo_instance_t;

// Manager state
static bool s_initialized = false;
static servo_instance_t s_servos[CLEARPATH_SERVO_MANAGER_MAX_SERVOS];
static TaskHandle_t s_task_handle = NULL;
static SemaphoreHandle_t s_config_mutex = NULL;
static clearpath_servo_unit_t s_unit_type = CLEARPATH_SERVO_UNIT_MM;  // Global unit type (all axes must use same)

/**
 * @brief Convert units to steps for a servo
 * 
 * @param servo_index Servo index
 * @param units Position in units
 * @return int32_t Position in steps
 */
static int32_t units_to_steps(uint8_t servo_index, float units)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0;
    }
    
    // Access config under mutex protection to avoid race condition
    uint32_t steps_per_unit = 1;  // Default
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            steps_per_unit = s_servos[servo_index].config.steps_per_unit;
            if (steps_per_unit == 0) {
                steps_per_unit = 1;  // Default to 1:1 if not configured
            }
        }
        xSemaphoreGive(s_config_mutex);
    }
    
    // Convert: steps = units * steps_per_unit
    // Check for overflow: int32_t max is 2,147,483,647
    // Use float intermediate to detect overflow before casting
    float steps_float = units * steps_per_unit;
    if (steps_float > 2147483647.0f || steps_float < -2147483648.0f) {
        ESP_LOGW(TAG, "units_to_steps overflow: units=%.2f, steps_per_unit=%u", units, steps_per_unit);
        return (steps_float > 0) ? INT32_MAX : INT32_MIN;
    }
    return (int32_t)steps_float;
}

/**
 * @brief Convert steps to units for a servo
 * 
 * @param servo_index Servo index
 * @param steps Position in steps
 * @return float Position in units
 */
static float steps_to_units(uint8_t servo_index, int32_t steps)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0.0f;
    }
    
    // Access config under mutex protection to avoid race condition
    uint32_t steps_per_unit = 1;  // Default
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            steps_per_unit = s_servos[servo_index].config.steps_per_unit;
            if (steps_per_unit == 0) {
                steps_per_unit = 1;  // Default to 1:1 if not configured
            }
        }
        xSemaphoreGive(s_config_mutex);
    }
    
    // Convert: units = steps / steps_per_unit
    return (float)steps / steps_per_unit;
}

/**
 * @brief Convert feedrate from units/minute to steps/second
 * 
 * @param servo_index Servo index (for steps_per_unit)
 * @param feedrate_units_per_min Feedrate in units/minute
 * @return uint32_t Feedrate in steps/second
 */
static uint32_t feedrate_units_per_min_to_steps_per_sec(uint8_t servo_index, float feedrate_units_per_min)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0;
    }
    
    // Access config under mutex protection to avoid race condition
    uint32_t steps_per_unit = 1;  // Default
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            steps_per_unit = s_servos[servo_index].config.steps_per_unit;
            if (steps_per_unit == 0) {
                steps_per_unit = 1;  // Default to 1:1 if not configured
            }
        }
        xSemaphoreGive(s_config_mutex);
    }
    
    // Convert: steps/sec = (units/min * steps/unit) / 60
    float steps_per_min = feedrate_units_per_min * steps_per_unit;
    float steps_per_sec = steps_per_min / 60.0f;
    
    // Check for overflow: uint32_t max is 4,294,967,295
    if (steps_per_sec > 4294967295.0f) {
        ESP_LOGW(TAG, "feedrate conversion overflow: feedrate=%.2f, steps_per_unit=%u", 
                 feedrate_units_per_min, steps_per_unit);
        return UINT32_MAX;
    }
    if (steps_per_sec < 0.0f) {
        return 0;
    }
    
    return (uint32_t)steps_per_sec;
}

// Coordinated motion types
typedef enum {
    COORDINATED_MOTION_NONE = 0,
    COORDINATED_MOTION_LINEAR,
    COORDINATED_MOTION_CIRCULAR
} coordinated_motion_type_t;

// Coordinated motion group structure
#define MAX_COORDINATED_AXES 4

typedef struct {
    coordinated_motion_type_t type;
    bool active;
    uint8_t axis_count;
    uint8_t axis_indices[MAX_COORDINATED_AXES];
    int32_t start_positions[MAX_COORDINATED_AXES];
    int32_t target_positions[MAX_COORDINATED_AXES];
    int32_t total_distance[MAX_COORDINATED_AXES];
    int32_t path_length;  // Total path length in steps (for linear: sqrt of sum of squares)
    uint32_t feedrate;  // Feedrate in steps/second along the path
    uint64_t start_time_us;
    uint64_t total_time_us;  // Total time needed for move (calculated from path_length / feedrate)
    uint32_t original_vel_max[MAX_COORDINATED_AXES];  // Store original vel_max to restore after move
    // Bresenham algorithm state (shared for linear and circular)
    // For linear interpolation (Bresenham line algorithm)
    int32_t bresenham_dx[MAX_COORDINATED_AXES];      // Distance per axis (absolute)
    int32_t bresenham_dy[MAX_COORDINATED_AXES];      // Not used for linear (kept for compatibility)
    int32_t bresenham_error[MAX_COORDINATED_AXES];   // Error term per axis
    int32_t bresenham_step_dir[MAX_COORDINATED_AXES]; // Step direction per axis (+1 or -1)
    int32_t bresenham_current[MAX_COORDINATED_AXES];  // Current position per axis
    int32_t bresenham_steps_generated;               // Total steps generated along path
    int32_t bresenham_steps_total;                   // Total steps in path
    // For circular interpolation (Bresenham circle algorithm)
    int32_t center_x;
    int32_t center_y;
    int32_t radius;
    int32_t start_angle_deg_x10;  // Start angle in 0.1 degree units (360 degrees = 3600)
    int32_t end_angle_deg_x10;    // End angle in 0.1 degree units
    int32_t angle_diff_x10;        // Angle difference in 0.1 degree units
    int32_t arc_length;  // Arc length in steps
    int32_t bresenham_x;      // Current X position relative to center (in first octant)
    int32_t bresenham_y;      // Current Y position relative to center (in first octant)
    int32_t bresenham_circle_error;  // Decision parameter for circle
    int32_t bresenham_octant; // Current octant (0-7)
    bool bresenham_clockwise;  // Direction
    uint64_t bresenham_last_step_time_us;  // Last step generation time for feedrate control
    uint32_t bresenham_step_interval_us;  // Step interval in microseconds (for feedrate)
    // For helical arcs (XYZ): Z axis linear interpolation
    int32_t z_start_position;  // Z start position in steps
    int32_t z_end_position;    // Z end position in steps
    int32_t z_total_distance; // Z total distance in steps
    int32_t z_current_position; // Z current position in steps
    int32_t z_error;           // Z axis error term for linear interpolation
    int32_t z_step_dir;        // Z step direction (+1 or -1)
} coordinated_motion_group_t;

static coordinated_motion_group_t s_coordinated_group = {0};

// Default configuration
#define DEFAULT_GPIO_STEP_0    8
#define DEFAULT_GPIO_DIR_0     9
#define DEFAULT_GPIO_ENABLE_0  10
#define DEFAULT_GPIO_HLFB_0    11
#define DEFAULT_GPIO_HOMING_SENSOR_0  -1  // Not configured by default
#define DEFAULT_VEL_MAX        1000
#define DEFAULT_ACCEL_MAX      10000
#define DEFAULT_JERK_MAX       0  // 0 = use default (10% of accel_max)
#define DEFAULT_PROFILE_TYPE   CLEARPATH_SERVO_PROFILE_TRAPEZOIDAL
#define DEFAULT_HOMING_VELOCITY 0  // 0 = use vel_max

// Sine lookup table (0-90 degrees, scaled by 1000)
// sin(0°) = 0, sin(90°) = 1000
#define SIN_TABLE_SIZE 91
static const int16_t s_sin_table[SIN_TABLE_SIZE] = {
    0, 17, 35, 52, 70, 87, 105, 122, 139, 156, 174, 191, 208, 225, 242, 259,
    276, 292, 309, 326, 342, 358, 375, 391, 407, 423, 438, 454, 469, 485, 500,
    515, 530, 545, 559, 574, 588, 602, 616, 629, 643, 656, 669, 682, 695, 707,
    719, 731, 743, 755, 766, 777, 788, 799, 809, 819, 829, 839, 848, 857, 866,
    875, 883, 891, 899, 906, 914, 921, 927, 934, 940, 946, 951, 956, 961, 966,
    970, 974, 978, 982, 985, 988, 990, 993, 995, 996, 998, 999, 1000
};

/**
 * @brief Calculate sin using lookup table with 0.1 degree resolution via linear interpolation
 * 
 * @param angle_deg_x10 Angle in 0.1 degree units (0-3600, where 3600 = 360.0 degrees)
 * @param scale Scale factor (1000 = 1.0)
 * @return int32_t sin(angle) * scale
 */
static int32_t sin_lookup(int32_t angle_deg_x10, int32_t scale)
{
    // Normalize to 0-3600 (0-360 degrees)
    angle_deg_x10 = angle_deg_x10 % 3600;
    if (angle_deg_x10 < 0) angle_deg_x10 += 3600;
    
    int32_t result;
    
    // Convert to integer degrees and fractional part (0.1 degree units)
    int32_t angle_deg_int = angle_deg_x10 / 10;  // Integer degrees
    int32_t angle_frac_x10 = angle_deg_x10 % 10;  // Fractional part in 0.1 degree units (0-9)
    
    // Get base angle for lookup (0-90 degrees)
    int32_t base_angle;
    bool negate = false;
    
    if (angle_deg_int <= 90) {
        base_angle = angle_deg_int;
    } else if (angle_deg_int <= 180) {
        base_angle = 180 - angle_deg_int;
    } else if (angle_deg_int <= 270) {
        base_angle = angle_deg_int - 180;
        negate = true;
    } else {
        base_angle = 360 - angle_deg_int;
        negate = true;
    }
    
    // Lookup sin values for base angle and next angle (for interpolation)
    int32_t sin_base = s_sin_table[base_angle];
    int32_t sin_next;
    
    if (base_angle < 90) {
        sin_next = s_sin_table[base_angle + 1];
    } else {
        // At 90 degrees, sin is maximum (1000), next would be sin(91) = sin(89) = s_sin_table[89]
        // This correctly interpolates from 1000 down to ~999.8 for angles 90.0 to 90.9
        sin_next = s_sin_table[89];
    }
    
    // Linear interpolation: sin = sin_base + (sin_next - sin_base) * (fraction / 10)
    int32_t sin_interp = sin_base + ((sin_next - sin_base) * angle_frac_x10) / 10;
    
    // Apply scale and sign
    result = ((int32_t)sin_interp * scale) / 1000;
    if (negate) {
        result = -result;
    }
    
    return result;
}

/**
 * @brief Calculate cos using lookup table
 * 
 * @param angle_deg Angle in degrees (0-360)
 * @param scale Scale factor (1000 = 1.0)
 * @return int32_t cos(angle) * scale
 */
static int32_t cos_lookup(int32_t angle_deg, int32_t scale)
{
    // cos(θ) = sin(θ + 90)
    return sin_lookup(angle_deg + 90, scale);
}

// Forward declaration for servo info structure
typedef struct {
    clearpath_servo_handle_t *handle;
    int gpio_dir;
} servo_info_t;

/**
 * @brief Generate steps for coordinated linear motion using Bresenham line algorithm
 * 
 * This function generates steps directly using Bresenham's line algorithm.
 * It's called from step_generation_task and generates steps based on feedrate timing.
 * 
 * @param group Coordinated motion group (must be protected by mutex)
 * @param servo_infos Array of servo info structures (handles and GPIO dir)
 * @param max_steps_per_interval Maximum steps to generate in this interval (overrun protection)
 * @return Number of steps generated (0 if complete or error)
 */
static int32_t generate_bresenham_linear_steps(coordinated_motion_group_t *group,
                                                servo_info_t *servo_infos,
                                                int32_t max_steps_per_interval)
{
    if (!group->active || group->type != COORDINATED_MOTION_LINEAR) {
        return 0;
    }

    // Check if move is complete
    if (group->bresenham_steps_generated >= group->bresenham_steps_total) {
        // Move complete - set all axes to target positions and stop
        for (uint8_t i = 0; i < group->axis_count; i++) {
            if (group->axis_indices[i] < CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
                clearpath_servo_handle_t *handle = servo_infos[group->axis_indices[i]].handle;
                if (handle != NULL) {
                    clearpath_servo_set_position(handle, group->target_positions[i]);
                    clearpath_servo_set_state(handle, CLEARPATH_SERVO_STATE_IDLE);
                    // Restore original vel_max before stopping
                    if (i < MAX_COORDINATED_AXES) {
                        clearpath_servo_set_vel_max(handle, group->original_vel_max[i]);
                    }
                    clearpath_servo_stop(handle);
                }
            }
        }
        group->active = false;
        return 0;
    }

    // Check feedrate timing - only generate step if enough time has passed
    uint64_t current_time_us = esp_timer_get_time();
    if (current_time_us < group->bresenham_last_step_time_us) {
        // Timer wrap-around
        group->bresenham_last_step_time_us = current_time_us;
    }
    
    uint64_t time_since_last_step = current_time_us - group->bresenham_last_step_time_us;
    if (time_since_last_step < group->bresenham_step_interval_us) {
        // Not time for next step yet
        return 0;
    }

    // Generate steps using Bresenham line algorithm
    // Multi-axis Bresenham: for each step along path, step the axis with maximum error
    // Error calculation: error[i] = distance[i] * steps_generated - current[i] * path_length
    // Step the axis where error[i] >= path_length/2 (or maximum error)
    int32_t steps_generated = 0;
    const int32_t max_steps_this_interval = (max_steps_per_interval > 0) ? max_steps_per_interval : 100;
    
    while (steps_generated < max_steps_this_interval && 
           group->bresenham_steps_generated < group->bresenham_steps_total) {
        
        // Find axis with maximum error that needs to step
        // Step when: error[i] >= path_length/2
        int32_t max_error = INT32_MIN;
        int32_t max_error_axis = -1;
        
        for (uint8_t i = 0; i < group->axis_count; i++) {
            if (group->axis_indices[i] >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
                continue;
            }
            
            // Check if this axis has remaining distance and needs to step
            if (group->bresenham_dx[i] > 0) {
                // Calculate if this axis should step: error >= path_length/2
                // Or use maximum error approach
                if (group->bresenham_error[i] > max_error) {
                    max_error = group->bresenham_error[i];
                    max_error_axis = i;
                }
            }
        }
        
        if (max_error_axis < 0 || max_error < (group->bresenham_steps_total / 2)) {
            // No axis needs to step yet, or move complete
            // Check if all axes are at target
            bool all_at_target = true;
            for (uint8_t i = 0; i < group->axis_count; i++) {
                if (group->axis_indices[i] < CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
                    if (group->bresenham_current[i] != group->target_positions[i]) {
                        all_at_target = false;
                        break;
                    }
                }
            }
            if (all_at_target) {
                break;  // Move complete
            }
            // Advance path step without stepping any axis (rare case)
            group->bresenham_steps_generated++;
            steps_generated++;
            continue;
        }
        
        // Generate step for selected axis
        uint8_t axis_idx = group->axis_indices[max_error_axis];
        if (axis_idx >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
            break;
        }
        
        clearpath_servo_handle_t *handle = servo_infos[axis_idx].handle;
        if (handle == NULL) {
            group->active = false;
            return steps_generated;
        }
        
        // Set direction GPIO
        gpio_set_level(servo_infos[axis_idx].gpio_dir, 
                      (group->bresenham_step_dir[max_error_axis] > 0) ? 1 : 0);
        
        // Generate step pulse
        esp_err_t ret = clearpath_servo_generate_step_pulse(handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to generate step pulse for axis %d", max_error_axis);
            group->active = false;
            return steps_generated;
        }
        
        // Update position
        clearpath_servo_increment_position(handle, group->bresenham_step_dir[max_error_axis]);
        
        // Update Bresenham state
        group->bresenham_current[max_error_axis] += group->bresenham_step_dir[max_error_axis];
        // Update error: subtract path_length when this axis steps
        group->bresenham_error[max_error_axis] -= group->bresenham_steps_total;
        
        // Update error for all axes: add their distance (they advance along path)
        for (uint8_t i = 0; i < group->axis_count; i++) {
            if (group->bresenham_dx[i] > 0) {
                group->bresenham_error[i] += group->bresenham_dx[i];
            }
        }
        
        group->bresenham_steps_generated++;
        steps_generated++;
        group->bresenham_last_step_time_us = current_time_us;
    }
    
    return steps_generated;
}

// Old process_coordinated_circular function removed - replaced with generate_bresenham_circular_steps

/**
 * @brief Generate steps for coordinated circular motion using Bresenham circle algorithm
 * 
 * This function generates steps directly using Bresenham's circle algorithm.
 * It's called from step_generation_task and generates steps based on feedrate timing.
 * 
 * @param group Coordinated motion group (must be protected by mutex)
 * @param servo_infos Array of servo info structures (handles and GPIO dir)
 * @param max_steps_per_interval Maximum steps to generate in this interval (overrun protection)
 * @return Number of steps generated (0 if complete or error)
 */
static int32_t generate_bresenham_circular_steps(coordinated_motion_group_t *group,
                                                  servo_info_t *servo_infos,
                                                  int32_t max_steps_per_interval)
{
    if (!group->active || group->type != COORDINATED_MOTION_CIRCULAR) {
        return 0;
    }

    // Check if move is complete
    if (group->bresenham_steps_generated >= group->bresenham_steps_total) {
        // Arc complete - set axes to final positions and stop
        uint8_t x_axis_idx = group->axis_indices[0];
        uint8_t y_axis_idx = group->axis_indices[1];
        
        if (x_axis_idx < CLEARPATH_SERVO_MANAGER_MAX_SERVOS && 
            y_axis_idx < CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
            clearpath_servo_handle_t *x_handle = servo_infos[x_axis_idx].handle;
            clearpath_servo_handle_t *y_handle = servo_infos[y_axis_idx].handle;
            
            if (x_handle != NULL && y_handle != NULL) {
                // Calculate final positions using end angle
                int32_t cos_val = cos_lookup(group->end_angle_deg_x10, 1000);
                int32_t sin_val = sin_lookup(group->end_angle_deg_x10, 1000);
                
                int32_t final_x = group->center_x + ((int64_t)group->radius * cos_val) / 1000;
                int32_t final_y = group->center_y + ((int64_t)group->radius * sin_val) / 1000;
                
                clearpath_servo_set_position(x_handle, final_x);
                clearpath_servo_set_state(x_handle, CLEARPATH_SERVO_STATE_IDLE);
                if (group->axis_count >= 1) {
                    clearpath_servo_set_vel_max(x_handle, group->original_vel_max[0]);
                }
                clearpath_servo_stop(x_handle);
                
                clearpath_servo_set_position(y_handle, final_y);
                clearpath_servo_set_state(y_handle, CLEARPATH_SERVO_STATE_IDLE);
                if (group->axis_count >= 2) {
                    clearpath_servo_set_vel_max(y_handle, group->original_vel_max[1]);
                }
                clearpath_servo_stop(y_handle);
            }
        }
        
        // Handle Z axis for helical arcs
        if (group->axis_count >= 3) {
            uint8_t z_axis_idx = group->axis_indices[2];
            if (z_axis_idx < CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
                clearpath_servo_handle_t *z_handle = servo_infos[z_axis_idx].handle;
                if (z_handle != NULL) {
                    clearpath_servo_set_position(z_handle, group->z_end_position);
                    clearpath_servo_set_state(z_handle, CLEARPATH_SERVO_STATE_IDLE);
                    if (group->axis_count >= 3) {
                        clearpath_servo_set_vel_max(z_handle, group->original_vel_max[2]);
                    }
                    clearpath_servo_stop(z_handle);
                }
            }
        }
        
        group->active = false;
        return 0;
    }

    // Check feedrate timing - only generate step if enough time has passed
    uint64_t current_time_us = esp_timer_get_time();
    if (current_time_us < group->bresenham_last_step_time_us) {
        // Timer wrap-around
        group->bresenham_last_step_time_us = current_time_us;
    }
    
    uint64_t time_since_last_step = current_time_us - group->bresenham_last_step_time_us;
    if (time_since_last_step < group->bresenham_step_interval_us) {
        // Not time for next step yet
        return 0;
    }

    // Generate steps using Bresenham circle algorithm
    // Bresenham works in first octant (0-45 degrees), we transform to actual coordinates
    int32_t steps_generated = 0;
    const int32_t max_steps_this_interval = (max_steps_per_interval > 0) ? max_steps_per_interval : 100;
    
    uint8_t x_axis_idx = group->axis_indices[0];
    uint8_t y_axis_idx = group->axis_indices[1];
    
    if (x_axis_idx >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS || 
        y_axis_idx >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        group->active = false;
        return 0;
    }
    
    clearpath_servo_handle_t *x_handle = servo_infos[x_axis_idx].handle;
    clearpath_servo_handle_t *y_handle = servo_infos[y_axis_idx].handle;
    
    if (x_handle == NULL || y_handle == NULL) {
        group->active = false;
        return 0;
    }
    
    while (steps_generated < max_steps_this_interval && 
           group->bresenham_steps_generated < group->bresenham_steps_total) {
        
        // Bresenham circle decision: if error < 0, step X only; else step X and Y
        bool step_x = false;
        bool step_y = false;
        
        if (group->bresenham_circle_error < 0) {
            // Step X only
            group->bresenham_x++;
            group->bresenham_circle_error += 2 * group->bresenham_x + 1;
            step_x = true;
        } else {
            // Step X and Y
            group->bresenham_x++;
            group->bresenham_y--;
            group->bresenham_circle_error += 2 * (group->bresenham_x - group->bresenham_y) + 1;
            step_x = true;
            step_y = true;
        }
        
        // Transform from first octant to actual coordinates based on current octant
        // Standard Bresenham circle octant transformations:
        // First octant point (x, y) maps to other octants as:
        // Octant 0 (0-45°):   (x, y)   -> (x, y)
        // Octant 1 (45-90°):  (x, y)   -> (y, x)   [swap]
        // Octant 2 (90-135°): (x, y)   -> (-y, x)  [swap, negate first]
        // Octant 3 (135-180°): (x, y)  -> (-x, y)  [negate first]
        // Octant 4 (180-225°): (x, y)  -> (-x, -y) [negate both]
        // Octant 5 (225-270°): (x, y)  -> (-y, -x) [swap, negate both]
        // Octant 6 (270-315°): (x, y)  -> (y, -x)  [swap, negate second]
        // Octant 7 (315-360°): (x, y)  -> (x, -y)  [negate second]
        //
        // For steps: in first octant, x increases (+1), y decreases (-1)
        // We need to map these step directions to actual X/Y axes
        
        int32_t actual_x_step = 0;
        int32_t actual_y_step = 0;
        int32_t octant = group->bresenham_octant;
        
        // Step directions in first octant
        int32_t first_oct_x_step = step_x ? 1 : 0;   // X increases in first octant
        int32_t first_oct_y_step = step_y ? -1 : 0;  // Y decreases in first octant
        
        // Apply octant transformation
        switch (octant) {
            case 0:  // 0-45°: (x, y) -> (x, y)
                actual_x_step = first_oct_x_step;
                actual_y_step = first_oct_y_step;
                break;
            case 1:  // 45-90°: (x, y) -> (y, x) [swap]
                actual_x_step = first_oct_y_step;
                actual_y_step = first_oct_x_step;
                break;
            case 2:  // 90-135°: (x, y) -> (-y, x) [swap, negate first]
                actual_x_step = -first_oct_y_step;
                actual_y_step = first_oct_x_step;
                break;
            case 3:  // 135-180°: (x, y) -> (-x, y) [negate first]
                actual_x_step = -first_oct_x_step;
                actual_y_step = first_oct_y_step;
                break;
            case 4:  // 180-225°: (x, y) -> (-x, -y) [negate both]
                actual_x_step = -first_oct_x_step;
                actual_y_step = -first_oct_y_step;
                break;
            case 5:  // 225-270°: (x, y) -> (-y, -x) [swap, negate both]
                actual_x_step = -first_oct_y_step;
                actual_y_step = -first_oct_x_step;
                break;
            case 6:  // 270-315°: (x, y) -> (y, -x) [swap, negate second]
                actual_x_step = first_oct_y_step;
                actual_y_step = -first_oct_x_step;
                break;
            case 7:  // 315-360°: (x, y) -> (x, -y) [negate second]
                actual_x_step = first_oct_x_step;
                actual_y_step = -first_oct_y_step;
                break;
        }
        
        // Apply clockwise/counterclockwise direction
        // Clockwise reverses the direction around the circle
        if (group->bresenham_clockwise) {
            actual_x_step = -actual_x_step;
            actual_y_step = -actual_y_step;
        }
        
        // Check for octant transition: when x >= y in first octant, we've reached 45°
        // At this point, we transition to the next octant
        // Note: For arcs, we may not complete a full octant, so this is a safety check
        if (group->bresenham_x >= group->bresenham_y && group->bresenham_x > 0) {
            // We've reached the 45° boundary in first octant
            // For full circles, we'd transition to next octant, but for arcs we track by step count
            // Transition octant if we're continuing (not at end)
            if (group->bresenham_steps_generated < group->bresenham_steps_total - 1) {
                // Transition to next octant
                if (group->bresenham_clockwise) {
                    group->bresenham_octant = (group->bresenham_octant - 1 + 8) % 8;
                } else {
                    group->bresenham_octant = (group->bresenham_octant + 1) % 8;
                }
                
                // At octant boundary (45°), swap x and y in first octant coordinates
                // This is because at 45°, x = y, and we're moving to the next octant
                int32_t temp = group->bresenham_x;
                group->bresenham_x = group->bresenham_y;
                group->bresenham_y = temp;
                
                // Reset error for new octant (at boundary, error should be recalculated)
                // Simplified: use current error, algorithm will correct
            }
        }
        
        // Generate step pulses for X and Y (circular arc)
        if (step_x) {
            gpio_set_level(servo_infos[x_axis_idx].gpio_dir, (actual_x_step > 0) ? 1 : 0);
            esp_err_t ret = clearpath_servo_generate_step_pulse(x_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to generate X step pulse");
                group->active = false;
                return steps_generated;
            }
            clearpath_servo_increment_position(x_handle, actual_x_step);
        }
        
        if (step_y) {
            gpio_set_level(servo_infos[y_axis_idx].gpio_dir, (actual_y_step > 0) ? 1 : 0);
            esp_err_t ret = clearpath_servo_generate_step_pulse(y_handle);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to generate Y step pulse");
                group->active = false;
                return steps_generated;
            }
            clearpath_servo_increment_position(y_handle, actual_y_step);
        }
        
        // Handle Z axis for helical arcs (linear interpolation synchronized with arc)
        if (group->axis_count >= 3) {
            uint8_t z_axis_idx = group->axis_indices[2];
            if (z_axis_idx < CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
                clearpath_servo_handle_t *z_handle = servo_infos[z_axis_idx].handle;
                if (z_handle != NULL && group->z_total_distance != 0) {
                    // Use Bresenham line algorithm for Z axis
                    // Z moves linearly proportionally to arc progress
                    // Error term accumulates: error += abs(z_total_distance)
                    // When error >= arc_length, step Z
                    group->z_error += abs(group->z_total_distance);
                    
                    if (group->z_error >= group->bresenham_steps_total) {
                        // Step Z axis
                        group->z_error -= group->bresenham_steps_total;
                        group->z_current_position += group->z_step_dir;
                        
                        gpio_set_level(servo_infos[z_axis_idx].gpio_dir, (group->z_step_dir > 0) ? 1 : 0);
                        esp_err_t ret = clearpath_servo_generate_step_pulse(z_handle);
                        if (ret != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to generate Z step pulse");
                            group->active = false;
                            return steps_generated;
                        }
                        clearpath_servo_increment_position(z_handle, group->z_step_dir);
                    }
                }
            }
        }
        
        group->bresenham_steps_generated++;
        steps_generated++;
        group->bresenham_last_step_time_us = current_time_us;
        
        // Check if we've reached the end angle
        // Calculate current angle from Bresenham position and octant
        // In first octant: angle = atan2(y, x) but we need to map through octant
        // Simplified: check if we've generated enough steps (arc_length)
        // More accurate: calculate current angle and compare to end angle
        if (group->bresenham_steps_generated >= group->bresenham_steps_total) {
            break;
        }
        
        // Additional check: if we've completed a full circle (x >= radius in first octant)
        // This shouldn't happen for arcs, but protects against infinite loops
        if (group->bresenham_x >= group->radius || group->bresenham_y <= 0) {
            // Reached 45° in first octant - would transition to next octant
            // For arcs, we should have completed by step count, but check anyway
            if (group->bresenham_steps_generated >= group->bresenham_steps_total) {
                break;
            }
        }
    }
    
    return steps_generated;
}

/**
 * @brief Generate step pulse for a servo using RMT
 * 
 * This function generates a single step pulse using RMT hardware and updates position.
 * Called from the step generation task.
 */
static void generate_step(servo_instance_t *servo)
{
    if (servo == NULL || !servo->initialized || servo->handle == NULL) {
        return;
    }

    // Generate step pulse using RMT hardware peripheral
    // RMT provides precise timing (typically 10 microseconds pulse width)
    esp_err_t ret = clearpath_servo_generate_step_pulse(servo->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to generate step pulse for servo");
        return;
    }

    // Update position based on direction
    // Direction GPIO: HIGH = forward (positive), LOW = reverse (negative)
    // Note: GPIO read is atomic, but direction could change between read and increment.
    // This is acceptable for step generation - we use the direction at the time of step.
    bool dir = gpio_get_level(servo->config.gpio_dir);
    int32_t delta = dir ? 1 : -1;
    
    // Atomically increment position to avoid race condition
    // This avoids the TOCTOU race condition of get_position + set_position
    clearpath_servo_increment_position(servo->handle, delta);
}

/**
 * @brief Step generation task
 * 
 * This task runs at configurable frequency (default 5 kHz) to generate step pulses
 * based on velocity and acceleration profiles. Step pulses are generated using the
 * RMT hardware peripheral for precise timing.
 * 
 * RMT hardware provides accurate pulse timing independent of task scheduling,
 * enabling higher step rates than software GPIO toggling.
 */
static void step_generation_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t task_interval = pdMS_TO_TICKS(1); // Check every 1ms
    uint64_t last_update_us = esp_timer_get_time();
    uint64_t last_assembly_update_us = 0;

    ESP_LOGI(TAG, "Step generation task started");

    while (1) {
        uint64_t now_us = esp_timer_get_time();
        uint64_t delta_us = now_us - last_update_us;

        if (delta_us >= STEP_GENERATION_TASK_INTERVAL_US) {
            // Process each servo - copy handles and config under mutex protection
            servo_info_t servo_infos[CLEARPATH_SERVO_MANAGER_MAX_SERVOS] = {0};
            
            if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                for (int i = 0; i < CLEARPATH_SERVO_MANAGER_MAX_SERVOS; i++) {
                    if (s_servos[i].initialized) {
                        servo_infos[i].handle = s_servos[i].handle;
                        servo_infos[i].gpio_dir = s_servos[i].config.gpio_dir;
                    }
                }
                xSemaphoreGive(s_config_mutex);
            }
            
            // Check if coordinated motion is active
            bool has_coordinated_move = false;
            if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                has_coordinated_move = s_coordinated_group.active;
                xSemaphoreGive(s_config_mutex);
            }
            
            if (has_coordinated_move) {
                // Process coordinated motion using Bresenham algorithms (linear or circular)
                // Generate steps directly instead of updating targets
                if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    // Calculate max steps per interval based on feedrate (overrun protection)
                    int32_t max_steps_per_interval = 100;  // Default limit
                    if (s_coordinated_group.feedrate > 0) {
                        // max_steps = (feedrate * delta_us) / 1000000 + 1
                        int64_t max_steps_64 = ((int64_t)s_coordinated_group.feedrate * (int64_t)delta_us) / 1000000LL + 1;
                        if (max_steps_64 > 0 && max_steps_64 < INT32_MAX) {
                            max_steps_per_interval = (int32_t)max_steps_64;
                        }
                    }
                    
                    if (s_coordinated_group.type == COORDINATED_MOTION_LINEAR) {
                        generate_bresenham_linear_steps(&s_coordinated_group, servo_infos, max_steps_per_interval);
                    } else if (s_coordinated_group.type == COORDINATED_MOTION_CIRCULAR) {
                        generate_bresenham_circular_steps(&s_coordinated_group, servo_infos, max_steps_per_interval);
                    }
                    xSemaphoreGive(s_config_mutex);
                }
            }
            
            for (int i = 0; i < CLEARPATH_SERVO_MANAGER_MAX_SERVOS; i++) {
                if (servo_infos[i].handle == NULL) {
                    continue;
                }

                // Get servo state and parameters
                clearpath_servo_state_t state = clearpath_servo_get_state(servo_infos[i].handle);
                
                // For coordinated motion, the coordinated handler updates targets dynamically
                // We still need to process velocity profiles and generate steps
                // The coordinated handler runs first and updates target positions
                // Then we process normally (which will use the updated targets)
                int32_t current_pos = clearpath_servo_get_position(servo_infos[i].handle);
                int32_t current_vel = clearpath_servo_get_velocity(servo_infos[i].handle);
                int32_t target_pos = clearpath_servo_get_target_position(servo_infos[i].handle);
                int32_t target_vel = clearpath_servo_get_target_velocity(servo_infos[i].handle);
                uint32_t vel_max = clearpath_servo_get_vel_max(servo_infos[i].handle);
                uint32_t accel_max = clearpath_servo_get_accel_max(servo_infos[i].handle);
                uint32_t jerk_max = clearpath_servo_get_jerk_max(servo_infos[i].handle);
                clearpath_servo_profile_type_t profile_type = clearpath_servo_get_profile_type(servo_infos[i].handle);
                int32_t current_accel = clearpath_servo_get_current_acceleration(servo_infos[i].handle);

                // Integer math: delta_time_us is in microseconds
                // For calculations: multiply by delta_us, divide by 1,000,000
                // Use 64-bit intermediate to avoid overflow

                // Motion profile implementation (trapezoidal or S-curve)
                int32_t new_velocity = current_vel;
                int32_t new_acceleration = current_accel;
                bool should_generate_step = false;

                if (state == CLEARPATH_SERVO_STATE_MOVING || state == CLEARPATH_SERVO_STATE_COORDINATED) {
                    // Position move: trapezoidal or S-curve profile
                    int32_t steps_remaining = target_pos - current_pos;
                    int32_t steps_remaining_abs = (steps_remaining < 0) ? -steps_remaining : steps_remaining;

                    if (steps_remaining_abs == 0) {
                        // Reached target, stop
                        new_velocity = 0;
                        new_acceleration = 0;
                        clearpath_servo_stop(servo_infos[i].handle);
                    } else {
                        // Determine direction
                        int32_t direction = (steps_remaining > 0) ? 1 : -1;
                        int32_t target_vel_signed = (target_vel > 0) ? (int32_t)vel_max : -(int32_t)vel_max;
                        if (target_vel != 0) {
                            target_vel_signed = target_vel;
                        }

                        if (profile_type == CLEARPATH_SERVO_PROFILE_S_CURVE) {
                            // S-curve profile: jerk-limited acceleration/deceleration
                            // Calculate deceleration distance for S-curve (more complex)
                            // For S-curve: decel_distance includes jerk-up, constant accel, and jerk-down phases
                            int32_t decel_distance = 0;
                            if (current_vel != 0) {
                                int32_t vel_abs = (current_vel < 0) ? -current_vel : current_vel;
                                int32_t accel_abs = (current_accel < 0) ? -current_accel : current_accel;
                                
                                // S-curve deceleration: v^2/(2*a) + v*a/(2*j) for jerk phases
                                // Simplified: use current acceleration if available, otherwise use accel_max
                                int32_t effective_accel = (accel_abs > 0) ? accel_abs : (int32_t)accel_max;
                                
                                // Distance for constant deceleration phase
                                int64_t decel_dist_64 = ((int64_t)vel_abs * (int64_t)vel_abs) / (2 * effective_accel);
                                
                                // Add distance for jerk-down phase (acceleration ramps from current to 0)
                                if (accel_abs > 0 && jerk_max > 0) {
                                    int64_t jerk_down_time_64 = ((int64_t)accel_abs * 1000000LL) / (int64_t)jerk_max;  // microseconds
                                    int64_t jerk_down_dist_64 = ((int64_t)vel_abs * jerk_down_time_64) / 1000000LL;
                                    decel_dist_64 += jerk_down_dist_64 / 2;  // Average velocity during jerk-down
                                }
                                
                                decel_distance = (int32_t)decel_dist_64;
                                if (decel_distance < 1) decel_distance = 1;
                            }

                            // Determine phase for S-curve
                            if (steps_remaining_abs <= decel_distance) {
                                // DECEL phase: approaching target, decelerate with S-curve
                                int32_t vel_abs = (current_vel < 0) ? -current_vel : current_vel;
                                int32_t accel_abs = (current_accel < 0) ? -current_accel : current_accel;
                                
                                if (accel_abs > 0) {
                                    // Jerk-down phase: reduce acceleration
                                    int64_t jerk_change_64 = ((int64_t)jerk_max * (int64_t)delta_us) / 1000000LL;
                                    int32_t jerk_change = (int32_t)jerk_change_64;
                                    if (jerk_change < 1) jerk_change = 1;
                                    
                                    if (current_accel > 0) {
                                        new_acceleration = current_accel - jerk_change;
                                        if (new_acceleration < 0) new_acceleration = 0;
                                    } else if (current_accel < 0) {
                                        new_acceleration = current_accel + jerk_change;
                                        if (new_acceleration > 0) new_acceleration = 0;
                                    }
                                } else {
                                    // Constant deceleration phase: maintain max deceleration
                                    new_acceleration = (direction > 0) ? -(int32_t)accel_max : (int32_t)accel_max;
                                }
                                
                                // Update velocity based on acceleration
                                int64_t vel_change_64 = ((int64_t)new_acceleration * (int64_t)delta_us) / 1000000LL;
                                int32_t vel_change = (int32_t)vel_change_64;
                                
                                if (current_vel > 0) {
                                    new_velocity = current_vel + vel_change;
                                    if (new_velocity < 0) {
                                        new_velocity = 0;
                                        new_acceleration = 0;
                                    }
                                } else if (current_vel < 0) {
                                    new_velocity = current_vel + vel_change;
                                    if (new_velocity > 0) {
                                        new_velocity = 0;
                                        new_acceleration = 0;
                                    }
                                } else {
                                    new_velocity = 0;
                                    new_acceleration = 0;
                                }
                            } else {
                                // ACCEL or CONSTANT phase for S-curve
                                int32_t vel_abs = (current_vel < 0) ? -current_vel : current_vel;
                                int32_t target_vel_abs = (target_vel_signed < 0) ? -target_vel_signed : target_vel_signed;
                                
                                if (vel_abs < target_vel_abs) {
                                    // ACCEL phase: increase acceleration with jerk, then constant acceleration
                                    int32_t accel_abs = (current_accel < 0) ? -current_accel : current_accel;
                                    
                                    if (accel_abs < (int32_t)accel_max) {
                                        // Jerk-up phase: increase acceleration
                                        int64_t jerk_change_64 = ((int64_t)jerk_max * (int64_t)delta_us) / 1000000LL;
                                        int32_t jerk_change = (int32_t)jerk_change_64;
                                        if (jerk_change < 1) jerk_change = 1;
                                        
                                        if (direction > 0) {
                                            new_acceleration = current_accel + jerk_change;
                                            if (new_acceleration > (int32_t)accel_max) new_acceleration = (int32_t)accel_max;
                                        } else {
                                            new_acceleration = current_accel - jerk_change;
                                            if (new_acceleration < -(int32_t)accel_max) new_acceleration = -(int32_t)accel_max;
                                        }
                                    } else {
                                        // Constant acceleration phase: maintain max acceleration
                                        new_acceleration = (direction > 0) ? (int32_t)accel_max : -(int32_t)accel_max;
                                    }
                                    
                                    // Update velocity based on acceleration
                                    int64_t vel_change_64 = ((int64_t)new_acceleration * (int64_t)delta_us) / 1000000LL;
                                    int32_t vel_change = (int32_t)vel_change_64;
                                    
                                    if (direction > 0) {
                                        new_velocity = current_vel + vel_change;
                                        if (new_velocity > target_vel_signed) {
                                            new_velocity = target_vel_signed;
                                            new_acceleration = 0;  // Start decelerating
                                        }
                                        if (new_velocity > (int32_t)vel_max) {
                                            new_velocity = (int32_t)vel_max;
                                            new_acceleration = 0;  // Reached max velocity
                                        }
                                    } else {
                                        new_velocity = current_vel + vel_change;
                                        if (new_velocity < target_vel_signed) {
                                            new_velocity = target_vel_signed;
                                            new_acceleration = 0;
                                        }
                                        if (new_velocity < -(int32_t)vel_max) {
                                            new_velocity = -(int32_t)vel_max;
                                            new_acceleration = 0;
                                        }
                                    }
                                } else {
                                    // CONSTANT velocity phase: maintain target velocity, zero acceleration
                                    new_velocity = target_vel_signed;
                                    new_acceleration = 0;
                                }
                            }
                        } else {
                            // Trapezoidal profile: constant acceleration/deceleration
                            // Calculate deceleration distance: v^2 / (2 * a)
                            int32_t decel_distance = 0;
                            if (current_vel != 0) {
                                int32_t vel_abs = (current_vel < 0) ? -current_vel : current_vel;
                                decel_distance = (int32_t)((int64_t)vel_abs * (int64_t)vel_abs) / (2 * (int32_t)accel_max);
                                if (decel_distance < 1) decel_distance = 1;
                            }

                            // Determine phase: ACCEL, CONSTANT, or DECEL
                            if (steps_remaining_abs <= decel_distance) {
                                // DECEL phase: approaching target, decelerate
                                int64_t vel_change_64 = ((int64_t)accel_max * (int64_t)delta_us) / 1000000LL;
                                int32_t vel_change = (int32_t)vel_change_64;
                                if (vel_change < 1) vel_change = 1;
                                
                                if (current_vel > 0) {
                                    new_velocity = current_vel - vel_change;
                                    if (new_velocity < 0) new_velocity = 0;
                                } else if (current_vel < 0) {
                                    new_velocity = current_vel + vel_change;
                                    if (new_velocity > 0) new_velocity = 0;
                                } else {
                                    new_velocity = 0;
                                }
                                new_acceleration = 0;  // Trapezoidal doesn't track acceleration
                            } else {
                                // ACCEL or CONSTANT phase
                                int32_t vel_abs = (current_vel < 0) ? -current_vel : current_vel;
                                int32_t target_vel_abs = (target_vel_signed < 0) ? -target_vel_signed : target_vel_signed;
                                
                                if (vel_abs < target_vel_abs) {
                                    // ACCEL phase: accelerate toward target velocity
                                    int64_t vel_change_64 = ((int64_t)accel_max * (int64_t)delta_us) / 1000000LL;
                                    int32_t vel_change = (int32_t)vel_change_64;
                                    if (vel_change < 1) vel_change = 1;
                                    
                                    if (direction > 0) {
                                        new_velocity = current_vel + vel_change;
                                        if (new_velocity > target_vel_signed) new_velocity = target_vel_signed;
                                        if (new_velocity > (int32_t)vel_max) new_velocity = (int32_t)vel_max;
                                    } else {
                                        new_velocity = current_vel - vel_change;
                                        if (new_velocity < target_vel_signed) new_velocity = target_vel_signed;
                                        if (new_velocity < -(int32_t)vel_max) new_velocity = -(int32_t)vel_max;
                                    }
                                    new_acceleration = 0;  // Trapezoidal doesn't track acceleration
                                } else {
                                    // CONSTANT phase: maintain target velocity
                                    new_velocity = target_vel_signed;
                                    new_acceleration = 0;
                                }
                            }
                        }

                        // Update acceleration for S-curve profile tracking
                        if (profile_type == CLEARPATH_SERVO_PROFILE_S_CURVE) {
                            clearpath_servo_set_current_acceleration(servo_infos[i].handle, new_acceleration);
                        }

                        // Set direction GPIO based on target direction
                        gpio_set_level(servo_infos[i].gpio_dir, (direction > 0) ? 1 : 0);

                        // Calculate steps to generate based on velocity (integer math)
                        // steps = (velocity * delta_us) / 1000000
                        int64_t steps_64 = ((int64_t)new_velocity * (int64_t)delta_us) / 1000000LL;
                        int32_t steps_to_generate = (int32_t)steps_64;
                        
                        // Generate steps (limit to reasonable number per interval)
                        // max_steps = (vel_max * delta_us) / 1000000 + 1
                        int64_t max_steps_64 = ((int64_t)vel_max * (int64_t)delta_us) / 1000000LL + 1;
                        int32_t max_steps_per_interval = (int32_t)max_steps_64;
                        if (steps_to_generate > max_steps_per_interval) {
                            steps_to_generate = max_steps_per_interval;
                        }
                        if (steps_to_generate < -max_steps_per_interval) {
                            steps_to_generate = -max_steps_per_interval;
                        }

                        // Generate step pulses
                        int32_t steps_generated = 0;
                        int32_t step_direction = (steps_to_generate >= 0) ? 1 : -1;
                        int32_t steps_abs = (steps_to_generate < 0) ? -steps_to_generate : steps_to_generate;
                        
                        for (int32_t s = 0; s < steps_abs && steps_generated < max_steps_per_interval; s++) {
                            esp_err_t ret = clearpath_servo_generate_step_pulse(servo_infos[i].handle);
                            if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "Failed to generate step pulse for servo %d", i);
                                break;
                            }
                            
                            // Atomically increment position
                            clearpath_servo_increment_position(servo_infos[i].handle, step_direction);
                            steps_generated++;
                        }

                        should_generate_step = (steps_generated > 0);
                    }
                    
                    // Update velocity for profile tracking
                    clearpath_servo_set_current_velocity(servo_infos[i].handle, new_velocity);
                } else if (state == CLEARPATH_SERVO_STATE_HOMING) {
                    // Homing mode: move at homing velocity until sensor triggers
                    // Check timeout first
                    if (clearpath_servo_is_homing_timeout(servo_infos[i].handle)) {
                        // Timeout reached - stop homing
                        clearpath_servo_stop(servo_infos[i].handle);
                        ESP_LOGW(TAG, "Homing timeout for servo %d", i);
                        new_velocity = 0;
                    } else {
                        // Check if homing sensor is triggered
                        bool sensor_triggered = clearpath_servo_get_homing_sensor_status(servo_infos[i].handle);
                        
                        if (sensor_triggered) {
                            // Sensor triggered - stop motion and set position to 0
                            // Use internal function to properly mark homing complete
                            clearpath_servo_stop(servo_infos[i].handle);
                            clearpath_servo_set_position(servo_infos[i].handle, 0);
                            // Mark homing complete by updating state (homing_complete flag is internal)
                            // The stop() function already transitions state to IDLE
                            ESP_LOGI(TAG, "Homing complete for servo %d - position set to 0", i);
                            new_velocity = 0;
                            } else {
                                // Continue homing motion - use velocity mode logic with profile support
                                // Homing velocity is already set in target_velocity by clearpath_servo_home()
                                if (target_vel == 0) {
                                    new_velocity = 0;
                                    new_acceleration = 0;
                                    clearpath_servo_stop(servo_infos[i].handle);
                                } else {
                                    if (profile_type == CLEARPATH_SERVO_PROFILE_S_CURVE) {
                                        // S-curve homing: ramp with jerk-limited acceleration
                                        int32_t vel_diff = target_vel - current_vel;
                                        int32_t vel_diff_abs = (vel_diff < 0) ? -vel_diff : vel_diff;
                                        
                                        if (vel_diff_abs > 0) {
                                            int32_t accel_abs = (current_accel < 0) ? -current_accel : current_accel;
                                            
                                            if (accel_abs < (int32_t)accel_max) {
                                                int64_t jerk_change_64 = ((int64_t)jerk_max * (int64_t)delta_us) / 1000000LL;
                                                int32_t jerk_change = (int32_t)jerk_change_64;
                                                if (jerk_change < 1) jerk_change = 1;
                                                
                                                if (vel_diff > 0) {
                                                    new_acceleration = current_accel + jerk_change;
                                                    if (new_acceleration > (int32_t)accel_max) new_acceleration = (int32_t)accel_max;
                                                } else {
                                                    new_acceleration = current_accel - jerk_change;
                                                    if (new_acceleration < -(int32_t)accel_max) new_acceleration = -(int32_t)accel_max;
                                                }
                                            } else {
                                                new_acceleration = (vel_diff > 0) ? (int32_t)accel_max : -(int32_t)accel_max;
                                            }
                                            
                                            int64_t vel_change_64 = ((int64_t)new_acceleration * (int64_t)delta_us) / 1000000LL;
                                            int32_t vel_change = (int32_t)vel_change_64;
                                            
                                            new_velocity = current_vel + vel_change;
                                            if ((vel_diff > 0 && new_velocity > target_vel) || (vel_diff < 0 && new_velocity < target_vel)) {
                                                new_velocity = target_vel;
                                                new_acceleration = 0;
                                            }
                                        } else {
                                            new_velocity = target_vel;
                                            new_acceleration = 0;
                                        }
                                        clearpath_servo_set_current_acceleration(servo_infos[i].handle, new_acceleration);
                                    } else {
                                        // Trapezoidal homing: constant acceleration ramp
                                        int64_t vel_change_64 = ((int64_t)accel_max * (int64_t)delta_us) / 1000000LL;
                                        int32_t vel_change = (int32_t)vel_change_64;
                                        if (vel_change < 1) vel_change = 1;
                                        
                                        if (current_vel < target_vel) {
                                            new_velocity = current_vel + vel_change;
                                            if (new_velocity > target_vel) new_velocity = target_vel;
                                        } else if (current_vel > target_vel) {
                                            new_velocity = current_vel - vel_change;
                                            if (new_velocity < target_vel) new_velocity = target_vel;
                                        } else {
                                            new_velocity = target_vel;
                                        }
                                        new_acceleration = 0;
                                    }

                                // Set direction GPIO
                                gpio_set_level(servo_infos[i].gpio_dir, (target_vel >= 0) ? 1 : 0);

                                // Calculate and generate steps (integer math)
                                // steps = (velocity * delta_us) / 1000000
                                int64_t steps_64 = ((int64_t)new_velocity * (int64_t)delta_us) / 1000000LL;
                                int32_t steps_to_generate = (int32_t)steps_64;
                                
                                // max_steps = (vel_max * delta_us) / 1000000 + 1
                                int64_t max_steps_64 = ((int64_t)vel_max * (int64_t)delta_us) / 1000000LL + 1;
                                int32_t max_steps_per_interval = (int32_t)max_steps_64;
                                if (steps_to_generate > max_steps_per_interval) {
                                    steps_to_generate = max_steps_per_interval;
                                }
                                if (steps_to_generate < -max_steps_per_interval) {
                                    steps_to_generate = -max_steps_per_interval;
                                }

                                int32_t steps_generated = 0;
                                int32_t step_direction = (steps_to_generate >= 0) ? 1 : -1;
                                int32_t steps_abs = (steps_to_generate < 0) ? -steps_to_generate : steps_to_generate;
                                
                                for (int32_t s = 0; s < steps_abs && steps_generated < max_steps_per_interval; s++) {
                                    esp_err_t ret = clearpath_servo_generate_step_pulse(servo_infos[i].handle);
                                    if (ret != ESP_OK) {
                                        ESP_LOGE(TAG, "Failed to generate step pulse for servo %d", i);
                                        break;
                                    }
                                    
                                    clearpath_servo_increment_position(servo_infos[i].handle, step_direction);
                                    steps_generated++;
                                }

                                should_generate_step = (steps_generated > 0);
                                
                                // Update velocity for profile tracking
                                clearpath_servo_set_current_velocity(servo_infos[i].handle, new_velocity);
                            }
                        }
                    }
                } else if (state == CLEARPATH_SERVO_STATE_VELOCITY_MODE) {
                    // Velocity mode: continuous motion at target velocity
                    if (target_vel == 0) {
                        new_velocity = 0;
                        new_acceleration = 0;
                        clearpath_servo_stop(servo_infos[i].handle);
                    } else {
                        if (profile_type == CLEARPATH_SERVO_PROFILE_S_CURVE) {
                            // S-curve velocity mode: ramp with jerk-limited acceleration
                            int32_t vel_diff = target_vel - current_vel;
                            int32_t vel_diff_abs = (vel_diff < 0) ? -vel_diff : vel_diff;
                            
                            if (vel_diff_abs > 0) {
                                // Need to accelerate/decelerate
                                int32_t accel_abs = (current_accel < 0) ? -current_accel : current_accel;
                                
                                if (accel_abs < (int32_t)accel_max) {
                                    // Jerk-up phase
                                    int64_t jerk_change_64 = ((int64_t)jerk_max * (int64_t)delta_us) / 1000000LL;
                                    int32_t jerk_change = (int32_t)jerk_change_64;
                                    if (jerk_change < 1) jerk_change = 1;
                                    
                                    if (vel_diff > 0) {
                                        new_acceleration = current_accel + jerk_change;
                                        if (new_acceleration > (int32_t)accel_max) new_acceleration = (int32_t)accel_max;
                                    } else {
                                        new_acceleration = current_accel - jerk_change;
                                        if (new_acceleration < -(int32_t)accel_max) new_acceleration = -(int32_t)accel_max;
                                    }
                                } else {
                                    // Constant acceleration phase
                                    new_acceleration = (vel_diff > 0) ? (int32_t)accel_max : -(int32_t)accel_max;
                                }
                                
                                // Update velocity
                                int64_t vel_change_64 = ((int64_t)new_acceleration * (int64_t)delta_us) / 1000000LL;
                                int32_t vel_change = (int32_t)vel_change_64;
                                
                                new_velocity = current_vel + vel_change;
                                if ((vel_diff > 0 && new_velocity > target_vel) || (vel_diff < 0 && new_velocity < target_vel)) {
                                    new_velocity = target_vel;
                                    new_acceleration = 0;
                                }
                            } else {
                                // At target velocity
                                new_velocity = target_vel;
                                new_acceleration = 0;
                            }
                            clearpath_servo_set_current_acceleration(servo_infos[i].handle, new_acceleration);
                        } else {
                            // Trapezoidal velocity mode: constant acceleration ramp
                            int64_t vel_change_64 = ((int64_t)accel_max * (int64_t)delta_us) / 1000000LL;
                            int32_t vel_change = (int32_t)vel_change_64;
                            if (vel_change < 1) vel_change = 1;
                            
                            if (current_vel < target_vel) {
                                new_velocity = current_vel + vel_change;
                                if (new_velocity > target_vel) new_velocity = target_vel;
                            } else if (current_vel > target_vel) {
                                new_velocity = current_vel - vel_change;
                                if (new_velocity < target_vel) new_velocity = target_vel;
                            } else {
                                new_velocity = target_vel;
                            }
                            new_acceleration = 0;
                        }

                        // Update velocity for profile tracking
                        clearpath_servo_set_current_velocity(servo_infos[i].handle, new_velocity);

                        // Set direction GPIO
                        gpio_set_level(servo_infos[i].gpio_dir, (target_vel >= 0) ? 1 : 0);

                        // Calculate and generate steps (integer math)
                        // steps = (velocity * delta_us) / 1000000
                        int64_t steps_64 = ((int64_t)new_velocity * (int64_t)delta_us) / 1000000LL;
                        int32_t steps_to_generate = (int32_t)steps_64;
                        
                        // max_steps = (vel_max * delta_us) / 1000000 + 1
                        int64_t max_steps_64 = ((int64_t)vel_max * (int64_t)delta_us) / 1000000LL + 1;
                        int32_t max_steps_per_interval = (int32_t)max_steps_64;
                        if (steps_to_generate > max_steps_per_interval) {
                            steps_to_generate = max_steps_per_interval;
                        }
                        if (steps_to_generate < -max_steps_per_interval) {
                            steps_to_generate = -max_steps_per_interval;
                        }

                        int32_t steps_generated = 0;
                        int32_t step_direction = (steps_to_generate >= 0) ? 1 : -1;
                        int32_t steps_abs = (steps_to_generate < 0) ? -steps_to_generate : steps_to_generate;
                        
                        for (int32_t s = 0; s < steps_abs && steps_generated < max_steps_per_interval; s++) {
                            esp_err_t ret = clearpath_servo_generate_step_pulse(servo_infos[i].handle);
                            if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "Failed to generate step pulse for servo %d", i);
                                break;
                            }
                            
                            clearpath_servo_increment_position(servo_infos[i].handle, step_direction);
                            steps_generated++;
                        }

                        should_generate_step = (steps_generated > 0);
                    }
                } else {
                    // IDLE state: no motion
                    new_velocity = 0;
                }

                // Velocity already updated above for MOVING and HOMING states
                // Only update here for other states if needed
                if (state != CLEARPATH_SERVO_STATE_MOVING && 
                    state != CLEARPATH_SERVO_STATE_COORDINATED &&
                    state != CLEARPATH_SERVO_STATE_HOMING &&
                    state != CLEARPATH_SERVO_STATE_VELOCITY_MODE &&
                    new_velocity != current_vel) {
                    clearpath_servo_set_current_velocity(servo_infos[i].handle, new_velocity);
                }

                // Update last step time if steps were generated
                if (should_generate_step && s_config_mutex != NULL && 
                    xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    s_servos[i].last_step_time_us = now_us;
                    xSemaphoreGive(s_config_mutex);
                }
            }

            last_update_us = now_us;
        }

        // Update assembly data periodically
        if (now_us - last_assembly_update_us > (ASSEMBLY_UPDATE_INTERVAL_MS * 1000)) {
            SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
            if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Update servo 0 feedback (bytes 61-71)
                // Check initialization under config mutex for thread safety
                bool servo0_initialized = false;
                clearpath_servo_handle_t *servo0_handle = NULL;
                if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    servo0_initialized = s_servos[0].initialized;
                    servo0_handle = s_servos[0].handle;
                    xSemaphoreGive(s_config_mutex);
                }
                
                if (servo0_initialized && servo0_handle != NULL) {
                    int32_t position = clearpath_servo_get_position(servo0_handle);
                    int32_t velocity = clearpath_servo_get_velocity(servo0_handle);
                    
                    // Write position (bytes 61-64)
                    memcpy(&INPUT_ASSEMBLY_100[61], &position, sizeof(int32_t));
                    
                    // Write velocity (bytes 65-66)
                    int16_t vel_16 = (int16_t)velocity;
                    memcpy(&INPUT_ASSEMBLY_100[65], &vel_16, sizeof(int16_t));
                    
                    // Write status (byte 67)
                    uint8_t status = 0;
                    if (clearpath_servo_steps_complete(servo0_handle)) status |= 0x01; // Move complete
                    if (clearpath_servo_is_enabled(servo0_handle)) status |= 0x02; // Enabled
                    if (clearpath_servo_is_fault(servo0_handle)) status |= 0x04; // Fault
                    if (clearpath_servo_get_hlfb_status(servo0_handle)) status |= 0x08; // In position (from HLFB)
                    INPUT_ASSEMBLY_100[67] = status;
                }

                // TODO: Update servo 1 feedback (bytes 68-71) if needed

                xSemaphoreGive(assembly_mutex);
                last_assembly_update_us = now_us;
            }
        }

        // Read commands from Output Assembly 150
        SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
        if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            // Parse commands for each servo
            // Expanded layout: 5 bytes per servo (1 byte command + 4 bytes int32_t data)
            // Bytes 32-51: Servo commands (20 bytes total for 4 servos)
            // Note: Output Assembly 150 is currently 40 bytes (g_assembly_data096[40])
            //       This supports 1 servo fully (bytes 32-36) with 32-bit range
            //       For 4 servos, assembly needs to be expanded to at least 52 bytes
            #define OUTPUT_ASSEMBLY_150_SIZE 40  // Current size, expand if needed for more servos
            for (int i = 0; i < CLEARPATH_SERVO_MANAGER_MAX_SERVOS; i++) {
                uint8_t byte_offset = 32 + (i * 5); // 5 bytes per servo
                // Safety check: ensure we don't read beyond assembly bounds
                if (byte_offset + 4 >= OUTPUT_ASSEMBLY_150_SIZE) break;
                
                // Get servo handle under config mutex
                clearpath_servo_handle_t *servo_handle = NULL;
                if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    if (s_servos[i].initialized) {
                        servo_handle = s_servos[i].handle;
                    }
                    xSemaphoreGive(s_config_mutex);
                }
                
                if (servo_handle == NULL) {
                    continue; // Servo not initialized, skip
                }

                // Read command from assembly
                uint8_t cmd_type = OUTPUT_ASSEMBLY_150[byte_offset];
                
                // Command format:
                // cmd_type: 0=stop, 1=move_abs, 2=move_rel, 3=move_velocity, 4=enable, 5=disable, 6=home
                // For move/home commands, bytes byte_offset+1 to byte_offset+4 contain int32_t value (little-endian)
                //   - move_abs/move_rel: position/steps
                //   - move_velocity: velocity
                //   - home: direction (positive=forward, negative=reverse), timeout_ms in upper 16 bits (optional)
                
                switch (cmd_type) {
                    case 0: // Stop
                        clearpath_servo_stop(servo_handle);
                        break;
                        
                    case 1: // Move absolute - read 32-bit position
                        {
                            int32_t position = 0;
                            memcpy(&position, &OUTPUT_ASSEMBLY_150[byte_offset + 1], sizeof(int32_t));
                            clearpath_servo_move(servo_handle, position, CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE);
                        }
                        break;
                        
                    case 2: // Move relative - read 32-bit steps
                        {
                            int32_t steps = 0;
                            memcpy(&steps, &OUTPUT_ASSEMBLY_150[byte_offset + 1], sizeof(int32_t));
                            clearpath_servo_move(servo_handle, steps, CLEARPATH_SERVO_MOVE_TARGET_RELATIVE);
                        }
                        break;
                        
                    case 3: // Move velocity - read 32-bit velocity
                        {
                            int32_t velocity = 0;
                            memcpy(&velocity, &OUTPUT_ASSEMBLY_150[byte_offset + 1], sizeof(int32_t));
                            clearpath_servo_move_velocity(servo_handle, velocity);
                        }
                        break;
                        
                    case 4: // Enable
                        clearpath_servo_set_enable(servo_handle, true);
                        break;
                        
                    case 5: // Disable
                        clearpath_servo_set_enable(servo_handle, false);
                        break;
                        
                    case 6: // Home - read 32-bit value: lower 16 bits = direction, upper 16 bits = timeout_ms
                        {
                            int32_t value = 0;
                            memcpy(&value, &OUTPUT_ASSEMBLY_150[byte_offset + 1], sizeof(int32_t));
                            int32_t direction = (int16_t)(value & 0xFFFF);  // Sign-extend lower 16 bits
                            uint32_t timeout_ms = (uint32_t)((value >> 16) & 0xFFFF);  // Upper 16 bits
                            clearpath_servo_home(servo_handle, direction, timeout_ms);
                        }
                        break;
                        
                    default:
                        // Unknown command, ignore
                        break;
                }
            }
            
            xSemaphoreGive(assembly_mutex);
        }

        vTaskDelay(task_interval);
    }
}

/**
 * @brief Initialize servo manager
 */
esp_err_t clearpath_servo_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Servo manager already initialized");
        return ESP_OK;
    }

    s_config_mutex = xSemaphoreCreateMutex();
    if (s_config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create config mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(s_servos, 0, sizeof(s_servos));

    // Initialize servo 0 with default configuration
    clearpath_servo_manager_config_t config0 = {
        .gpio_step = DEFAULT_GPIO_STEP_0,
        .gpio_dir = DEFAULT_GPIO_DIR_0,
        .gpio_enable = DEFAULT_GPIO_ENABLE_0,
        .gpio_hlfb = DEFAULT_GPIO_HLFB_0,
        .gpio_homing_sensor = DEFAULT_GPIO_HOMING_SENSOR_0,
        .vel_max = DEFAULT_VEL_MAX,
        .accel_max = DEFAULT_ACCEL_MAX,
        .jerk_max = DEFAULT_JERK_MAX,
        .profile_type = DEFAULT_PROFILE_TYPE,
        .hlfb_mode = CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE,
        .hlfb_active_high = true,
        .homing_sensor_type = CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN,
        .homing_velocity = DEFAULT_HOMING_VELOCITY,
        .steps_per_unit = 800,  // Default: 800 steps/mm (or steps/inch if unit type is inches)
        .enabled = false
    };

    clearpath_servo_config_t servo_config0 = {
        .gpio_step = config0.gpio_step,
        .gpio_dir = config0.gpio_dir,
        .gpio_enable = config0.gpio_enable,
        .gpio_hlfb = config0.gpio_hlfb,
        .gpio_homing_sensor = config0.gpio_homing_sensor,
        .vel_max = config0.vel_max,
        .accel_max = config0.accel_max,
        .jerk_max = config0.jerk_max,
        .profile_type = config0.profile_type,
        .hlfb_mode = config0.hlfb_mode,
        .hlfb_active_high = config0.hlfb_active_high,
        .homing_sensor_type = config0.homing_sensor_type,
        .homing_velocity = config0.homing_velocity
    };

    clearpath_servo_handle_t *handle0 = NULL;
    esp_err_t ret = clearpath_servo_init(&servo_config0, &handle0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo 0");
        vSemaphoreDelete(s_config_mutex);
        return ret;
    }

    // Create step generation task first (before marking servo as initialized)
    xTaskCreatePinnedToCore(
        step_generation_task,
        "servo_step_gen",
        4096,
        NULL,
        10,  // High priority for step generation
        &s_task_handle,
        1    // Pin to core 1 (I/O core)
    );

    if (s_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create step generation task");
        clearpath_servo_deinit(handle0);
        vSemaphoreDelete(s_config_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Mark servo as initialized only after task creation succeeds
    s_servos[0].handle = handle0;
    s_servos[0].config = config0;
    s_servos[0].initialized = true;
    s_servos[0].last_position = 0;

    s_initialized = true;
    ESP_LOGI(TAG, "Servo manager initialized with %d servo(s)", 1);
    return ESP_OK;
}

bool clearpath_servo_manager_is_initialized(void)
{
    return s_initialized;
}

esp_err_t clearpath_servo_manager_get_config(uint8_t servo_index, clearpath_servo_manager_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_servos[servo_index].initialized) {
        memcpy(config, &s_servos[servo_index].config, sizeof(clearpath_servo_manager_config_t));
    } else {
        memset(config, 0, sizeof(clearpath_servo_manager_config_t));
    }

    xSemaphoreGive(s_config_mutex);
    return ESP_OK;
}

esp_err_t clearpath_servo_manager_set_config(uint8_t servo_index, const clearpath_servo_manager_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // TODO: Reinitialize servo with new configuration if already initialized
    memcpy(&s_servos[servo_index].config, config, sizeof(clearpath_servo_manager_config_t));

    xSemaphoreGive(s_config_mutex);
    return ESP_OK;
}

int32_t clearpath_servo_manager_get_position(uint8_t servo_index)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0;
    }

    clearpath_servo_handle_t *handle = NULL;
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            handle = s_servos[servo_index].handle;
        }
        xSemaphoreGive(s_config_mutex);
    }

    if (handle == NULL) {
        return 0;
    }

    return clearpath_servo_get_position(handle);
}

int32_t clearpath_servo_manager_get_velocity(uint8_t servo_index)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0;
    }

    clearpath_servo_handle_t *handle = NULL;
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            handle = s_servos[servo_index].handle;
        }
        xSemaphoreGive(s_config_mutex);
    }

    if (handle == NULL) {
        return 0;
    }

    return clearpath_servo_get_velocity(handle);
}

uint8_t clearpath_servo_manager_get_status(uint8_t servo_index)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0;
    }

    clearpath_servo_handle_t *handle = NULL;
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            handle = s_servos[servo_index].handle;
        }
        xSemaphoreGive(s_config_mutex);
    }

    if (handle == NULL) {
        return 0;
    }

    uint8_t status = 0;
    if (clearpath_servo_steps_complete(handle)) status |= 0x01;
    if (clearpath_servo_is_enabled(handle)) status |= 0x02;
    if (clearpath_servo_is_fault(handle)) status |= 0x04;
    if (clearpath_servo_get_hlfb_status(handle)) status |= 0x08;

    return status;
}

uint8_t clearpath_servo_manager_get_count(void)
{
    uint8_t count = 0;
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < CLEARPATH_SERVO_MANAGER_MAX_SERVOS; i++) {
            if (s_servos[i].initialized) {
                count++;
            }
        }
        xSemaphoreGive(s_config_mutex);
    }
    return count;
}

esp_err_t clearpath_servo_manager_home(uint8_t servo_index, int32_t direction, uint32_t timeout_ms)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return ESP_ERR_INVALID_ARG;
    }

    clearpath_servo_handle_t *handle = NULL;
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            handle = s_servos[servo_index].handle;
        }
        xSemaphoreGive(s_config_mutex);
    }

    if (handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return clearpath_servo_home(handle, direction, timeout_ms);
}

bool clearpath_servo_manager_is_homing_complete(uint8_t servo_index)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return false;
    }

    clearpath_servo_handle_t *handle = NULL;
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            handle = s_servos[servo_index].handle;
        }
        xSemaphoreGive(s_config_mutex);
    }

    if (handle == NULL) {
        return false;
    }

    return clearpath_servo_is_homing_complete(handle);
}

bool clearpath_servo_manager_get_homing_sensor_status(uint8_t servo_index)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return false;
    }

    clearpath_servo_handle_t *handle = NULL;
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (s_servos[servo_index].initialized) {
            handle = s_servos[servo_index].handle;
        }
        xSemaphoreGive(s_config_mutex);
    }

    if (handle == NULL) {
        return false;
    }

    return clearpath_servo_get_homing_sensor_status(handle);
}

esp_err_t clearpath_servo_manager_set_unit_type(clearpath_servo_unit_t unit_type)
{
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    s_unit_type = unit_type;
    
    xSemaphoreGive(s_config_mutex);
    return ESP_OK;
}

clearpath_servo_unit_t clearpath_servo_manager_get_unit_type(void)
{
    return s_unit_type;
}

esp_err_t clearpath_servo_manager_move_linear(
    uint8_t axis_count,
    const uint8_t *axis_indices,
    const float *target_positions,
    float feedrate
)
{
    if (axis_count < 2 || axis_count > MAX_COORDINATED_AXES) {
        ESP_LOGE(TAG, "Invalid axis count: %d (must be 2-4)", axis_count);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (axis_indices == NULL || target_positions == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if servos are initialized and have steps_per_unit configured
    for (uint8_t i = 0; i < axis_count; i++) {
        if (axis_indices[i] >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
            ESP_LOGE(TAG, "Invalid axis index: %d", axis_indices[i]);
            xSemaphoreGive(s_config_mutex);
            return ESP_ERR_INVALID_ARG;
        }
        if (!s_servos[axis_indices[i]].initialized) {
            ESP_LOGE(TAG, "Servo %d not initialized", axis_indices[i]);
            xSemaphoreGive(s_config_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        if (s_servos[axis_indices[i]].config.steps_per_unit == 0) {
            ESP_LOGE(TAG, "Servo %d has steps_per_unit not configured", axis_indices[i]);
            xSemaphoreGive(s_config_mutex);
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    // Stop any existing coordinated motion
    s_coordinated_group.active = false;
    
    // Initialize coordinated motion group
    memset(&s_coordinated_group, 0, sizeof(coordinated_motion_group_t));
    s_coordinated_group.type = COORDINATED_MOTION_LINEAR;
    s_coordinated_group.active = true;
    s_coordinated_group.axis_count = axis_count;
    s_coordinated_group.start_time_us = esp_timer_get_time();
    
    // Convert target positions from units to steps
    // Validate axis_count doesn't exceed array bounds
    if (axis_count > MAX_COORDINATED_AXES) {
        ESP_LOGE(TAG, "axis_count %d exceeds MAX_COORDINATED_AXES %d", axis_count, MAX_COORDINATED_AXES);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    int32_t target_positions_steps[MAX_COORDINATED_AXES];
    for (uint8_t i = 0; i < axis_count; i++) {
        target_positions_steps[i] = units_to_steps(axis_indices[i], target_positions[i]);
    }
    
    // Convert feedrate from units/minute to steps/second
    // Use first axis for feedrate conversion (all should have same unit type)
    uint32_t feedrate_steps_per_sec = feedrate_units_per_min_to_steps_per_sec(axis_indices[0], feedrate);
    if (feedrate_steps_per_sec == 0) {
        ESP_LOGE(TAG, "Invalid feedrate: 0 steps/sec");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    s_coordinated_group.feedrate = feedrate_steps_per_sec;
    
    // Copy axis indices and get start/target positions (in steps)
    // Calculate path length: sqrt(sum of squares of distances)
    int64_t path_length_squared_64 = 0;
    
    for (uint8_t i = 0; i < axis_count; i++) {
        s_coordinated_group.axis_indices[i] = axis_indices[i];
        
        clearpath_servo_handle_t *handle = s_servos[axis_indices[i]].handle;
        if (handle == NULL) {
            ESP_LOGE(TAG, "Servo %d handle is NULL", axis_indices[i]);
            xSemaphoreGive(s_config_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        s_coordinated_group.start_positions[i] = clearpath_servo_get_position(handle);
        s_coordinated_group.target_positions[i] = target_positions_steps[i];
        
        // Calculate distance for this axis (in steps)
        int32_t distance = s_coordinated_group.target_positions[i] - s_coordinated_group.start_positions[i];
        if (distance < 0) distance = -distance;
        s_coordinated_group.total_distance[i] = distance;
        
        // Accumulate squared distance for path length calculation
        path_length_squared_64 += ((int64_t)distance * (int64_t)distance);
    }
    
    // Calculate path length: sqrt(sum of squares)
    // Use integer square root algorithm (binary search method)
    int32_t path_length = 0;
    if (path_length_squared_64 > 0) {
        // Integer square root using binary search
        // Limit high to prevent overflow in mid * mid calculation
        // sqrt(INT64_MAX) ≈ 3,037,000,499, but we need int32_t result
        // So limit to sqrt(INT32_MAX^2) = INT32_MAX = 2,147,483,647
        int64_t max_sqrt = 2147483647LL;  // INT32_MAX
        int64_t low = 0;
        int64_t high = (path_length_squared_64 < max_sqrt) ? path_length_squared_64 : max_sqrt;
        int64_t mid = 0;
        int32_t iteration_count = 0;
        const int32_t max_iterations = 64;  // Prevent infinite loop
        
        while (low <= high && iteration_count < max_iterations) {
            mid = (low + high) / 2;
            // Check for overflow before squaring
            if (mid > 2147483647LL) {
                high = mid - 1;
                iteration_count++;
                continue;
            }
            int64_t square = mid * mid;
            
            if (square == path_length_squared_64) {
                path_length = (int32_t)mid;
                break;
            } else if (square < path_length_squared_64) {
                low = mid + 1;
                path_length = (int32_t)mid;  // Keep track of best approximation
            } else {
                high = mid - 1;
            }
            iteration_count++;
        }
        
        if (iteration_count >= max_iterations) {
            ESP_LOGW(TAG, "Square root calculation reached max iterations");
        }
    }
    
    s_coordinated_group.path_length = path_length;
    
    // Check for zero path length
    if (path_length == 0) {
        ESP_LOGW(TAG, "Zero path length, no movement needed");
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }
    
    // Calculate total time needed: time = path_length / feedrate_steps_per_sec
    // time_us = (path_length * 1,000,000) / feedrate_steps_per_sec
    // feedrate_steps_per_sec is already validated to be > 0 above
    // Check for overflow: path_length * 1000000LL could overflow
    int64_t path_length_64 = (int64_t)path_length;
    int64_t time_us_64 = (path_length_64 * 1000000LL) / feedrate_steps_per_sec;
    
    // Check for overflow/underflow in time calculation
    if (time_us_64 < 0 || time_us_64 > INT64_MAX / 2) {
        ESP_LOGE(TAG, "Total time calculation overflow: path_length=%d, feedrate=%u", 
                 path_length, feedrate_steps_per_sec);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    s_coordinated_group.total_time_us = (uint64_t)time_us_64;
    
    // Initialize Bresenham line algorithm state
    s_coordinated_group.bresenham_steps_total = path_length;
    s_coordinated_group.bresenham_steps_generated = 0;
    s_coordinated_group.bresenham_last_step_time_us = esp_timer_get_time();
    
    // Calculate step interval for feedrate control: step_interval_us = 1000000 / feedrate
    if (feedrate_steps_per_sec > 0) {
        s_coordinated_group.bresenham_step_interval_us = 1000000ULL / feedrate_steps_per_sec;
    } else {
        s_coordinated_group.bresenham_step_interval_us = 1000;  // Default 1ms
    }
    
    // Initialize Bresenham state for each axis
    for (uint8_t i = 0; i < axis_count; i++) {
        int32_t distance = s_coordinated_group.target_positions[i] - s_coordinated_group.start_positions[i];
        s_coordinated_group.bresenham_dx[i] = (distance < 0) ? -distance : distance;  // Absolute distance
        s_coordinated_group.bresenham_step_dir[i] = (distance >= 0) ? 1 : -1;
        s_coordinated_group.bresenham_current[i] = s_coordinated_group.start_positions[i];
        // Initialize error: error[i] = distance[i] * steps_generated - (current[i] - start[i]) * path_length
        // At start: error[i] = 0 (steps_generated = 0, current = start)
        s_coordinated_group.bresenham_error[i] = 0;
    }
    
    // Set all axes to coordinated state
    // Each axis will move at the velocity needed to complete its distance in total_time
    for (uint8_t i = 0; i < axis_count; i++) {
        clearpath_servo_handle_t *handle = s_servos[axis_indices[i]].handle;
        if (handle == NULL) {
            ESP_LOGE(TAG, "Servo %d handle is NULL", axis_indices[i]);
            xSemaphoreGive(s_config_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        // Set coordinated state
        clearpath_servo_set_state(handle, CLEARPATH_SERVO_STATE_COORDINATED);
        
        // Store original vel_max to restore after move
        uint32_t original_vel_max = clearpath_servo_get_vel_max(handle);
        if (i < MAX_COORDINATED_AXES) {
            s_coordinated_group.original_vel_max[i] = original_vel_max;
        }
        
        // Calculate required velocity for this axis: velocity = distance / time
        // velocity_steps_per_sec = (distance * 1,000,000) / time_us
        uint32_t axis_velocity = 0;
        if (s_coordinated_group.total_time_us > 0) {
            int64_t distance_64 = (int64_t)s_coordinated_group.total_distance[i];
            int64_t axis_velocity_64 = (distance_64 * 1000000LL) / s_coordinated_group.total_time_us;
            
            // Check for overflow/underflow
            if (axis_velocity_64 < 0) {
                axis_velocity = 0;
            } else if (axis_velocity_64 > UINT32_MAX) {
                ESP_LOGW(TAG, "Axis velocity overflow, clamping to max");
                axis_velocity = UINT32_MAX;
            } else {
                axis_velocity = (uint32_t)axis_velocity_64;
            }
        }
        
        // Limit to servo's maximum velocity
        if (axis_velocity > original_vel_max) {
            axis_velocity = original_vel_max;
        }
        
        // Temporarily set vel_max to calculated velocity for this move
        if (axis_velocity > 0) {
            clearpath_servo_set_vel_max(handle, axis_velocity);
        }
        
        // Start axis moving to target (in steps)
        clearpath_servo_move(handle, target_positions_steps[i], CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE);
    }
    
    xSemaphoreGive(s_config_mutex);
    
    ESP_LOGI(TAG, "Started linear coordinated move: %d axes, feedrate=%.2f units/min (%.2f steps/sec)", 
             axis_count, feedrate, (float)feedrate_steps_per_sec);
    return ESP_OK;
}

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
)
{
    if (axis_x_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS ||
        axis_y_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        ESP_LOGE(TAG, "Invalid axis indices");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if servos are initialized and have steps_per_unit configured
    if (!s_servos[axis_x_index].initialized || !s_servos[axis_y_index].initialized) {
        ESP_LOGE(TAG, "Servos not initialized");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_servos[axis_x_index].config.steps_per_unit == 0 || 
        s_servos[axis_y_index].config.steps_per_unit == 0) {
        ESP_LOGE(TAG, "Servos have steps_per_unit not configured");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert positions from units to steps
    int32_t center_x_steps = units_to_steps(axis_x_index, center_x);
    int32_t center_y_steps = units_to_steps(axis_y_index, center_y);
    int32_t radius_steps = units_to_steps(axis_x_index, radius);  // Use X axis for radius conversion
    
    // Validate radius is positive
    if (radius_steps <= 0) {
        ESP_LOGE(TAG, "Invalid radius: %d steps (must be > 0)", radius_steps);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert feedrate from units/minute to steps/second
    uint32_t feedrate_steps_per_sec = feedrate_units_per_min_to_steps_per_sec(axis_x_index, feedrate);
    if (feedrate_steps_per_sec == 0) {
        ESP_LOGE(TAG, "Invalid feedrate: 0 steps/sec");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert angles to 0.1 degree units (multiply by 10)
    int32_t start_angle_deg_x10 = start_angle_deg * 10;
    int32_t end_angle_deg_x10 = end_angle_deg * 10;
    
    // Normalize angles to 0-3600 (0-360 degrees in 0.1 degree units)
    start_angle_deg_x10 = start_angle_deg_x10 % 3600;
    if (start_angle_deg_x10 < 0) start_angle_deg_x10 += 3600;
    end_angle_deg_x10 = end_angle_deg_x10 % 3600;
    if (end_angle_deg_x10 < 0) end_angle_deg_x10 += 3600;
    
    // Calculate angle difference in 0.1 degree units
    int32_t angle_diff_x10 = end_angle_deg_x10 - start_angle_deg_x10;
    if (clockwise) {
        if (angle_diff_x10 > 0) {
            angle_diff_x10 -= 3600;
        }
    } else {
        if (angle_diff_x10 < 0) {
            angle_diff_x10 += 3600;
        }
    }
    
    // Check for zero angle difference (no movement needed)
    if (angle_diff_x10 == 0) {
        ESP_LOGW(TAG, "Zero angle difference, no arc movement needed");
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }
    
    // Calculate arc length: arc_length = radius * angle (in radians)
    // angle_rad = angle_deg * π / 180
    // Check for overflow in intermediate calculation
    // Use absolute angle difference (supports multi-turn arcs)
    int64_t radius_64 = (int64_t)radius_steps;
    int64_t angle_abs_64 = (int64_t)abs(angle_diff_x10);
    
    // Check for overflow: radius * angle_diff could be very large
    if (radius_64 > INT64_MAX / angle_abs_64) {
        ESP_LOGE(TAG, "Arc length calculation overflow: radius=%d, angle_diff=%d", 
                 radius_steps, angle_diff_x10);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    int64_t arc_length_64 = (radius_64 * angle_abs_64 * 314159LL) / 180000LL;
    
    // Check for overflow when casting to int32_t
    if (arc_length_64 > INT32_MAX || arc_length_64 < INT32_MIN) {
        ESP_LOGE(TAG, "Arc length overflow: %lld steps", arc_length_64);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    int32_t arc_length = (int32_t)arc_length_64;
    
    // Check for zero arc length
    if (arc_length == 0) {
        ESP_LOGW(TAG, "Zero arc length, no movement needed");
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }
    
    // Calculate total time needed: time = arc_length / feedrate
    // time_us = (arc_length * 1,000,000) / feedrate_steps_per_sec
    // feedrate_steps_per_sec is already validated to be > 0 above
    int64_t total_time_us_64 = ((int64_t)arc_length * 1000000LL) / feedrate_steps_per_sec;
    
    // Check for overflow/underflow in time calculation
    if (total_time_us_64 < 0 || total_time_us_64 > INT64_MAX / 2) {
        ESP_LOGE(TAG, "Arc total time calculation overflow: arc_length=%d, feedrate=%u", 
                 arc_length, feedrate_steps_per_sec);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t total_time_us = (uint64_t)total_time_us_64;
    
    // Stop any existing coordinated motion
    s_coordinated_group.active = false;
    
    // Initialize circular interpolation
    memset(&s_coordinated_group, 0, sizeof(coordinated_motion_group_t));
    s_coordinated_group.type = COORDINATED_MOTION_CIRCULAR;
    s_coordinated_group.active = true;
    s_coordinated_group.axis_count = 2;  // Will be updated to 3 if Z axis is provided
    s_coordinated_group.axis_indices[0] = axis_x_index;
    s_coordinated_group.axis_indices[1] = axis_y_index;
    // Z axis fields initialized to 0 (no Z movement)
    s_coordinated_group.z_start_position = 0;
    s_coordinated_group.z_end_position = 0;
    s_coordinated_group.z_total_distance = 0;
    s_coordinated_group.z_current_position = 0;
    s_coordinated_group.z_error = 0;
    s_coordinated_group.z_step_dir = 0;
    s_coordinated_group.center_x = center_x_steps;
    s_coordinated_group.center_y = center_y_steps;
    s_coordinated_group.radius = radius_steps;
    s_coordinated_group.feedrate = feedrate_steps_per_sec;
    s_coordinated_group.start_angle_deg_x10 = start_angle_for_bresenham_x10;
    s_coordinated_group.end_angle_deg_x10 = end_angle_for_bresenham_x10;
    s_coordinated_group.angle_diff_x10 = angle_diff_x10;
    s_coordinated_group.arc_length = arc_length;
    s_coordinated_group.current_angle_deg_x10 = start_angle_for_bresenham_x10;
    s_coordinated_group.start_time_us = esp_timer_get_time();
    s_coordinated_group.total_time_us = total_time_us;
    
    // Calculate starting positions using normalized start angle
    int32_t start_angle_deg = start_angle_for_bresenham_x10 / 10;
    int32_t cos_val = cos_lookup(start_angle_for_bresenham_x10, 1000);
    int32_t sin_val = sin_lookup(start_angle_for_bresenham_x10, 1000);
    
    int32_t start_x = center_x_steps + ((int64_t)radius_steps * cos_val) / 1000;
    int32_t start_y = center_y_steps + ((int64_t)radius_steps * sin_val) / 1000;
    
    s_coordinated_group.start_positions[0] = start_x;
    s_coordinated_group.start_positions[1] = start_y;
    
    // Move to starting position first (if not already there)
    clearpath_servo_handle_t *x_handle = s_servos[axis_x_index].handle;
    clearpath_servo_handle_t *y_handle = s_servos[axis_y_index].handle;
    
    // Validate handles are not NULL
    if (x_handle == NULL || y_handle == NULL) {
        ESP_LOGE(TAG, "Servo handles are NULL");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    int32_t current_x = clearpath_servo_get_position(x_handle);
    int32_t current_y = clearpath_servo_get_position(y_handle);
    
    // If not at start position, move there first
    if (current_x != start_x || current_y != start_y) {
        // Use linear move to get to start position
        xSemaphoreGive(s_config_mutex);
        
        // Wait for move to start position to complete
        clearpath_servo_move(x_handle, start_x, CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE);
        clearpath_servo_move(y_handle, start_y, CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE);
        
        // Wait a bit for move to complete (simplified - in production, check completion)
        vTaskDelay(pdMS_TO_TICKS(100));
        
        if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        
        // Re-validate handles after re-taking mutex (servos could have been deinitialized)
        x_handle = s_servos[axis_x_index].handle;
        y_handle = s_servos[axis_y_index].handle;
        if (x_handle == NULL || y_handle == NULL || 
            !s_servos[axis_x_index].initialized || !s_servos[axis_y_index].initialized) {
            ESP_LOGE(TAG, "Servos deinitialized during move to start position");
            xSemaphoreGive(s_config_mutex);
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    // Set both axes to coordinated state
    clearpath_servo_set_state(x_handle, CLEARPATH_SERVO_STATE_COORDINATED);
    clearpath_servo_set_state(y_handle, CLEARPATH_SERVO_STATE_COORDINATED);
    
    // Store original vel_max to restore after move
    uint32_t original_vel_max_x = clearpath_servo_get_vel_max(x_handle);
    uint32_t original_vel_max_y = clearpath_servo_get_vel_max(y_handle);
    s_coordinated_group.original_vel_max[0] = original_vel_max_x;
    s_coordinated_group.original_vel_max[1] = original_vel_max_y;
    
    // Set feedrate for both axes (limit velocity to feedrate_steps_per_sec)
    // For circular interpolation, both axes need to maintain feedrate along the arc
    // The coordinated handler will update targets dynamically, so we set a reasonable vel_max
    uint32_t max_vel_for_arc = feedrate_steps_per_sec;
    if (max_vel_for_arc < original_vel_max_x) {
        clearpath_servo_set_vel_max(x_handle, max_vel_for_arc);
    }
    if (max_vel_for_arc < original_vel_max_y) {
        clearpath_servo_set_vel_max(y_handle, max_vel_for_arc);
    }
    
    // Initialize Bresenham circle algorithm state
    s_coordinated_group.bresenham_steps_total = arc_length;
    s_coordinated_group.bresenham_steps_generated = 0;
    s_coordinated_group.bresenham_last_step_time_us = esp_timer_get_time();
    s_coordinated_group.bresenham_clockwise = clockwise;
    
    // Calculate step interval for feedrate control: step_interval_us = 1000000 / feedrate
    if (feedrate_steps_per_sec > 0) {
        s_coordinated_group.bresenham_step_interval_us = 1000000ULL / feedrate_steps_per_sec;
    } else {
        s_coordinated_group.bresenham_step_interval_us = 1000;  // Default 1ms
    }
    
    // Initialize Bresenham circle: determine starting octant and position
    // Octants: 0=0-45°, 1=45-90°, 2=90-135°, 3=135-180°, 4=180-225°, 5=225-270°, 6=270-315°, 7=315-360°
    // Bresenham works in first octant (0-45°), we transform coordinates based on current octant
    // Use normalized start angle for octant calculation (0-360 degrees)
    int32_t start_angle_deg = start_angle_for_bresenham_x10 / 10;  // Convert to integer degrees
    start_angle_deg = start_angle_deg % 360;
    if (start_angle_deg < 0) start_angle_deg += 360;
    
    // Determine starting octant (0-7)
    int32_t octant = start_angle_deg / 45;
    if (octant >= 8) octant = 7;
    s_coordinated_group.bresenham_octant = octant;
    
    // Calculate start position in first octant coordinates
    // Map start angle to first octant: angle_in_octant = start_angle % 45
    int32_t angle_in_octant = start_angle_deg % 45;  // Angle within current octant (0-44°)
    
    // In first octant: x = radius * sin(angle), y = radius * cos(angle) for angle 0-45°
    // But we need to map the angle to first octant coordinates
    // For any octant, we can map to first octant by taking angle % 45
    int32_t cos_angle = cos_lookup(angle_in_octant * 10, 1000);  // angle_in_octant in 0.1° units
    int32_t sin_angle = sin_lookup(angle_in_octant * 10, 1000);
    
    // In first octant coordinates: x increases from 0, y decreases from radius
    s_coordinated_group.bresenham_x = ((int64_t)radius_steps * sin_angle) / 1000;
    s_coordinated_group.bresenham_y = ((int64_t)radius_steps * cos_angle) / 1000;
    
    // Initialize error parameter: error = 1 - radius (standard Bresenham initialization)
    // The error will be updated incrementally as we step
    s_coordinated_group.bresenham_circle_error = 1 - radius_steps;
    
    // Note: For arcs, we track progress by step count (arc_length)
    // The octant will transition automatically as we step around the circle
    
    // Start circular motion by setting initial targets (in steps)
    int32_t cos_val_start = cos_lookup(start_angle_for_bresenham_x10, 1000);
    int32_t sin_val_start = sin_lookup(start_angle_for_bresenham_x10, 1000);
    int32_t target_x = center_x_steps + ((int64_t)radius_steps * cos_val_start) / 1000;
    int32_t target_y = center_y_steps + ((int64_t)radius_steps * sin_val_start) / 1000;
    
    clearpath_servo_move(x_handle, target_x, CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE);
    clearpath_servo_move(y_handle, target_y, CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE);
    
    xSemaphoreGive(s_config_mutex);
    
    ESP_LOGI(TAG, "Started circular interpolation: center=(%.2f,%.2f) units, radius=%.2f units, angle=%d to %d, feedrate=%.2f units/min (%.2f steps/sec)",
             center_x, center_y, radius, start_angle_deg, end_angle_deg, feedrate, (float)feedrate_steps_per_sec);
    return ESP_OK;
}

esp_err_t clearpath_servo_manager_move_arc_to_point(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    float end_x,
    float end_y,
    bool absolute,
    float radius,
    bool clockwise,
    float feedrate
)
{
    if (axis_x_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS ||
        axis_y_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        ESP_LOGE(TAG, "Invalid axis indices");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if servos are initialized and have steps_per_unit configured
    if (!s_servos[axis_x_index].initialized || !s_servos[axis_y_index].initialized) {
        ESP_LOGE(TAG, "Servos not initialized");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_servos[axis_x_index].config.steps_per_unit == 0 || 
        s_servos[axis_y_index].config.steps_per_unit == 0) {
        ESP_LOGE(TAG, "Servos have steps_per_unit not configured");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get current positions (start point)
    clearpath_servo_handle_t *x_handle = s_servos[axis_x_index].handle;
    clearpath_servo_handle_t *y_handle = s_servos[axis_y_index].handle;
    
    if (x_handle == NULL || y_handle == NULL) {
        ESP_LOGE(TAG, "Servo handles are NULL");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    int32_t start_x_steps = clearpath_servo_get_position(x_handle);
    int32_t start_y_steps = clearpath_servo_get_position(y_handle);
    
    // Convert to units for calculations
    float start_x = steps_to_units(axis_x_index, start_x_steps);
    float start_y = steps_to_units(axis_y_index, start_y_steps);
    
    // Calculate end point
    float end_x_abs, end_y_abs;
    if (absolute) {
        end_x_abs = end_x;
        end_y_abs = end_y;
    } else {
        end_x_abs = start_x + end_x;
        end_y_abs = start_y + end_y;
    }
    
    // Calculate distance from start to end
    float dx = end_x_abs - start_x;
    float dy = end_y_abs - start_y;
    float distance = sqrtf(dx * dx + dy * dy);
    
    // Validate radius: must be >= half the distance (for arc to exist)
    float min_radius = distance / 2.0f;
    if (radius < min_radius) {
        ESP_LOGE(TAG, "Radius %.2f is too small (minimum %.2f for distance %.2f)", 
                 radius, min_radius, distance);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate center point
    // Method: Find midpoint, then calculate perpendicular distance to center
    // There are two possible centers (major and minor arc) - we choose based on direction
    
    float mid_x = (start_x + end_x_abs) / 2.0f;
    float mid_y = (start_y + end_y_abs) / 2.0f;
    
    // Perpendicular distance from midpoint to center
    // h = sqrt(r^2 - (d/2)^2) where d is distance from start to end
    float half_distance = distance / 2.0f;
    float h_squared = radius * radius - half_distance * half_distance;
    
    if (h_squared < 0) {
        // Shouldn't happen if radius >= min_radius, but check anyway
        ESP_LOGE(TAG, "Invalid radius calculation: radius=%.2f, distance=%.2f", radius, distance);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    float h = sqrtf(h_squared);
    
    // Perpendicular vector (normalized, rotated 90°)
    // Perpendicular to (dx, dy) is (-dy, dx) or (dy, -dx)
    // Normalize: perp = (-dy, dx) / distance
    float perp_x = -dy / distance;
    float perp_y = dx / distance;
    
    // Two possible centers: midpoint ± h * perpendicular
    // This creates two arcs: minor arc (< 180°) and major arc (> 180°)
    // For G-code compatibility with positive radius: always use minor arc
    // The clockwise parameter determines the direction around the circle, not which arc
    
    // Calculate both possible centers
    float center1_x = mid_x + h * perp_x;
    float center1_y = mid_y + h * perp_y;
    float center2_x = mid_x - h * perp_x;
    float center2_y = mid_y - h * perp_y;
    
    // Calculate angles for both centers to determine which gives minor arc
    float angle1_start = atan2f(start_y - center1_y, start_x - center1_x);
    float angle1_end = atan2f(end_y_abs - center1_y, end_x_abs - center1_x);
    float angle1_diff = fabsf(angle1_end - angle1_start);
    if (angle1_diff > 3.14159265f) angle1_diff = 2.0f * 3.14159265f - angle1_diff;  // Get smaller angle
    
    float angle2_start = atan2f(start_y - center2_y, start_x - center2_x);
    float angle2_end = atan2f(end_y_abs - center2_y, end_x_abs - center2_x);
    float angle2_diff = fabsf(angle2_end - angle2_start);
    if (angle2_diff > 3.14159265f) angle2_diff = 2.0f * 3.14159265f - angle2_diff;  // Get smaller angle
    
    // Choose center that gives minor arc (smaller angle difference)
    float center_x, center_y;
    if (angle1_diff < angle2_diff) {
        center_x = center1_x;
        center_y = center1_y;
    } else {
        center_x = center2_x;
        center_y = center2_y;
    }
    
    // Calculate start and end angles from center
    float start_angle_rad = atan2f(start_y - center_y, start_x - center_x);
    float end_angle_rad = atan2f(end_y_abs - center_y, end_x_abs - center_x);
    
    // Convert to degrees
    int32_t start_angle_deg = (int32_t)(start_angle_rad * 180.0f / 3.14159265f);
    int32_t end_angle_deg = (int32_t)(end_angle_rad * 180.0f / 3.14159265f);
    
    // Normalize to 0-360
    start_angle_deg = start_angle_deg % 360;
    if (start_angle_deg < 0) start_angle_deg += 360;
    end_angle_deg = end_angle_deg % 360;
    if (end_angle_deg < 0) end_angle_deg += 360;
    
    xSemaphoreGive(s_config_mutex);
    
    // Call the existing move_arc function with calculated center and angles
    return clearpath_servo_manager_move_arc(
        axis_x_index,
        axis_y_index,
        center_x,
        center_y,
        radius,
        start_angle_deg,
        end_angle_deg,
        clockwise,
        feedrate
    );
}

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
)
{
    if (axis_x_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS ||
        axis_y_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS ||
        axis_z_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        ESP_LOGE(TAG, "Invalid axis indices");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if servos are initialized and have steps_per_unit configured
    if (!s_servos[axis_x_index].initialized || 
        !s_servos[axis_y_index].initialized || 
        !s_servos[axis_z_index].initialized) {
        ESP_LOGE(TAG, "Servos not initialized");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_servos[axis_x_index].config.steps_per_unit == 0 || 
        s_servos[axis_y_index].config.steps_per_unit == 0 ||
        s_servos[axis_z_index].config.steps_per_unit == 0) {
        ESP_LOGE(TAG, "Servos have steps_per_unit not configured");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Convert positions from units to steps
    int32_t center_x_steps = units_to_steps(axis_x_index, center_x);
    int32_t center_y_steps = units_to_steps(axis_y_index, center_y);
    int32_t radius_steps = units_to_steps(axis_x_index, radius);
    int32_t z_start_steps = units_to_steps(axis_z_index, z_start);
    int32_t z_end_steps = units_to_steps(axis_z_index, z_end);
    
    // Validate radius is positive
    if (radius_steps <= 0) {
        ESP_LOGE(TAG, "Invalid radius: %d steps (must be > 0)", radius_steps);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate Z distance
    int32_t z_distance_steps = z_end_steps - z_start_steps;
    
    // Convert feedrate from units/minute to steps/second
    // For helical arcs, feedrate is along the 3D path
    // Path length = sqrt(arc_length^2 + z_distance^2)
    // We'll use the arc length for feedrate calculation (approximation)
    uint32_t feedrate_steps_per_sec = feedrate_units_per_min_to_steps_per_sec(axis_x_index, feedrate);
    if (feedrate_steps_per_sec == 0) {
        ESP_LOGE(TAG, "Invalid feedrate: 0 steps/sec");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Convert angles to 0.1 degree units (multiply by 10)
    // Allow angles > 360 degrees for multi-turn helixes
    int32_t start_angle_deg_x10 = start_angle_deg * 10;
    int32_t end_angle_deg_x10 = end_angle_deg * 10;
    
    // Calculate angle difference in 0.1 degree units
    // For multi-turn helixes, we preserve the full angle difference
    int32_t angle_diff_x10 = end_angle_deg_x10 - start_angle_deg_x10;
    
    // For clockwise, reverse the direction
    if (clockwise) {
        angle_diff_x10 = -angle_diff_x10;
    }
    
    // Normalize start angle to 0-3600 for Bresenham initialization (octant calculation)
    // But preserve full angle difference for arc length calculation
    int32_t start_angle_normalized_x10 = start_angle_deg_x10 % 3600;
    if (start_angle_normalized_x10 < 0) start_angle_normalized_x10 += 3600;
    
    // Store normalized angles for Bresenham initialization
    int32_t start_angle_for_bresenham_x10 = start_angle_normalized_x10;
    int32_t end_angle_for_bresenham_x10 = start_angle_normalized_x10 + angle_diff_x10;
    
    // Check for zero angle difference (no movement needed)
    if (angle_diff_x10 == 0 && z_distance_steps == 0) {
        ESP_LOGW(TAG, "Zero angle difference and Z distance, no arc movement needed");
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }
    
    // Calculate arc length: arc_length = radius * angle (in radians)
    // Use absolute angle difference (supports multi-turn helixes)
    int64_t radius_64 = (int64_t)radius_steps;
    int64_t angle_abs_64 = (int64_t)abs(angle_diff_x10);
    
    // Check for overflow: radius * angle_diff could be very large
    if (radius_64 > INT64_MAX / angle_abs_64) {
        ESP_LOGE(TAG, "Arc length calculation overflow: radius=%d, angle_diff=%d", 
                 radius_steps, angle_diff_x10);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    int64_t arc_length_64 = (radius_64 * angle_abs_64 * 314159LL) / 1800000LL;
    
    // Check for overflow when casting to int32_t
    if (arc_length_64 > INT32_MAX || arc_length_64 < INT32_MIN) {
        ESP_LOGE(TAG, "Arc length overflow: %lld steps", arc_length_64);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    int32_t arc_length = (int32_t)arc_length_64;
    
    // Calculate 3D path length for feedrate (helical path)
    // path_length = sqrt(arc_length^2 + z_distance^2)
    int64_t arc_length_sq_64 = (int64_t)arc_length * (int64_t)arc_length;
    int64_t z_distance_sq_64 = (int64_t)z_distance_steps * (int64_t)z_distance_steps;
    int64_t path_length_sq_64 = arc_length_sq_64 + z_distance_sq_64;
    
    // Use arc_length for step generation (Z will be interpolated proportionally)
    // The feedrate is along the 3D path, but we generate steps based on arc_length
    
    // Check for zero arc length
    if (arc_length == 0 && z_distance_steps == 0) {
        ESP_LOGW(TAG, "Zero arc length and Z distance, no movement needed");
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }
    
    // Calculate total time needed: time = path_length / feedrate
    // Use arc_length for timing (approximation - actual path is 3D)
    int64_t total_time_us_64 = ((int64_t)arc_length * 1000000LL) / feedrate_steps_per_sec;
    
    // Check for overflow/underflow in time calculation
    if (total_time_us_64 < 0 || total_time_us_64 > INT64_MAX / 2) {
        ESP_LOGE(TAG, "Helical arc total time calculation overflow: arc_length=%d, feedrate=%u", 
                 arc_length, feedrate_steps_per_sec);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint64_t total_time_us = (uint64_t)total_time_us_64;
    
    // Stop any existing coordinated motion
    s_coordinated_group.active = false;
    
    // Initialize circular interpolation with Z axis
    memset(&s_coordinated_group, 0, sizeof(coordinated_motion_group_t));
    s_coordinated_group.type = COORDINATED_MOTION_CIRCULAR;
    s_coordinated_group.active = true;
    s_coordinated_group.axis_count = 3;  // X, Y, Z
    s_coordinated_group.axis_indices[0] = axis_x_index;
    s_coordinated_group.axis_indices[1] = axis_y_index;
    s_coordinated_group.axis_indices[2] = axis_z_index;
    s_coordinated_group.center_x = center_x_steps;
    s_coordinated_group.center_y = center_y_steps;
    s_coordinated_group.radius = radius_steps;
    s_coordinated_group.feedrate = feedrate_steps_per_sec;
    s_coordinated_group.start_angle_deg_x10 = start_angle_for_bresenham_x10;
    s_coordinated_group.end_angle_deg_x10 = end_angle_for_bresenham_x10;
    s_coordinated_group.angle_diff_x10 = angle_diff_x10;
    s_coordinated_group.arc_length = arc_length;
    s_coordinated_group.start_time_us = esp_timer_get_time();
    s_coordinated_group.total_time_us = total_time_us;
    
    // Initialize Z axis state for linear interpolation
    s_coordinated_group.z_start_position = z_start_steps;
    s_coordinated_group.z_end_position = z_end_steps;
    s_coordinated_group.z_total_distance = z_distance_steps;
    s_coordinated_group.z_current_position = z_start_steps;
    s_coordinated_group.z_error = 0;
    s_coordinated_group.z_step_dir = (z_distance_steps > 0) ? 1 : ((z_distance_steps < 0) ? -1 : 0);
    
    // Calculate starting positions using normalized start angle
    int32_t cos_val_start = cos_lookup(start_angle_for_bresenham_x10, 1000);
    int32_t sin_val_start = sin_lookup(start_angle_for_bresenham_x10, 1000);
    
    int32_t start_x = center_x_steps + ((int64_t)radius_steps * cos_val_start) / 1000;
    int32_t start_y = center_y_steps + ((int64_t)radius_steps * sin_val_start) / 1000;
    
    s_coordinated_group.start_positions[0] = start_x;
    s_coordinated_group.start_positions[1] = start_y;
    s_coordinated_group.start_positions[2] = z_start_steps;
    
    // Move to starting position first (if not already there)
    clearpath_servo_handle_t *x_handle = s_servos[axis_x_index].handle;
    clearpath_servo_handle_t *y_handle = s_servos[axis_y_index].handle;
    clearpath_servo_handle_t *z_handle = s_servos[axis_z_index].handle;
    
    // Validate handles are not NULL
    if (x_handle == NULL || y_handle == NULL || z_handle == NULL) {
        ESP_LOGE(TAG, "Servo handles are NULL");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    int32_t current_x = clearpath_servo_get_position(x_handle);
    int32_t current_y = clearpath_servo_get_position(y_handle);
    int32_t current_z = clearpath_servo_get_position(z_handle);
    
    // If not at start position, move there first
    if (current_x != start_x || current_y != start_y || current_z != z_start_steps) {
        // Use linear move to get to start position
        uint8_t axes[] = {axis_x_index, axis_y_index, axis_z_index};
        float targets[] = {
            steps_to_units(axis_x_index, start_x),
            steps_to_units(axis_y_index, start_y),
            steps_to_units(axis_z_index, z_start_steps)
        };
        xSemaphoreGive(s_config_mutex);
        
        // Move to start position
        esp_err_t ret = clearpath_servo_manager_move_linear(3, axes, targets, feedrate);
        if (ret != ESP_OK) {
            return ret;
        }
        
        // Wait for move to complete
        while (!clearpath_servo_manager_is_coordinated_move_complete()) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }
    
    // Store original velocity limits
    s_coordinated_group.original_vel_max[0] = clearpath_servo_get_vel_max(x_handle);
    s_coordinated_group.original_vel_max[1] = clearpath_servo_get_vel_max(y_handle);
    s_coordinated_group.original_vel_max[2] = clearpath_servo_get_vel_max(z_handle);
    
    // Set coordinated state for all axes
    clearpath_servo_set_state(x_handle, CLEARPATH_SERVO_STATE_COORDINATED);
    clearpath_servo_set_state(y_handle, CLEARPATH_SERVO_STATE_COORDINATED);
    clearpath_servo_set_state(z_handle, CLEARPATH_SERVO_STATE_COORDINATED);
    
    // Initialize Bresenham circle: determine starting octant and position
    // Use normalized start angle for octant calculation (0-360 degrees)
    int32_t start_angle_deg = start_angle_for_bresenham_x10 / 10;
    start_angle_deg = start_angle_deg % 360;
    if (start_angle_deg < 0) start_angle_deg += 360;
    
    // Determine starting octant (0-7)
    int32_t octant = start_angle_deg / 45;
    if (octant >= 8) octant = 7;
    s_coordinated_group.bresenham_octant = octant;
    
    // Calculate start position in first octant coordinates
    int32_t angle_in_octant = start_angle_deg % 45;
    
    int32_t cos_angle = cos_lookup(angle_in_octant * 10, 1000);
    int32_t sin_angle = sin_lookup(angle_in_octant * 10, 1000);
    
    s_coordinated_group.bresenham_x = ((int64_t)radius_steps * sin_angle) / 1000;
    s_coordinated_group.bresenham_y = ((int64_t)radius_steps * cos_angle) / 1000;
    
    // Initialize error parameter
    s_coordinated_group.bresenham_circle_error = 1 - radius_steps;
    
    // Initialize Bresenham step generation state
    s_coordinated_group.bresenham_steps_total = arc_length;
    s_coordinated_group.bresenham_steps_generated = 0;
    s_coordinated_group.bresenham_last_step_time_us = esp_timer_get_time();
    s_coordinated_group.bresenham_clockwise = clockwise;
    
    // Calculate step interval for feedrate control
    if (feedrate_steps_per_sec > 0) {
        s_coordinated_group.bresenham_step_interval_us = 1000000ULL / feedrate_steps_per_sec;
    } else {
        s_coordinated_group.bresenham_step_interval_us = 1000;  // Default 1ms
    }
    
    xSemaphoreGive(s_config_mutex);
    
    ESP_LOGI(TAG, "Started helical arc: center=(%.2f,%.2f) units, radius=%.2f units, angle=%d to %d, Z=%.2f to %.2f, feedrate=%.2f units/min (%.2f steps/sec)",
             center_x, center_y, radius, start_angle_deg, end_angle_deg, z_start, z_end, feedrate, (float)feedrate_steps_per_sec);
    return ESP_OK;
}

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
)
{
    if (axis_x_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS ||
        axis_y_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS ||
        axis_z_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        ESP_LOGE(TAG, "Invalid axis indices");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    // Check if servos are initialized and have steps_per_unit configured
    if (!s_servos[axis_x_index].initialized || 
        !s_servos[axis_y_index].initialized || 
        !s_servos[axis_z_index].initialized) {
        ESP_LOGE(TAG, "Servos not initialized");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    if (s_servos[axis_x_index].config.steps_per_unit == 0 || 
        s_servos[axis_y_index].config.steps_per_unit == 0 ||
        s_servos[axis_z_index].config.steps_per_unit == 0) {
        ESP_LOGE(TAG, "Servos have steps_per_unit not configured");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Get current positions (start point)
    clearpath_servo_handle_t *x_handle = s_servos[axis_x_index].handle;
    clearpath_servo_handle_t *y_handle = s_servos[axis_y_index].handle;
    clearpath_servo_handle_t *z_handle = s_servos[axis_z_index].handle;
    
    if (x_handle == NULL || y_handle == NULL || z_handle == NULL) {
        ESP_LOGE(TAG, "Servo handles are NULL");
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    int32_t start_x_steps = clearpath_servo_get_position(x_handle);
    int32_t start_y_steps = clearpath_servo_get_position(y_handle);
    int32_t start_z_steps = clearpath_servo_get_position(z_handle);
    
    // Convert to units for calculations
    float start_x = steps_to_units(axis_x_index, start_x_steps);
    float start_y = steps_to_units(axis_y_index, start_y_steps);
    float start_z = steps_to_units(axis_z_index, start_z_steps);
    
    // Calculate end point
    float end_x_abs, end_y_abs, end_z_abs;
    if (absolute) {
        end_x_abs = end_x;
        end_y_abs = end_y;
        end_z_abs = end_z;
    } else {
        end_x_abs = start_x + end_x;
        end_y_abs = start_y + end_y;
        end_z_abs = start_z + end_z;
    }
    
    // Calculate distance from start to end in XY plane
    float dx = end_x_abs - start_x;
    float dy = end_y_abs - start_y;
    float distance_xy = sqrtf(dx * dx + dy * dy);
    
    // Check for zero distance in XY plane (straight line in Z only)
    if (distance_xy < 0.0001f) {
        // If XY distance is zero, just do a linear move in Z
        // This is a degenerate case - could also return error
        ESP_LOGW(TAG, "Zero XY distance, performing linear Z move only");
        xSemaphoreGive(s_config_mutex);
        // Use linear move for all three axes (X and Y stay same, Z moves)
        // move_linear requires at least 2 axes, so we include all three
        uint8_t axes[] = {axis_x_index, axis_y_index, axis_z_index};
        float targets[] = {start_x, start_y, end_z_abs};
        return clearpath_servo_manager_move_linear(3, axes, targets, feedrate);
    }
    
    // Validate radius: must be >= half the distance in XY plane (for arc to exist)
    float min_radius = distance_xy / 2.0f;
    if (radius < min_radius) {
        ESP_LOGE(TAG, "Radius %.2f is too small (minimum %.2f for XY distance %.2f)", 
                 radius, min_radius, distance_xy);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate center point in XY plane (same as 2D arc calculation)
    float mid_x = (start_x + end_x_abs) / 2.0f;
    float mid_y = (start_y + end_y_abs) / 2.0f;
    
    // Perpendicular distance from midpoint to center
    float half_distance = distance_xy / 2.0f;
    float h_squared = radius * radius - half_distance * half_distance;
    
    if (h_squared < 0) {
        ESP_LOGE(TAG, "Invalid radius calculation: radius=%.2f, distance_xy=%.2f", radius, distance_xy);
        xSemaphoreGive(s_config_mutex);
        return ESP_ERR_INVALID_ARG;
    }
    
    float h = sqrtf(h_squared);
    
    // Perpendicular vector (normalized, rotated 90°)
    float perp_x = -dy / distance_xy;
    float perp_y = dx / distance_xy;
    
    // Calculate both possible centers
    float center1_x = mid_x + h * perp_x;
    float center1_y = mid_y + h * perp_y;
    float center2_x = mid_x - h * perp_x;
    float center2_y = mid_y - h * perp_y;
    
    // Calculate angles for both centers to determine which gives minor arc
    float angle1_start = atan2f(start_y - center1_y, start_x - center1_x);
    float angle1_end = atan2f(end_y_abs - center1_y, end_x_abs - center1_x);
    float angle1_diff = fabsf(angle1_end - angle1_start);
    if (angle1_diff > 3.14159265f) angle1_diff = 2.0f * 3.14159265f - angle1_diff;
    
    float angle2_start = atan2f(start_y - center2_y, start_x - center2_x);
    float angle2_end = atan2f(end_y_abs - center2_y, end_x_abs - center2_x);
    float angle2_diff = fabsf(angle2_end - angle2_start);
    if (angle2_diff > 3.14159265f) angle2_diff = 2.0f * 3.14159265f - angle2_diff;
    
    // Choose center that gives minor arc (smaller angle difference)
    float center_x, center_y;
    if (angle1_diff < angle2_diff) {
        center_x = center1_x;
        center_y = center1_y;
    } else {
        center_x = center2_x;
        center_y = center2_y;
    }
    
    // Calculate start and end angles from center
    float start_angle_rad = atan2f(start_y - center_y, start_x - center_x);
    float end_angle_rad = atan2f(end_y_abs - center_y, end_x_abs - center_x);
    
    // Convert to degrees
    float start_angle_deg_float = start_angle_rad * 180.0f / 3.14159265f;
    float end_angle_deg_float = end_angle_rad * 180.0f / 3.14159265f;
    
    // Normalize to 0-360
    while (start_angle_deg_float < 0.0f) start_angle_deg_float += 360.0f;
    while (start_angle_deg_float >= 360.0f) start_angle_deg_float -= 360.0f;
    while (end_angle_deg_float < 0.0f) end_angle_deg_float += 360.0f;
    while (end_angle_deg_float >= 360.0f) end_angle_deg_float -= 360.0f;
    
    // Convert to integer degrees
    // Note: move_arc_helical() will handle the clockwise direction and angle difference calculation
    // We just need to provide the start and end angles from the center
    int32_t start_angle_deg = (int32_t)start_angle_deg_float;
    int32_t end_angle_deg = (int32_t)end_angle_deg_float;
    
    xSemaphoreGive(s_config_mutex);
    
    // Call the existing move_arc_helical function with calculated center and angles
    return clearpath_servo_manager_move_arc_helical(
        axis_x_index,
        axis_y_index,
        axis_z_index,
        center_x,
        center_y,
        radius,
        start_angle_deg,
        end_angle_deg,
        start_z,
        end_z_abs,
        clockwise,
        feedrate
    );
}

bool clearpath_servo_manager_is_coordinated_move_complete(void)
{
    if (s_config_mutex == NULL) {
        return true;
    }
    
    if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    
    bool complete = !s_coordinated_group.active;
    xSemaphoreGive(s_config_mutex);
    
    return complete;
}

esp_err_t clearpath_servo_manager_stop_coordinated_move(void)
{
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    if (s_coordinated_group.active) {
        // Stop all axes in the coordinated group
        for (uint8_t i = 0; i < s_coordinated_group.axis_count; i++) {
            clearpath_servo_handle_t *handle = s_servos[s_coordinated_group.axis_indices[i]].handle;
            if (handle != NULL) {
                // Restore original vel_max before stopping
                if (i < MAX_COORDINATED_AXES) {
                    clearpath_servo_set_vel_max(handle, s_coordinated_group.original_vel_max[i]);
                }
                clearpath_servo_set_state(handle, CLEARPATH_SERVO_STATE_IDLE);
                clearpath_servo_stop(handle);
            }
        }
        
        s_coordinated_group.active = false;
    }
    
    xSemaphoreGive(s_config_mutex);
    
    ESP_LOGI(TAG, "Stopped coordinated move");
    return ESP_OK;
}
