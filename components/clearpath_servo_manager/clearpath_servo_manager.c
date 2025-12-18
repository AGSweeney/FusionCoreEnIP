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

// Default configuration
#define DEFAULT_GPIO_STEP_0    8
#define DEFAULT_GPIO_DIR_0     9
#define DEFAULT_GPIO_ENABLE_0  10
#define DEFAULT_GPIO_HLFB_0    11
#define DEFAULT_GPIO_HOMING_SENSOR_0  -1  // Not configured by default
#define DEFAULT_VEL_MAX        1000
#define DEFAULT_ACCEL_MAX      10000
#define DEFAULT_HOMING_VELOCITY 0  // 0 = use vel_max

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
            typedef struct {
                clearpath_servo_handle_t *handle;
                int gpio_dir;
            } servo_info_t;
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
            
            for (int i = 0; i < CLEARPATH_SERVO_MANAGER_MAX_SERVOS; i++) {
                if (servo_infos[i].handle == NULL) {
                    continue;
                }

                // Get servo state and parameters
                clearpath_servo_state_t state = clearpath_servo_get_state(servo_infos[i].handle);
                int32_t current_pos = clearpath_servo_get_position(servo_infos[i].handle);
                int32_t current_vel = clearpath_servo_get_velocity(servo_infos[i].handle);
                int32_t target_pos = clearpath_servo_get_target_position(servo_infos[i].handle);
                int32_t target_vel = clearpath_servo_get_target_velocity(servo_infos[i].handle);
                uint32_t vel_max = clearpath_servo_get_vel_max(servo_infos[i].handle);
                uint32_t accel_max = clearpath_servo_get_accel_max(servo_infos[i].handle);

                // Integer math: delta_time_us is in microseconds
                // For calculations: multiply by delta_us, divide by 1,000,000
                // Use 64-bit intermediate to avoid overflow

                // Trapezoidal velocity profile implementation
                int32_t new_velocity = current_vel;
                bool should_generate_step = false;

                if (state == CLEARPATH_SERVO_STATE_MOVING) {
                    // Position move: trapezoidal profile
                    int32_t steps_remaining = target_pos - current_pos;
                    int32_t steps_remaining_abs = (steps_remaining < 0) ? -steps_remaining : steps_remaining;

                    if (steps_remaining_abs == 0) {
                        // Reached target, stop
                        new_velocity = 0;
                        clearpath_servo_stop(servo_infos[i].handle);
                    } else {
                        // Determine direction
                        int32_t direction = (steps_remaining > 0) ? 1 : -1;
                        int32_t target_vel_signed = (target_vel > 0) ? (int32_t)vel_max : -(int32_t)vel_max;
                        if (target_vel != 0) {
                            target_vel_signed = target_vel;
                        }

                        // Calculate deceleration distance: v^2 / (2 * a)
                        // Distance needed to decelerate from current velocity to 0
                        int32_t decel_distance = 0;
                        if (current_vel != 0) {
                            int32_t vel_abs = (current_vel < 0) ? -current_vel : current_vel;
                            decel_distance = (int32_t)((int64_t)vel_abs * (int64_t)vel_abs) / (2 * (int32_t)accel_max);
                            if (decel_distance < 1) decel_distance = 1; // Minimum 1 step
                        }

                        // Determine phase: ACCEL, CONSTANT, or DECEL
                        if (steps_remaining_abs <= decel_distance) {
                            // DECEL phase: approaching target, decelerate
                            // Integer math: vel_change = (accel_max * delta_us) / 1000000
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
                        } else {
                            // ACCEL or CONSTANT phase
                            int32_t vel_abs = (current_vel < 0) ? -current_vel : current_vel;
                            int32_t target_vel_abs = (target_vel_signed < 0) ? -target_vel_signed : target_vel_signed;
                            
                            if (vel_abs < target_vel_abs) {
                                // ACCEL phase: accelerate toward target velocity
                                // Integer math: vel_change = (accel_max * delta_us) / 1000000
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
                            } else {
                                // CONSTANT phase: maintain target velocity
                                new_velocity = target_vel_signed;
                            }
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
                            // Continue homing motion - use velocity mode logic
                            // Homing velocity is already set in target_velocity by clearpath_servo_home()
                            if (target_vel == 0) {
                                new_velocity = 0;
                                clearpath_servo_stop(servo_infos[i].handle);
                            } else {
                                // Ramp to target velocity if not already there
                                // Integer math: vel_change = (accel_max * delta_us) / 1000000
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
                        }
                    }
                } else if (state == CLEARPATH_SERVO_STATE_VELOCITY_MODE) {
                    // Velocity mode: continuous motion at target velocity
                    if (target_vel == 0) {
                        new_velocity = 0;
                        clearpath_servo_stop(servo_infos[i].handle);
                    } else {
                        // Ramp to target velocity if not already there
                        // Integer math: vel_change = (accel_max * delta_us) / 1000000
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

                // Update velocity in servo handle
                if (new_velocity != current_vel) {
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
        .hlfb_mode = CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE,
        .hlfb_active_high = true,
        .homing_sensor_type = CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN,
        .homing_velocity = DEFAULT_HOMING_VELOCITY,
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
