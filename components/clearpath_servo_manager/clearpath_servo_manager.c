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
 * 1. Background task reads commands from Output Assembly 150 (bytes 32-39)
 * 2. Task generates step pulses based on velocity/acceleration profiles
 * 3. Position tracking updates based on step count
 * 4. HLFB monitoring updates status flags
 * 5. Status/feedback written to Input Assembly 100 (bytes 61-71)
 * 
 * Step Generation:
 * - Background task generates step pulses (default 5 kHz, configurable)
 * - Trapezoidal velocity profile (accel → constant → decel)
 * - Updates position counter on each step
 * - Checks move completion based on position and HLFB
 * - Note: ESP32-P4 software timing limits practical step rates to ~1-10 kHz
 *   depending on system load, task priority, and GPIO toggle speed.
 *   ESP32-P4 hardware peripherals (LEDC, MCPWM, RMT) could achieve much
 *   higher rates (up to 40 MHz with LEDC) but require different implementation
 * 
 * Assembly Data Layout:
 * - Output Assembly 150, Bytes 32-39: Servo commands (2 bytes per servo)
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
#define STEP_PULSE_WIDTH_US 10  // Step pulse width in microseconds
#define ASSEMBLY_UPDATE_INTERVAL_MS 10  // Update assembly data every 10ms

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
#define DEFAULT_VEL_MAX        1000
#define DEFAULT_ACCEL_MAX      10000

/**
 * @brief Generate step pulse for a servo
 * 
 * This function generates a single step pulse and updates position.
 * Called from the step generation task.
 */
static void generate_step(servo_instance_t *servo)
{
    if (servo == NULL || !servo->initialized || servo->handle == NULL) {
        return;
    }

    // Generate step pulse: LOW → HIGH → LOW
    gpio_set_level(servo->config.gpio_step, 1);
    // Small delay for pulse width (using busy wait for precise timing)
    uint64_t start_us = esp_timer_get_time();
    while ((esp_timer_get_time() - start_us) < STEP_PULSE_WIDTH_US) {
        // Busy wait for precise timing
    }
    gpio_set_level(servo->config.gpio_step, 0);

    // Update position based on direction
    // Direction GPIO: HIGH = forward (positive), LOW = reverse (negative)
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
 * based on velocity and acceleration profiles.
 * 
 * Note: ESP32-P4 software timing limits practical step rates to ~1-10 kHz depending
 * on system load, task priority, and GPIO toggle speed. ESP32-P4 hardware peripherals
 * (LEDC up to 40 MHz, MCPWM, or RMT) could achieve much higher rates but require
 * different implementation.
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
            // Process each servo
            for (int i = 0; i < CLEARPATH_SERVO_MANAGER_MAX_SERVOS; i++) {
                servo_instance_t *servo = &s_servos[i];
                
                if (!servo->initialized || servo->handle == NULL) {
                    continue;
                }

                // Check if servo needs to move
                int32_t current_pos = clearpath_servo_get_position(servo->handle);
                int32_t target_pos = 0; // TODO: Get from servo handle
                int32_t current_vel = clearpath_servo_get_velocity(servo->handle);
                int32_t target_vel = 0; // TODO: Get from servo handle

                // Simple step generation logic
                // TODO: Implement proper trapezoidal velocity profile
                if (current_pos != target_pos || current_vel != 0) {
                    // Generate step
                    generate_step(servo);
                    servo->last_step_time_us = now_us;
                }
            }

            last_update_us = now_us;
        }

        // Update assembly data periodically
        if (now_us - last_assembly_update_us > (ASSEMBLY_UPDATE_INTERVAL_MS * 1000)) {
            SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
            if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                // Update servo 0 feedback (bytes 61-71)
                if (s_servos[0].initialized && s_servos[0].handle != NULL) {
                    int32_t position = clearpath_servo_get_position(s_servos[0].handle);
                    int32_t velocity = clearpath_servo_get_velocity(s_servos[0].handle);
                    
                    // Write position (bytes 61-64)
                    memcpy(&INPUT_ASSEMBLY_100[61], &position, sizeof(int32_t));
                    
                    // Write velocity (bytes 65-66)
                    int16_t vel_16 = (int16_t)velocity;
                    memcpy(&INPUT_ASSEMBLY_100[65], &vel_16, sizeof(int16_t));
                    
                    // Write status (byte 67)
                    uint8_t status = 0;
                    if (clearpath_servo_steps_complete(s_servos[0].handle)) status |= 0x01; // Move complete
                    if (clearpath_servo_is_enabled(s_servos[0].handle)) status |= 0x02; // Enabled
                    if (clearpath_servo_is_fault(s_servos[0].handle)) status |= 0x04; // Fault
                    if (clearpath_servo_get_hlfb_status(s_servos[0].handle)) status |= 0x08; // In position (from HLFB)
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
            // Read servo 0 command (bytes 32-33)
            // TODO: Parse command and execute move/velocity commands
            // Command format: Byte 32 = command type, Byte 33 = reserved/data
            
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
        .vel_max = DEFAULT_VEL_MAX,
        .accel_max = DEFAULT_ACCEL_MAX,
        .hlfb_mode = CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE,
        .hlfb_active_high = true,
        .enabled = false
    };

    clearpath_servo_config_t servo_config0 = {
        .gpio_step = config0.gpio_step,
        .gpio_dir = config0.gpio_dir,
        .gpio_enable = config0.gpio_enable,
        .gpio_hlfb = config0.gpio_hlfb,
        .vel_max = config0.vel_max,
        .accel_max = config0.accel_max,
        .hlfb_mode = config0.hlfb_mode,
        .hlfb_active_high = config0.hlfb_active_high
    };

    clearpath_servo_handle_t *handle0 = NULL;
    esp_err_t ret = clearpath_servo_init(&servo_config0, &handle0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize servo 0");
        vSemaphoreDelete(s_config_mutex);
        return ret;
    }

    s_servos[0].handle = handle0;
    s_servos[0].config = config0;
    s_servos[0].initialized = true;
    s_servos[0].last_position = 0;

    // Create step generation task
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

    if (!s_servos[servo_index].initialized || s_servos[servo_index].handle == NULL) {
        return 0;
    }

    return clearpath_servo_get_position(s_servos[servo_index].handle);
}

int32_t clearpath_servo_manager_get_velocity(uint8_t servo_index)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0;
    }

    if (!s_servos[servo_index].initialized || s_servos[servo_index].handle == NULL) {
        return 0;
    }

    return clearpath_servo_get_velocity(s_servos[servo_index].handle);
}

uint8_t clearpath_servo_manager_get_status(uint8_t servo_index)
{
    if (servo_index >= CLEARPATH_SERVO_MANAGER_MAX_SERVOS) {
        return 0;
    }

    if (!s_servos[servo_index].initialized || s_servos[servo_index].handle == NULL) {
        return 0;
    }

    uint8_t status = 0;
    if (clearpath_servo_steps_complete(s_servos[servo_index].handle)) status |= 0x01;
    if (clearpath_servo_is_enabled(s_servos[servo_index].handle)) status |= 0x02;
    if (clearpath_servo_is_fault(s_servos[servo_index].handle)) status |= 0x04;
    if (clearpath_servo_get_hlfb_status(s_servos[servo_index].handle)) status |= 0x08;

    return status;
}

uint8_t clearpath_servo_manager_get_count(void)
{
    uint8_t count = 0;
    for (int i = 0; i < CLEARPATH_SERVO_MANAGER_MAX_SERVOS; i++) {
        if (s_servos[i].initialized) {
            count++;
        }
    }
    return count;
}
