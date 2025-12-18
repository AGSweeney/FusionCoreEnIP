/**
 * @file clearpath_servo.c
 * @brief ClearPath Servo Driver Implementation
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * 
 * Step Generation Algorithm:
 * -------------------------
 * 
 * The driver uses a background task to generate step pulses based on velocity
 * and acceleration profiles. Step generation follows a trapezoidal velocity profile:
 * 
 * 1. Acceleration phase: Velocity ramps from 0 to target velocity
 * 2. Constant velocity phase: Maintains target velocity
 * 3. Deceleration phase: Velocity ramps from target to 0
 * 
 * Position tracking is maintained by counting step pulses. The driver supports:
 * - Absolute position moves (move to specific step count)
 * - Relative position moves (move N steps from current position)
 * - Continuous velocity moves (move at constant velocity until stopped)
 * 
 * HLFB (High Level Feedback) monitoring provides status information:
 * - Enabled state
 * - Move complete status
 * - Fault conditions
 * - In-position status
 * 
 * Note: Step generation is performed by the manager component's background task.
 * This driver provides the control logic and state management.
 * 
 * Attribution:
 * ------------
 * This implementation is inspired by the Teknic ClearCore library API design.
 * The step and direction control concepts, HLFB feedback interpretation, and
 * velocity/acceleration profile management follow patterns established in the
 * ClearCore library for ClearPath servo control.
 * 
 * References:
 * - Teknic ClearCore Library: https://github.com/Teknic-Inc/ClearCore-library
 * - ClearCore Step and Direction API: https://teknic-inc.github.io/ClearCore-library/_move_gen.html
 * - ClearPath Servo User Manual: https://www.teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf
 * - ESP32-P4 RMT (Remote Control) API: Hardware step pulse generation
 */

#include "clearpath_servo.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "clearpath_servo";

// RMT configuration constants
#define RMT_CLK_DIV 80  // 80 MHz / 80 = 1 MHz (1 us resolution)
#define STEP_PULSE_WIDTH_TICKS 10  // 10 microseconds pulse width
#define RMT_TX_QUEUE_SIZE 0  // No queue needed for simple pulse generation

// Static RMT channel allocation (ESP32-P4 has multiple RMT channels)
// Bitmask: bit N = 1 means channel N is in use, bit N = 0 means channel N is free
static uint8_t s_rmt_channel_bitmask = 0;
static SemaphoreHandle_t s_rmt_channel_mutex = NULL;
#define MAX_RMT_CHANNELS 8  // ESP32-P4 typically has 8 RMT channels

// Initialize RMT channel allocation mutex (called once)
// Note: This function is not fully thread-safe - if called concurrently from
// multiple threads, multiple mutexes may be created (small memory leak).
// This is acceptable since initialization is typically single-threaded.
static void init_rmt_channel_mutex(void)
{
    if (s_rmt_channel_mutex == NULL) {
        s_rmt_channel_mutex = xSemaphoreCreateMutex();
    }
}

/**
 * @brief Servo handle structure
 */
struct clearpath_servo_handle_s {
    clearpath_servo_config_t config;
    int32_t position;
    int32_t target_position;
    int32_t current_velocity;
    int32_t target_velocity;
    uint32_t vel_max;
    uint32_t accel_max;
    clearpath_servo_state_t state;
    bool enabled;
    SemaphoreHandle_t mutex;
    bool initialized;
    rmt_channel_t rmt_channel;  // RMT channel for step pulse generation
    rmt_item32_t step_pulse_item;  // Pre-configured RMT pulse item
    bool homing_complete;  // Homing operation completion flag
    uint64_t homing_start_time_us;  // Homing start timestamp for timeout
    uint32_t homing_timeout_ms;  // Homing timeout in milliseconds (0 = no timeout)
};

esp_err_t clearpath_servo_init(const clearpath_servo_config_t *config, clearpath_servo_handle_t **handle_out)
{
    if (config == NULL || handle_out == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->gpio_step < 0 || config->gpio_dir < 0) {
        ESP_LOGE(TAG, "Step and direction GPIO pins must be specified");
        return ESP_ERR_INVALID_ARG;
    }

    // Validate GPIO pin numbers are within valid range (ESP32 typically 0-48)
    if (config->gpio_step > 48 || config->gpio_dir > 48 ||
        (config->gpio_enable >= 0 && config->gpio_enable > 48) ||
        (config->gpio_hlfb >= 0 && config->gpio_hlfb > 48)) {
        ESP_LOGE(TAG, "GPIO pin number out of valid range (0-48)");
        return ESP_ERR_INVALID_ARG;
    }

    clearpath_servo_handle_t *handle = calloc(1, sizeof(clearpath_servo_handle_t));
    if (handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate servo handle");
        return ESP_ERR_NO_MEM;
    }

    memcpy(&handle->config, config, sizeof(clearpath_servo_config_t));
    handle->position = 0;
    handle->target_position = 0;
    handle->current_velocity = 0;
    handle->target_velocity = 0;
    handle->vel_max = config->vel_max > 0 ? config->vel_max : 1000;
    handle->accel_max = config->accel_max > 0 ? config->accel_max : 10000;
    handle->state = CLEARPATH_SERVO_STATE_IDLE;
    handle->enabled = false;
    handle->initialized = false;  // Set to true only after all initialization succeeds
    handle->homing_complete = false;
    handle->homing_start_time_us = 0;
    handle->homing_timeout_ms = 0;

    handle->mutex = xSemaphoreCreateMutex();
    if (handle->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(handle);
        return ESP_ERR_NO_MEM;
    }

    // Allocate RMT channel for step pulse generation
    // Find first free channel using bitmask (thread-safe)
    init_rmt_channel_mutex();
    if (s_rmt_channel_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create RMT channel mutex");
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    
    int ch = 0;
    if (xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
        for (ch = 0; ch < MAX_RMT_CHANNELS; ch++) {
            if (!(s_rmt_channel_bitmask & (1U << ch))) {
                // Found free channel
                handle->rmt_channel = (rmt_channel_t)ch;
                s_rmt_channel_bitmask |= (1U << ch);  // Mark as used
                break;
            }
        }
        xSemaphoreGive(s_rmt_channel_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire RMT channel mutex");
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return ESP_ERR_TIMEOUT;
    }
    
    if (ch >= MAX_RMT_CHANNELS) {
        ESP_LOGE(TAG, "No available RMT channels (all %d channels in use)", MAX_RMT_CHANNELS);
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return ESP_ERR_NO_MEM;
    }
    
    // Configure RMT for step pulse generation
    rmt_config_t rmt_config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)config->gpio_step, handle->rmt_channel);
    rmt_config.clk_div = RMT_CLK_DIV;  // 1 MHz clock (1 us resolution)
    rmt_config.tx_config.loop_en = false;
    rmt_config.tx_config.carrier_en = false;
    rmt_config.tx_config.idle_output_en = true;
    rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    
    esp_err_t ret = rmt_config(&rmt_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure RMT channel %d", handle->rmt_channel);
        // Free the channel in bitmask since we failed to configure it
        if (s_rmt_channel_mutex != NULL && xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
            s_rmt_channel_bitmask &= ~(1U << handle->rmt_channel);
            xSemaphoreGive(s_rmt_channel_mutex);
        }
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return ret;
    }
    
    ret = rmt_driver_install(handle->rmt_channel, RMT_TX_QUEUE_SIZE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install RMT driver for channel %d", handle->rmt_channel);
        // Free the channel in bitmask since we failed to install driver
        if (s_rmt_channel_mutex != NULL && xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
            s_rmt_channel_bitmask &= ~(1U << handle->rmt_channel);
            xSemaphoreGive(s_rmt_channel_mutex);
        }
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return ret;
    }
    
    // Pre-configure step pulse item: HIGH for STEP_PULSE_WIDTH_TICKS, then return to idle (LOW)
    // RMT will automatically return to idle level after the item completes
    handle->step_pulse_item.level0 = 1;
    handle->step_pulse_item.duration0 = STEP_PULSE_WIDTH_TICKS;
    handle->step_pulse_item.level1 = 0;
    handle->step_pulse_item.duration1 = 1;  // Minimum duration, RMT returns to idle after item
    
    // Configure direction GPIO (still uses regular GPIO)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << config->gpio_dir),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure dir GPIO");
        rmt_driver_uninstall(handle->rmt_channel);
        // Free the channel in bitmask
        if (s_rmt_channel_mutex != NULL && xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
            s_rmt_channel_bitmask &= ~(1U << handle->rmt_channel);
            xSemaphoreGive(s_rmt_channel_mutex);
        }
        vSemaphoreDelete(handle->mutex);
        free(handle);
        return ret;
    }

    // Configure enable GPIO if specified
    if (config->gpio_enable >= 0) {
        io_conf.pin_bit_mask = (1ULL << config->gpio_enable);
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure enable GPIO");
            rmt_driver_uninstall(handle->rmt_channel);
            // Free the channel in bitmask
            if (s_rmt_channel_mutex != NULL && xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
                s_rmt_channel_bitmask &= ~(1U << handle->rmt_channel);
                xSemaphoreGive(s_rmt_channel_mutex);
            }
            vSemaphoreDelete(handle->mutex);
            free(handle);
            return ret;
        }
        gpio_set_level(config->gpio_enable, 0); // Disable by default
    }

    // Configure HLFB GPIO if specified (input)
    if (config->gpio_hlfb >= 0) {
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << config->gpio_hlfb);
        io_conf.pull_down_en = config->hlfb_active_high ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = config->hlfb_active_high ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure HLFB GPIO");
            rmt_driver_uninstall(handle->rmt_channel);
            // Free the channel in bitmask
            if (s_rmt_channel_mutex != NULL && xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
                s_rmt_channel_bitmask &= ~(1U << handle->rmt_channel);
                xSemaphoreGive(s_rmt_channel_mutex);
            }
            vSemaphoreDelete(handle->mutex);
            free(handle);
            return ret;
        }
    }

    // Configure homing sensor GPIO if specified (input)
    if (config->gpio_homing_sensor >= 0) {
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << config->gpio_homing_sensor);
        // For NO sensor: pull-down (triggers when HIGH), for NC sensor: pull-up (triggers when LOW)
        if (config->homing_sensor_type == CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN) {
            io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        } else {
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        }
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure homing sensor GPIO");
            rmt_driver_uninstall(handle->rmt_channel);
            // Free the channel in bitmask
            if (s_rmt_channel_mutex != NULL && xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
                s_rmt_channel_bitmask &= ~(1U << handle->rmt_channel);
                xSemaphoreGive(s_rmt_channel_mutex);
            }
            vSemaphoreDelete(handle->mutex);
            free(handle);
            return ret;
        }
    }

    // Initialize GPIO states
    gpio_set_level(config->gpio_dir, 0);

    // Mark as initialized only after all initialization succeeds
    handle->initialized = true;

    *handle_out = handle;
    ESP_LOGI(TAG, "Servo initialized: Step=%d, Dir=%d, Enable=%d, HLFB=%d, Homing=%d",
             config->gpio_step, config->gpio_dir, config->gpio_enable, config->gpio_hlfb, config->gpio_homing_sensor);
    return ESP_OK;
}

esp_err_t clearpath_servo_deinit(clearpath_servo_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bool was_initialized = false;
    
    // Mark as uninitialized first to prevent new operations
    if (handle->mutex != NULL && xSemaphoreTake(handle->mutex, portMAX_DELAY) == pdTRUE) {
        was_initialized = handle->initialized;
        handle->initialized = false;
        xSemaphoreGive(handle->mutex);
    } else {
        // If mutex unavailable, assume initialized and proceed with cleanup
        was_initialized = true;
    }

    // Wait a bit for any in-flight operations to complete
    vTaskDelay(pdMS_TO_TICKS(10));

    // Uninstall RMT driver if it was initialized
    if (was_initialized) {
        rmt_driver_uninstall(handle->rmt_channel);
        // Mark channel as free in bitmask (thread-safe)
        if (s_rmt_channel_mutex != NULL && 
            handle->rmt_channel < MAX_RMT_CHANNELS) {
            if (xSemaphoreTake(s_rmt_channel_mutex, portMAX_DELAY) == pdTRUE) {
                s_rmt_channel_bitmask &= ~(1U << handle->rmt_channel);
                xSemaphoreGive(s_rmt_channel_mutex);
            }
        }
    }

    // Delete mutex and free handle
    if (handle->mutex != NULL) {
        vSemaphoreDelete(handle->mutex);
        handle->mutex = NULL;
    }

    free(handle);
    return ESP_OK;
}

esp_err_t clearpath_servo_move(clearpath_servo_handle_t *handle, int32_t steps, clearpath_servo_move_target_t target_type)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (target_type == CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE) {
        handle->target_position = steps;
    } else {
        handle->target_position = handle->position + steps;
    }

    handle->state = CLEARPATH_SERVO_STATE_MOVING;
    handle->target_velocity = 0; // Will be set by step generation task

    // Set direction GPIO
    int32_t steps_to_move = handle->target_position - handle->position;
    gpio_set_level(handle->config.gpio_dir, (steps_to_move >= 0) ? 1 : 0);

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t clearpath_servo_move_velocity(clearpath_servo_handle_t *handle, int32_t velocity)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    handle->target_velocity = velocity;
    handle->state = CLEARPATH_SERVO_STATE_VELOCITY_MODE;

    // Set direction GPIO
    gpio_set_level(handle->config.gpio_dir, (velocity >= 0) ? 1 : 0);

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t clearpath_servo_stop(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    handle->target_velocity = 0;
    handle->current_velocity = 0;
    
    // If stopping from homing state, mark homing as complete
    if (handle->state == CLEARPATH_SERVO_STATE_HOMING) {
        handle->homing_complete = true;
    }
    
    handle->state = CLEARPATH_SERVO_STATE_IDLE;
    handle->target_position = handle->position;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t clearpath_servo_set_vel_max(clearpath_servo_handle_t *handle, uint32_t vel_max)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    handle->vel_max = vel_max;
    handle->config.vel_max = vel_max;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t clearpath_servo_set_accel_max(clearpath_servo_handle_t *handle, uint32_t accel_max)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    handle->accel_max = accel_max;
    handle->config.accel_max = accel_max;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

int32_t clearpath_servo_get_position(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        // If mutex take fails, return 0 to avoid race condition
        // Caller should retry if this is critical
        return 0;
    }

    int32_t pos = handle->position;
    xSemaphoreGive(handle->mutex);
    return pos;
}

esp_err_t clearpath_servo_set_position(clearpath_servo_handle_t *handle, int32_t position)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    handle->position = position;
    handle->target_position = position;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

/**
 * @brief Atomically increment/decrement position by delta
 * 
 * Internal function for step generation to avoid race conditions.
 * This is more efficient than get_position + set_position.
 * 
 * @param handle Servo handle
 * @param delta Position delta (+1 or -1 typically)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_increment_position(clearpath_servo_handle_t *handle, int32_t delta)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    handle->position += delta;
    // Update target_position if we're in a move and have reached/passed it
    if (handle->state == CLEARPATH_SERVO_STATE_MOVING) {
        int32_t steps_remaining = handle->target_position - handle->position;
        if ((delta > 0 && steps_remaining <= 0) || (delta < 0 && steps_remaining >= 0)) {
            // Reached or passed target, mark as complete
            handle->target_position = handle->position;
        }
    }

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

/**
 * @brief Generate a step pulse using RMT
 * 
 * Internal function for step generation. Generates a precise step pulse
 * using the RMT peripheral for accurate timing.
 * 
 * @param handle Servo handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_generate_step_pulse(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Generate step pulse using RMT
    // The pulse item is pre-configured: HIGH for STEP_PULSE_WIDTH_TICKS, then LOW
    esp_err_t ret = rmt_write_items(handle->rmt_channel, &handle->step_pulse_item, 1, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to generate RMT step pulse");
        return ret;
    }

    return ESP_OK;
}

esp_err_t clearpath_servo_reset_position(clearpath_servo_handle_t *handle)
{
    return clearpath_servo_set_position(handle, 0);
}

bool clearpath_servo_steps_complete(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return true;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool complete = (handle->state == CLEARPATH_SERVO_STATE_IDLE) ||
                    (handle->state == CLEARPATH_SERVO_STATE_MOVING && 
                     handle->position == handle->target_position);

    xSemaphoreGive(handle->mutex);
    return complete;
}

int32_t clearpath_servo_get_velocity(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        // If mutex take fails, return 0 to avoid race condition
        // Caller should retry if this is critical
        return 0;
    }

    int32_t vel = handle->current_velocity;
    xSemaphoreGive(handle->mutex);
    return vel;
}

bool clearpath_servo_get_hlfb_status(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized || handle->config.gpio_hlfb < 0) {
        return false;
    }

    // GPIO read is atomic, but we should validate handle is still valid
    // Note: Config is read-only after init, so accessing gpio_hlfb is safe
    return gpio_get_level(handle->config.gpio_hlfb);
}

clearpath_servo_hlfb_mode_t clearpath_servo_get_hlfb_mode(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return CLEARPATH_SERVO_HLFB_MODE_ENABLED;
    }

    // Config is read-only after init, so accessing hlfb_mode is safe without mutex
    return handle->config.hlfb_mode;
}

bool clearpath_servo_is_enabled(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return false;
    }

    if (handle->config.gpio_hlfb < 0) {
        // Fall back to enable GPIO state - need mutex for thread safety
        if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
            return false;
        }
        bool enabled = handle->enabled;
        xSemaphoreGive(handle->mutex);
        return enabled;
    }

    bool hlfb = clearpath_servo_get_hlfb_status(handle);
    // Note: hlfb_active_high means HLFB signal is active when high
    // If hlfb_active_high is true, then hlfb=true means enabled
    // If hlfb_active_high is false, then hlfb=false means enabled (active low)
    if (handle->config.hlfb_mode == CLEARPATH_SERVO_HLFB_MODE_ENABLED) {
        return handle->config.hlfb_active_high ? hlfb : !hlfb;
    }

    // Fall back to enable GPIO state - need mutex for thread safety
    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    bool enabled = handle->enabled;
    xSemaphoreGive(handle->mutex);
    return enabled;
}

bool clearpath_servo_is_move_complete(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return true;
    }

    // First check step count
    if (clearpath_servo_steps_complete(handle)) {
        return true;
    }

    // Then check HLFB if configured
    if (handle->config.gpio_hlfb >= 0 && 
        handle->config.hlfb_mode == CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE) {
        bool hlfb = clearpath_servo_get_hlfb_status(handle);
        if (handle->config.hlfb_active_high) {
            return hlfb;
        } else {
            return !hlfb;
        }
    }

    return false;
}

bool clearpath_servo_is_fault(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return false;
    }

    if (handle->config.gpio_hlfb < 0 || 
        handle->config.hlfb_mode != CLEARPATH_SERVO_HLFB_MODE_FAULT) {
        return false;
    }

    bool hlfb = clearpath_servo_get_hlfb_status(handle);
    if (handle->config.hlfb_active_high) {
        return !hlfb; // Fault is active low
    } else {
        return hlfb; // Fault is active high
    }
}

esp_err_t clearpath_servo_set_enable(clearpath_servo_handle_t *handle, bool enable)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (handle->config.gpio_enable < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    gpio_set_level(handle->config.gpio_enable, enable ? 1 : 0);
    handle->enabled = enable;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t clearpath_servo_home(clearpath_servo_handle_t *handle, int32_t direction, uint32_t timeout_ms)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (handle->config.gpio_homing_sensor < 0) {
        ESP_LOGE(TAG, "Homing sensor GPIO not configured");
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Set homing state
    handle->state = CLEARPATH_SERVO_STATE_HOMING;
    handle->homing_complete = false;
    handle->homing_start_time_us = esp_timer_get_time();
    handle->homing_timeout_ms = timeout_ms;
    
    // Set homing velocity (use configured homing velocity or vel_max)
    uint32_t homing_vel = handle->config.homing_velocity > 0 ? 
                          handle->config.homing_velocity : handle->vel_max;
    handle->target_velocity = (direction >= 0) ? (int32_t)homing_vel : -(int32_t)homing_vel;
    
    // Set direction GPIO
    gpio_set_level(handle->config.gpio_dir, (direction >= 0) ? 1 : 0);

    xSemaphoreGive(handle->mutex);
    ESP_LOGI(TAG, "Homing started: direction=%d, velocity=%d steps/s, timeout=%lu ms", 
             direction, homing_vel, timeout_ms);
    return ESP_OK;
}

/**
 * @brief Mark homing as complete and set position to zero
 * 
 * Internal function called by manager when homing sensor triggers.
 * 
 * @param handle Servo handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_complete_homing(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (handle->state == CLEARPATH_SERVO_STATE_HOMING) {
        handle->position = 0;
        handle->target_position = 0;
        handle->current_velocity = 0;
        handle->target_velocity = 0;
        handle->state = CLEARPATH_SERVO_STATE_IDLE;
        handle->homing_complete = true;
        ESP_LOGI(TAG, "Homing complete - position set to 0");
    }

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

/**
 * @brief Check if homing timeout has been reached
 * 
 * @param handle Servo handle
 * @return true if timeout reached, false otherwise
 */
bool clearpath_servo_is_homing_timeout(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return false;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool timeout = false;
    if (handle->state == CLEARPATH_SERVO_STATE_HOMING && handle->homing_timeout_ms > 0) {
        uint64_t now_us = esp_timer_get_time();
        uint64_t elapsed_ms = (now_us - handle->homing_start_time_us) / 1000;
        if (elapsed_ms >= handle->homing_timeout_ms) {
            timeout = true;
        }
    }

    xSemaphoreGive(handle->mutex);
    return timeout;
}

bool clearpath_servo_is_homing_complete(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return false;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool complete = handle->homing_complete;
    xSemaphoreGive(handle->mutex);
    return complete;
}

bool clearpath_servo_get_homing_sensor_status(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized || handle->config.gpio_homing_sensor < 0) {
        return false;
    }

    // GPIO read is atomic
    bool sensor_level = gpio_get_level(handle->config.gpio_homing_sensor);
    
    // Interpret based on sensor type:
    // NO (Normally Open): triggers when HIGH (closed)
    // NC (Normally Closed): triggers when LOW (opened)
    if (handle->config.homing_sensor_type == CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN) {
        return sensor_level;  // HIGH = triggered
    } else {
        return !sensor_level;  // LOW = triggered
    }
}

int32_t clearpath_servo_get_target_position(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    int32_t target_pos = handle->target_position;
    xSemaphoreGive(handle->mutex);
    return target_pos;
}

int32_t clearpath_servo_get_target_velocity(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    int32_t target_vel = handle->target_velocity;
    xSemaphoreGive(handle->mutex);
    return target_vel;
}

uint32_t clearpath_servo_get_vel_max(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    uint32_t vel_max = handle->vel_max;
    xSemaphoreGive(handle->mutex);
    return vel_max;
}

uint32_t clearpath_servo_get_accel_max(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return 0;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    uint32_t accel_max = handle->accel_max;
    xSemaphoreGive(handle->mutex);
    return accel_max;
}

clearpath_servo_state_t clearpath_servo_get_state(clearpath_servo_handle_t *handle)
{
    if (handle == NULL || !handle->initialized) {
        return CLEARPATH_SERVO_STATE_IDLE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return CLEARPATH_SERVO_STATE_IDLE;
    }

    clearpath_servo_state_t state = handle->state;
    xSemaphoreGive(handle->mutex);
    return state;
}

esp_err_t clearpath_servo_set_current_velocity(clearpath_servo_handle_t *handle, int32_t velocity)
{
    if (handle == NULL || !handle->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(handle->mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    handle->current_velocity = velocity;
    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}
