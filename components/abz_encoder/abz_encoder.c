/**
 * @file abz_encoder.c
 * @brief ABZ Rotary Encoder Driver Implementation using PCNT Hardware
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * 
 * PCNT Hardware Quadrature Decoding:
 * -----------------------------------
 * 
 * This implementation uses the ESP32-P4 PCNT (Pulse Counter) peripheral for
 * hardware-based quadrature decoding. The PCNT peripheral automatically:
 * - Decodes A/B quadrature signals
 * - Determines direction (forward/reverse)
 * - Counts pulses with configurable resolution (1x or 4x)
 * - Filters glitches
 * 
 * Resolution modes:
 * - 1x resolution: Counts only on A channel edges (half quadrature)
 * - 4x resolution: Counts on all A/B edges (full quadrature)
 * 
 * The PCNT hardware handles all quadrature decoding, eliminating the need for
 * software state machines or GPIO interrupts on A/B channels.
 */

#include "abz_encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <limits.h>

static const char *TAG = "abz_encoder";

esp_err_t abz_encoder_init(abz_encoder_t *encoder, abz_encoder_resolution_t resolution, int gpio_a, int gpio_b)
{
    if (encoder == NULL) {
        ESP_LOGE(TAG, "Encoder handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (resolution != ABZ_ENCODER_RESOLUTION_1X && resolution != ABZ_ENCODER_RESOLUTION_4X) {
        ESP_LOGE(TAG, "Invalid resolution mode: %d", resolution);
        return ESP_ERR_INVALID_ARG;
    }

    if (gpio_a < 0 || gpio_b < 0) {
        ESP_LOGE(TAG, "Invalid GPIO pins: A=%d, B=%d", gpio_a, gpio_b);
        return ESP_ERR_INVALID_ARG;
    }

    memset(encoder, 0, sizeof(abz_encoder_t));
    encoder->resolution = resolution;
    encoder->last_direction = ABZ_ENCODER_DIRECTION_FORWARD;
    encoder->pcnt_offset = 0;

    // Create mutex for thread-safe access
    encoder->mutex = xSemaphoreCreateMutex();
    if (encoder->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure PCNT unit for quadrature decoding
    pcnt_unit_config_t unit_config = {
        .high_limit = INT16_MAX,
        .low_limit = INT16_MIN,
    };
    esp_err_t ret = pcnt_new_unit(&unit_config, &encoder->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT unit: %s", esp_err_to_name(ret));
        vSemaphoreDelete((SemaphoreHandle_t)encoder->mutex);
        return ret;
    }

    // Configure PCNT channel for A/B quadrature signals
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = gpio_a,
        .level_gpio_num = gpio_b,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ret = pcnt_new_channel(encoder->pcnt_unit, &chan_config, &pcnt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create PCNT channel: %s", esp_err_to_name(ret));
        pcnt_del_unit(encoder->pcnt_unit);
        vSemaphoreDelete((SemaphoreHandle_t)encoder->mutex);
        return ret;
    }

    // Set edge and level actions for quadrature decoding
    // Forward: A rising edge when B is low, A falling edge when B is high
    // Reverse: A rising edge when B is high, A falling edge when B is low
    ret = pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCNT edge actions: %s", esp_err_to_name(ret));
        pcnt_del_unit(encoder->pcnt_unit);
        vSemaphoreDelete((SemaphoreHandle_t)encoder->mutex);
        return ret;
    }

    // Set level action for direction detection
    ret = pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PCNT level actions: %s", esp_err_to_name(ret));
        pcnt_del_unit(encoder->pcnt_unit);
        vSemaphoreDelete((SemaphoreHandle_t)encoder->mutex);
        return ret;
    }

    // Configure glitch filter (optional, helps with noisy signals)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,  // Filter pulses shorter than 1us
    };
    pcnt_unit_set_glitch_filter(encoder->pcnt_unit, &filter_config);

    // Enable PCNT unit
    ret = pcnt_unit_enable(encoder->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable PCNT unit: %s", esp_err_to_name(ret));
        pcnt_del_unit(encoder->pcnt_unit);
        vSemaphoreDelete((SemaphoreHandle_t)encoder->mutex);
        return ret;
    }

    // Clear counter and start counting
    ret = pcnt_unit_clear_count(encoder->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to clear PCNT count: %s", esp_err_to_name(ret));
        pcnt_unit_disable(encoder->pcnt_unit);
        pcnt_del_unit(encoder->pcnt_unit);
        vSemaphoreDelete((SemaphoreHandle_t)encoder->mutex);
        return ret;
    }

    ret = pcnt_unit_start(encoder->pcnt_unit);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start PCNT unit: %s", esp_err_to_name(ret));
        pcnt_unit_disable(encoder->pcnt_unit);
        pcnt_del_unit(encoder->pcnt_unit);
        vSemaphoreDelete((SemaphoreHandle_t)encoder->mutex);
        return ret;
    }

    encoder->initialized = true;
    ESP_LOGI(TAG, "Encoder initialized with PCNT: GPIO_A=%d, GPIO_B=%d, %dx resolution", 
             gpio_a, gpio_b, resolution);
    return ESP_OK;
}

esp_err_t abz_encoder_deinit(abz_encoder_t *encoder)
{
    if (encoder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (encoder->initialized && encoder->pcnt_unit != NULL) {
        pcnt_unit_stop(encoder->pcnt_unit);
        pcnt_unit_disable(encoder->pcnt_unit);
        pcnt_del_unit(encoder->pcnt_unit);
        encoder->pcnt_unit = NULL;
    }

    if (encoder->mutex != NULL) {
        SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
        vSemaphoreDelete(mutex);
        encoder->mutex = NULL;
    }

    encoder->initialized = false;
    return ESP_OK;
}

esp_err_t abz_encoder_update_position(abz_encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized || encoder->pcnt_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex - blocking since this is called from task context
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read PCNT hardware counter
    int16_t pcnt_count = 0;
    esp_err_t ret = pcnt_unit_get_count(encoder->pcnt_unit, &pcnt_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PCNT count: %s", esp_err_to_name(ret));
        xSemaphoreGive(mutex);
        return ret;
    }

    // Calculate delta from PCNT counter
    int32_t delta = (int32_t)pcnt_count - encoder->pcnt_offset;
    
    // Apply resolution scaling if needed
    // PCNT hardware counts in 4x mode by default
    if (encoder->resolution == ABZ_ENCODER_RESOLUTION_1X) {
        // For 1x resolution, divide by 4 (PCNT counts all edges, we only want A edges)
        // Use rounding division to preserve precision: (delta + sign(delta)*2) / 4
        if (delta >= 0) {
            delta = (delta + 2) / 4;  // Round up for positive
        } else {
            delta = (delta - 2) / 4;  // Round down for negative
        }
    }

    // Update position and direction
    if (delta != 0) {
        encoder->position += delta;
        encoder->last_direction = (delta > 0) ? ABZ_ENCODER_DIRECTION_FORWARD : ABZ_ENCODER_DIRECTION_REVERSE;
        
        // Reset PCNT offset to track relative changes
        encoder->pcnt_offset = pcnt_count;
    }

    xSemaphoreGive(mutex);
    return ESP_OK;
}

esp_err_t abz_encoder_process_index(abz_encoder_t *encoder, bool reset_position)
{
    if (encoder == NULL || !encoder->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    encoder->index_detected = true;

    if (reset_position) {
        encoder->position = 0;
        ESP_LOGI(TAG, "Index pulse detected - position reset to 0");
    } else {
        ESP_LOGI(TAG, "Index pulse detected - position preserved");
    }

    xSemaphoreGive(mutex);
    return ESP_OK;
}

int32_t abz_encoder_get_position(const abz_encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) {
        return 0;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return 0;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    int32_t position = encoder->position;
    xSemaphoreGive(mutex);
    return position;
}

esp_err_t abz_encoder_set_position(abz_encoder_t *encoder, int32_t position)
{
    if (encoder == NULL || !encoder->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    encoder->position = position;
    
    // Reset PCNT counter and offset to match new position
    if (encoder->pcnt_unit != NULL) {
        pcnt_unit_clear_count(encoder->pcnt_unit);
        encoder->pcnt_offset = 0;
    }
    
    xSemaphoreGive(mutex);
    return ESP_OK;
}

esp_err_t abz_encoder_reset_position(abz_encoder_t *encoder)
{
    return abz_encoder_set_position(encoder, 0);
}

bool abz_encoder_get_index_detected(const abz_encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) {
        return false;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return false;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool index_detected = encoder->index_detected;
    xSemaphoreGive(mutex);
    return index_detected;
}

esp_err_t abz_encoder_clear_index(abz_encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    encoder->index_detected = false;
    xSemaphoreGive(mutex);
    return ESP_OK;
}

abz_encoder_direction_t abz_encoder_get_direction(const abz_encoder_t *encoder)
{
    if (encoder == NULL || !encoder->initialized) {
        return ABZ_ENCODER_DIRECTION_FORWARD;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return ABZ_ENCODER_DIRECTION_FORWARD;
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ABZ_ENCODER_DIRECTION_FORWARD;
    }

    abz_encoder_direction_t direction = encoder->last_direction;
    xSemaphoreGive(mutex);
    return direction;
}

int16_t abz_encoder_calculate_velocity(int32_t position_delta, uint32_t time_delta_ms)
{
    if (time_delta_ms == 0) {
        return 0;
    }
    
    // Calculate velocity in counts per second
    // velocity = (delta / time_ms) * 1000
    int64_t velocity = ((int64_t)position_delta * 1000) / (int64_t)time_delta_ms;
    
    // Clamp to int16_t range
    if (velocity > INT16_MAX) {
        return INT16_MAX;
    } else if (velocity < INT16_MIN) {
        return INT16_MIN;
    }
    
    return (int16_t)velocity;
}
