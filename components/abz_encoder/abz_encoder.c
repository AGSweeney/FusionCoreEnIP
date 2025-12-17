/**
 * @file abz_encoder.c
 * @brief ABZ Rotary Encoder Driver Implementation
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * 
 * Quadrature Decoding Algorithm:
 * ------------------------------
 * 
 * The encoder uses a 4-state state machine to decode quadrature signals:
 * 
 * State encoding: (A << 1) | B
 * - State 0: A=0, B=0 (00)
 * - State 1: A=0, B=1 (01)
 * - State 2: A=1, B=0 (10)
 * - State 3: A=1, B=1 (11)
 * 
 * Forward rotation sequence: 00 → 01 → 11 → 10 → 00 (clockwise)
 * Reverse rotation sequence: 00 → 10 → 11 → 01 → 00 (counter-clockwise)
 * 
 * State transition table:
 * 
 * | Previous | Current | Direction | Delta |
 * |----------|---------|-----------|-------|
 * | 00       | 01      | Forward   | +1    |
 * | 00       | 10      | Reverse   | -1    |
 * | 01       | 00      | Reverse   | -1    |
 * | 01       | 11      | Forward   | +1    |
 * | 11       | 01      | Reverse   | -1    |
 * | 11       | 10      | Forward   | +1    |
 * | 10       | 11      | Forward   | +1    |
 * | 10       | 00      | Reverse   | -1    |
 * 
 * In 4x resolution mode, all state transitions are counted.
 * In 1x resolution mode, only transitions involving A channel edges are counted.
 */

#include "abz_encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <limits.h>

static const char *TAG = "abz_encoder";

// Quadrature state transition table
// Format: [previous_state][current_state] = delta
// Positive delta = forward, negative delta = reverse
// Forward sequence: 00 → 01 → 11 → 10 → 00
// Reverse sequence: 00 → 10 → 11 → 01 → 00
static const int8_t quadrature_table[4][4] = {
    // Previous state 0 (00)
    { 0,  1, -1,  0},  // Current: 00, 01, 10, 11
    // Previous state 1 (01)
    {-1,  0,  0,  1},  // Current: 00, 01, 10, 11
    // Previous state 2 (10)
    { 1,  0,  0, -1},  // Current: 00, 01, 10, 11
    // Previous state 3 (11)
    { 0, -1,  1,  0}   // Current: 00, 01, 10, 11
};

esp_err_t abz_encoder_init(abz_encoder_t *encoder, abz_encoder_resolution_t resolution)
{
    if (encoder == NULL) {
        ESP_LOGE(TAG, "Encoder handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (resolution != ABZ_ENCODER_RESOLUTION_1X && resolution != ABZ_ENCODER_RESOLUTION_4X) {
        ESP_LOGE(TAG, "Invalid resolution mode: %d", resolution);
        return ESP_ERR_INVALID_ARG;
    }

    memset(encoder, 0, sizeof(abz_encoder_t));
    encoder->resolution = resolution;
    encoder->initialized = true;
    encoder->last_direction = ABZ_ENCODER_DIRECTION_FORWARD;

    // Create mutex for thread-safe access
    encoder->mutex = xSemaphoreCreateMutex();
    if (encoder->mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        encoder->initialized = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Encoder initialized with %dx resolution", resolution);
    return ESP_OK;
}

esp_err_t abz_encoder_deinit(abz_encoder_t *encoder)
{
    if (encoder == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (encoder->mutex != NULL) {
        SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
        vSemaphoreDelete(mutex);
        encoder->mutex = NULL;
    }

    encoder->initialized = false;
    return ESP_OK;
}

esp_err_t abz_encoder_process_quadrature(abz_encoder_t *encoder, bool a_state, bool b_state)
{
    if (encoder == NULL || !encoder->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t mutex = (SemaphoreHandle_t)encoder->mutex;
    if (mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Take mutex - blocking since this is called from task context, not ISR
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Encode current state: (A << 1) | B
    uint8_t current_state = ((uint8_t)a_state << 1) | (uint8_t)b_state;
    
    // Get delta from transition table
    int8_t delta = quadrature_table[encoder->state][current_state];
    
    // Update position based on delta
    if (delta != 0) {
        // Determine direction from delta sign
        if (delta > 0) {
            encoder->last_direction = ABZ_ENCODER_DIRECTION_FORWARD;
        } else {
            encoder->last_direction = ABZ_ENCODER_DIRECTION_REVERSE;
        }

        // In 1x resolution mode, only count transitions involving A channel
        if (encoder->resolution == ABZ_ENCODER_RESOLUTION_1X) {
            // Only count if A channel changed (state transition involves A edge)
            uint8_t prev_a = (encoder->state >> 1) & 0x01;
            uint8_t curr_a = (current_state >> 1) & 0x01;
            if (prev_a != curr_a) {
                encoder->position += delta;
            }
        } else {
            // 4x resolution: count all valid transitions
            encoder->position += delta;
        }
    }

    // Update state
    encoder->state = current_state;

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
