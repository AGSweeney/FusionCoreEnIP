/**
 * @file abz_encoder_manager.c
 * @brief ABZ Encoder Manager Implementation
 * 
 * **NOTE: This component is scaffolded but NOT included in the build.**
 * 
 * Architecture:
 * ------------
 * 
 * GPIO Interrupt Flow:
 *   A/B/Z GPIO → ISR Handler → Queue → Task → Quadrature Decoder → Assembly Data
 * 
 * 1. GPIO interrupts configured on A/B channels (both edges) and Z channel (rising edge)
 * 2. ISR handler reads GPIO states and sends to queue (non-blocking)
 * 3. Task processes queue events and updates encoder position
 * 4. Velocity calculated periodically from position delta
 * 5. Assembly data updated with mutex protection
 * 
 * Assembly Data Layout (Input Assembly 100, Bytes 61-71):
 * - Bytes 61-64: Position (int32_t, little-endian)
 * - Bytes 65-66: Velocity (int16_t, counts/second, little-endian)
 * - Byte 67: Status flags (bit 0=index detected, bit 1=direction, bit 2=initialized)
 * - Bytes 68-71: Reserved (4 bytes for future expansion)
 */

#include "abz_encoder_manager.h"
#include "abz_encoder.h"
#include "fusion_core_assembly.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "abz_encoder_manager";

// Encoder event structure for queue
typedef struct {
    uint32_t gpio_num;      /**< GPIO number that triggered interrupt */
    uint32_t level;         /**< GPIO level (for A/B channels) */
    uint64_t timestamp_us;  /**< Timestamp in microseconds */
} encoder_event_t;

// Manager state
static bool s_initialized = false;
static abz_encoder_t s_encoder;
static abz_encoder_config_t s_config;
static QueueHandle_t s_event_queue = NULL;
static TaskHandle_t s_task_handle = NULL;
static SemaphoreHandle_t s_config_mutex = NULL;

// Velocity calculation state
static int32_t s_last_position = 0;
static uint64_t s_last_velocity_update_us = 0;
static int16_t s_current_velocity = 0;

// Default configuration
#define DEFAULT_GPIO_A           8
#define DEFAULT_GPIO_B           9
#define DEFAULT_GPIO_Z           10
#define DEFAULT_RESOLUTION       ABZ_ENCODER_RESOLUTION_4X
#define DEFAULT_VELOCITY_INTERVAL_MS  100
#define DEFAULT_INDEX_RESET      false
#define EVENT_QUEUE_SIZE         32

/**
 * @brief GPIO interrupt service routine
 * 
 * This ISR runs in IRAM context and must be fast.
 * It only reads GPIO states and sends to queue.
 */
static void IRAM_ATTR encoder_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Note: esp_timer_get_time() may not be IRAM-safe on all ESP32 variants
    // For production, consider using a simpler timestamp or IRAM-safe timer
    uint64_t timestamp_us = 0;
    #ifdef CONFIG_ESP_TIMER_PROFILING
    // esp_timer_get_time() is IRAM-safe when profiling is enabled
    timestamp_us = esp_timer_get_time();
    #else
    // Fallback: use 0 timestamp, task will use its own timing
    timestamp_us = 0;
    #endif
    
    encoder_event_t event = {
        .gpio_num = gpio_num,
        .level = gpio_get_level(gpio_num),
        .timestamp_us = timestamp_us
    };
    
    // Send to queue from ISR
    if (s_event_queue != NULL) {
        xQueueSendFromISR(s_event_queue, &event, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Encoder processing task
 * 
 * This task processes encoder events from the queue and updates position.
 */
static void encoder_task(void *pvParameters)
{
    (void)pvParameters;
    encoder_event_t event;
    
    ESP_LOGI(TAG, "Encoder task started");
    
    while (1) {
        // Wait for event from queue
        if (xQueueReceive(s_event_queue, &event, portMAX_DELAY)) {
            if (!s_initialized) {
                continue;
            }
            
            // Read current GPIO states and config with mutex protection
            abz_encoder_config_t config_copy;
            if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(&config_copy, &s_config, sizeof(abz_encoder_config_t));
                xSemaphoreGive(s_config_mutex);
            } else {
                // Fallback to direct access if mutex unavailable (shouldn't happen)
                memcpy(&config_copy, &s_config, sizeof(abz_encoder_config_t));
            }
            
            bool a_state = gpio_get_level(config_copy.gpio_a);
            bool b_state = gpio_get_level(config_copy.gpio_b);
            bool z_state = gpio_get_level(config_copy.gpio_z);
            
            // Process based on which GPIO triggered
            if (event.gpio_num == config_copy.gpio_a || event.gpio_num == config_copy.gpio_b) {
                // Quadrature signal change
                abz_encoder_process_quadrature(&s_encoder, a_state, b_state);
            } else if (event.gpio_num == config_copy.gpio_z && z_state == 1) {
                // Index pulse (rising edge)
                abz_encoder_process_index(&s_encoder, config_copy.index_reset_position);
            }
            
            // Update assembly data periodically
            static uint64_t last_assembly_update_us = 0;
            uint64_t now_us = esp_timer_get_time();
            if (now_us - last_assembly_update_us > 10000) {  // Update every 10ms
                SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
                if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    int32_t position = abz_encoder_get_position(&s_encoder);
                    bool index_detected = abz_encoder_get_index_detected(&s_encoder);
                    abz_encoder_direction_t direction = abz_encoder_get_direction(&s_encoder);
                    
                    // Write position (bytes 61-64)
                    memcpy(&INPUT_ASSEMBLY_100[61], &position, sizeof(int32_t));
                    
                    // Calculate velocity (read config with mutex)
                    uint32_t velocity_interval_ms = DEFAULT_VELOCITY_INTERVAL_MS;
                    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                        velocity_interval_ms = s_config.velocity_interval_ms;
                        xSemaphoreGive(s_config_mutex);
                    }
                    
                    uint64_t time_delta_us = now_us - s_last_velocity_update_us;
                    if (time_delta_us >= (velocity_interval_ms * 1000)) {
                        int32_t position_delta = position - s_last_position;
                        s_current_velocity = abz_encoder_calculate_velocity(
                            position_delta, 
                            time_delta_us / 1000
                        );
                        s_last_position = position;
                        s_last_velocity_update_us = now_us;
                    }
                    
                    // Write velocity (bytes 65-66)
                    memcpy(&INPUT_ASSEMBLY_100[65], &s_current_velocity, sizeof(int16_t));
                    
                    // Write status flags (byte 67)
                    uint8_t status = 0;
                    if (index_detected) status |= 0x01;
                    if (direction == ABZ_ENCODER_DIRECTION_FORWARD) status |= 0x02;
                    if (s_initialized) status |= 0x04;
                    INPUT_ASSEMBLY_100[67] = status;
                    
                    xSemaphoreGive(assembly_mutex);
                    last_assembly_update_us = now_us;
                }
            }
        }
    }
}

/**
 * @brief Configure GPIO for encoder
 */
static esp_err_t configure_gpio(void)
{
    // Validate GPIO pin numbers
    if (s_config.gpio_a < 0 || s_config.gpio_a > 48 ||
        s_config.gpio_b < 0 || s_config.gpio_b > 48 ||
        s_config.gpio_z < 0 || s_config.gpio_z > 48) {
        ESP_LOGE(TAG, "Invalid GPIO pin numbers: A=%d, B=%d, Z=%d", 
                 s_config.gpio_a, s_config.gpio_b, s_config.gpio_z);
        return ESP_ERR_INVALID_ARG;
    }

    // Configure A channel (both edges)
    gpio_config_t io_conf_a = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << s_config.gpio_a),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    esp_err_t ret = gpio_config(&io_conf_a);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO A: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure B channel (both edges)
    gpio_config_t io_conf_b = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << s_config.gpio_b),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    ret = gpio_config(&io_conf_b);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO B: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure Z channel (rising edge for index pulse)
    gpio_config_t io_conf_z = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << s_config.gpio_z),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    ret = gpio_config(&io_conf_z);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO Z: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Install GPIO ISR service if not already installed
    // Note: gpio_install_isr_service() is safe to call multiple times
    // It will return ESP_ERR_INVALID_STATE if already installed, which we can ignore
    esp_err_t isr_ret = gpio_install_isr_service(0);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(isr_ret));
        return isr_ret;
    }
    
    // Add ISR handlers
    gpio_isr_handler_add(s_config.gpio_a, encoder_isr_handler, (void*)s_config.gpio_a);
    gpio_isr_handler_add(s_config.gpio_b, encoder_isr_handler, (void*)s_config.gpio_b);
    gpio_isr_handler_add(s_config.gpio_z, encoder_isr_handler, (void*)s_config.gpio_z);
    
    ESP_LOGI(TAG, "GPIO configured: A=%d, B=%d, Z=%d", 
             s_config.gpio_a, s_config.gpio_b, s_config.gpio_z);
    
    return ESP_OK;
}

esp_err_t abz_encoder_manager_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Encoder manager already initialized");
        return ESP_OK;
    }
    
    // TODO: Load configuration from NVS
    // For now, use defaults
    s_config.gpio_a = DEFAULT_GPIO_A;
    s_config.gpio_b = DEFAULT_GPIO_B;
    s_config.gpio_z = DEFAULT_GPIO_Z;
    s_config.resolution = DEFAULT_RESOLUTION;
    s_config.velocity_interval_ms = DEFAULT_VELOCITY_INTERVAL_MS;
    s_config.index_reset_position = DEFAULT_INDEX_RESET;
    s_config.enabled = true;
    
    if (!s_config.enabled) {
        ESP_LOGI(TAG, "Encoder is disabled in configuration");
        return ESP_OK;
    }
    
    // Create mutex
    s_config_mutex = xSemaphoreCreateMutex();
    if (s_config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create config mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize encoder driver
    esp_err_t ret = abz_encoder_init(&s_encoder, s_config.resolution);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize encoder driver");
        return ret;
    }
    
    // Configure GPIO
    ret = configure_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO");
        return ret;
    }
    
    // Create event queue
    s_event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(encoder_event_t));
    if (s_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Create task
    xTaskCreate(encoder_task, "encoder_task", 4096, NULL, 10, &s_task_handle);
    if (s_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder task");
        vQueueDelete(s_event_queue);
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize velocity calculation state
    s_last_position = abz_encoder_get_position(&s_encoder);
    s_last_velocity_update_us = esp_timer_get_time();
    s_current_velocity = 0;
    
    s_initialized = true;
    ESP_LOGI(TAG, "Encoder manager initialized successfully");
    
    return ESP_OK;
}

bool abz_encoder_manager_is_initialized(void)
{
    return s_initialized;
}

esp_err_t abz_encoder_manager_get_config(abz_encoder_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (s_config_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    memcpy(config, &s_config, sizeof(abz_encoder_config_t));
    xSemaphoreGive(s_config_mutex);
    return ESP_OK;
}

esp_err_t abz_encoder_manager_set_config(const abz_encoder_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // TODO: Validate GPIO pins, save to NVS
    
    if (s_config_mutex != NULL && xSemaphoreTake(s_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&s_config, config, sizeof(abz_encoder_config_t));
        xSemaphoreGive(s_config_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

int32_t abz_encoder_manager_get_position(void)
{
    if (!s_initialized) {
        return 0;
    }
    return abz_encoder_get_position(&s_encoder);
}

esp_err_t abz_encoder_manager_reset_position(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return abz_encoder_reset_position(&s_encoder);
}

int16_t abz_encoder_manager_get_velocity(void)
{
    // s_current_velocity is updated atomically (single write operation)
    // Reading int16_t is atomic on ESP32, so this is safe without mutex
    return s_current_velocity;
}

bool abz_encoder_manager_get_index_detected(void)
{
    if (!s_initialized) {
        return false;
    }
    return abz_encoder_get_index_detected(&s_encoder);
}

abz_encoder_direction_t abz_encoder_manager_get_direction(void)
{
    if (!s_initialized) {
        return ABZ_ENCODER_DIRECTION_FORWARD;
    }
    return abz_encoder_get_direction(&s_encoder);
}
