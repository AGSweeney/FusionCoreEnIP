#ifndef VL53L1X_CONFIG_H
#define VL53L1X_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief VL53L1x configuration structure
 * 
 * Contains all configurable parameters for the VL53L1x sensor
 */
typedef struct {
    // Distance Mode Configuration
    uint16_t distance_mode;           // 1=SHORT (<1.3m), 2=LONG (<4m, default)
    uint16_t timing_budget_ms;        // 15, 20, 33, 50, 100 (default), 200, 500
    uint32_t inter_measurement_ms;    // Must be >= timing_budget, default: 100
    
    // Region of Interest (ROI) Configuration
    uint16_t roi_x_size;              // 4-16 (default: 16)
    uint16_t roi_y_size;              // 4-16 (default: 16)
    uint8_t roi_center_spad;          // 0-199 (default: 199 = center)
    
    // Calibration Values
    int16_t offset_mm;                // -128 to +127 mm (default: 0)
    uint16_t xtalk_cps;               // 0-65535 cps (default: 0)
    
    // Threshold Configuration
    uint16_t signal_threshold_kcps;   // 0-65535 kcps (default: 1024)
    uint16_t sigma_threshold_mm;      // 0-65535 mm (default: 15)
    
    // Distance Threshold Detection (for interrupts)
    uint16_t threshold_low_mm;        // 0-4000 mm (default: 0 = disabled)
    uint16_t threshold_high_mm;       // 0-4000 mm (default: 0 = disabled)
    uint8_t threshold_window;         // 0=below, 1=above, 2=out, 3=in (default: 0)
    
    // Interrupt Configuration
    uint8_t interrupt_polarity;        // 0=active low, 1=active high (default: 1)
    
    // I2C Configuration (for multiple sensors)
    uint8_t i2c_address;              // 0x29-0x7F (default: 0x29)
} vl53l1x_config_t;

/**
 * @brief Load configuration from NVS storage
 * 
 * @param config Pointer to configuration structure to populate
 * @return true on success, false if no saved config exists or error occurred
 */
bool vl53l1x_config_load(vl53l1x_config_t *config);

/**
 * @brief Save configuration to NVS storage
 * 
 * @param config Pointer to configuration structure to save
 * @return true on success, false on error
 */
bool vl53l1x_config_save(const vl53l1x_config_t *config);

/**
 * @brief Initialize config structure with default values
 * 
 * @param config Pointer to configuration structure to initialize
 */
void vl53l1x_config_get_defaults(vl53l1x_config_t *config);

/**
 * @brief Validate configuration values are within valid ranges
 * 
 * @param config Pointer to configuration structure to validate
 * @return true if valid, false if any value is out of range
 */
bool vl53l1x_config_validate(const vl53l1x_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // VL53L1X_CONFIG_H

