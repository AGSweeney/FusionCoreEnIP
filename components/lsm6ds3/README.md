# LSM6DS3 ESP-IDF Driver

ESP-IDF driver for the **LSM6DS3 6-axis accelerometer and gyroscope** (IMU - Inertial Measurement Unit).

## Overview

The LSM6DS3 is a high-performance 6-axis motion sensor that combines a 3-axis accelerometer and 3-axis gyroscope on a single chip. This driver provides a complete interface for ESP-IDF projects, supporting sensor fusion algorithms, calibration, and real-time orientation tracking.

**Key Features:**
- 3-axis accelerometer (2g, 4g, 8g, 16g ranges)
- 3-axis gyroscope (125, 250, 500, 1000, 2000 DPS ranges)
- I2C and SPI communication interfaces
- Sensor fusion algorithms (Complementary Filter, Madgwick Filter)
- Roll, Pitch, and Ground Angle calculations
- Hardware and software offset compensation
- NVS calibration storage
- Block data update mode
- Real-time orientation tracking

## Product Information

- **Chip**: LSM6DS3
- **Manufacturer**: STMicroelectronics
- **Type**: 6-DOF (6 Degrees of Freedom) IMU
- **Interface**: I2C (default address 0x6A or 0x6B) or SPI

## Specifications

| Parameter | Value |
|-----------|-------|
| Accelerometer Full-Scale Range | ±2g, ±4g, ±8g, ±16g |
| Accelerometer Output Data Rate | 12.5 Hz to 6.66 kHz |
| Gyroscope Full-Scale Range | ±125, ±250, ±500, ±1000, ±2000 DPS |
| Gyroscope Output Data Rate | 12.5 Hz to 1.66 kHz |
| Operating Voltage | 1.71V to 3.6V |
| I2C Interface | Standard/Fast mode (up to 400 kHz) |
| SPI Interface | Up to 10 MHz |
| Temperature Range | -40°C to +85°C |

## Hardware Connection

### I2C Interface

The L2C interface uses standard I2C pins:

| Pin | Function | ESP32 Connection |
|-----|----------|------------------|
| VCC | Power (3.3V) | 3.3V |
| GND | Ground | GND |
| SDA | I2C Data | GPIO (configured in I2C bus) |
| SCL | I2C Clock | GPIO (configured in I2C bus) |

**I2C Addresses:**
- **0x6A**: Default address (SDO/SA0 pin low)
- **0x6B**: Alternative address (SDO/SA0 pin high)

### SPI Interface

The SPI interface uses standard SPI pins (MOSI, MISO, SCLK, CS).

## Installation

### Using ESP-IDF Component Manager

Add this component to your project's `idf_component.yml`:

```yaml
dependencies:
  lsm6ds3:
    path: components/lsm6ds3
```

### Manual Installation

1. Copy the `lsm6ds3` component directory to your project's `components` folder
2. Add `lsm6ds3` to your component's `REQUIRES` in `CMakeLists.txt`:

```cmake
idf_component_register(
    ...
    REQUIRES lsm6ds3
)
```

## Quick Start

### Basic Initialization

```c
#include "lsm6ds3.h"
#include "i2c_bus_manager.h"

// Get I2C bus handle
i2c_master_bus_handle_t i2c_bus = i2c_bus_manager_get_primary_bus();

// Configure LSM6DS3
lsm6ds3_config_t config = {
    .interface = LSM6DS3_INTERFACE_I2C,
    .bus.i2c.bus_handle = i2c_bus,
    .bus.i2c.address = 0x6A  // or 0x6B
};

// Initialize device
lsm6ds3_handle_t handle;
esp_err_t ret = lsm6ds3_init(&handle, &config);
if (ret != ESP_OK) {
    ESP_LOGE("LSM6DS3", "Initialization failed");
    return;
}

// Configure sensor settings
lsm6ds3_set_accel_odr(&handle, LSM6DS3_XL_ODR_104Hz);
lsm6ds3_set_accel_full_scale(&handle, LSM6DS3_XL_FS_4g);
lsm6ds3_set_gyro_odr(&handle, LSM6DS3_G_ODR_104Hz);
lsm6ds3_set_gyro_full_scale(&handle, LSM6DS3_G_FS_500dps);
```

### Reading Sensor Data

```c
// Read accelerometer data (in mg)
float accel_mg[3];
lsm6ds3_read_accel(&handle, accel_mg);
ESP_LOGI("LSM6DS3", "Accel: X=%.2f mg, Y=%.2f mg, Z=%.2f mg", 
         accel_mg[0], accel_mg[1], accel_mg[2]);

// Read gyroscope data (in mdps)
float gyro_mdps[3];
lsm6ds3_read_gyro(&handle, gyro_mdps);
ESP_LOGI("LSM6DS3", "Gyro: X=%.2f mdps, Y=%.2f mdps, Z=%.2f mdps", 
         gyro_mdps[0], gyro_mdps[1], gyro_mdps[2]);
```

## Sensor Fusion

The driver includes sensor fusion algorithms to calculate orientation angles from accelerometer and gyroscope data.

### Complementary Filter

The complementary filter combines accelerometer and gyroscope data to provide stable orientation tracking:

```c
#include "lsm6ds3_fusion.h"

// Initialize complementary filter
lsm6ds3_complementary_filter_t filter;
lsm6ds3_complementary_init(&filter, 0.96f, 50.0f);  // alpha=0.96, 50Hz

// Update filter with sensor data
float accel_g[3] = {accel_mg[0]/1000.0f, accel_mg[1]/1000.0f, accel_mg[2]/1000.0f};
float gyro_dps[3] = {gyro_mdps[0]/1000.0f, gyro_mdps[1]/1000.0f, gyro_mdps[2]/1000.0f};
float dt = 0.02f;  // 20ms = 50Hz
lsm6ds3_complementary_update(&filter, accel_g, gyro_dps, dt);

// Get orientation angles
float roll, pitch;
lsm6ds3_complementary_get_angles(&filter, &roll, &pitch);
ESP_LOGI("LSM6DS3", "Roll: %.2f°, Pitch: %.2f°", roll, pitch);
```

### Madgwick Filter

The Madgwick filter provides quaternion-based orientation estimation:

```c
// Initialize Madgwick filter
lsm6ds3_madgwick_filter_t madgwick;
lsm6ds3_madgwick_init(&madgwick, 0.1f, 50.0f);  // beta=0.1, 50Hz

// Update filter
lsm6ds3_madgwick_update(&madgwick, accel_g, gyro_dps, dt);

// Get Euler angles
lsm6ds3_euler_angles_t angles;
lsm6ds3_madgwick_get_euler(&madgwick, &angles);
ESP_LOGI("LSM6DS3", "Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
         angles.roll, angles.pitch, angles.yaw);
```

### Ground Angle Calculation

Calculate the angle from vertical using roll and pitch:

```c
float ground_angle = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
ESP_LOGI("LSM6DS3", "Ground Angle: %.2f°", ground_angle);
```

## Calibration

The LSM6DS3 supports both accelerometer and gyroscope calibration to improve accuracy.

### Accelerometer Calibration

```c
// Calibrate accelerometer (device should be stationary)
esp_err_t ret = lsm6ds3_calibrate_accel(&handle, 100, 10);
if (ret == ESP_OK) {
    ESP_LOGI("LSM6DS3", "Accelerometer calibrated");
    
    // Save calibration to NVS
    lsm6ds3_save_calibration_to_nvs(&handle, "lsm6ds3");
}
```

### Gyroscope Calibration

```c
// Calibrate gyroscope (device must be completely still)
esp_err_t ret = lsm6ds3_calibrate_gyro(&handle, 100, 10);
if (ret == ESP_OK) {
    ESP_LOGI("LSM6DS3", "Gyroscope calibrated");
    
    // Save calibration to NVS
    lsm6ds3_save_calibration_to_nvs(&handle, "lsm6ds3");
}
```

### Loading Calibration

```c
// Load calibration from NVS
lsm6ds3_load_calibration_from_nvs(&handle, "lsm6ds3");
```

### Manual Offset Setting

```c
// Set accelerometer offset manually
float accel_offset_mg[3] = {-10.5f, 5.2f, -2.1f};
lsm6ds3_set_accel_offset(&handle, accel_offset_mg);

// Set gyroscope offset manually
float gyro_offset_mdps[3] = {1.2f, -0.8f, 0.5f};
lsm6ds3_set_gyro_offset(&handle, gyro_offset_mdps);
```

## Angle Zero Offsets

Set reference zero points for roll, pitch, and yaw angles:

```c
#include "lsm6ds3_fusion.h"

lsm6ds3_angle_zero_t zero_ref;

// Set zero offsets
lsm6ds3_set_angle_zero(&zero_ref, 0.0f, 0.0f, 0.0f);

// Or set individually
lsm6ds3_set_roll_zero(&zero_ref, 0.0f);
lsm6ds3_set_pitch_zero(&zero_ref, 0.0f);
lsm6ds3_set_yaw_zero(&zero_ref, 0.0f);

// Apply zero offsets to angles
float roll = 45.0f, pitch = 30.0f, yaw = 0.0f;
lsm6ds3_apply_angle_zero(&zero_ref, &roll, &pitch, &yaw);

// Save to NVS
lsm6ds3_save_angle_zero_to_nvs(&zero_ref, "lsm6ds3");

// Load from NVS
lsm6ds3_load_angle_zero_from_nvs(&zero_ref, "lsm6ds3");
```

## Configuration

### Accelerometer Settings

```c
// Set output data rate
lsm6ds3_set_accel_odr(&handle, LSM6DS3_XL_ODR_104Hz);

// Set full-scale range
lsm6ds3_set_accel_full_scale(&handle, LSM6DS3_XL_FS_4g);
```

**Available ODR values:**
- `LSM6DS3_XL_ODR_12_5Hz` - 12.5 Hz
- `LSM6DS3_XL_ODR_26Hz` - 26 Hz
- `LSM6DS3_XL_ODR_52Hz` - 52 Hz
- `LSM6DS3_XL_ODR_104Hz` - 104 Hz (recommended)
- `LSM6DS3_XL_ODR_208Hz` - 208 Hz
- `LSM6DS3_XL_ODR_416Hz` - 416 Hz
- `LSM6DS3_XL_ODR_833Hz` - 833 Hz
- `LSM6DS3_XL_ODR_1_66kHz` - 1.66 kHz
- `LSM6DS3_XL_ODR_3_33kHz` - 3.33 kHz
- `LSM6DS3_XL_ODR_6_66kHz` - 6.66 kHz

**Available Full-Scale ranges:**
- `LSM6DS3_XL_FS_2g` - ±2g
- `LSM6DS3_XL_FS_4g` - ±4g (recommended)
- `LSM6DS3_XL_FS_8g` - ±8g
- `LSM6DS3_XL_FS_16g` - ±16g

### Gyroscope Settings

```c
// Set output data rate
lsm6ds3_set_gyro_odr(&handle, LSM6DS3_G_ODR_104Hz);

// Set full-scale range
lsm6ds3_set_gyro_full_scale(&handle, LSM6DS3_G_FS_500dps);
```

**Available ODR values:**
- `LSM6DS3_G_ODR_12_5Hz` - 12.5 Hz
- `LSM6DS3_G_ODR_26Hz` - 26 Hz
- `LSM6DS3_G_ODR_52Hz` - 52 Hz
- `LSM6DS3_G_ODR_104Hz` - 104 Hz (recommended)
- `LSM6DS3_G_ODR_208Hz` - 208 Hz
- `LSM6DS3_G_ODR_416Hz` - 416 Hz
- `LSM6DS3_G_ODR_833Hz` - 833 Hz
- `LSM6DS3_G_ODR_1_66kHz` - 1.66 kHz

**Available Full-Scale ranges:**
- `LSM6DS3_G_FS_125dps` - ±125 DPS
- `LSM6DS3_G_FS_250dps` - ±250 DPS
- `LSM6DS3_G_FS_500dps` - ±500 DPS (recommended)
- `LSM6DS3_G_FS_1000dps` - ±1000 DPS
- `LSM6DS3_G_FS_2000dps` - ±2000 DPS

## Assembly Data Format

The LSM6DS3 manager writes orientation data to EtherNet/IP Input Assembly 100 at bytes 16-23:

| Byte Range | Field | Type | Description |
|------------|-------|------|-------------|
| 16-17 | Roll | int16_t | Roll angle scaled by 100 (-180.00° to +180.00°) |
| 18-19 | Pitch | int16_t | Pitch angle scaled by 100 (-180.00° to +180.00°) |
| 20-21 | Ground Angle | int16_t | Ground angle scaled by 100 (-180.00° to +180.00°) |
| 22-23 | Reserved | - | Reserved for future expansion |

**Data Format:**
- All values are stored as **little-endian** signed 16-bit integers
- Angles are scaled by 100 (fixed-point format)
- Example: `45.66°` → `4566` (stored as `0x11D6`)

See [ASSEMBLY_DATA.md](ASSEMBLY_DATA.md) for complete assembly data format documentation.

## API Reference

### Initialization

- `lsm6ds3_init()` - Initialize LSM6DS3 device
- `lsm6ds3_deinit()` - Deinitialize LSM6DS3 device
- `lsm6ds3_reset()` - Software reset
- `lsm6ds3_get_device_id()` - Read device ID

### Configuration

- `lsm6ds3_set_accel_odr()` - Set accelerometer output data rate
- `lsm6ds3_set_accel_full_scale()` - Set accelerometer full-scale range
- `lsm6ds3_set_gyro_odr()` - Set gyroscope output data rate
- `lsm6ds3_set_gyro_full_scale()` - Set gyroscope full-scale range
- `lsm6ds3_enable_block_data_update()` - Enable/disable block data update

### Data Reading

- `lsm6ds3_read_accel()` - Read accelerometer data (mg)
- `lsm6ds3_read_gyro()` - Read gyroscope data (mdps)

### Calibration

- `lsm6ds3_calibrate_accel()` - Calibrate accelerometer
- `lsm6ds3_calibrate_gyro()` - Calibrate gyroscope
- `lsm6ds3_set_accel_offset()` - Set accelerometer software offset
- `lsm6ds3_set_gyro_offset()` - Set gyroscope software offset
- `lsm6ds3_get_accel_offset()` - Get accelerometer offset
- `lsm6ds3_get_gyro_offset()` - Get gyroscope offset
- `lsm6ds3_clear_accel_calibration()` - Clear accelerometer calibration
- `lsm6ds3_clear_gyro_calibration()` - Clear gyroscope calibration
- `lsm6ds3_save_calibration_to_nvs()` - Save calibration to NVS
- `lsm6ds3_load_calibration_from_nvs()` - Load calibration from NVS

### Sensor Fusion

- `lsm6ds3_complementary_init()` - Initialize complementary filter
- `lsm6ds3_complementary_update()` - Update complementary filter
- `lsm6ds3_complementary_get_angles()` - Get roll/pitch from complementary filter
- `lsm6ds3_madgwick_init()` - Initialize Madgwick filter
- `lsm6ds3_madgwick_update()` - Update Madgwick filter
- `lsm6ds3_madgwick_get_euler()` - Get Euler angles from Madgwick filter
- `lsm6ds3_calculate_angle_from_vertical()` - Calculate ground angle

### Angle Zero Offsets

- `lsm6ds3_set_angle_zero()` - Set all angle zero offsets
- `lsm6ds3_set_roll_zero()` - Set roll zero offset
- `lsm6ds3_set_pitch_zero()` - Set pitch zero offset
- `lsm6ds3_set_yaw_zero()` - Set yaw zero offset
- `lsm6ds3_apply_angle_zero()` - Apply zero offsets to angles
- `lsm6ds3_clear_angle_zero()` - Clear all zero offsets
- `lsm6ds3_save_angle_zero_to_nvs()` - Save zero offsets to NVS
- `lsm6ds3_load_angle_zero_from_nvs()` - Load zero offsets from NVS

## Manager Component

The `lsm6ds3_manager` component provides a high-level interface for the LSM6DS3 sensor, including automatic initialization, sensor fusion, and assembly data updates.

### Manager API

- `lsm6ds3_manager_init()` - Initialize LSM6DS3 manager
- `lsm6ds3_manager_is_initialized()` - Check if manager is initialized
- `lsm6ds3_manager_calibrate_gyro()` - Calibrate gyroscope
- `lsm6ds3_manager_get_calibration_status()` - Get calibration status
- `lsm6ds3_manager_get_gyro_offsets()` - Get gyroscope offsets
- `lsm6ds3_manager_set_angle_zero()` - Set angle zero offsets
- `lsm6ds3_manager_clear_angle_zero()` - Clear angle zero offsets
- `lsm6ds3_manager_get_angle_zero()` - Get angle zero offsets
- `lsm6ds3_manager_save_calibration()` - Save calibration to NVS
- `lsm6ds3_manager_load_calibration()` - Load calibration from NVS

## Examples

### Complete Example

```c
#include "lsm6ds3.h"
#include "lsm6ds3_fusion.h"
#include "i2c_bus_manager.h"
#include "esp_log.h"

void lsm6ds3_example(void)
{
    // Initialize I2C bus
    i2c_master_bus_handle_t i2c_bus = i2c_bus_manager_get_primary_bus();
    
    // Configure LSM6DS3
    lsm6ds3_config_t config = {
        .interface = LSM6DS3_INTERFACE_I2C,
        .bus.i2c.bus_handle = i2c_bus,
        .bus.i2c.address = 0x6A
    };
    
    // Initialize device
    lsm6ds3_handle_t handle;
    if (lsm6ds3_init(&handle, &config) != ESP_OK) {
        ESP_LOGE("LSM6DS3", "Initialization failed");
        return;
    }
    
    // Configure sensors
    lsm6ds3_set_accel_odr(&handle, LSM6DS3_XL_ODR_104Hz);
    lsm6ds3_set_accel_full_scale(&handle, LSM6DS3_XL_FS_4g);
    lsm6ds3_set_gyro_odr(&handle, LSM6DS3_G_ODR_104Hz);
    lsm6ds3_set_gyro_full_scale(&handle, LSM6DS3_G_FS_500dps);
    
    // Load calibration
    lsm6ds3_load_calibration_from_nvs(&handle, "lsm6ds3");
    
    // Initialize sensor fusion
    lsm6ds3_complementary_filter_t filter;
    lsm6ds3_complementary_init(&filter, 0.96f, 50.0f);
    
    // Main loop
    while (1) {
        float accel_mg[3], gyro_mdps[3];
        
        // Read sensor data
        lsm6ds3_read_accel(&handle, accel_mg);
        lsm6ds3_read_gyro(&handle, gyro_mdps);
        
        // Convert to g and dps
        float accel_g[3] = {accel_mg[0]/1000.0f, accel_mg[1]/1000.0f, accel_mg[2]/1000.0f};
        float gyro_dps[3] = {gyro_mdps[0]/1000.0f, gyro_mdps[1]/1000.0f, gyro_mdps[2]/1000.0f};
        
        // Update filter
        lsm6ds3_complementary_update(&filter, accel_g, gyro_dps, 0.02f);
        
        // Get angles
        float roll, pitch;
        lsm6ds3_complementary_get_angles(&filter, &roll, &pitch);
        
        // Calculate ground angle
        float ground_angle = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
        
        ESP_LOGI("LSM6DS3", "Roll: %.2f°, Pitch: %.2f°, Ground: %.2f°", 
                 roll, pitch, ground_angle);
        
        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz
    }
}
```

## License

MIT License

Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>

This driver incorporates code from STMicroelectronics:
- Register driver files (`driver/lsm6ds3_reg.c` and `driver/lsm6ds3_reg.h`)
- Copyright (c) 2018-2025 STMicroelectronics
- Licensed under BSD 3-Clause License

The full BSD 3-Clause license text is included in the register driver files.

## Related Documentation

- [Assembly Data Format](ASSEMBLY_DATA.md) - Complete assembly data format documentation
- [Driver README](driver/README.md) - Low-level register driver documentation
- [LSM6DS3 Datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3.pdf) - Official STMicroelectronics datasheet

