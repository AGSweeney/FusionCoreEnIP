# VL53L1x_ULD Library for ESP-IDF

ESP-IDF wrapper component for the STMicroelectronics VL53L1x Time-of-Flight (ToF) distance sensor using the VL53L1X User Level Driver (ULD).

## Overview

The VL53L1x is a long-range ToF ranging sensor capable of measuring distances up to 4 meters. This library provides a high-level ESP-IDF wrapper around the STMicroelectronics VL53L1X ULD, making it easy to integrate the sensor into ESP32 projects.

## Features

### Core Capabilities

- **Distance Measurement**: Up to 4 meters in Long mode, 1.3 meters in Short mode
- **High Update Rate**: Up to 30 Hz (Long mode) or 50 Hz (Short mode)
- **I2C Interface**: Standard I2C communication (default address: 0x29)
- **Multiple Distance Modes**: Short and Long range modes
- **Configurable Timing Budget**: 15ms to 500ms
- **ROI Configuration**: Configurable Region of Interest (ROI) with size and center control
- **Calibration Support**: Offset, crosstalk (xtalk), and temperature calibration
- **Status Information**: Comprehensive range status, signal rate, ambient light, SPAD count
- **Threshold Detection**: Configurable distance thresholds with window detection modes
- **Interrupt Support**: Configurable interrupt polarity and threshold-based interrupts
- **Multiple Sensor Support**: Dynamic I2C address change for multiple sensors on same bus
- **ESP-IDF Integration**: Native ESP-IDF I2C master driver support

## Hardware Requirements

- ESP32 series microcontroller (ESP32, ESP32-S2, ESP32-S3, ESP32-P4, etc.)
- VL53L1x sensor module
- I2C connections (default pins: SDA **GPIO 7**, SCL **GPIO 8**; configurable via Kconfig)
  - VCC (3.3V)
  - GND
  - Optional: XSHUT (shutdown pin)
  - Optional: GPIO1 (interrupt pin)

## Installation

This component is designed to be used as an ESP-IDF component. Add it to your project's `components` directory or use it as a managed component via `idf_component.yml`.

### Adding to CMakeLists.txt

```cmake
idf_component_register(
    ...
    PRIV_REQUIRES
        vl53l1x_uld
    ...
)
```

## Quick Start

### Basic Usage

```c
#include "vl53l1x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Initialize I2C handle
vl53l1x_i2c_handle_t vl53l1x_i2c_handle = VL53L1X_I2C_INIT;
vl53l1x_i2c_handle.scl_gpio = GPIO_NUM_8;  // Default SCL GPIO (configurable)
vl53l1x_i2c_handle.sda_gpio = GPIO_NUM_7;  // Default SDA GPIO (configurable)

vl53l1x_handle_t vl53l1x_handle = VL53L1X_INIT;
vl53l1x_handle.i2c_handle = &vl53l1x_i2c_handle;

// Initialize sensor
if (!vl53l1x_init(&vl53l1x_handle)) {
    ESP_LOGE("APP", "VL53L1X initialization failed");
    return;
}

// Add device
vl53l1x_device_handle_t vl53l1x_device = VL53L1X_DEVICE_INIT;
vl53l1x_device.vl53l1x_handle = &vl53l1x_handle;
vl53l1x_device.i2c_address = 0x29;  // Default address

if (!vl53l1x_add_device(&vl53l1x_device)) {
    ESP_LOGE("APP", "Failed to add VL53L1X device");
    return;
}

// Read distance
while (1) {
    uint16_t distance_mm = vl53l1x_get_mm(&vl53l1x_device);
    ESP_LOGI("APP", "Distance: %d mm", distance_mm);
    vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz update rate
}
```

## API Reference

### Initialization Functions

#### `vl53l1x_init()`
Initialize the VL53L1x handle and I2C bus.

```c
bool vl53l1x_init(vl53l1x_handle_t *vl53l1x_handle);
```

**Parameters:**
- `vl53l1x_handle`: Pointer to VL53L1x handle structure

**Returns:**
- `true` on success, `false` on failure

#### `vl53l1x_add_device()`
Add a VL53L1x device to the handle. Configures the sensor and starts ranging.

```c
bool vl53l1x_add_device(vl53l1x_device_handle_t *device);
```

**Parameters:**
- `device`: Pointer to device handle structure (must have `vl53l1x_handle` and `i2c_address` set)

**Returns:**
- `true` on success, `false` on failure

**Note:** The device is automatically configured in LONG distance mode.

### Measurement Functions

#### `vl53l1x_get_mm()`
Get distance measurement in millimeters.

```c
uint16_t vl53l1x_get_mm(const vl53l1x_device_handle_t *device);
```

**Parameters:**
- `device`: Pointer to device handle

**Returns:**
- Distance in millimeters (0 = no target or out of range)

**Note:** This function automatically clears the interrupt after reading.

### Calibration Functions

#### `vl53l1x_calibrate_offset()`
Calibrate the sensor offset at a known distance.

```c
bool vl53l1x_calibrate_offset(const vl53l1x_device_handle_t *device, 
                               uint16_t target_distance_mm, 
                               int16_t *offset);
```

**Parameters:**
- `device`: Pointer to device handle
- `target_distance_mm`: Known distance to target in mm
- `offset`: Pointer to store calculated offset value

**Returns:**
- `true` on success, `false` on failure

**Note:** Stop ranging before calibration, restart after.

#### `vl53l1x_calibrate_xtalk()`
Calibrate crosstalk compensation.

```c
bool vl53l1x_calibrate_xtalk(const vl53l1x_device_handle_t *device,
                              uint16_t target_distance_mm,
                              uint16_t *xtalk);
```

**Parameters:**
- `device`: Pointer to device handle
- `target_distance_mm`: Distance to target (typically 600mm at inflection point)
- `xtalk`: Pointer to store calculated xtalk value

**Returns:**
- `true` on success, `false` on failure

### ROI Configuration Functions

#### `vl53l1x_set_roi()`
Set the Region of Interest (ROI) size.

```c
bool vl53l1x_set_roi(const vl53l1x_device_handle_t *device,
                     uint16_t x_size, 
                     uint16_t y_size);
```

**Parameters:**
- `device`: Pointer to device handle
- `x_size`: ROI width (minimum 4)
- `y_size`: ROI height (minimum 4)

**Returns:**
- `true` on success, `false` on failure

#### `vl53l1x_set_roi_center()`
Set the ROI center SPAD.

```c
bool vl53l1x_set_roi_center(const vl53l1x_device_handle_t *device,
                             uint8_t center_spad);
```

**Parameters:**
- `device`: Pointer to device handle
- `center_spad`: Center SPAD index (0-255)

**Returns:**
- `true` on success, `false` on failure

#### `vl53l1x_get_roi_center()`
Get the current ROI center SPAD.

```c
bool vl53l1x_get_roi_center(const vl53l1x_device_handle_t *device,
                             uint8_t *center_spad);
```

**Parameters:**
- `device`: Pointer to device handle
- `center_spad`: Pointer to store center SPAD value

**Returns:**
- `true` on success, `false` on failure

### Diagnostic Functions

#### `vl53l1x_log_sensor_id()`
Log the sensor ID (should be 0xEEAC).

```c
void vl53l1x_log_sensor_id(const vl53l1x_device_handle_t *device);
```

#### `vl53l1x_log_ambient_light()`
Log the ambient light level.

```c
void vl53l1x_log_ambient_light(const vl53l1x_device_handle_t *device);
```

#### `vl53l1x_log_signal_rate()`
Log the signal rate.

```c
void vl53l1x_log_signal_rate(const vl53l1x_device_handle_t *device);
```

## Sensor Data Outputs

The VL53L1x sensor provides comprehensive measurement data:

### Primary Data

1. **Distance** (`uint16_t`)
   - Units: millimeters (mm)
   - Range: 0-4000mm (Long mode) or 0-1300mm (Short mode)
   - Value of 0 typically indicates no target detected or out of range

2. **Range Status** (`uint8_t`)
   - Indicates measurement validity and error conditions
   - Status codes:
     - `0` - No error (valid measurement)
     - `1` - Sigma failed (measurement uncertainty too high)
     - `2` - Signal failed (signal too weak)
     - `3` - Target out of range
     - `4` - Signal failed
     - `5` - Range valid but wrapped
     - `6` - Target out of range
     - `7` - Wrap-around (target beyond max range)
     - `9` - Range valid but wrapped
     - `10` - Target out of range
     - `11-13` - Range valid but wrapped
     - `255` - Invalid/unknown status

3. **Ambient Light Level** (`uint16_t`)
   - Units: kcps (kilo counts per second)
   - Measures background ambient light interference
   - Useful for determining measurement quality

4. **Signal Rate** (`uint16_t`)
   - Units: kcps (kilo counts per second)
   - Total returned signal strength
   - Higher values indicate stronger target reflection

5. **Signal Per SPAD** (`uint16_t`)
   - Units: kcps/SPAD (kilo counts per second per SPAD)
   - Normalized signal strength per enabled SPAD
   - Useful for comparing measurements across different ROI configurations

6. **Number of Enabled SPADs** (`uint16_t`)
   - Count of active SPAD (Single Photon Avalanche Diode) elements
   - Depends on ROI configuration
   - More SPADs generally provide better accuracy

7. **Sensor ID** (`uint16_t`)
   - Should read `0xEEAC` for VL53L1X
   - Used for device identification and validation

### Complete Result Structure

All measurement data can be retrieved in a single read operation:

```c
VL53L1X_ERROR VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t *pResult);
```

**Result Structure:**
```c
typedef struct {
    uint8_t Status;      // Range status (see above)
    uint16_t Distance;   // Distance in mm
    uint16_t Ambient;    // Ambient light level (kcps)
    uint16_t SigPerSPAD; // Signal per SPAD (kcps/SPAD)
    uint16_t NumSPADs;   // Number of enabled SPADs
} VL53L1X_Result_t;
```

## Configuration Capabilities

The VL53L1x library provides extensive configuration options:

### 1. Distance Mode Configuration

```c
VL53L1X_ERROR VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DistanceMode);
VL53L1X_ERROR VL53L1X_GetDistanceMode(uint16_t dev, uint16_t *pDistanceMode);
```

**Distance Modes:**
- `1` = SHORT (up to 1.3m, better ambient immunity, faster updates)
- `2` = LONG (up to 4m, default, better for longer ranges)

**When to use:**
- **SHORT mode**: Indoor applications, close-range detection (<1.3m), high ambient light
- **LONG mode**: Outdoor applications, longer ranges (1-4m), lower ambient light

### 2. Timing Budget Configuration

```c
VL53L1X_ERROR VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t TimingBudgetInMs);
VL53L1X_ERROR VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t *pTimingBudgetInMs);
```

**Available Timing Budgets:** 15, 20, 33, 50, 100 (default), 200, 500 ms

**Trade-offs:**
- **Shorter timing budget** (15-33ms): Faster updates, lower accuracy, higher power consumption
- **Longer timing budget** (100-500ms): Slower updates, higher accuracy, better noise immunity

**Note:** Inter-measurement period must be ≥ timing budget.

### 3. Inter-Measurement Period

```c
VL53L1X_ERROR VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t InterMeasurementInMs);
VL53L1X_ERROR VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t *pIM);
```

Controls the delay between consecutive measurements in continuous mode.
- Must be ≥ timing budget
- Default: 100ms
- Lower values = higher update rate (up to sensor limits)

### 4. Region of Interest (ROI) Configuration

**ROI Size:**
```c
VL53L1X_ERROR VL53L1X_SetROI(uint16_t dev, uint16_t X, uint16_t Y);
VL53L1X_ERROR VL53L1X_GetROI_XY(uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y);
```
- X, Y: ROI width and height (minimum 4, maximum 16)
- Smaller ROI = faster measurements, narrower field of view
- Larger ROI = slower measurements, wider field of view

**ROI Center:**
```c
VL53L1X_ERROR VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter);
VL53L1X_ERROR VL53L1X_GetROICenter(uint16_t dev, uint8_t *ROICenter);
```
- Center SPAD index: 0-199
- Allows fine-tuning the measurement center point
- Default: 199 (center of SPAD array)

### 5. Offset Calibration

```c
VL53L1X_ERROR VL53L1X_SetOffset(uint16_t dev, int16_t OffsetValue);
VL53L1X_ERROR VL53L1X_GetOffset(uint16_t dev, int16_t *Offset);
```

**Purpose:** Compensates for systematic distance measurement errors
- Units: millimeters (mm)
- Can be positive or negative
- Typically calibrated at 100mm distance with grey 17% target
- Should be recalibrated if sensor is replaced or environment changes significantly

### 6. Crosstalk (Xtalk) Calibration

```c
VL53L1X_ERROR VL53L1X_SetXtalk(uint16_t dev, uint16_t XtalkValue);
VL53L1X_ERROR VL53L1X_GetXtalk(uint16_t dev, uint16_t *Xtalk);
```

**Purpose:** Compensates for photons reflected from cover glass
- Units: cps (counts per second)
- Typically calibrated at inflection point (~600mm) with grey 17% target
- Important for accurate measurements at medium distances
- Should be recalibrated if cover glass is changed

### 7. Temperature Calibration

```c
VL53L1X_ERROR VL53L1X_StartTemperatureUpdate(uint16_t dev);
```

**Purpose:** Updates internal temperature compensation
- Should be called when temperature changes by >8°C
- Automatically called during initialization
- Improves measurement accuracy over temperature variations

### 8. Signal Threshold Configuration

```c
VL53L1X_ERROR VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t signal);
VL53L1X_ERROR VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t *signal);
```

**Purpose:** Sets minimum signal strength threshold for valid measurements
- Units: kcps (kilo counts per second)
- Default: 1024 kcps
- Lower values = more sensitive, may include noise
- Higher values = less sensitive, may miss weak targets

### 9. Sigma Threshold Configuration

```c
VL53L1X_ERROR VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t sigma);
VL53L1X_ERROR VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t *signal);
```

**Purpose:** Sets maximum measurement uncertainty threshold
- Units: millimeters (mm)
- Default: 15 mm
- Lower values = stricter quality requirements
- Higher values = more lenient, may accept noisier measurements

### 10. Distance Threshold Detection

```c
VL53L1X_ERROR VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t ThreshLow,
                                           uint16_t ThreshHigh, uint8_t Window,
                                           uint8_t IntOnNoTarget);
VL53L1X_ERROR VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t *window);
VL53L1X_ERROR VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t *low);
VL53L1X_ERROR VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t *high);
```

**Purpose:** Configures distance-based interrupt generation

**Window Modes:**
- `0` = Below threshold (interrupt when distance < ThreshLow)
- `1` = Above threshold (interrupt when distance > ThreshHigh)
- `2` = Out of window (interrupt when distance < ThreshLow OR distance > ThreshHigh)
- `3` = In window (interrupt when ThreshLow ≤ distance ≤ ThreshHigh)

**Example:**
```c
// Trigger interrupt when target is between 100mm and 300mm
VL53L1X_SetDistanceThreshold(dev, 100, 300, 3, 0);
```

### 11. Interrupt Configuration

```c
VL53L1X_ERROR VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t IntPol);
VL53L1X_ERROR VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t *pIntPol);
VL53L1X_ERROR VL53L1X_ClearInterrupt(uint16_t dev);
VL53L1X_ERROR VL53L1X_CheckForDataReady(uint16_t dev, uint8_t *isDataReady);
```

**Interrupt Polarity:**
- `1` = Active high (default)
- `0` = Active low

**Interrupt Events:**
- New sample ready (default)
- Distance threshold crossed (when configured)

### 12. I2C Address Configuration

```c
VL53L1X_ERROR VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address);
bool vl53l1x_update_device_address(vl53l1x_device_handle_t *device, uint8_t new_address);
```

**Purpose:** Change I2C address for multiple sensors on same bus
- Default address: `0x29`
- Allows up to 16 sensors on one I2C bus (with address changes)
- Use XSHUT pin to change address during initialization

### 13. Ranging Control

```c
VL53L1X_ERROR VL53L1X_StartRanging(uint16_t dev);
VL53L1X_ERROR VL53L1X_StopRanging(uint16_t dev);
```

**Purpose:** Start/stop continuous ranging mode
- Start ranging begins continuous measurements
- Stop ranging halts measurements (useful before calibration)

### 14. Boot State and Sensor ID

```c
VL53L1X_ERROR VL53L1X_BootState(uint16_t dev, uint8_t *state);
VL53L1X_ERROR VL53L1X_GetSensorId(uint16_t dev, uint16_t *id);
```

**Purpose:** Check sensor initialization status and verify device type
- Boot state: `0` = booted, `1` = not booted
- Sensor ID: Should be `0xEEAC` for VL53L1X

## Advanced API (VL53L1X_api.h)

For advanced configuration, you can use the low-level API functions directly. All functions above are available through the `VL53L1X_api.h` header. The wrapper functions in `vl53l1x.h` provide a simplified interface for common operations.

## EtherNet/IP Integration

When integrated with EtherNet/IP (OpENer), the sensor data is written to the input assembly buffer. The byte layout is as follows:

### Input Assembly Byte Layout

The sensor data is written to bytes 0-8 of the input assembly buffer (little-endian format):

| Byte(s) | Data Type | Description | Units |
|---------|-----------|-------------|-------|
| 0-1 | `uint16_t` | **Distance** | millimeters (mm) |
| 2 | `uint8_t` | **Status** | Range status code (0 = valid, see status codes above) |
| 3-4 | `uint16_t` | **Ambient** | Ambient light level | kcps (kilo counts per second) |
| 5-6 | `uint16_t` | **SigPerSPAD** | Signal per SPAD | kcps/SPAD |
| 7-8 | `uint16_t` | **NumSPADs** | Number of enabled SPADs | count |

### Example: Reading Sensor Data from Input Assembly

```c
// Assuming g_assembly_data064 is your input assembly buffer
uint16_t distance_mm = g_assembly_data064[0] | (g_assembly_data064[1] << 8);
uint8_t status = g_assembly_data064[2];
uint16_t ambient_kcps = g_assembly_data064[3] | (g_assembly_data064[4] << 8);
uint16_t sig_per_spad = g_assembly_data064[5] | (g_assembly_data064[6] << 8);
uint16_t num_spads = g_assembly_data064[7] | (g_assembly_data064[8] << 8);

// Check if measurement is valid
if (status == 0) {
    printf("Distance: %d mm (valid)\n", distance_mm);
    printf("Ambient: %d kcps\n", ambient_kcps);
    printf("Signal per SPAD: %d kcps/SPAD\n", sig_per_spad);
    printf("Number of SPADs: %d\n", num_spads);
} else {
    printf("Measurement invalid (status: %d)\n", status);
}
```

### Data Update Rate

The sensor data is updated at the configured rate (default: 10 Hz / 100ms). The input assembly buffer is continuously updated by the sensor task, independent of EtherNet/IP communication.

**Note:** Bytes 9-31 of the input assembly are available for other application data and are not overwritten by the sensor task.

## Hardware Configuration

### I2C Configuration

The I2C handle structure allows configuration of:
- `i2c_port`: I2C port number (default: `I2C_NUM_0`)
- `scl_gpio`: SCL GPIO pin number (default `GPIO_NUM_8`)
- `sda_gpio`: SDA GPIO pin number (default `GPIO_NUM_7`)
- `scl_speed_hz`: I2C clock speed (default: 400000 Hz)

### Device Configuration

The device handle structure includes:
- `i2c_address`: I2C address (default: `0x29`)
- `xshut_gpio`: XSHUT GPIO pin (optional, `GPIO_NUM_NC` if not used)
  - Used for hardware reset and address change
  - Pull low to reset sensor
- `interrupt_gpio`: Interrupt GPIO pin (optional, `GPIO_NUM_NC` if not used)
  - Used for interrupt-based data ready detection
  - Reduces CPU polling overhead
- `scl_speed_hz`: I2C clock speed for this device (default: 400000 Hz)

## Update Rates

The maximum update rate depends on the distance mode and timing budget:

- **Short Mode:**
  - Up to 50 Hz (20ms timing budget)
  - Up to 100 Hz for distances < 1m (15ms timing budget)

- **Long Mode:**
  - Up to 30 Hz (33ms timing budget)
  - Default: 10 Hz (100ms timing budget)

**Note:** The inter-measurement period must be set to match or exceed the timing budget.

## Calibration Procedure

### Offset Calibration

1. Stop ranging: `VL53L1X_StopRanging(device->dev)`
2. Place a target at a known distance (recommended: 100mm)
3. Wait for sensor to stabilize (100ms)
4. Call `vl53l1x_calibrate_offset()` with the known distance
5. Restart ranging: `VL53L1X_StartRanging(device->dev)`

### Crosstalk (Xtalk) Calibration

1. Stop ranging: `VL53L1X_StopRanging(device->dev)`
2. Place a target at the inflection point (typically 600mm)
3. Wait for sensor to stabilize (100ms)
4. Call `vl53l1x_calibrate_xtalk()` with the target distance
5. Restart ranging: `VL53L1X_StartRanging(device->dev)`

## Example Code

See `example.c` in this directory for a complete example including:
- Sensor initialization
- ROI configuration
- Calibration procedures
- Continuous measurement loop with status checking

## Troubleshooting

### Sensor Not Found

- Verify I2C wiring (SDA/SCL)
- Check I2C address (default: 0x29)
- Use `i2c_find_first_addr()` to scan for devices
- Ensure pull-up resistors are present (typically 2.2kΩ)

### Invalid Measurements

- Check range status using `VL53L1X_GetRangeStatus()`
- Verify target is within range (1.3m for Short, 4m for Long)
- Check ambient light levels using `vl53l1x_log_ambient_light()`
- Ensure timing budget is appropriate for update rate

### Initialization Failures

- Verify GPIO pins are correctly configured
- Check I2C bus initialization
- Ensure sensor is powered (3.3V)
- Check for I2C bus conflicts

## License

See `LICENSE` file and `vl53l1x_uld_esp_wrapper/core/LICENSE.txt` for license information.

## References

- [VL53L1X Datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)
- [VL53L1X User Manual](https://www.st.com/resource/en/user_manual/um2556-vl53l1x-ultralite-driver-user-manual-stmicroelectronics.pdf)
- [STMicroelectronics VL53L1X Product Page](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)
