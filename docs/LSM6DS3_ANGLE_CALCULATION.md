# LSM6DS3 Angle Calculation Implementation

## Overview

This document describes the complete implementation of angle calculation from the LSM6DS3 6-axis IMU sensor, including sensor fusion filtering and ground truth (angle from vertical) calculation.

## Architecture

The implementation uses a **Complementary Filter** to fuse accelerometer and gyroscope data, providing stable roll and pitch angles. The ground truth angle (angle from vertical) is then calculated from these fused angles.

### Components

1. **LSM6DS3 Driver** (`components/lsm6ds3/`)
   - Sensor initialization and data reading
   - Calibration support
   - Raw sensor data conversion

2. **Sensor Fusion** (`components/lsm6ds3/lsm6ds3_fusion.c`)
   - Complementary filter implementation
   - Madgwick filter (available but not used)
   - Angle from vertical calculation

3. **Main Application** (`main/main.c`)
   - Filter initialization
   - Real-time sensor data processing
   - Ground truth calculation

---

## Filter Implementation: Complementary Filter

### Configuration

The complementary filter is initialized with the following parameters:

```c
// From main/main.c:1962
lsm6ds3_complementary_init(&s_lsm6ds3_filter, 0.96f, 104.0f);
```

- **Alpha (α)**: `0.96` - Filter coefficient (0.0 to 1.0)
  - Higher values (closer to 1.0) trust gyroscope more
  - Lower values trust accelerometer more
  - `0.96` means 96% gyroscope, 4% accelerometer

- **Sample Rate**: `104 Hz` - Sensor update frequency

### Filter Algorithm

The complementary filter combines:
- **Gyroscope**: High-frequency, accurate for short-term changes, but drifts over time
- **Accelerometer**: Low-frequency, accurate for absolute orientation when stationary, but noisy during motion

**Update Equation** (from `lsm6ds3_fusion.c:236-273`):

```c
// Calculate accelerometer-based angles
float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

if (accel_norm > 0.0f) {
    // Normalize accelerometer vector
    float accel_x = accel[0] / accel_norm;
    float accel_y = accel[1] / accel_norm;
    float accel_z = accel[2] / accel_norm;
    
    // Calculate roll and pitch from accelerometer
    float accel_roll = atan2f(accel_y, accel_z) * RAD_TO_DEG;
    float accel_pitch = asinf(-accel_x) * RAD_TO_DEG;
    
    // Complementary filter fusion
    filter->roll = filter->alpha * (filter->roll + gyro[0] * dt) + (1.0f - filter->alpha) * accel_roll;
    filter->pitch = filter->alpha * (filter->pitch + gyro[1] * dt) + (1.0f - filter->alpha) * accel_pitch;
}
```

**Mathematical Formulation**:

```
roll_new = α × (roll_old + gyro_x × dt) + (1 - α) × accel_roll
pitch_new = α × (pitch_old + gyro_y × dt) + (1 - α) × accel_pitch
```

Where:
- `α = 0.96` (filter coefficient)
- `dt` = time delta between samples (1/sample_rate ≈ 0.0096 seconds)
- `gyro_x, gyro_y` = gyroscope rates in degrees/second
- `accel_roll, accel_pitch` = angles calculated from accelerometer

### Initialization

On first update, the filter initializes from accelerometer data only:

```c
// From lsm6ds3_fusion.c:257-260
if (!filter->initialized) {
    filter->roll = accel_roll;
    filter->pitch = accel_pitch;
    filter->initialized = true;
}
```

---

## Angle Calculation Steps

### Step 1: Read Raw Sensor Data

**Location**: `main/main.c:2230-2240`

```c
float accel_mg[3];  // Accelerometer in milli-g
float gyro_mdps[3]; // Gyroscope in milli-degrees per second

// Read accelerometer (converted to g)
lsm6ds3_read_accel(&s_lsm6ds3_handle, accel_mg);
float accel_g[3] = {
    accel_mg[0] / 1000.0f,  // Convert mg to g
    accel_mg[1] / 1000.0f,
    accel_mg[2] / 1000.0f
};

// Read gyroscope (converted to degrees/second)
lsm6ds3_read_gyro(&s_lsm6ds3_handle, gyro_mdps);
float gyro_dps[3] = {
    gyro_mdps[0] / 1000.0f,  // Convert mdps to dps
    gyro_mdps[1] / 1000.0f,
    gyro_mdps[2] / 1000.0f
};
```

**Data Units**:
- Accelerometer: milli-g (mg) → converted to g (gravity units)
- Gyroscope: milli-degrees per second (mdps) → converted to degrees/second (dps)

### Step 2: Update Complementary Filter

**Location**: `main/main.c:2243-2249`

```c
// Calculate time delta (dt) from period_ms (typically 20ms for 50Hz update rate)
// dt = period_ms / 1000.0f (e.g., 20ms = 0.02s for 50Hz update rate)
float dt = (float)period_ms / 1000.0f;

// Update filter with new sensor data
esp_err_t fusion_err = lsm6ds3_complementary_update(&s_lsm6ds3_filter, accel_g, gyro_dps, dt);

if (fusion_err == ESP_OK) {
    // Get fused roll and pitch angles
    lsm6ds3_complementary_get_angles(&s_lsm6ds3_filter, &roll, &pitch);
}
```

**Note**: The actual update rate is determined by `period_ms` (typically 20ms = 50Hz), not the sensor's 104Hz sample rate. The sensor is sampled at 104Hz, but the filter is updated at the task rate (50Hz).

**Filter Update Process** (from `lsm6ds3_fusion.c:236-273`):

1. **Normalize accelerometer vector**:
   ```c
   float accel_norm = sqrtf(accel[0]² + accel[1]² + accel[2]²);
   accel_x = accel[0] / accel_norm;
   accel_y = accel[1] / accel_norm;
   accel_z = accel[2] / accel_norm;
   ```

2. **Calculate accelerometer-based angles**:
   ```c
   accel_roll = atan2(accel_y, accel_z) × 180/π
   accel_pitch = asin(-accel_x) × 180/π
   ```

3. **Apply complementary filter**:
   ```c
   roll = α × (roll_old + gyro_x × dt) + (1 - α) × accel_roll
   pitch = α × (pitch_old + gyro_y × dt) + (1 - α) × accel_pitch
   ```

### Step 3: Calculate Ground Truth (Angle from Vertical)

**Location**: `main/main.c:2252`

```c
// Calculate ground angle from vertical using fused roll and pitch
signed_ground_angle = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
```

**Ground Truth Calculation** (from `lsm6ds3_fusion.c:420-435`):

```c
float lsm6ds3_calculate_angle_from_vertical(float roll_deg, float pitch_deg)
{
    // Convert to radians
    float roll_rad = roll_deg * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    
    // Calculate cosine components
    float cos_roll = cosf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    
    // 3D angle formula: cos(angle) = cos(roll) × cos(pitch)
    float cos_angle = cos_roll * cos_pitch;
    
    // Clamp to valid range [-1, 1] to prevent acos domain errors
    cos_angle = fmaxf(-1.0f, fminf(1.0f, cos_angle));
    
    // Calculate angle from vertical
    float angle_rad = acosf(cos_angle);
    float angle_deg = angle_rad * RAD_TO_DEG;
    
    return angle_deg;
}
```

**Mathematical Formula**:

The angle from vertical (ground truth) is calculated using the 3D angle formula:

```
angle_from_vertical = arccos(cos(roll) × cos(pitch))
```

This formula calculates the angle between the device's Z-axis and the vertical (gravity) direction, accounting for both roll and pitch rotations.

**Physical Interpretation**:
- `0°` = Device is vertical (Z-axis pointing up)
- `90°` = Device is horizontal (Z-axis pointing sideways)
- `180°` = Device is inverted (Z-axis pointing down)

### Step 4: Apply Sign Convention

**Location**: `main/main.c:2253-2255`

```c
// Apply sign convention: negative for angles > 90°
if (signed_ground_angle > 90.0f) {
    signed_ground_angle = -signed_ground_angle;
}
```

This ensures that angles beyond 90° are represented as negative values, providing a signed angle measurement.

---

## Complete Data Flow

```
┌─────────────────┐
│  LSM6DS3 Sensor │
└────────┬────────┘
         │
         ├─── Accelerometer (mg) ───┐
         │                           │
         └─── Gyroscope (mdps) ─────┤
                                     │
                                     ▼
                          ┌──────────────────┐
                          │  Unit Conversion  │
                          │  mg → g           │
                          │  mdps → dps       │
                          └─────────┬─────────┘
                                    │
                                    ▼
                          ┌──────────────────┐
                          │ Complementary    │
                          │ Filter (α=0.96)  │
                          │                  │
                          │ Fuses:           │
                          │ - Gyro (96%)     │
                          │ - Accel (4%)     │
                          └─────────┬────────┘
                                    │
                                    ├─── Roll
                                    │
                                    └─── Pitch
                                         │
                                         ▼
                          ┌──────────────────────────┐
                          │ Ground Truth Calculation  │
                          │                          │
                          │ angle = arccos(          │
                          │   cos(roll) × cos(pitch) │
                          │ )                        │
                          └─────────┬────────────────┘
                                    │
                                    ▼
                          ┌──────────────────┐
                          │ Signed Ground    │
                          │ Angle            │
                          │                  │
                          │ >90° → negative  │
                          └──────────────────┘
```

---

## Code Sections Reference

### Filter Initialization

**File**: `main/main.c`  
**Lines**: 1962-1964

```c
err = lsm6ds3_complementary_init(&s_lsm6ds3_filter, 0.96f, 104.0f);
if (err != ESP_OK) {
    ESP_LOGW(TAG, "LSM6DS3: Failed to initialize complementary filter: %s (continuing anyway)", esp_err_to_name(err));
}
```

### Sensor Reading and Filter Update

**File**: `main/main.c`  
**Lines**: 2230-2261

```c
// Read sensor data
float accel_mg_temp[3];
float gyro_mdps_temp[3];
esp_err_t accel_err = lsm6ds3_read_accel(&s_lsm6ds3_handle, accel_mg_temp);
esp_err_t gyro_err = lsm6ds3_read_gyro(&s_lsm6ds3_handle, gyro_mdps_temp);

if (accel_err == ESP_OK && gyro_err == ESP_OK) {
    // Convert units: mg to g, mdps to dps
    float accel_g[3] = {
        accel_mg_temp[0] / 1000.0f,
        accel_mg_temp[1] / 1000.0f,
        accel_mg_temp[2] / 1000.0f
    };
    float gyro_dps[3] = {
        gyro_mdps_temp[0] / 1000.0f,
        gyro_mdps_temp[1] / 1000.0f,
        gyro_mdps_temp[2] / 1000.0f
    };
    
    // Calculate time delta from period_ms (typically 20ms for 50Hz)
    float dt = (float)period_ms / 1000.0f;
    
    // Update filter with new sensor data
    esp_err_t fusion_err = lsm6ds3_complementary_update(&s_lsm6ds3_filter, accel_g, gyro_dps, dt);
    
    if (fusion_err == ESP_OK) {
        float roll, pitch;
        lsm6ds3_complementary_get_angles(&s_lsm6ds3_filter, &roll, &pitch);
        
        // Calculate ground angle from vertical
        signed_ground_angle = lsm6ds3_calculate_angle_from_vertical(roll, pitch);
        
        // Apply sign convention: negative for angles > 90°
        if (signed_ground_angle > 90.0f) {
            signed_ground_angle = -signed_ground_angle;
        }
    } else {
        ESP_LOGW(TAG, "LSM6DS3: Complementary filter update failed: %s", esp_err_to_name(fusion_err));
    }
}
```

### Complementary Filter Implementation

**File**: `components/lsm6ds3/lsm6ds3_fusion.c`  
**Lines**: 236-273

```c
esp_err_t lsm6ds3_complementary_update(lsm6ds3_complementary_filter_t *filter, 
                                        const float accel[3], 
                                        const float gyro[3], 
                                        float dt)
{
    // Normalize accelerometer
    float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    
    if (accel_norm > 0.0f) {
        float accel_x = accel[0] / accel_norm;
        float accel_y = accel[1] / accel_norm;
        float accel_z = accel[2] / accel_norm;
        
        // Calculate angles from accelerometer
        float accel_roll = atan2f(accel_y, accel_z) * RAD_TO_DEG;
        float accel_pitch = asinf(-accel_x) * RAD_TO_DEG;
        
        if (!filter->initialized) {
            // Initialize from accelerometer
            filter->roll = accel_roll;
            filter->pitch = accel_pitch;
            filter->initialized = true;
        } else {
            // Complementary filter fusion
            filter->roll = filter->alpha * (filter->roll + gyro[0] * dt) + 
                          (1.0f - filter->alpha) * accel_roll;
            filter->pitch = filter->alpha * (filter->pitch + gyro[1] * dt) + 
                           (1.0f - filter->alpha) * accel_pitch;
        }
    } else {
        // If accelerometer is invalid, use gyroscope only
        if (filter->initialized) {
            filter->roll += gyro[0] * dt;
            filter->pitch += gyro[1] * dt;
        }
    }
    
    return ESP_OK;
}
```

### Ground Truth Calculation

**File**: `components/lsm6ds3/lsm6ds3_fusion.c`  
**Lines**: 420-435

```c
float lsm6ds3_calculate_angle_from_vertical(float roll_deg, float pitch_deg)
{
    // Convert to radians
    float roll_rad = roll_deg * DEG_TO_RAD;
    float pitch_rad = pitch_deg * DEG_TO_RAD;
    
    // Calculate cosine components
    float cos_roll = cosf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    
    // 3D angle formula
    float cos_angle = cos_roll * cos_pitch;
    
    // Clamp to valid range [-1, 1]
    cos_angle = fmaxf(-1.0f, fminf(1.0f, cos_angle));
    
    // Calculate angle from vertical
    float angle_rad = acosf(cos_angle);
    float angle_deg = angle_rad * RAD_TO_DEG;
    
    return angle_deg;
}
```

---

## Configuration Parameters

### Filter Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `alpha` | `0.96` | Complementary filter coefficient (96% gyro, 4% accel) |
| `sample_rate` | `104 Hz` | Sensor update frequency |
| `dt` | `~0.0096 s` | Time delta (1/sample_rate) |

### Sensor Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Accelerometer Range | `±2g` | Full-scale range |
| Gyroscope Range | `±250 DPS` | Full-scale range |
| Output Data Rate | `104 Hz` | Both sensors |

### Angle Calculation Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `RAD_TO_DEG` | `180.0 / π` | Radians to degrees conversion |
| `DEG_TO_RAD` | `π / 180.0` | Degrees to radians conversion |
| `PI` | `3.14159265358979323846` | Mathematical constant π |

---

## Mathematical Formulas

### 1. Accelerometer Angle Calculation

```
roll_accel = atan2(accel_y, accel_z) × 180/π
pitch_accel = asin(-accel_x) × 180/π
```

### 2. Complementary Filter Fusion

```
roll_new = α × (roll_old + gyro_x × dt) + (1 - α) × roll_accel
pitch_new = α × (pitch_old + gyro_y × dt) + (1 - α) × pitch_accel
```

Where:
- `α = 0.96` (filter coefficient)
- `dt = 1/104 ≈ 0.0096` seconds

### 3. Ground Truth (Angle from Vertical)

```
angle_from_vertical = arccos(cos(roll) × cos(pitch))
```

This calculates the 3D angle between the device's Z-axis and the vertical direction.

### 4. Sign Convention

```
if (angle > 90°):
    signed_angle = -angle
else:
    signed_angle = angle
```

---

## Calibration

The LSM6DS3 driver supports calibration for both accelerometer and gyroscope:

### Accelerometer Calibration

**Location**: `components/lsm6ds3/lsm6ds3.c:675-712`

- Collects samples while device is stationary
- Calculates offset for X and Y axes
- Calculates Z-axis offset assuming 1g gravity

```c
handle->calibration.accel_offset_mg[0] = sum[0] / samples;
handle->calibration.accel_offset_mg[1] = sum[1] / samples;
handle->calibration.accel_offset_mg[2] = (sum[2] / samples) - GRAVITY_MG;  // Subtract 1g
```

### Gyroscope Calibration

**Location**: `components/lsm6ds3/lsm6ds3.c:714-751`

- Collects samples while device is stationary
- Calculates average offset (should be near zero)

```c
handle->calibration.gyro_offset_mdps[0] = sum[0] / samples;
handle->calibration.gyro_offset_mdps[1] = sum[1] / samples;
handle->calibration.gyro_offset_mdps[2] = sum[2] / samples;
```

Calibration offsets are automatically applied when reading sensor data (see `lsm6ds3.c:477-481` and `552-556`).

---

## Error Handling

### Filter Update Errors

**Location**: `main/main.c:2260`

```c
if (fusion_err != ESP_OK) {
    ESP_LOGW(TAG, "LSM6DS3: Complementary filter update failed: %s", esp_err_to_name(fusion_err));
    // Continue with previous angle values
}
```

### Invalid Accelerometer Data

**Location**: `lsm6ds3_fusion.c:265-269`

If accelerometer norm is zero or invalid, the filter falls back to gyroscope-only integration:

```c
if (accel_norm <= 0.0f) {
    if (filter->initialized) {
        filter->roll += gyro[0] * dt;
        filter->pitch += gyro[1] * dt;
    }
}
```

### Domain Error Prevention

**Location**: `lsm6ds3_fusion.c:429`

The ground truth calculation clamps the cosine value to prevent `acos()` domain errors:

```c
cos_angle = fmaxf(-1.0f, fminf(1.0f, cos_angle));
```

---

## Performance Characteristics

### Update Rate

- **Sensor Sample Rate**: 104 Hz (sensor internal rate)
- **Filter Update Rate**: 50 Hz (task update rate, typically 20ms period)
- **Angle Calculation**: 50 Hz (per filter update)
- **Ground Truth Calculation**: 50 Hz (per filter update)

**Note**: The sensor samples at 104Hz internally, but the complementary filter is updated at the task rate (typically 50Hz with 20ms period). The `dt` parameter passed to the filter reflects the actual time between filter updates.

### Computational Complexity

- **Complementary Filter Update**: O(1) - constant time
- **Ground Truth Calculation**: O(1) - constant time
- **Total per sample**: ~10-20 floating-point operations

### Memory Usage

- **Filter State**: ~32 bytes (`lsm6ds3_complementary_filter_t`)
- **Temporary Variables**: ~100 bytes (stack)
- **Total**: <200 bytes per sensor instance

---

## Notes

1. **Filter Selection**: The implementation uses a complementary filter rather than the Madgwick filter (which is also available but not used). The complementary filter is simpler and sufficient for roll/pitch estimation.

2. **Yaw Calculation**: Yaw is not calculated because it requires a magnetometer. The complementary filter only provides roll and pitch.

3. **Ground Truth**: The "ground truth" angle represents the angle from vertical (gravity direction), which is useful for applications requiring absolute orientation relative to gravity.

4. **Sign Convention**: Angles greater than 90° are negated to provide a signed measurement, useful for distinguishing between orientations on opposite sides of vertical.

5. **Calibration**: Sensor calibration is important for accurate angle measurements. The driver supports both software calibration (applied in software) and hardware calibration (using sensor registers).

---

## References

- **LSM6DS3 Driver**: `components/lsm6ds3/lsm6ds3.c`
- **Sensor Fusion**: `components/lsm6ds3/lsm6ds3_fusion.c`
- **Main Application**: `main/main.c` (lines 1962, 2230-2260, 2325-2337)
- **STMicroelectronics Register Driver**: `components/lsm6ds3/driver/lsm6ds3_reg.c` (BSD 3-Clause License)

---

**Document Version**: 1.0  
**Last Updated**: 2025-01-XX  
**Author**: Generated from codebase analysis

