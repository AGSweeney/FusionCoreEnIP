# ClearPath Servo Manager Component

**Status: Scaffolded - NOT included in build**

Multi-servo management component for Teknic ClearPath servos providing unit-based motion control, coordinated multi-axis interpolation, step pulse generation via RMT hardware, and EtherNet/IP assembly integration.

## Features

- **Multi-Servo Support**: Manages up to 4 ClearPath servos simultaneously
- **Unit-Based Motion**: Work in real-world units (mm or inches) instead of steps
- **Step Generation**: Background task generates step pulses using RMT hardware (default 5 kHz)
- **Velocity Profiles**: Trapezoidal and S-curve acceleration/deceleration profiles
- **Multi-Axis Coordinated Motion**:
  - Linear interpolation (2-4 axes) using Bresenham line algorithm
  - Circular interpolation (2-axis arcs) using Bresenham circle algorithm
  - Helical interpolation (3-axis XYZ) using Bresenham circle + line algorithms
  - Direct step generation with feedrate control via step timing
  - Low CPU usage (integer-only math, no trigonometry during step generation)
  - Multi-turn support (angles > 360°)
- **HLFB Monitoring**: Reads High Level Feedback for status information
- **Assembly Integration**: Reads commands from Output Assembly 150, writes feedback to Input Assembly 100
- **Thread-Safe**: Mutex protection for all shared data access
- **Integer Math**: All calculations use 64-bit integer intermediates (no floating point in motion calculations)
- **Overflow Protection**: Comprehensive overflow/underflow detection and clamping
- **Timer Wrap-Around Handling**: Handles timer rollover edge cases

## Key Concepts

### Servo Index = Motor Axis Number

**Important**: In this system, the **servo index is the same as the motor axis number**. There is no separate mapping—you assign each servo index to a physical axis:

- **Servo Index 0** = X axis (or your first axis)
- **Servo Index 1** = Y axis (or your second axis)
- **Servo Index 2** = Z axis (or your third axis)
- **Servo Index 3** = A axis (or your fourth axis)

When API functions refer to `axis_x_index` or `axis_y_index`, they mean the servo index assigned to that axis. For example:
- `axis_x_index = 0` means use servo index 0 for the X axis
- `axis_y_index = 1` means use servo index 1 for the Y axis

The system supports up to 4 servos (indices 0-3), and you configure which physical axis each servo index controls through your application setup.

## Quick Start

### Basic Initialization

```c
#include "clearpath_servo_manager.h"

// Initialize the manager
esp_err_t ret = clearpath_servo_manager_init();
if (ret != ESP_OK) {
    ESP_LOGE("APP", "Failed to initialize servo manager");
    return;
}

// Check initialization status
if (clearpath_servo_manager_is_initialized()) {
    uint8_t servo_count = clearpath_servo_manager_get_count();
    ESP_LOGI("APP", "Servo manager initialized with %d servos", servo_count);
}
```

### Configure Units and Steps Per Unit

```c
// Set unit type (all axes must use same unit type)
clearpath_servo_manager_set_unit_type(CLEARPATH_SERVO_UNIT_MM);

// Configure steps per unit for each servo
clearpath_servo_manager_config_t config;

// X axis: 800 steps/mm
clearpath_servo_manager_get_config(0, &config);
config.steps_per_unit = 800;
clearpath_servo_manager_set_config(0, &config);

// Y axis: 400 steps/mm
clearpath_servo_manager_get_config(1, &config);
config.steps_per_unit = 400;
clearpath_servo_manager_set_config(1, &config);

// Z axis: 200 steps/mm
clearpath_servo_manager_get_config(2, &config);
config.steps_per_unit = 200;
clearpath_servo_manager_set_config(2, &config);
```

### Basic Motion Control

Single-axis motion is performed via the `clearpath_servo` driver component directly, or via EtherNet/IP assembly commands. The manager provides coordinated motion functions that work in units:

```c
// Coordinated 2-axis linear move in units
uint8_t axes[] = {0, 1};  // X and Y axes
float targets[] = {100.0f, 50.0f};  // 100mm X, 50mm Y
float feedrate = 1000.0f;  // 1000 mm/min along the path

clearpath_servo_manager_move_linear(2, axes, targets, feedrate);

// Wait for completion
while (!clearpath_servo_manager_is_coordinated_move_complete()) {
    vTaskDelay(pdMS_TO_TICKS(10));
}
```

## Architecture

### Data Flow

```
Output Assembly 150 → Manager Task → Step Generation (RMT) → GPIO → ClearPath Servo
ClearPath Servo → HLFB GPIO → Manager Task → Status → Input Assembly 100
```

### Component Structure

1. **Background Task**: Generates step pulses at 5 kHz frequency
2. **Step Generation**: Uses ESP32-P4 RMT hardware for precise timing (10 μs pulse width)
3. **Velocity Profiles**: Trapezoidal or S-curve acceleration/deceleration
4. **Position Tracking**: Updates position counter based on step count
5. **HLFB Monitoring**: Reads HLFB GPIO for status feedback
6. **Assembly Updates**: Reads commands from Output Assembly 150, writes status to Input Assembly 100
7. **Coordinated Motion**: Handles multi-axis linear and circular interpolation with unit conversion

## Unit-Based Motion Control

The manager supports working in real-world units (millimeters or inches) instead of steps. This simplifies motion programming and makes code more maintainable.

**Configuration:**
- Set unit type: `clearpath_servo_manager_set_unit_type(CLEARPATH_SERVO_UNIT_MM)`
- Configure `steps_per_unit` for each servo (e.g., 800 steps/mm)
- All coordinated motion functions accept positions in units

**Benefits:**
- Code works in real-world dimensions (mm or inches)
- No manual step calculations
- Easy to understand and maintain
- Automatic conversion to steps based on each axis's `steps_per_unit`

## API Reference

### Initialization and Configuration

```c
// Initialize manager
esp_err_t clearpath_servo_manager_init(void);

// Check if initialized
bool clearpath_servo_manager_is_initialized(void);

// Get servo count
uint8_t clearpath_servo_manager_get_count(void);

// Get/set servo configuration
esp_err_t clearpath_servo_manager_get_config(uint8_t servo_index, clearpath_servo_manager_config_t *config);
esp_err_t clearpath_servo_manager_set_config(uint8_t servo_index, const clearpath_servo_manager_config_t *config);

// Get/set unit type
esp_err_t clearpath_servo_manager_set_unit_type(clearpath_servo_unit_t unit_type);
clearpath_servo_unit_t clearpath_servo_manager_get_unit_type(void);
```

### Position and Status

```c
// Get current position (in steps)
int32_t clearpath_servo_manager_get_position(uint8_t servo_index);

// Get current velocity (in steps/second)
int32_t clearpath_servo_manager_get_velocity(uint8_t servo_index);

// Get status flags
uint8_t clearpath_servo_manager_get_status(uint8_t servo_index);
```

### Coordinated Motion

#### Linear Interpolation

```c
/**
 * @brief Start coordinated linear move
 * 
 * Moves multiple axes in a straight line to target positions.
 * All axes start and stop together, maintaining proportional motion.
 * 
 * @param axis_count Number of axes (2-4)
 * @param axis_indices Array of servo indices
 * @param target_positions Array of target positions in units (one per axis)
 * @param feedrate Feedrate in units/minute along the path
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_move_linear(
    uint8_t axis_count,
    const uint8_t *axis_indices,
    const float *target_positions,
    float feedrate
);
```

#### Circular Interpolation (Center Point)

```c
/**
 * @brief Start circular interpolation (arc move)
 * 
 * Moves X and Y axes in a circular arc.
 * 
 * @param axis_x_index X axis servo index (0-3)
 * @param axis_y_index Y axis servo index (0-3)
 * @param center_x Center X position in units
 * @param center_y Center Y position in units
 * @param radius Radius in units
 * @param start_angle_deg Start angle in degrees (can be > 360 for multi-turn arcs)
 * @param end_angle_deg End angle in degrees (can be > 360 for multi-turn arcs)
 * @param clockwise True for clockwise, false for counter-clockwise
 * @param feedrate Feedrate in units/minute along the arc path
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_move_arc(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    float center_x,
    float center_y,
    float radius,
    int32_t start_angle_deg,
    int32_t end_angle_deg,
    bool clockwise,
    float feedrate
);
```

#### Circular Interpolation (End Point + Radius)

```c
/**
 * @brief Start circular interpolation using end point and radius
 * 
 * Moves from current position to end point in a circular arc.
 * Similar to G-code G02/G03 with R parameter (radius mode).
 * 
 * The center is automatically calculated from current position, end point, and radius.
 * 
 * @param axis_x_index X axis servo index (0-3)
 * @param axis_y_index Y axis servo index (0-3)
 * @param end_x End X position in units (absolute if absolute=true, relative if absolute=false)
 * @param end_y End Y position in units (absolute if absolute=true, relative if absolute=false)
 * @param absolute If true, end_x/end_y are absolute; if false, relative to current position
 * @param radius Radius in units (must be >= half the distance from start to end)
 * @param clockwise True for clockwise (G02), false for counter-clockwise (G03)
 * @param feedrate Feedrate in units/minute along the arc path
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if radius is too small
 */
esp_err_t clearpath_servo_manager_move_arc_to_point(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    float end_x,
    float end_y,
    bool absolute,
    float radius,
    bool clockwise,
    float feedrate
);
```

#### Helical Arc (Center Point)

```c
/**
 * @brief Start helical arc move (XYZ circular interpolation)
 * 
 * Moves X and Y axes in a circular arc while simultaneously moving Z axis linearly.
 * This creates a helical (spiral) motion path. Similar to G-code G02/G03 with Z axis.
 * 
 * @param axis_x_index X axis servo index (0-3)
 * @param axis_y_index Y axis servo index (0-3)
 * @param axis_z_index Z axis servo index (0-3)
 * @param center_x Center X position in units
 * @param center_y Center Y position in units
 * @param radius Radius in units
 * @param start_angle_deg Start angle in degrees (can be > 360 for multi-turn helixes)
 * @param end_angle_deg End angle in degrees (can be > 360 for multi-turn helixes)
 * @param z_start Z start position in units
 * @param z_end Z end position in units
 * @param clockwise True for clockwise, false for counter-clockwise
 * @param feedrate Feedrate in units/minute along the helical path
 * @return esp_err_t ESP_OK on success
 */
esp_err_t clearpath_servo_manager_move_arc_helical(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    uint8_t axis_z_index,
    float center_x,
    float center_y,
    float radius,
    int32_t start_angle_deg,
    int32_t end_angle_deg,
    float z_start,
    float z_end,
    bool clockwise,
    float feedrate
);
```

#### Helical Arc (End Point + Radius)

```c
/**
 * @brief Start helical arc move using end point and radius
 * 
 * Moves X and Y axes in a circular arc while simultaneously moving Z axis linearly,
 * from current position to end point. Similar to G-code G02/G03 with R parameter and Z axis.
 * 
 * The center is automatically calculated from current position, end point, and radius.
 * 
 * @param axis_x_index X axis servo index (0-3)
 * @param axis_y_index Y axis servo index (0-3)
 * @param axis_z_index Z axis servo index (0-3)
 * @param end_x End X position in units (absolute if absolute=true, relative if absolute=false)
 * @param end_y End Y position in units (absolute if absolute=true, relative if absolute=false)
 * @param end_z End Z position in units (absolute if absolute=true, relative if absolute=false)
 * @param absolute If true, end_x/end_y/end_z are absolute; if false, relative to current position
 * @param radius Radius in units (must be >= half the distance from start to end in XY plane)
 * @param clockwise True for clockwise (G02), false for counter-clockwise (G03)
 * @param feedrate Feedrate in units/minute along the helical path
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if radius is too small
 */
esp_err_t clearpath_servo_manager_move_arc_helical_to_point(
    uint8_t axis_x_index,
    uint8_t axis_y_index,
    uint8_t axis_z_index,
    float end_x,
    float end_y,
    float end_z,
    bool absolute,
    float radius,
    bool clockwise,
    float feedrate
);
```

#### Move Control

```c
// Check if coordinated move is complete
bool clearpath_servo_manager_is_coordinated_move_complete(void);

// Stop coordinated move
esp_err_t clearpath_servo_manager_stop_coordinated_move(void);
```

### Homing

```c
// Start homing move
esp_err_t clearpath_servo_manager_home(uint8_t servo_index, int32_t direction, uint32_t timeout_ms);

// Check if homing is complete
bool clearpath_servo_manager_is_homing_complete(uint8_t servo_index);

// Get homing sensor status
bool clearpath_servo_manager_get_homing_sensor_status(uint8_t servo_index);
```

## Usage Examples

### Linear Interpolation (2-Axis)

```c
// 2-axis linear move: X=100mm, Y=50mm at 1000 mm/min along the path
uint8_t axes[] = {0, 1};  // Servo 0 (X), Servo 1 (Y)
float targets[] = {100.0f, 50.0f};  // 100mm X, 50mm Y
float feedrate = 1000.0f;  // 1000 mm/min along the path (not per axis)

esp_err_t ret = clearpath_servo_manager_move_linear(2, axes, targets, feedrate);
if (ret == ESP_OK) {
    while (!clearpath_servo_manager_is_coordinated_move_complete()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI("APP", "Linear move complete");
}
```

### Linear Interpolation (3-Axis)

```c
// X-Y-Z move: 100mm X, 50mm Y, 25mm Z at 1000 mm/min
uint8_t axes[] = {0, 1, 2};
float targets[] = {100.0f, 50.0f, 25.0f};
clearpath_servo_manager_move_linear(3, axes, targets, 1000.0f);
```

### Circular Interpolation (Center Point)

```c
// Draw a full circle: center at (100mm, 100mm), radius 50mm, clockwise, 1000 mm/min
clearpath_servo_manager_move_arc(
    0,      // X axis servo index
    1,      // Y axis servo index
    100.0f, // Center X (mm)
    100.0f, // Center Y (mm)
    50.0f,  // Radius (mm)
    0,      // Start angle (degrees)
    360,    // End angle (degrees)
    true,   // Clockwise
    1000.0f // Feedrate (mm/min along arc)
);

// Draw a 90-degree arc (quarter circle)
clearpath_servo_manager_move_arc(0, 1, 100.0f, 100.0f, 50.0f, 0, 90, true, 1000.0f);
```

### Circular Interpolation (End Point + Radius)

```c
// Absolute end point: move to (150mm, 100mm) with 50mm radius, clockwise
clearpath_servo_manager_move_arc_to_point(
    0,      // X axis servo index
    1,      // Y axis servo index
    150.0f, // End X (absolute, mm)
    100.0f, // End Y (absolute, mm)
    true,   // Absolute coordinates
    50.0f,  // Radius (mm) - must be >= half the distance from start to end
    true,   // Clockwise (G02)
    1000.0f // Feedrate (mm/min along arc)
);

// Relative end point: move 50mm in X, 30mm in Y with 40mm radius, counter-clockwise
clearpath_servo_manager_move_arc_to_point(
    0,      // X axis servo index
    1,      // Y axis servo index
    50.0f,  // End X (relative, mm)
    30.0f,  // End Y (relative, mm)
    false,  // Relative coordinates
    40.0f,  // Radius (mm)
    false,  // Counter-clockwise (G03)
    1000.0f // Feedrate (mm/min)
);
```

### Helical Arc (Center Point)

```c
// Helical arc: 90-degree arc in XY plane, while moving 10mm in Z
clearpath_servo_manager_move_arc_helical(
    0,      // X axis servo index
    1,      // Y axis servo index
    2,      // Z axis servo index
    100.0f, // Center X (mm)
    100.0f, // Center Y (mm)
    50.0f,  // Radius (mm)
    0,      // Start angle (degrees)
    90,     // End angle (degrees)
    0.0f,   // Z start position (mm)
    10.0f,  // Z end position (mm)
    true,   // Clockwise
    1000.0f // Feedrate (mm/min along helical path)
);

// Multi-turn helix: 3 full rotations (1080°) with 30mm Z travel
clearpath_servo_manager_move_arc_helical(
    0, 1, 2,           // X, Y, Z axes
    200.0f, 200.0f,     // Center X, Y
    50.0f,              // Radius
    0, 1080,            // 3 full turns (3 × 360°)
    0.0f, 30.0f,        // Z: 0 to 30mm
    true,               // Clockwise
    2000.0f             // Feedrate
);
```

### Helical Arc (End Point + Radius)

```c
// Absolute end point: move to (150mm, 100mm, 20mm) with 50mm radius, clockwise
clearpath_servo_manager_move_arc_helical_to_point(
    0,      // X axis servo index
    1,      // Y axis servo index
    2,      // Z axis servo index
    150.0f, // End X (absolute, mm)
    100.0f, // End Y (absolute, mm)
    20.0f,  // End Z (absolute, mm)
    true,   // Absolute coordinates
    50.0f,  // Radius (mm) - must be >= half the distance from start to end in XY plane
    true,   // Clockwise (G02)
    1000.0f // Feedrate (mm/min along helical path)
);

// Relative end point: move 50mm in X, 30mm in Y, 10mm in Z with 40mm radius
clearpath_servo_manager_move_arc_helical_to_point(
    0, 1, 2,    // X, Y, Z axes
    50.0f,      // End X (relative, mm)
    30.0f,      // End Y (relative, mm)
    10.0f,      // End Z (relative, mm)
    false,      // Relative coordinates
    40.0f,      // Radius (mm)
    false,      // Counter-clockwise (G03)
    1000.0f     // Feedrate
);
```

**Angle Convention:**
- 0° = +X direction (right)
- 90° = +Y direction (up)
- 180° = -X direction (left)
- 270° = -Y direction (down)

## Coordinated Motion Algorithms

### Linear Interpolation

Uses **Bresenham's line algorithm** for direct step generation:

**How it works:**
1. Convert target positions from units to steps using each axis's `steps_per_unit`
2. Calculate path length: `sqrt(sum of squares of axis distances)`
3. Initialize Bresenham algorithm state for each axis (distance, step direction, error term)
4. Generate steps directly: for each step along the path, step the axis with maximum error term
5. Control feedrate via step timing: `step_interval = 1,000,000 / feedrate_steps_per_sec`

**Benefits:**
- Integer-only math (no floating point)
- Direct step generation (no intermediate target calculations)
- ~75-90% CPU reduction compared to parametric approach
- Maintains synchronized motion naturally
- Proven algorithm (standard in GRBL and other CNC controllers)

### Circular Interpolation

Uses **Bresenham's circle algorithm** with full octant support:

**How it works:**
1. Convert center, radius from units to steps
2. Calculate arc length: `arc_length = radius × angle_radians`
3. Initialize Bresenham circle algorithm in first octant (0-45°)
4. Transform steps to actual coordinates based on current octant (0-7)
5. Handle octant transitions at 45° boundaries
6. Generate steps directly for X and Y axes
7. Control feedrate via step timing

**Octant Support:**
- Full 8-octant support (0-360°)
- Automatic octant transitions at 45° boundaries
- Correct coordinate transformations for all octants
- Supports clockwise and counter-clockwise directions

**Benefits:**
- Integer-only math (no trigonometry during step generation)
- Direct step generation
- Low CPU usage
- Full circle support (0-360° and beyond for multi-turn arcs)

### Helical Interpolation

Combines **Bresenham circle algorithm** (X/Y) with **Bresenham line algorithm** (Z):

**How it works:**
1. X and Y follow circular arc using Bresenham circle algorithm
2. Z moves linearly using Bresenham line algorithm
3. Z steps are generated proportionally to arc progress (error term accumulates `abs(z_total_distance)`, steps when `error >= arc_length`)
4. All three axes start and stop together
5. Feedrate is along the 3D helical path

**Benefits:**
- Synchronized 3-axis motion
- Integer-only math
- Direct step generation
- Supports multi-turn helixes (angles > 360°)

### Step Generation and Feedrate Control

**Step Generation:**
- Steps are generated directly by Bresenham algorithms (no intermediate target calculations)
- Step generation occurs in the `step_generation_task` (5 kHz frequency)
- Feedrate is controlled via step timing: `step_interval_us = 1,000,000 / feedrate_steps_per_sec`
- Steps are generated only when enough time has passed (feedrate control)

**Overrun Protection:**
- Maximum steps per interval are limited based on feedrate and task interval
- Prevents step generation from overwhelming the system
- Formula: `max_steps = (feedrate * delta_us) / 1,000,000 + 1`

**Race Condition Protection:**
- All access to coordinated motion state is protected by `s_config_mutex`
- Servo handles are copied under mutex protection before use
- Step generation functions assume mutex is already held

## Step Generation

This implementation uses the ESP32-P4 RMT (Remote Control) hardware peripheral for step pulse generation, providing precise timing independent of task scheduling.

### RMT Hardware Configuration

- **Clock divider**: 80 (1 MHz resolution, 1 microsecond per tick)
- **Step pulse width**: 10 microseconds (configurable via RMT)
- **Precise hardware timing**: Accurate pulse generation independent of CPU load

### Step Generation Task

The manager includes a background task (default 5 kHz, configurable) that:

1. Reads commands from Output Assembly 150
2. Generates step pulses using RMT hardware based on velocity/acceleration profiles
3. Updates position tracking
4. Monitors HLFB for status feedback
5. Writes status/feedback to Input Assembly 100
6. Processes coordinated motion (linear/circular interpolation)

**Step Rate**: Limited by task frequency (default 5 kHz) and RMT configuration. RMT hardware enables much higher rates than software GPIO toggling.

### Velocity Profiles

Step generation supports two motion profile types with integer-only calculations:

#### Trapezoidal Profile (Default)

- **Acceleration Phase**: Velocity ramps from 0 to target velocity with constant acceleration
- **Constant Velocity Phase**: Maintains target velocity until deceleration begins
- **Deceleration Phase**: Velocity ramps from target to 0 with constant deceleration

#### S-Curve Profile

- **Jerk-Limited Acceleration**: Smoother motion with reduced mechanical stress
- **Jerk Rate**: Controlled by `jerk_max` (default: 10% of `accel_max` per second)
- **Phases**: Jerk-up → Constant acceleration → Constant velocity → Constant deceleration → Jerk-down

All calculations use 64-bit integer intermediates to prevent overflow, then convert to 32-bit results. This provides precise motion control without floating-point operations, suitable for embedded systems.

## Assembly Data Layout

### Output Assembly 150 (Control Commands)

**Bytes 32-51: Servo Commands (20 bytes total, 5 bytes per servo, supports 4 servos)**

| Byte Range | Servo | Field | Type | Description |
|------------|-------|-------|------|-------------|
| 32 | Servo 0 | Command Type | uint8_t | Command type (see below) |
| 33-36 | Servo 0 | Value | int32_t | Position (move_abs/move_rel) or velocity (move_velocity) |
| 37 | Servo 1 | Command Type | uint8_t | Command type |
| 38-41 | Servo 1 | Value | int32_t | Position or velocity |
| 42 | Servo 2 | Command Type | uint8_t | Position or velocity |
| 43-46 | Servo 2 | Value | int32_t | Position or velocity |
| 47 | Servo 3 | Command Type | uint8_t | Command type |
| 48-51 | Servo 3 | Value | int32_t | Position or velocity |

**Command Types:**
- 0: Stop - Value field ignored
- 1: Move Absolute - Value is target position (int32_t, steps)
- 2: Move Relative - Value is relative steps (int32_t, steps)
- 3: Move Velocity - Value is target velocity (int32_t, steps/second)
- 4: Enable - Value field ignored
- 5: Disable - Value field ignored
- 6: Home - Value format: lower 16 bits = direction (int16_t), upper 16 bits = timeout_ms (uint16_t)

**Note**: All values are little-endian int32_t, providing full 32-bit range for positions and velocities (±2,147,483,647 steps or steps/second).

### Input Assembly 100 (Status/Feedback)

**Bytes 61-71: Servo Feedback (11 bytes)**

| Byte Range | Field | Type | Description |
|------------|-------|------|-------------|
| 61-64 | Position | int32_t | Servo 0 position in steps (little-endian) |
| 65-66 | Velocity | int16_t | Servo 0 velocity in steps/second (little-endian) |
| 67 | Status | uint8_t | Servo 0 status flags |
| 68-71 | Reserved | - | Reserved for Servo 1 or future expansion |

**Status Byte (Byte 67) Bits:**
- Bit 0: Move complete (1 = complete, 0 = moving)
- Bit 1: Enabled (1 = enabled, 0 = disabled)
- Bit 2: Fault (1 = fault detected, 0 = no fault)
- Bit 3: In position (1 = in position, 0 = not in position)
- Bits 4-7: Reserved

## Homing

The component supports homing operations to establish a reference position (zero point) using a homing sensor.

### Homing Sensor Configuration

- **GPIO Pin**: Configure `gpio_homing_sensor` in servo configuration (-1 if not used)
- **Sensor Type**: 
  - `CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN` (NO): Sensor triggers when GPIO goes HIGH (closed)
  - `CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_CLOSED` (NC): Sensor triggers when GPIO goes LOW (opened)
- **Homing Velocity**: Configure `homing_velocity` (0 = use `vel_max`)

### Homing Process

1. Call `clearpath_servo_manager_home()` with direction and timeout
2. Servo moves at homing velocity in specified direction
3. When sensor triggers, motion stops immediately
4. Position is automatically set to 0
5. Homing state transitions to IDLE

### Homing via EtherNet/IP

Command type 6 in Output Assembly 150:
- Byte 0: Command type = 6
- Bytes 1-4: int32_t value
  - Lower 16 bits: direction (int16_t, positive=forward, negative=reverse)
  - Upper 16 bits: timeout_ms (uint16_t, 0=no timeout)

**Example**: Home forward with 5 second timeout
- Value = (5000 << 16) | 1 = 0x00050001

## Error Handling and Safety

### Overflow Protection

- **Unit Conversion**: Float-to-int conversions check for INT32_MAX/INT32_MIN overflow
- **Feedrate Conversion**: Checks for UINT32_MAX overflow
- **Path Calculations**: Square root algorithm includes iteration limits and overflow checks
- **Time Calculations**: All time calculations check for overflow before casting
- **Velocity Calculations**: Axis velocities are clamped to valid ranges

### Timer Wrap-Around Handling

- Detects timer rollover (when `current_time < start_time`)
- Handles wrap-around gracefully with appropriate elapsed time calculation
- Logs warnings when wrap-around is detected

### Input Validation

- **Axis Count**: Validated to be 2-4 for coordinated motion
- **Axis Indices**: Checked for bounds and initialization
- **Steps Per Unit**: Must be configured (> 0) before coordinated motion
- **Feedrate**: Must be > 0 (validated after conversion)
- **Radius**: Must be > 0 for circular interpolation
- **Radius Validation**: For end-point arcs, radius must be >= half the distance from start to end

## Limitations

- Maximum servos: 4 (indices 0-3)
- Position counter: 32-bit signed integer (±2,147,483,647 steps)
- Step generation: RMT hardware-based, default 5 kHz task frequency
- Unit mixing: All axes must use the same unit type (mm or inches)
- Coordinated motion: Requires all axes to have `steps_per_unit` configured

## Attribution

Step generation algorithms, trapezoidal velocity profiles, and servo control concepts are based on patterns from the Teknic ClearCore library. The high-frequency step generation task and velocity/acceleration management follow similar principles to ClearCore's StepGenerator implementation.

**References:**
- [Teknic ClearCore Library](https://github.com/Teknic-Inc/ClearCore-library)
- [ClearCore Step Generation](https://teknic-inc.github.io/ClearCore-library/_move_gen.html)
- [ClearPath Servo User Manual](https://www.teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf)
