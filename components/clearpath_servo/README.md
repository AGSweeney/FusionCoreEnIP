# ClearPath Servo Driver Component

**Status: Scaffolded - NOT included in build**

Low-level driver for Teknic ClearPath servos providing step/direction control, position tracking, velocity profiles, and HLFB feedback monitoring. Designed to work with the `clearpath_servo_manager` component for multi-servo coordination and step generation.

## Features

- **Step/Direction Control**: Standard interface for ClearPath servos
- **Position Tracking**: 32-bit signed position counter (±2,147,483,647 steps)
- **Velocity Control**: Configurable maximum velocity (steps/second)
- **Acceleration Control**: Configurable acceleration/deceleration limits
- **Motion Profiles**: Trapezoidal (constant acceleration) and S-curve (jerk-limited)
- **Absolute/Relative Moves**: Support for both move types
- **Continuous Velocity Mode**: Move at constant velocity until stopped
- **Homing Support**: Move to sensor and set position to zero
- **HLFB Monitoring**: High Level Feedback status interpretation
- **Enable Control**: Optional enable GPIO for servo enable/disable
- **Thread-Safe**: Mutex protection for concurrent access

## Hardware Requirements

### Level Shifting

**Important**: ESP32-P4 GPIO outputs are 3.3V logic. ClearPath servos expect 5V logic levels. Level shifting is required using a device such as:
- **SN74AHCT14** (hex inverting Schmitt trigger) - recommended
- Similar level shifter ICs

**Connection Requirements:**
- **Step GPIO**: Output → Level shifter → ClearPath Step input (5V)
- **Direction GPIO**: Output → Level shifter → ClearPath Direction input (5V)
- **Enable GPIO**: Output → Level shifter → ClearPath Enable input (5V, optional)
- **HLFB GPIO**: ClearPath HLFB output (5V) → Level shifter → Input (3.3V, optional)
- **Homing Sensor**: Sensor output → Level shifter → Input (3.3V, optional)

### GPIO Configuration

Each servo requires:
- **Step GPIO**: Output, generates step pulses (requires 3.3V→5V level shifting)
- **Direction GPIO**: Output, sets direction (HIGH=forward, LOW=reverse, requires level shifting)
- **Enable GPIO**: Output, optional, enables/disables servo (requires level shifting)
- **HLFB GPIO**: Input, High Level Feedback status (5V from servo, requires 5V→3.3V level shifting)
- **Homing Sensor GPIO**: Input, optional, homing sensor signal (may require level shifting)

## Quick Start

### Initialization

```c
#include "clearpath_servo.h"

clearpath_servo_config_t config = {
    .gpio_step = 8,
    .gpio_dir = 9,
    .gpio_enable = 10,      // Optional, use -1 if not used
    .gpio_hlfb = 11,        // Optional, use -1 if not used
    .gpio_homing_sensor = 12,  // Optional, use -1 if not used
    .vel_max = 1000,        // Maximum velocity in steps/second
    .accel_max = 10000,     // Maximum acceleration in steps/second²
    .jerk_max = 0,          // Maximum jerk (0 = use default: 10% of accel_max)
    .profile_type = CLEARPATH_SERVO_PROFILE_TRAPEZOIDAL,
    .hlfb_mode = CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE,
    .hlfb_active_high = true,
    .homing_sensor_type = CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN,
    .homing_velocity = 0    // 0 = use vel_max
};

clearpath_servo_handle_t *servo;
esp_err_t ret = clearpath_servo_init(&config, &servo);
if (ret != ESP_OK) {
    ESP_LOGE("APP", "Failed to initialize servo");
    return;
}
```

## API Reference

### Position Moves

```c
// Relative move: move 1000 steps forward
clearpath_servo_move(servo, 1000, CLEARPATH_SERVO_MOVE_TARGET_RELATIVE);

// Absolute move: move to position 5000
clearpath_servo_move(servo, 5000, CLEARPATH_SERVO_MOVE_TARGET_ABSOLUTE);

// Check if move is complete
if (clearpath_servo_steps_complete(servo)) {
    // Move finished
}
```

### Velocity Control

```c
// Move at constant velocity (1000 steps/second forward)
clearpath_servo_move_velocity(servo, 1000);

// Move in reverse at 500 steps/second
clearpath_servo_move_velocity(servo, -500);

// Stop motion
clearpath_servo_stop(servo);
```

### Configuration

```c
// Set maximum velocity
clearpath_servo_set_vel_max(servo, 2000);  // 2000 steps/second

// Set maximum acceleration
clearpath_servo_set_accel_max(servo, 50000);  // 50000 steps/second²

// Set maximum jerk (for S-curve profile)
clearpath_servo_set_jerk_max(servo, 5000);  // 5000 steps/second³

// Set motion profile type
clearpath_servo_set_profile_type(servo, CLEARPATH_SERVO_PROFILE_S_CURVE);
```

### Position and Status

```c
// Get current position
int32_t position = clearpath_servo_get_position(servo);

// Get current velocity
int32_t velocity = clearpath_servo_get_velocity(servo);

// Reset position to zero
clearpath_servo_reset_position(servo);

// Set position (for calibration)
clearpath_servo_set_position(servo, 0);
```

### HLFB (High Level Feedback)

```c
// Read raw HLFB GPIO status
bool hlfb = clearpath_servo_get_hlfb_status(servo);

// Check if servo is enabled (from HLFB)
bool enabled = clearpath_servo_is_enabled(servo);

// Check if move is complete (from HLFB)
bool complete = clearpath_servo_is_move_complete(servo);

// Check for fault condition (from HLFB)
bool fault = clearpath_servo_is_fault(servo);
```

### Enable Control

```c
// Enable servo
clearpath_servo_set_enable(servo, true);

// Disable servo
clearpath_servo_set_enable(servo, false);
```

### Homing

```c
// Start homing move (forward direction, 5 second timeout)
clearpath_servo_home(servo, 1, 5000);  // direction=forward, timeout=5 seconds

// Start homing move (reverse direction, 10 second timeout)
clearpath_servo_home(servo, -1, 10000);  // direction=reverse, timeout=10 seconds

// Check if homing is complete
if (clearpath_servo_is_homing_complete(servo)) {
    // Position is now set to 0
}

// Check homing sensor status
bool sensor_triggered = clearpath_servo_get_homing_sensor_status(servo);
```

**Homing Behavior:**
- Moves at configured homing velocity (or `vel_max` if `homing_velocity` is 0)
- Stops immediately when sensor triggers
- Sets position to 0 when sensor triggers
- Supports timeout (0 = no timeout)
- Sensor type configurable: NO (Normally Open) or NC (Normally Closed)

## Motion Profiles

### Trapezoidal Profile (Default)

- **Acceleration Phase**: Velocity ramps from 0 to target with constant acceleration
- **Constant Velocity Phase**: Maintains target velocity
- **Deceleration Phase**: Velocity ramps from target to 0 with constant deceleration

### S-Curve Profile

- **Jerk-Limited Acceleration**: Smoother motion with reduced mechanical stress
- **Jerk Rate**: Controlled by `jerk_max` (default: 10% of `accel_max` per second)
- **Phases**: Jerk-up → Constant acceleration → Constant velocity → Constant deceleration → Jerk-down

## HLFB Modes

HLFB (High Level Feedback) can be configured in ClearPath MSP software to report different status conditions:

- **CLEARPATH_SERVO_HLFB_MODE_ENABLED**: HLFB indicates enabled state
- **CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE**: HLFB indicates move complete
- **CLEARPATH_SERVO_HLFB_MODE_IN_POSITION**: HLFB indicates in position
- **CLEARPATH_SERVO_HLFB_MODE_FAULT**: HLFB indicates fault condition
- **CLEARPATH_SERVO_HLFB_MODE_CUSTOM**: User-defined HLFB behavior

The HLFB mode should match the configuration set in Teknic Motion Studio (MSP) software.

## Step Generation

**Note**: This driver component provides control logic and state management. Actual step pulse generation is performed by the `clearpath_servo_manager` component's background task using the ESP32-P4 RMT (Remote Control) hardware peripheral.

The manager component:
- Generates step pulses via RMT hardware (precise timing, typically 10 microsecond pulse width)
- Coordinates step timing in background task (default 5 kHz)
- Implements velocity profiles with integer-only calculations
- Updates position tracking based on step count

See `components/clearpath_servo_manager/README.md` for detailed step generation implementation.

## Integration with Manager

This driver is designed to work with `clearpath_servo_manager`, which provides:
- Background step generation task
- Multi-servo management
- Coordinated motion (linear/circular interpolation)
- EtherNet/IP assembly integration
- Configuration management (NVS)

For multi-servo coordination and unit-based motion control, use the manager component API.

## Limitations

- Position counter: 32-bit signed integer (±2,147,483,647 steps)
- Step generation: Performed by manager component using RMT hardware
- RMT timing resolution: 1 MHz (1 microsecond per tick)
- Step pulse width: 10 microseconds (configurable via RMT)
- Integer math: All internal calculations use integer arithmetic

## Attribution

This implementation is inspired by and follows the API design patterns of the Teknic ClearCore library. The API function names and concepts (Move, MoveVelocity, VelMax, AccelMax, StepsComplete, HLFB) are based on the ClearCore library API for compatibility and familiarity.

**References:**
- [Teknic ClearCore Library](https://github.com/Teknic-Inc/ClearCore-library)
- [ClearCore Library Documentation](https://teknic-inc.github.io/ClearCore-library/)
- [ClearPath Servo User Manual](https://www.teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf)
