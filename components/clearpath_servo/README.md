# ClearPath Servo Driver Component

**Status: Scaffolded - NOT included in build**

This component provides step and direction control for Teknic ClearPath servos, similar to the ClearCore library API. It handles step pulse generation, position tracking, velocity control, and acceleration/deceleration limits.

## Features

- **Step and Direction Control**: Standard step/direction interface for ClearPath servos
- **Position Tracking**: 32-bit signed position counter
- **Velocity Control**: Configurable maximum velocity (steps per second)
- **Acceleration Control**: Configurable acceleration/deceleration limits
- **Absolute and Relative Moves**: Support for both move types
- **Continuous Velocity Mode**: Move at constant velocity until stopped
- **HLFB Support**: High Level Feedback monitoring for status information
- **Enable Control**: Optional enable GPIO for servo enable/disable
- **Thread-Safe**: Mutex protection for concurrent access

## Hardware Requirements

### Level Shifting

**Important**: ESP32-P4 GPIO outputs are 3.3V logic, which may not reliably drive ClearPath servo inputs. Level shifting is required using a device such as SN74AHCT14 (hex inverting Schmitt trigger) or similar level shifter to convert 3.3V logic to 5V logic levels that ClearPath servos expect.

See the hookup schematic in `clearpath_servo_manager` component documentation for connection details.

### GPIO Connections

Each servo requires:
- **Step GPIO**: Output, generates step pulses (requires level shifting to 5V)
- **Direction GPIO**: Output, sets direction (HIGH=forward, LOW=reverse, requires level shifting to 5V)
- **Enable GPIO**: Output, optional, enables/disables servo (requires level shifting to 5V)
- **HLFB GPIO**: Input, High Level Feedback status (5V from servo, may need level shifting to 3.3V)

## API Reference

### Initialization

```c
#include "clearpath_servo.h"

clearpath_servo_config_t config = {
    .gpio_step = 8,
    .gpio_dir = 9,
    .gpio_enable = 10,  // Optional, use -1 if not used
    .gpio_hlfb = 11,    // Optional, use -1 if not used
    .vel_max = 1000,    // Maximum velocity in steps/second
    .accel_max = 10000, // Maximum acceleration in steps/second²
    .hlfb_mode = CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE,
    .hlfb_active_high = true
};

clearpath_servo_handle_t *servo;
esp_err_t ret = clearpath_servo_init(&config, &servo);
```

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

## Step Generation

**Note**: This driver component provides the control logic and state management. Actual step pulse generation is performed by the `clearpath_servo_manager` component's background task using the ESP32-P4 RMT (Remote Control) hardware peripheral.

The manager component handles:
- Step pulse generation via RMT hardware peripheral (precise timing, typically 10 microsecond pulse width)
- Background task coordinates step timing (default 5 kHz, configurable)
- Trapezoidal velocity profiles (acceleration → constant velocity → deceleration)
- Position tracking and step counting
- Integration with EtherNet/IP assembly data

**RMT Hardware Step Generation**: This implementation uses the ESP32-P4 RMT peripheral for step pulse generation, providing precise timing independent of task scheduling. RMT hardware enables accurate pulse widths and higher step rates compared to software GPIO toggling. The RMT clock divider is configured for 1 MHz resolution (1 microsecond per tick), allowing precise control of step pulse timing.

## HLFB Modes

HLFB (High Level Feedback) can be configured in ClearPath MSP software to report different status conditions:

- **CLEARPATH_SERVO_HLFB_MODE_ENABLED**: HLFB indicates enabled state
- **CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE**: HLFB indicates move complete
- **CLEARPATH_SERVO_HLFB_MODE_IN_POSITION**: HLFB indicates in position
- **CLEARPATH_SERVO_HLFB_MODE_FAULT**: HLFB indicates fault condition
- **CLEARPATH_SERVO_HLFB_MODE_CUSTOM**: User-defined HLFB behavior

The HLFB mode should match the configuration set in Teknic Motion Studio (MSP) software.

## Integration Notes

This component is designed to work with `clearpath_servo_manager` which handles:
- Background step generation task
- Assembly data integration (EtherNet/IP)
- Multi-servo management
- Configuration management (NVS)
- Status updates

See `components/clearpath_servo_manager/README.md` for integration details.

## Limitations

- Position counter is 32-bit signed integer (±2,147,483,647 steps)
- Step generation uses RMT hardware peripheral for precise pulse timing
- Step pulse width: 10 microseconds (configurable via RMT)
- Step generation frequency depends on FreeRTOS task scheduling (default 5 kHz)
- **Maximum step rate**: Limited by task frequency and RMT configuration. RMT hardware enables higher rates than software GPIO toggling
- Acceleration/deceleration profiles are simplified (trapezoidal)
- RMT hardware provides accurate pulse timing independent of CPU load

## Future Enhancements

- Hardware timer-based step generation for higher precision
- S-curve acceleration profiles
- Position capture on HLFB events
- Limit switch support
- Homing routines
- Multi-axis coordinated motion

## Attribution

This implementation is inspired by and follows the API design patterns of the Teknic ClearCore library. The API function names and concepts (Move, MoveVelocity, VelMax, AccelMax, StepsComplete, HLFB) are based on the ClearCore library API for compatibility and familiarity with Teknic's established interface.

**References:**
- [Teknic ClearCore Library](https://github.com/Teknic-Inc/ClearCore-library) - API design inspiration
- [ClearCore Library Documentation](https://teknic-inc.github.io/ClearCore-library/) - API reference
- [ClearCore Step and Direction Control](https://teknic-inc.github.io/ClearCore-library/_move_gen.html) - Step generation concepts
- [ClearPath Servo User Manual](https://www.teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf) - Hardware specifications
- ESP32-P4 RMT (Remote Control) API documentation - Hardware step pulse generation
- ESP32-P4 GPIO API documentation
- FreeRTOS task and mutex documentation
