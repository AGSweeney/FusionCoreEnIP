# ClearPath Servo Manager Component

**Status: Scaffolded - NOT included in build**

This component manages multiple ClearPath servos, generates step pulses, monitors HLFB feedback, and integrates with EtherNet/IP assembly data. It integrates with the `clearpath_servo` driver component.

## Features

- **Multi-Servo Support**: Manages up to 4 ClearPath servos
- **Step Generation**: Background task generates step pulses (default 5 kHz, configurable)
- **Velocity Profiles**: Trapezoidal acceleration/deceleration profiles
- **HLFB Monitoring**: Reads High Level Feedback for status information
- **Assembly Integration**: Reads commands from Output Assembly 150, writes feedback to Input Assembly 100
- **Configuration Management**: NVS storage for GPIO pins and settings
- **Thread-Safe**: Mutex protection for assembly data access

## Architecture

### Data Flow

```
Output Assembly 150 → Manager Task → Step Generation → GPIO → ClearPath Servo
ClearPath Servo → HLFB GPIO → Manager Task → Status → Input Assembly 100
```

### Component Structure

1. **Background Task**: Background task generates step pulses (default 5 kHz, configurable)
2. **Step Generation**: Trapezoidal velocity profiles with acceleration/deceleration
3. **Position Tracking**: Updates position counter based on step count
4. **HLFB Monitoring**: Reads HLFB GPIO for status feedback
5. **Assembly Updates**: Reads commands from Output Assembly 150, writes status to Input Assembly 100

## Assembly Data Layout

### Output Assembly 150 (Control Commands)

**Bytes 32-51: Servo Commands (20 bytes total, 5 bytes per servo, supports 4 servos)**

| Byte Range | Servo | Field | Type | Description |
|------------|-------|-------|------|-------------|
| 32 | Servo 0 | Command Type | uint8_t | Command type (see below) |
| 33-36 | Servo 0 | Value | int32_t | Position (move_abs/move_rel) or velocity (move_velocity) |
| 37 | Servo 1 | Command Type | uint8_t | Command type |
| 38-41 | Servo 1 | Value | int32_t | Position or velocity |
| 42 | Servo 2 | Command Type | uint8_t | Command type |
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
- 6: Home - Value format: lower 16 bits = direction (int16_t, positive=forward, negative=reverse), upper 16 bits = timeout_ms (uint16_t, 0=no timeout)

**Note**: All values are little-endian int32_t, providing full 32-bit range for positions and velocities (±2,147,483,647 steps or steps/second).

**Assembly Size Note**: Output Assembly 150 is currently 40 bytes (`g_assembly_data096[40]`), which supports 1 servo fully (bytes 32-36). For 4 servos with full 32-bit range, the assembly needs to be expanded to at least 52 bytes (bytes 32-51). The code includes bounds checking to prevent reading beyond the current assembly size.

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

**Note**: For multiple servos, Input Assembly 100 may need to be expanded beyond 72 bytes to support all servos.

## Hardware Connections

### Hookup Schematic

**Single Servo Connection:**

```
ESP32                    SN74AHCT14 (Level Shifter)          ClearPath Servo
------                   -------------------------            ---------------
GPIO_X (Step)    ------> 1A (Input)                           Step+
                         |                                    |
                         | (inverted)                         |
                         v                                    |
                         1Y (Output) ------------------------> Step-
                         
GPIO_Y (Dir)     ------> 2A (Input)                           Dir+
                         |                                    |
                         | (inverted)                         |
                         v                                    |
                         2Y (Output) ------------------------> Dir-
                         
GPIO_Z (Enable)  ------> 3A (Input)                           Enable+
                         |                                    |
                         | (inverted)                         |
                         v                                    |
                         3Y (Output) ------------------------> Enable-
                         
                         
ClearPath Servo          Level Shifter (5V→3.3V)              ESP32
---------------          -------------------------            ------
HLFB+            ------> Input (5V)                           GPIO_W (HLFB)
                         |                                    |
                         | (level shifted)                    |
                         v                                    |
                         Output (3.3V) ----------------------> 
                         
HLFB-            ------> GND                                  GND
                         
GND              ------> GND (Pin 7)                          GND
                         VCC (Pin 14) <--- 5V Power Supply
                         
3.3V             ------> VCC (Pin 1) - Optional, if needed
```

### Power Connections

- ESP32-P4: 3.3V logic supply
- SN74AHCT14: VCC (Pin 14) = 5V, GND (Pin 7) = Ground
- ClearPath Servo: Requires appropriate motor power supply (typically 24-75V DC)

### Pin Mapping

- SN74AHCT14 is an inverting Schmitt trigger, so logic is inverted
- If non-inverting level shifter is used (e.g., SN74AHCT125), connect directly without inversion
- Each servo requires:
  - 3-4 GPIO outputs: Step, Direction, Enable (optional)
  - 1-2 GPIO inputs: HLFB (High Level Feedback), Homing Sensor (optional)
- For 4 servos: 12-16 GPIO outputs + 4-8 GPIO inputs = 16-24 GPIO pins total
- Enable pin is optional - can be tied high if not used
- HLFB is highly recommended for status monitoring and move completion detection
- Homing sensor is optional but required for homing operations

### HLFB Connection Options

1. **Direct Connection (if GPIO is 5V tolerant)**:
   - HLFB+ → ESP32 GPIO (with pull-down resistor recommended)
   - HLFB- → GND
   - Verify ESP32-P4 GPIO is 5V tolerant for selected pin

2. **Level Shifter (Recommended)**:
   - Use SN74LVC1T45, TXB0104, or similar bidirectional level shifter
   - Or use simple resistor divider (10kΩ + 20kΩ) for 5V→3.3V conversion
   - Provides protection and reliable signal levels

3. **HLFB Configuration**:
   - Configure HLFB mode via Teknic Motion Studio (MSP) software
   - Common modes: Enabled, Move Complete, In Position, Fault
   - HLFB can be configured as active-high or active-low
   - See `components/clearpath_servo/README.md` for detailed HLFB mode descriptions

## Configuration

### Default Configuration

```c
// Servo 0 default configuration
gpio_step = 8
gpio_dir = 9
gpio_enable = 10
gpio_hlfb = 11
gpio_homing_sensor = -1  // Not configured by default
vel_max = 1000 steps/second
accel_max = 10000 steps/second²
hlfb_mode = CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE
hlfb_active_high = true
homing_sensor_type = CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN
homing_velocity = 0  // 0 = use vel_max
```

### Configuration via NVS

Configuration is stored in NVS and can be loaded via `system_config` component. GPIO pins and servo parameters can be configured per servo instance.

## API Reference

### Initialization

```c
#include "clearpath_servo_manager.h"

esp_err_t ret = clearpath_servo_manager_init();
```

### Getting Configuration

```c
clearpath_servo_manager_config_t config;
esp_err_t ret = clearpath_servo_manager_get_config(0, &config);
```

### Reading Status

```c
// Get position
int32_t position = clearpath_servo_manager_get_position(0);

// Get velocity
int32_t velocity = clearpath_servo_manager_get_velocity(0);

// Get status flags
uint8_t status = clearpath_servo_manager_get_status(0);
bool move_complete = (status & 0x01) != 0;
bool enabled = (status & 0x02) != 0;
bool fault = (status & 0x04) != 0;
bool in_position = (status & 0x08) != 0;

// Get number of initialized servos
uint8_t count = clearpath_servo_manager_get_count();

// Homing operations
// Start homing move (forward direction, 5 second timeout)
clearpath_servo_manager_home(0, 1, 5000);

// Check if homing is complete
bool homing_done = clearpath_servo_manager_is_homing_complete(0);

// Check homing sensor status
bool sensor_triggered = clearpath_servo_manager_get_homing_sensor_status(0);
```

## Step Generation

This implementation uses the ESP32-P4 RMT (Remote Control) hardware peripheral for step pulse generation, providing precise timing independent of task scheduling.

### RMT Hardware Configuration

- **Clock divider**: 80 (1 MHz resolution, 1 microsecond per tick)
- **Step pulse width**: 10 microseconds (configurable)
- **Precise hardware timing**: Accurate pulse generation independent of CPU load

### Step Generation Task

The manager includes a background task (default 5 kHz, configurable) that:

1. Reads commands from Output Assembly 150
2. Generates step pulses using RMT hardware based on velocity/acceleration profiles
3. Updates position tracking
4. Monitors HLFB for status feedback
5. Writes status/feedback to Input Assembly 100

**Step Rate**: Limited by task frequency (default 5 kHz) and RMT configuration. RMT hardware enables much higher rates than software GPIO toggling.

### Velocity Profiles

Step generation uses trapezoidal velocity profiles with integer-only calculations:
- **Acceleration Phase**: Velocity ramps from 0 to target velocity using integer math
- **Constant Velocity Phase**: Maintains target velocity
- **Deceleration Phase**: Velocity ramps from target to 0, calculating deceleration distance as `v²/(2a)`

Acceleration and deceleration rates are controlled by `accel_max` configuration. All calculations use 64-bit integer intermediates to prevent overflow, then convert to 32-bit results. This provides precise motion control without floating-point operations, suitable for embedded systems.

## Homing

The component supports homing operations to establish a reference position (zero point) using a homing sensor.

### Homing Sensor Configuration

- **GPIO Pin**: Configure `gpio_homing_sensor` in servo configuration (-1 if not used)
- **Sensor Type**: 
  - `CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_OPEN` (NO): Sensor triggers when GPIO goes HIGH (closed)
  - `CLEARPATH_SERVO_HOMING_SENSOR_NORMALLY_CLOSED` (NC): Sensor triggers when GPIO goes LOW (opened)
- **Homing Velocity**: Configure `homing_velocity` (0 = use `vel_max`)

### Homing Process

1. Call `clearpath_servo_home()` or `clearpath_servo_manager_home()` with direction and timeout
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

Example: Home forward with 5 second timeout
- Value = (5000 << 16) | 1 = 0x00050001

### Sensor Wiring

Homing sensors typically require level shifting (5V→3.3V) similar to HLFB:
- NO sensor: Usually open circuit, closes (goes HIGH) when triggered
- NC sensor: Usually closed circuit, opens (goes LOW) when triggered
- Pull-up/pull-down resistors configured automatically based on sensor type

## Integration Notes

### Enabling the Component

To enable this component:

1. Ensure `clearpath_servo` driver component is available
2. Add component to build system (remove from .gitignore if needed)
3. Call `clearpath_servo_manager_init()` in `main.c` startup sequence
4. Configure GPIO pins via `system_config` or NVS
5. Update EDS file if assembly sizes change

### Dependencies

- `clearpath_servo` - Driver component
- `opener` - EtherNet/IP assembly access
- `system_config` - Configuration management
- `driver` - ESP-IDF GPIO driver (ESP32-P4)
- `freertos` - FreeRTOS tasks and synchronization

## Limitations

- **Step Generation**: Uses RMT hardware peripheral for precise timing
- **Step Rate**: Limited by task frequency (default 5 kHz) and RMT configuration
- **Position Range**: 32-bit signed integer (±2,147,483,647 steps)
- **Assembly Size**: Output Assembly 150 is currently 40 bytes, supporting 1 servo fully (bytes 32-36). For 4 servos, assembly needs expansion to at least 52 bytes (bytes 32-51)
- **Velocity Profiles**: Trapezoidal only (no S-curve acceleration)
- **RMT Channels**: Up to 8 RMT channels available (one per servo)
- **Integer Math**: All calculations use integer arithmetic (no floating point) for embedded system efficiency

## Future Enhancements

- Variable RMT pulse width configuration
- Higher frequency step generation using optimized RMT settings
- S-curve acceleration profiles
- Support for all 4 servos in assembly data
- Limit switch support
- Multi-axis coordinated motion
- Position capture on HLFB events
- Homing with index pulse (Z channel) support for encoders

## Attribution

Step generation algorithms, trapezoidal velocity profiles, and servo control concepts are based on patterns from the Teknic ClearCore library. The high-frequency step generation task and velocity/acceleration management follow similar principles to ClearCore's StepGenerator and MotorManager implementations.

**References:**
- [Teknic ClearCore Library](https://github.com/Teknic-Inc/ClearCore-library) - Step generation and motor control concepts
- [ClearCore Library Documentation](https://teknic-inc.github.io/ClearCore-library/) - API reference
- [ClearCore Motor Control](https://teknic-inc.github.io/ClearCore-library/_motor_driver_main.html) - Motor control patterns
- [ClearCore Step Generation](https://teknic-inc.github.io/ClearCore-library/_move_gen.html) - Step and direction control
- [ClearPath Servo User Manual](https://www.teknic.com/files/downloads/Clearpath-SC%20User%20Manual.pdf) - Hardware specifications
- `components/clearpath_servo/README.md` - Driver component documentation
- `docs/ASSEMBLY_DATA_LAYOUT.md` - Assembly data layout documentation
