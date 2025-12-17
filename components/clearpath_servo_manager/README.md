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

**Bytes 32-39: Servo Commands (8 bytes, supports 4 servos)**

| Byte Range | Servo | Field | Type | Description |
|------------|-------|-------|------|-------------|
| 32-33 | Servo 0 | Command | uint16_t | Command type and data |
| 34-35 | Servo 1 | Command | uint16_t | Command type and data |
| 36-37 | Servo 2 | Command | uint16_t | Command type and data |
| 38-39 | Servo 3 | Command | uint16_t | Command type and data |

**Command Format (2 bytes per servo):**
- Byte 0: Command type (0=stop, 1=move_abs, 2=move_rel, 3=move_vel)
- Byte 1: Reserved/data

**Note**: For more detailed control, the Output Assembly may need to be expanded to support 4 bytes per servo (position/velocity commands).

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
  - 1 GPIO input: HLFB (High Level Feedback)
- For 4 servos: 12-16 GPIO outputs + 4 GPIO inputs = 16-20 GPIO pins total
- Enable pin is optional - can be tied high if not used
- HLFB is highly recommended for status monitoring and move completion detection

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

## Configuration

### Default Configuration

```c
// Servo 0 default configuration
gpio_step = 8
gpio_dir = 9
gpio_enable = 10
gpio_hlfb = 11
vel_max = 1000 steps/second
accel_max = 10000 steps/second²
hlfb_mode = CLEARPATH_SERVO_HLFB_MODE_MOVE_COMPLETE
hlfb_active_high = true
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
```

## Step Generation

**Maximum Step Rate**: This ESP32-P4 implementation uses software timing via FreeRTOS tasks, which limits practical step pulse rates to approximately **1-10 kHz** depending on:
- Task priority and scheduling
- System load from other tasks
- GPIO toggle speed
- CPU frequency

For higher step rates, consider:
- Using ESP32-P4 hardware peripherals (LEDC up to 40 MHz, MCPWM, or RMT) for step generation
- Increasing task priority
- Reducing other system load
- Using dedicated hardware step generation ICs

The manager component includes a background task (default 5 kHz, configurable) that:

1. Reads commands from Output Assembly 150
2. Generates step pulses based on velocity/acceleration profiles
3. Updates position tracking
4. Monitors HLFB for status feedback
5. Writes status/feedback to Input Assembly 100

### Velocity Profiles

Step generation uses trapezoidal velocity profiles:
- **Acceleration Phase**: Velocity ramps from 0 to target velocity
- **Constant Velocity Phase**: Maintains target velocity
- **Deceleration Phase**: Velocity ramps from target to 0

Acceleration and deceleration rates are controlled by `accel_max` configuration.

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

- Step generation uses software timing (no hardware timers)
- Maximum step rate limited by task frequency and GPIO toggle speed
- **Maximum step rate**: ESP32-P4 software timing limits practical rates to approximately **1-10 kHz** depending on system load, task priority, and GPIO toggle speed. ESP32-P4 hardware peripherals (LEDC up to 40 MHz, MCPWM, or RMT) could achieve much higher rates but require different implementation
- Position tracking is 32-bit signed integer (±2,147,483,647 steps)
- Assembly data layout supports 1-2 servos fully (may need expansion for 4 servos)
- Trapezoidal velocity profiles only (no S-curve acceleration)

## Future Enhancements

- ESP32-P4 hardware peripheral-based step generation (LEDC, MCPWM, or RMT) for higher precision and rates up to 40 MHz
- S-curve acceleration profiles
- Support for all 4 servos in assembly data
- Limit switch support
- Homing routines
- Multi-axis coordinated motion
- Position capture on HLFB events

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
