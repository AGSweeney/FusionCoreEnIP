# PCF8575 I2C I/O Expander Driver

**Status: Scaffolded - NOT included in build**

This component provides a full-featured driver for the PCF8575 16-bit I2C I/O expander from Texas Instruments (formerly NXP).

## Features

- **16 I/O Pins**: Quasi-bidirectional I/O (P0-P15)
- **Input/Output Configuration**: Per-pin configuration
- **Read/Write Operations**: Port-wide and pin-level operations
- **State Caching**: Cached port state for read-modify-write operations
- **Interrupt Support**: Configuration for INT pin (hardware interrupt)
- **Thread-Safe**: Designed for use with mutex protection in manager layer
- **Error Handling**: Retry logic with exponential backoff

## PCF8575 Overview

The PCF8575 is a 16-bit I/O expander that provides additional GPIO pins via I2C:

- **I2C Address**: Base address 0x20, configurable via A0/A1/A2 pins (0x20-0x27)
- **I/O Pins**: 16 quasi-bidirectional pins (P0-P15)
- **Interrupt**: INT pin for input change detection
- **Quasi-Bidirectional**: 
  - Writing 1 = Input mode (high-impedance with weak pull-up)
  - Writing 0 = Output mode (drives low)
  - Reading always reads current pin state

## Quasi-Bidirectional I/O Behavior

The PCF8575 uses quasi-bidirectional I/O, which is different from standard GPIO:

1. **Input Mode**: Write 1 to a pin
   - Pin becomes high-impedance input
   - Weak internal pull-up resistor (~100μA)
   - Can be driven high or low externally

2. **Output Low**: Write 0 to a pin
   - Pin drives low (sinks current)
   - Strong low drive capability

3. **Output High**: Write 1 to a pin (same as input)
   - Pin becomes input with pull-up
   - For true high output, use external pull-up or rely on weak pull-up
   - Limited current sourcing capability (~100μA)

**Important Notes**:
- To drive a load high, use external pull-up resistor
- The weak pull-up is sufficient for logic-level signals
- For LED driving, use external pull-up or current-limiting resistor

## Hardware Connections

### I2C Interface

```
ESP32-P4                    PCF8575
--------                    -------
GPIO_X (SDA)  -----------> SDA
GPIO_Y (SCL)  -----------> SCL
3.3V          -----------> VCC
GND           -----------> GND
```

### Address Configuration

PCF8575 I2C address is set by A0, A1, A2 pins:

| A2 | A1 | A0 | Address |
|----|----|----|---------|
| 0  | 0  | 0  | 0x20    |
| 0  | 0  | 1  | 0x21    |
| 0  | 1  | 0  | 0x22    |
| 0  | 1  | 1  | 0x23    |
| 1  | 0  | 0  | 0x24    |
| 1  | 0  | 1  | 0x25    |
| 1  | 1  | 0  | 0x26    |
| 1  | 1  | 1  | 0x27    |

### Interrupt Pin (Optional)

```
PCF8575                    ESP32-P4
-------                    --------
INT   -------------------> GPIO_Z (with pull-up)
```

- INT goes low when any input pin changes state
- Requires pull-up resistor (internal or external)
- Can be used for interrupt-driven input monitoring

## API Reference

### Initialization

```c
#include "pcf8575.h"
#include "i2c_bus_manager.h"

// Get I2C bus handle
i2c_master_bus_handle_t bus_handle = i2c_bus_manager_get_primary_bus();

// Create I2C device handle
i2c_device_config_t dev_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x20,  // PCF8575 address
    .scl_speed_hz = 400000,
};

i2c_master_dev_handle_t i2c_dev;
esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, &i2c_dev);

// Initialize PCF8575
pcf8575_t pcf8575;
pcf8575_config_t config = {
    .initial_port_state = 0xFFFF,  // All inputs (default)
    .interrupt_gpio = -1,          // No interrupt pin
    .interrupt_active_low = true,
};

if (pcf8575_init(&pcf8575, i2c_dev, &config)) {
    ESP_LOGI(TAG, "PCF8575 initialized");
} else {
    ESP_LOGE(TAG, "Failed to initialize PCF8575");
}
```

### Port Operations

```c
// Read all 16 pins
uint16_t port_value;
pcf8575_read_port(&pcf8575, &port_value);

// Write all 16 pins
pcf8575_write_port(&pcf8575, 0x00FF);  // P0-P7 = outputs low, P8-P15 = inputs
```

### Pin Operations

```c
// Read a single pin
bool pin_level;
pcf8575_read_pin(&pcf8575, 0, &pin_level);

// Write a single pin
pcf8575_write_pin(&pcf8575, 0, false);  // Set pin 0 as output low
pcf8575_write_pin(&pcf8575, 0, true);  // Set pin 0 as input (with pull-up)

// Toggle a pin
pcf8575_toggle_pin(&pcf8575, 0);
```

### Configuration

```c
// Set pin as input
pcf8575_set_pin_input(&pcf8575, 0);

// Set pin as output
pcf8575_set_pin_output(&pcf8575, 0, false);  // Output low
pcf8575_set_pin_output(&pcf8575, 0, true);   // Input with pull-up (quasi-bidirectional)

// Configure multiple pins
pcf8575_set_pins_input(&pcf8575, 0xFF00);   // P8-P15 as inputs
pcf8575_set_pins_output(&pcf8575, 0x00FF, 0x0000);  // P0-P7 as outputs (all low)
```

### Update Operations

```c
// Update specific pins (read-modify-write)
pcf8575_update_pins(&pcf8575, 0x000F, 0x0005);  // Update P0-P3, set P0 and P2 high
```

### State Management

```c
// Get cached port state (last read/write value)
uint16_t cached = pcf8575_get_cached_state(&pcf8575);

// Check if initialized
if (pcf8575_is_initialized(&pcf8575)) {
    // Device is ready
}
```

## Usage Examples

### Example 1: Basic Input/Output

```c
// Configure P0-P7 as outputs, P8-P15 as inputs
pcf8575_write_port(&pcf8575, 0xFF00);  // P0-P7 = 0 (outputs), P8-P15 = 1 (inputs)

// Set output pins
pcf8575_write_pin(&pcf8575, 0, false);  // P0 = output low
pcf8575_write_pin(&pcf8575, 1, true);   // P1 = input (with pull-up)

// Read input pins
bool p8_level;
pcf8575_read_pin(&pcf8575, 8, &p8_level);
```

### Example 2: LED Control

```c
// Configure P0-P7 as outputs for LEDs (with external pull-ups for high drive)
pcf8575_set_pins_output(&pcf8575, 0x00FF, 0x0000);  // All LEDs off initially

// Turn on LED on P0 (requires external pull-up for high drive)
pcf8575_write_pin(&pcf8575, 0, true);   // Input with pull-up (LED on via pull-up)
pcf8575_write_pin(&pcf8575, 0, false); // Output low (LED off)

// Or use external pull-up and drive low to turn on
// (LED cathode to P0, anode to VCC via resistor)
pcf8575_write_pin(&pcf8575, 0, false); // LED on (sinks current)
pcf8575_write_pin(&pcf8575, 0, true);  // LED off (high-impedance)
```

### Example 3: Button Reading

```c
// Configure P8-P15 as inputs for buttons
pcf8575_set_pins_input(&pcf8575, 0xFF00);

// Read button on P8
bool button_pressed;
pcf8575_read_pin(&pcf8575, 8, &button_pressed);
if (!button_pressed) {  // Assuming active-low button
    ESP_LOGI(TAG, "Button pressed");
}
```

### Example 4: Read-Modify-Write

```c
// Update only specific pins without affecting others
uint16_t mask = 0x000F;      // Update P0-P3
uint16_t value = 0x0005;     // Set P0 and P2 high, P1 and P3 low
pcf8575_update_pins(&pcf8575, mask, value);
```

## I2C Communication

### Write Operation

The PCF8575 expects 2 bytes when writing:
- **Byte 0**: P0-P7 state (LSB)
- **Byte 1**: P8-P15 state (MSB)

### Read Operation

The PCF8575 returns 2 bytes when reading:
- **Byte 0**: P0-P7 state (LSB)
- **Byte 1**: P8-P15 state (MSB)

### Timing

- **I2C Speed**: Up to 400 kHz (standard mode)
- **Timeout**: 100ms default
- **Retry Logic**: 3 retries with exponential backoff

## Interrupt Support

The PCF8575 has an INT pin that goes low when any input pin changes state. To use interrupts:

1. Connect INT pin to ESP32 GPIO (with pull-up)
2. Configure GPIO interrupt in your application
3. When interrupt occurs, read port to get new state
4. INT clears after reading the port

**Note**: This driver does not implement interrupt handling - that should be done in a manager component.

## Limitations

1. **Quasi-Bidirectional**: Cannot drive high strongly (limited to ~100μA)
2. **No Direction Register**: Direction is implied by read/write operations
3. **No Pull-Up Configuration**: Pull-ups are always enabled for inputs
4. **No Polarity Inversion**: Input polarity cannot be inverted
5. **No Interrupt Masking**: INT pin triggers on any input change
6. **State Caching**: Cached state may become stale if pins are changed externally

## Thread Safety

This driver is **not thread-safe** by itself. For multi-threaded access:
- Use a mutex in the manager layer
- Or create separate device handles for different threads
- Cache state is not protected - use external synchronization

## Error Handling

The driver includes:
- Retry logic with exponential backoff (3 retries)
- Timeout handling (100ms default)
- Input validation (pin numbers, NULL pointers)
- Error logging via ESP_LOG

## Comparison with MCP23017

| Feature | PCF8575 | MCP23017 |
|---------|---------|----------|
| I/O Pins | 16 | 16 |
| Direction Control | Quasi-bidirectional | Dedicated direction register |
| Pull-Ups | Always on (inputs) | Configurable per pin |
| Interrupt | Simple (any change) | Configurable (mask, polarity, etc.) |
| Registers | 2 bytes (simple) | Multiple registers (complex) |
| Cost | Lower | Higher |
| Features | Basic | Advanced |

**Use PCF8575 when**:
- Simple I/O expansion needed
- Cost is a concern
- Quasi-bidirectional behavior is acceptable

**Use MCP23017 when**:
- Need true bidirectional I/O
- Need configurable pull-ups
- Need advanced interrupt features
- Need polarity inversion

## Integration Notes

### Adding to Build

This component is scaffolded and **NOT included in build** by default. To enable:

1. Remove from `.gitignore` if needed
2. Component will be automatically discovered by ESP-IDF
3. Include header: `#include "pcf8575.h"`

### Dependencies

- `esp_driver_i2c` - ESP-IDF I2C master driver
- I2C bus must be initialized before use

### Manager Component

A manager component should be created to:
- Handle multiple PCF8575 devices
- Provide thread-safe access
- Integrate with EtherNet/IP assembly data
- Handle interrupt GPIO configuration
- Provide configuration via NVS

## Future Enhancements

- Interrupt handling implementation
- Manager component for multi-device support
- EtherNet/IP assembly integration
- NVS configuration storage
- Web UI integration

## References

- [PCF8575 Datasheet](https://www.ti.com/lit/ds/symlink/pcf8575.pdf) - Texas Instruments
- [PCF8575 Application Note](https://www.ti.com/lit/an/slva689/slva689.pdf) - Quasi-bidirectional I/O explanation
