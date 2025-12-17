# ABZ Encoder Driver Component

**Status: Scaffolded - NOT included in build**

This component provides quadrature decoding functionality for ABZ rotary encoders. It implements a state machine to decode A/B channel quadrature signals and track position, direction, and index pulse detection.

## Features

- **Quadrature Decoding**: 4-state state machine for accurate position tracking
- **Resolution Modes**: Supports 1x and 4x resolution
- **Direction Detection**: Tracks forward/reverse rotation
- **Index Pulse Support**: Detects and handles Z channel index pulses
- **Thread-Safe**: Designed for use with interrupt handlers and tasks
- **Velocity Calculation**: Helper function for calculating counts per second

## Quadrature Decoding

### State Machine

The encoder uses a 4-state state machine to decode quadrature signals:

- **State 0**: A=0, B=0 (00)
- **State 1**: A=0, B=1 (01)
- **State 2**: A=1, B=0 (10)
- **State 3**: A=1, B=1 (11)

### Forward Rotation Sequence

```
00 → 01 → 11 → 10 → 00 (clockwise)
```

### Reverse Rotation Sequence

```
00 → 10 → 11 → 01 → 00 (counter-clockwise)
```

### Resolution Modes

**1x Resolution:**
- Counts only on A channel edges
- B channel used for direction detection
- Lower resolution, simpler processing

**4x Resolution:**
- Counts on all A and B channel edges
- Provides 4x the resolution
- More accurate position tracking

## API Reference

### Initialization

```c
abz_encoder_t encoder;
esp_err_t ret = abz_encoder_init(&encoder, ABZ_ENCODER_RESOLUTION_4X);
```

### Processing Quadrature Signals

```c
// Called from interrupt handler or task when A/B channels change
bool a_state = gpio_get_level(GPIO_A);
bool b_state = gpio_get_level(GPIO_B);
abz_encoder_process_quadrature(&encoder, a_state, b_state);
```

### Processing Index Pulse

```c
// Called when Z channel index pulse detected (rising edge)
abz_encoder_process_index(&encoder, true);  // Reset position to 0
// or
abz_encoder_process_index(&encoder, false); // Preserve position, set flag
```

### Reading Position and Status

```c
int32_t position = abz_encoder_get_position(&encoder);
bool index_detected = abz_encoder_get_index_detected(&encoder);
abz_encoder_direction_t direction = abz_encoder_get_direction(&encoder);
```

### Velocity Calculation

```c
int32_t position_delta = current_position - last_position;
uint32_t time_delta_ms = 100;  // Time between readings
int16_t velocity = abz_encoder_calculate_velocity(position_delta, time_delta_ms);
```

## Integration Notes

This component is designed to work with `abz_encoder_manager` which handles:
- GPIO interrupt configuration
- Interrupt service routine (ISR)
- Task for processing encoder events
- Assembly data updates
- Configuration management

See `components/abz_encoder_manager/README.md` for integration details.

## Hardware Connections

Typical ABZ encoder connections:

| Encoder Pin | ESP32 GPIO | Notes |
|-------------|------------|-------|
| A           | GPIO X      | Quadrature channel A (with pull-up) |
| B           | GPIO Y      | Quadrature channel B (with pull-up) |
| Z           | GPIO Z      | Index pulse (with pull-up) |
| VCC         | 3.3V or 5V | Power supply (check encoder specs) |
| GND         | GND        | Ground |

**Note**: Most encoders use open-collector outputs, so pull-up resistors are required. ESP32 internal pull-ups can be used, or external pull-ups (typically 10kΩ).

## Example Usage

```c
#include "abz_encoder.h"

abz_encoder_t encoder;

void encoder_init_example(void)
{
    // Initialize encoder with 4x resolution
    abz_encoder_init(&encoder, ABZ_ENCODER_RESOLUTION_4X);
}

void encoder_isr_example(void)
{
    // Read GPIO states
    bool a = gpio_get_level(GPIO_A);
    bool b = gpio_get_level(GPIO_B);
    
    // Process quadrature signal
    abz_encoder_process_quadrature(&encoder, a, b);
}

void encoder_read_example(void)
{
    int32_t pos = abz_encoder_get_position(&encoder);
    abz_encoder_direction_t dir = abz_encoder_get_direction(&encoder);
    
    printf("Position: %ld, Direction: %s\n", 
           pos, 
           (dir == ABZ_ENCODER_DIRECTION_FORWARD) ? "Forward" : "Reverse");
}
```

## Limitations

- Position counter is 32-bit signed integer (±2,147,483,647 counts)
- Maximum interrupt rate depends on ESP32 GPIO interrupt handling capability
- Velocity calculation accuracy depends on update interval
- No built-in debouncing (should be handled in hardware or ISR)

## Future Enhancements

- Support for multiple encoders
- Hardware debouncing configuration
- Position wrapping/overflow handling
- Velocity filtering (moving average)
- Encoder calibration and offset support

## References

- [Quadrature Encoder Basics](https://en.wikipedia.org/wiki/Rotary_encoder#Incremental_encoder)
- ESP32 GPIO Interrupt API documentation
- Existing encoder implementations in ESP-IDF examples
