# ABZ Encoder Driver Component

**Status: Scaffolded - NOT included in build**

This component provides quadrature decoding functionality for ABZ rotary encoders using the ESP32-P4 PCNT (Pulse Counter) hardware peripheral. PCNT hardware automatically decodes A/B channel quadrature signals, tracks position, and determines direction without CPU intervention.

## Features

- **Hardware Quadrature Decoding**: PCNT peripheral handles A/B channel decoding automatically
- **Resolution Modes**: Supports 1x and 4x resolution
- **Direction Detection**: Hardware-based direction detection
- **Index Pulse Support**: Detects and handles Z channel index pulses (via GPIO interrupt)
- **Thread-Safe**: Mutex protection for position access
- **Velocity Calculation**: Helper function for calculating counts per second
- **Glitch Filtering**: Built-in hardware filtering for noisy signals
- **High Performance**: No CPU overhead for quadrature decoding

## PCNT Hardware Quadrature Decoding

The ESP32-P4 PCNT (Pulse Counter) peripheral provides hardware-based quadrature decoding:

- **Automatic Decoding**: PCNT hardware decodes A/B quadrature signals without software intervention
- **Direction Detection**: Hardware determines forward/reverse rotation automatically
- **High Speed**: Can handle high-speed encoder signals without CPU load
- **Glitch Filtering**: Built-in hardware filter removes noise and spurious pulses

### Resolution Modes

**1x Resolution:**
- Counts only on A channel edges (half quadrature)
- B channel used for direction detection
- Lower resolution, simpler processing
- PCNT counts all edges, software divides by 4

**4x Resolution:**
- Counts on all A and B channel edges (full quadrature)
- Provides 4x the resolution
- More accurate position tracking
- PCNT hardware counts all edges directly

## API Reference

### Initialization

```c
abz_encoder_t encoder;
// Initialize with PCNT hardware - GPIO pins are configured automatically
esp_err_t ret = abz_encoder_init(&encoder, ABZ_ENCODER_RESOLUTION_4X, GPIO_A, GPIO_B);
```

### Updating Position from PCNT Hardware

```c
// Call periodically (e.g., every 1-10ms) to sync software position with PCNT hardware counter
// PCNT hardware handles all quadrature decoding automatically
abz_encoder_update_position(&encoder);
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
- PCNT hardware configuration (automatic quadrature decoding)
- GPIO interrupt for Z channel (index pulse)
- Periodic task to read PCNT counter and update position
- Assembly data updates
- Configuration management

**Note**: A/B channels are handled entirely by PCNT hardware - no GPIO interrupts needed. Only Z channel (index pulse) uses GPIO interrupt.

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
    // Initialize encoder with PCNT hardware
    // GPIO_A and GPIO_B are configured automatically by PCNT
    esp_err_t ret = abz_encoder_init(&encoder, ABZ_ENCODER_RESOLUTION_4X, GPIO_A, GPIO_B);
    if (ret != ESP_OK) {
        ESP_LOGE("ENCODER", "Failed to initialize encoder");
        return;
    }
}

void encoder_task_example(void *pvParameters)
{
    while (1) {
        // Periodically update position from PCNT hardware counter
        // PCNT hardware handles all quadrature decoding automatically
        abz_encoder_update_position(&encoder);
        
        // Read position and direction
        int32_t pos = abz_encoder_get_position(&encoder);
        abz_encoder_direction_t dir = abz_encoder_get_direction(&encoder);
        
        printf("Position: %ld, Direction: %s\n", 
               pos, 
               (dir == ABZ_ENCODER_DIRECTION_FORWARD) ? "Forward" : "Reverse");
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Update every 10ms
    }
}
```

## Limitations

- Position counter is 32-bit signed integer (±2,147,483,647 counts)
- PCNT hardware counter is 16-bit (±32,767), but software position extends to 32-bit
- Velocity calculation accuracy depends on update interval
- Hardware glitch filter configured for 1 microsecond (configurable)
- Position update frequency depends on task scheduling (typically 1-10ms)

## Future Enhancements

- Support for multiple encoders
- Hardware debouncing configuration
- Position wrapping/overflow handling
- Velocity filtering (moving average)
- Encoder calibration and offset support

## References

- [Quadrature Encoder Basics](https://en.wikipedia.org/wiki/Rotary_encoder#Incremental_encoder)
- [ESP32-P4 PCNT (Pulse Counter) API](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/pcnt.html) - Hardware quadrature decoding
- ESP32-P4 GPIO API documentation
- ESP-IDF PCNT examples
