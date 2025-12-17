# ABZ Encoder Manager Component

**Status: Scaffolded - NOT included in build**

This component manages ABZ encoder GPIO interrupts, processes quadrature signals, calculates velocity, and updates EtherNet/IP assembly data. It integrates with the `abz_encoder` driver component.

## Features

- **GPIO Interrupt Handling**: Configures interrupts on A, B, and Z channels
- **Queue-Based Processing**: ISR sends events to queue for deferred processing
- **Velocity Calculation**: Calculates counts per second from position delta
- **Assembly Data Integration**: Updates EtherNet/IP Input Assembly 100
- **Configuration Management**: NVS storage for GPIO pins and settings
- **Thread-Safe**: Mutex protection for assembly data access

## Architecture

### Data Flow

```
GPIO A/B/Z Signals → ISR Handler → Queue → Task → Quadrature Decoder → Position/Velocity → Assembly Data
```

### Component Structure

1. **GPIO Configuration**: Sets up interrupts on A (both edges), B (both edges), Z (rising edge)
2. **ISR Handler**: Fast interrupt handler that reads GPIO states and queues events
3. **Processing Task**: Dequeues events and updates encoder position
4. **Velocity Calculation**: Periodically calculates velocity from position delta
5. **Assembly Updates**: Writes position, velocity, and status to Input Assembly 100

## Assembly Data Layout

**Input Assembly 100, Bytes 61-71:**

| Byte Range | Field | Type | Description |
|------------|-------|------|-------------|
| 61-64 | Position | int32_t | Encoder position count (little-endian) |
| 65-66 | Velocity | int16_t | Velocity in counts/second (little-endian) |
| 67 | Status | uint8_t | Status flags (bit 0=index, bit 1=direction, bit 2=initialized) |
| 68-71 | Reserved | - | Reserved for future expansion |

**Status Byte (Byte 67) Bits:**
- Bit 0: Index pulse detected (1 = detected, 0 = not detected)
- Bit 1: Direction (1 = forward, 0 = reverse)
- Bit 2: Initialized (1 = initialized, 0 = not initialized)
- Bits 3-7: Reserved

## Configuration

### Default Configuration

```c
#define DEFAULT_GPIO_A           8
#define DEFAULT_GPIO_B           9
#define DEFAULT_GPIO_Z           10
#define DEFAULT_RESOLUTION       ABZ_ENCODER_RESOLUTION_4X
#define DEFAULT_VELOCITY_INTERVAL_MS  100
#define DEFAULT_INDEX_RESET      false
```

### Configuration Structure

```c
typedef struct {
    int gpio_a;                      // GPIO pin for A channel
    int gpio_b;                      // GPIO pin for B channel
    int gpio_z;                      // GPIO pin for Z channel (index)
    abz_encoder_resolution_t resolution; // Resolution mode (1x or 4x)
    uint32_t velocity_interval_ms;   // Velocity calculation interval (ms)
    bool index_reset_position;        // Reset position on index pulse
    bool enabled;                    // Encoder enabled flag
} abz_encoder_config_t;
```

## API Reference

### Initialization

```c
esp_err_t ret = abz_encoder_manager_init();
```

This function:
- Loads configuration from NVS (TODO: implement)
- Configures GPIO interrupts
- Creates event queue
- Creates processing task
- Initializes encoder driver

### Configuration

```c
abz_encoder_config_t config;
abz_encoder_manager_get_config(&config);

config.gpio_a = 8;
config.gpio_b = 9;
config.gpio_z = 10;
config.resolution = ABZ_ENCODER_RESOLUTION_4X;
abz_encoder_manager_set_config(&config);
```

### Reading Encoder Data

```c
int32_t position = abz_encoder_manager_get_position();
int16_t velocity = abz_encoder_manager_get_velocity();
bool index_detected = abz_encoder_manager_get_index_detected();
abz_encoder_direction_t direction = abz_encoder_manager_get_direction();
```

### Position Control

```c
abz_encoder_manager_reset_position();  // Reset position to 0
```

## Integration Steps

To enable this component in the build:

1. **Add to Build System**
   - Ensure `components/abz_encoder_manager/CMakeLists.txt` exists
   - ESP-IDF will auto-discover components in `components/` directory

2. **Initialize in main.c**
   ```c
   #include "abz_encoder_manager.h"
   
   void app_main(void)
   {
       // ... other initialization ...
       
       // Initialize encoder manager
       abz_encoder_manager_init();
       
       // ... rest of initialization ...
   }
   ```

3. **Add Assembly Definitions** (if not already present)
   ```c
   // In fusion_core_assembly.h
   #define ABZ_ENCODER_BYTE_START 61
   #define ABZ_ENCODER_BYTE_END 71
   ```

4. **Add Configuration to system_config**
   - Add encoder configuration structure to `system_config.h`
   - Add NVS save/load functions
   - Add Kconfig options for GPIO pins

5. **Update Assembly Documentation**
   - Update `docs/ASSEMBLY_DATA_LAYOUT.md` with encoder data format

## GPIO Interrupt Configuration

### A Channel (Quadrature)
- **Interrupt Type**: `GPIO_INTR_ANYEDGE` (both rising and falling)
- **Pull-up**: Enabled (for open-collector encoders)
- **ISR**: Fast, queues event for task processing

### B Channel (Quadrature)
- **Interrupt Type**: `GPIO_INTR_ANYEDGE` (both rising and falling)
- **Pull-up**: Enabled
- **ISR**: Fast, queues event for task processing

### Z Channel (Index)
- **Interrupt Type**: `GPIO_INTR_POSEDGE` (rising edge only)
- **Pull-up**: Enabled
- **ISR**: Fast, queues event for task processing

## Performance Considerations

- **Interrupt Rate**: Maximum interrupt rate depends on encoder speed and resolution
  - 4x resolution: 4 interrupts per quadrature cycle
  - Typical encoder: 100-1000 PPR (pulses per revolution)
  - At 1000 RPM with 1000 PPR: ~67k interrupts/second
  
- **Queue Size**: Default 32 events (adjustable)
  - Queue overflow indicates encoder is too fast or task is blocked
  - Monitor queue usage and increase size if needed

- **Task Priority**: Default priority 10 (adjustable)
  - Should be higher than sensor reading tasks
  - Lower than critical system tasks

- **Assembly Update Rate**: Updates every 10ms (configurable)
  - Balances accuracy with CPU usage
  - Velocity calculation uses configurable interval (default 100ms)

## Troubleshooting

### Encoder Not Responding
- Check GPIO pin assignments
- Verify pull-up resistors (internal or external)
- Check encoder power supply
- Verify encoder signal levels (3.3V compatible)

### Position Not Updating
- Check interrupt handler is being called (add logging)
- Verify queue is not overflowing
- Check task is running (monitor task state)
- Verify quadrature signals are connected correctly

### Incorrect Position Counts
- Verify A/B channel connections (may be swapped)
- Check resolution mode matches encoder
- Verify encoder PPR matches expected counts
- Check for electrical noise (add filtering if needed)

### Velocity Calculation Issues
- Verify velocity calculation interval is appropriate
- Check position delta is reasonable
- Monitor for position counter overflow

## Future Enhancements

- [ ] NVS configuration storage
- [ ] Kconfig options for GPIO pins
- [ ] Web UI configuration page
- [ ] REST API endpoints for encoder control
- [ ] Multiple encoder support
- [ ] Encoder calibration and offset support
- [ ] Position wrapping/overflow handling
- [ ] Velocity filtering (moving average)
- [ ] Debouncing configuration
- [ ] Encoder speed limit detection

## Dependencies

- `abz_encoder` driver component
- `fusion_core_assembly` for assembly data access
- ESP-IDF GPIO driver
- FreeRTOS (queue, task, mutex)
- ESP32 timer API for timestamps

## References

- `components/abz_encoder/README.md` - Encoder driver documentation
- `components/vl53l1x_manager/` - Example manager component pattern
- ESP-IDF GPIO Interrupt API documentation
- EtherNet/IP Assembly Data Layout documentation
