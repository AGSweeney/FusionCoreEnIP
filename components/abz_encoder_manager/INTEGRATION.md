# ABZ Encoder Manager Integration Guide

**Status: Scaffolded - NOT included in build**

This document describes how to integrate the ABZ encoder manager into the FusionCoreEnIP project.

## Prerequisites

- ABZ encoder hardware connected to ESP32 GPIO pins
- Encoder uses 3.3V or 5V logic levels (check encoder specifications)
- Pull-up resistors configured (internal or external)

## Integration Steps

### 1. Enable Components in Build

The components are currently scaffolded but not included in the build. To enable:

**Option A: Auto-discovery (Recommended)**
- ESP-IDF automatically discovers components in `components/` directory
- Ensure `CMakeLists.txt` files exist in both `abz_encoder/` and `abz_encoder_manager/`
- Build system will include them automatically

**Option B: Explicit inclusion**
- Add to main `CMakeLists.txt` if needed:
  ```cmake
  # Components are auto-discovered, but you can explicitly require them:
  # idf_build_set_property(COMPONENT_REQUIRES abz_encoder_manager)
  ```

### 2. Add Assembly Definitions

Add encoder byte offset definitions to `fusion_core_assembly.h`:

```c
// In components/opener/src/ports/ESP32/fusion_core/include/fusion_core_assembly.h

#define ABZ_ENCODER_BYTE_START 61
#define ABZ_ENCODER_BYTE_END 71
```

### 3. Initialize in main.c

Add encoder manager initialization to `main/main.c`:

```c
#include "abz_encoder_manager.h"

void app_main(void)
{
    // ... existing initialization ...
    
    // Initialize encoder manager
    ESP_LOGI(TAG, "Initializing ABZ encoder manager...");
    esp_err_t ret = abz_encoder_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize encoder manager: %s", esp_err_to_name(ret));
    }
    
    // ... rest of initialization ...
}
```

### 4. Add Configuration to system_config

Add encoder configuration to `components/system_config/`:

**In `system_config.h`:**
```c
typedef struct {
    int gpio_a;
    int gpio_b;
    int gpio_z;
    abz_encoder_resolution_t resolution;
    uint32_t velocity_interval_ms;
    bool index_reset_position;
    bool enabled;
} system_abz_encoder_config_t;

bool system_abz_encoder_config_load(system_abz_encoder_config_t *config);
bool system_abz_encoder_config_save(const system_abz_encoder_config_t *config);
bool system_abz_encoder_enabled_load(void);
```

**In `system_config.c`:**
```c
// Add NVS namespace and key definitions
#define NVS_NAMESPACE_ABZ_ENCODER "abz_enc"
#define NVS_KEY_CONFIG "config"
#define NVS_KEY_ENABLED "enabled"

// Implement load/save functions
bool system_abz_encoder_config_load(system_abz_encoder_config_t *config)
{
    // Load from NVS
    // Return default values if not found
}

bool system_abz_encoder_config_save(const system_abz_encoder_config_t *config)
{
    // Save to NVS
}

bool system_abz_encoder_enabled_load(void)
{
    // Load enabled flag from NVS
    // Default to false if not found
}
```

### 5. Add Kconfig Options

Add Kconfig options for encoder GPIO pins in `main/Kconfig.projbuild`:

```kconfig
menu "ABZ Encoder Configuration"
    config ABZ_ENCODER_ENABLED
        bool "Enable ABZ encoder"
        default n
        help
            Enable ABZ rotary encoder support
    
    config ABZ_ENCODER_GPIO_A
        int "GPIO pin for encoder A channel"
        depends on ABZ_ENCODER_ENABLED
        default 8
        help
            GPIO pin number for encoder A channel (quadrature)
    
    config ABZ_ENCODER_GPIO_B
        int "GPIO pin for encoder B channel"
        depends on ABZ_ENCODER_ENABLED
        default 9
        help
            GPIO pin number for encoder B channel (quadrature)
    
    config ABZ_ENCODER_GPIO_Z
        int "GPIO pin for encoder Z channel (index)"
        depends on ABZ_ENCODER_ENABLED
        default 10
        help
            GPIO pin number for encoder Z channel (index pulse)
    
    choice ABZ_ENCODER_RESOLUTION
        prompt "Encoder resolution mode"
        depends on ABZ_ENCODER_ENABLED
        default ABZ_ENCODER_RESOLUTION_4X
        help
            Select encoder resolution mode
            
        config ABZ_ENCODER_RESOLUTION_1X
            bool "1x resolution"
            help
                Count only on A channel edges (lower resolution)
        
        config ABZ_ENCODER_RESOLUTION_4X
            bool "4x resolution"
            help
                Count on all A and B channel edges (higher resolution)
    endchoice
    
    config ABZ_ENCODER_VELOCITY_INTERVAL_MS
        int "Velocity calculation interval (ms)"
        depends on ABZ_ENCODER_ENABLED
        default 100
        range 10 1000
        help
            Interval for calculating velocity from position delta
endmenu
```

### 6. Update Assembly Documentation

Update `docs/ASSEMBLY_DATA_LAYOUT.md` to document encoder data:

```markdown
### ABZ Encoder (Bytes 61-71)

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 61-64 | Position | int32_t | Encoder position count (little-endian) |
| 65-66 | Velocity | int16_t | Velocity in counts/second (little-endian) |
| 67 | Status | uint8_t | Status flags (bit 0=index, bit 1=direction, bit 2=initialized) |
| 68-71 | Reserved | - | Reserved bytes (unused) |

**Notes:**
- Position is a signed 32-bit integer (±2,147,483,647 counts)
- Velocity is calculated from position delta over time interval
- Status byte bits:
  - Bit 0: Index pulse detected (1 = detected, 0 = not detected)
  - Bit 1: Direction (1 = forward, 0 = reverse)
  - Bit 2: Initialized (1 = initialized, 0 = not initialized)
```

### 7. Update Device Counts (Optional)

If you want to add encoder count to device counts section:

**In `fusion_core_assembly.h`:**
```c
#define DEVICE_COUNT_ABZ_ENCODER 61  // Add encoder count byte
```

**In encoder manager init:**
```c
if (s_initialized) {
    INPUT_ASSEMBLY_100[DEVICE_COUNT_ABZ_ENCODER] = 1;
}
```

## Testing

1. **Hardware Verification**
   - Connect encoder A, B, Z channels to configured GPIO pins
   - Verify encoder power supply
   - Check signal levels with oscilloscope if available

2. **Software Testing**
   - Monitor serial output for initialization messages
   - Rotate encoder and verify position updates
   - Check assembly data via EtherNet/IP or REST API
   - Verify index pulse detection

3. **Performance Testing**
   - Test at various encoder speeds
   - Monitor queue usage (add logging if needed)
   - Verify velocity calculation accuracy
   - Check for position counter overflow

## Troubleshooting

### Build Errors
- Ensure all dependencies are included
- Check CMakeLists.txt syntax
- Verify component paths are correct

### Runtime Errors
- Check GPIO pin assignments
- Verify encoder is powered
- Monitor serial output for error messages
- Check queue is not overflowing

### Data Issues
- Verify assembly byte offsets match documentation
- Check mutex protection is working
- Monitor task execution
- Verify quadrature decoding is correct

## Next Steps

After integration:
1. Test with actual encoder hardware
2. Add web UI configuration page (optional)
3. Add REST API endpoints (optional)
4. Add encoder calibration support (optional)
5. Document encoder specifications and wiring

## References

- `components/abz_encoder/README.md` - Encoder driver documentation
- `components/abz_encoder_manager/README.md` - Manager component documentation
- `docs/ASSEMBLY_DATA_LAYOUT.md` - Assembly data layout documentation
- Existing manager components for integration patterns
