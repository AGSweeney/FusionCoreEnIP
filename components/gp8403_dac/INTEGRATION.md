# GP8403 DAC Integration Guide

This document describes how to integrate the GP8403 DAC driver into the ESP32-P4 EtherNet/IP project.

## Component Structure

```
components/gp8403_dac/
├── CMakeLists.txt          # Component build configuration
├── gp8403_dac.c            # Driver implementation
├── include/
│   └── gp8403_dac.h        # Driver API header
├── README.md               # Driver documentation
└── INTEGRATION.md          # This file
```

## Integration Steps

### 1. Add Component to Build

The component is automatically discovered by ESP-IDF's component system. No changes needed to `CMakeLists.txt` if the component is in `components/` directory.

### 2. Include in Main Application

Add the driver header to your source file:

```c
#include "gp8403_dac.h"
```

### 3. Initialize I2C Bus

The driver requires an initialized I2C master bus. If your project already has an I2C bus initialized (e.g., for IMU sensors), you can reuse it:

```c
// Get existing I2C bus handle (from your I2C initialization code)
extern i2c_master_bus_handle_t s_i2c_bus_handle;
```

Or initialize a new I2C bus:

```c
#include "driver/i2c_master.h"

i2c_master_bus_config_t i2c_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_21,  // Adjust to your pinout
    .scl_io_num = GPIO_NUM_22,   // Adjust to your pinout
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = {
        .enable_internal_pullup = true,
    },
};

i2c_master_bus_handle_t i2c_bus_handle;
esp_err_t err = i2c_master_bus_initialize(&i2c_bus_config, &i2c_bus_handle);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize I2C bus");
    return;
}
```

### 4. Initialize DAC Device

```c
gp8403_dac_config_t dac_config = {
    .bus_handle = i2c_bus_handle,
    .i2c_addr = GP8403_DAC_DEFAULT_I2C_ADDR,  // 0x58, or set via DIP switch
};

gp8403_dac_handle_t *dac_handle = NULL;
if (!gp8403_dac_init(&dac_config, &dac_handle)) {
    ESP_LOGE(TAG, "Failed to initialize GP8403 DAC");
    return;
}
```

### 5. Use DAC in Application

#### Example: Set Voltage from EtherNet/IP Output Assembly

If you want to control the DAC from EtherNet/IP Output Assembly data:

```c
// In your EtherNet/IP output assembly update function
void update_dac_from_assembly(const uint8_t *output_assembly_data)
{
    // Assuming bytes 0-1 are channel 0 voltage (scaled), bytes 2-3 are channel 1
    // Adjust based on your actual assembly layout
    
    // Example: Convert assembly bytes to voltage (0-10V)
    // If using 8-bit values: voltage = (byte / 255.0) * 10.0
    float voltage0 = ((float)output_assembly_data[0] / 255.0f) * 10.0f;
    float voltage1 = ((float)output_assembly_data[1] / 255.0f) * 10.0f;
    
    gp8403_dac_set_channels(dac_handle, voltage0, voltage1);
}
```

#### Example: Periodic Voltage Update

```c
void dac_update_task(void *pvParameters)
{
    gp8403_dac_handle_t *dac = (gp8403_dac_handle_t *)pvParameters;
    
    float voltage = 0.0f;
    bool increasing = true;
    
    while (1) {
        // Ramp voltage from 0V to 10V and back
        if (increasing) {
            voltage += 0.1f;
            if (voltage >= 10.0f) {
                voltage = 10.0f;
                increasing = false;
            }
        } else {
            voltage -= 0.1f;
            if (voltage <= 0.0f) {
                voltage = 0.0f;
                increasing = true;
            }
        }
        
        gp8403_dac_set_both_channels(dac, voltage);
        vTaskDelay(pdMS_TO_TICKS(100));  // Update every 100ms
    }
}
```

## Integration with EtherNet/IP Output Assembly

### Mapping DAC Channels to Assembly Data

Based on the project's `ASSEMBLY_DATA_LAYOUT.md`, the Output Assembly (150) has 32 bytes. You could map DAC channels to unused bytes or add them to the assembly:

**Option 1: Use Reserved Bytes**
- Map Channel 0 to a reserved byte (e.g., byte 30)
- Map Channel 1 to a reserved byte (e.g., byte 31)

**Option 2: Extend Assembly**
- Add DAC control bytes to the assembly definition
- Update EDS file to reflect new assembly size

### Example Integration Code

```c
// In your EtherNet/IP output assembly handler
void handle_output_assembly_150(const uint8_t *assembly_data, size_t length)
{
    if (length < 32) {
        return;
    }
    
    // Extract DAC control values from assembly
    // Example: Use bytes 30-31 for DAC channels (0-255 maps to 0-10V)
    uint8_t dac0_byte = assembly_data[30];
    uint8_t dac1_byte = assembly_data[31];
    
    // Convert to voltage
    float voltage0 = ((float)dac0_byte / 255.0f) * 10.0f;
    float voltage1 = ((float)dac1_byte / 255.0f) * 10.0f;
    
    // Update DAC
    if (dac_handle != NULL && gp8403_dac_is_initialized(dac_handle)) {
        gp8403_dac_set_channels(dac_handle, voltage0, voltage1);
    }
}
```

## Configuration Options

### I2C Address Selection

The module supports 8 I2C addresses via DIP switch (0x58 - 0x5F). Set the address in the configuration:

```c
gp8403_dac_config_t dac_config = {
    .bus_handle = i2c_bus_handle,
    .i2c_addr = 0x5A,  // Set based on DIP switch position
};
```

### Multiple DAC Modules

To use multiple DAC modules on the same I2C bus, initialize each with a different address:

```c
// DAC module 1 (DIP switch: 000 = 0x58)
gp8403_dac_config_t dac1_config = {
    .bus_handle = i2c_bus_handle,
    .i2c_addr = 0x58,
};
gp8403_dac_handle_t *dac1_handle = NULL;
gp8403_dac_init(&dac1_config, &dac1_handle);

// DAC module 2 (DIP switch: 001 = 0x59)
gp8403_dac_config_t dac2_config = {
    .bus_handle = i2c_bus_handle,
    .i2c_addr = 0x59,
};
gp8403_dac_handle_t *dac2_handle = NULL;
gp8403_dac_init(&dac2_config, &dac2_handle);
```

## Thread Safety

The driver is thread-safe when used with ESP-IDF's I2C master driver, which handles I2C bus locking internally. Multiple tasks can safely call DAC functions concurrently.

## Error Handling

Always check return values:

```c
if (!gp8403_dac_set_voltage(dac_handle, GP8403_CHANNEL_0, 5.0f)) {
    ESP_LOGE(TAG, "Failed to set DAC voltage");
    // Handle error
}
```

## Cleanup

When shutting down, deinitialize the DAC:

```c
gp8403_dac_deinit(&dac_handle);  // Sets handle to NULL
```

## Troubleshooting

### Device Not Found

1. **Check I2C Bus**: Verify I2C bus is initialized and working
2. **Verify Address**: Check DIP switch setting matches configured address
3. **Check Wiring**: Verify SDA, SCL, VCC, GND connections
4. **I2C Scanner**: Use an I2C scanner to verify device presence

### Incorrect Output

1. **Register Map**: Verify register addresses match GP8403 datasheet
2. **I2C Communication**: Enable debug logging to see I2C transactions
3. **Power Supply**: Ensure 3.3V or 5V power supply is stable
4. **Voltage Calculation**: Verify voltage-to-DAC conversion formula

### Build Errors

Ensure the component is included in your build:

```cmake
# In your component's CMakeLists.txt
idf_component_register(
    SRCS "your_file.c"
    REQUIRES gp8403_dac  # Add this
)
```

## Testing

### Basic Test

```c
void test_dac_basic(void)
{
    // Initialize DAC
    gp8403_dac_config_t config = {
        .bus_handle = i2c_bus_handle,
        .i2c_addr = GP8403_DAC_DEFAULT_I2C_ADDR,
    };
    gp8403_dac_handle_t *dac = NULL;
    if (!gp8403_dac_init(&config, &dac)) {
        ESP_LOGE(TAG, "DAC init failed");
        return;
    }
    
    // Test channel 0: 0V -> 5V -> 10V -> 0V
    float voltages[] = {0.0f, 5.0f, 10.0f, 0.0f};
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "Setting channel 0 to %.1fV", voltages[i]);
        gp8403_dac_set_voltage(dac, GP8403_CHANNEL_0, voltages[i]);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Test channel 1: ramp from 0V to 10V
    for (float v = 0.0f; v <= 10.0f; v += 0.5f) {
        gp8403_dac_set_voltage(dac, GP8403_CHANNEL_1, v);
        ESP_LOGI(TAG, "Channel 1: %.2fV", v);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Cleanup
    gp8403_dac_deinit(&dac);
}
```

## References

- [Driver README](README.md) - Complete API documentation
- [Product Page](https://www.dfrobot.com/product-2613.html) - Hardware specifications
- [GP8403 Datasheet](https://www.dfrobot.com/) - Chip documentation (check product page)

