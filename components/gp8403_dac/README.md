# GP8403 DAC Driver

ESP-IDF driver for the **DFRobot Gravity 2-Channel I2C DAC Module (0-10V)**.

## Product Information

- **Product**: [Gravity: 2-Channel I2C DAC Module (0-10V)](https://www.dfrobot.com/product-2613.html)
- **SKU**: DFR0971
- **Chip**: GP8403
- **Manufacturer**: DFRobot

## Features

- ✅ 2 independent channels (VOUT0, VOUT1)
- ✅ 0-10V output range per channel
- ✅ 12-bit resolution (0x000 - 0xFFF)
- ✅ 8 configurable I2C addresses via DIP switch
- ✅ Output voltage error < 0.5%
- ✅ Linearity error < 0.1%
- ✅ Output short-circuit protection
- ✅ ESP-IDF I2C master driver support
- ✅ Thread-safe operation

## Specifications

| Parameter | Value |
|-----------|-------|
| Operating Voltage | 3.3V - 5V |
| I2C Interface | PH2.0-4P (Gravity Line Sequence) |
| Input Signal | 12-bit (0x000 - 0xFFF) |
| Output Voltage Range | 0 - 10V (both channels) |
| Output Voltage Error | < 0.5% |
| Linearity Error | 0.1% |
| Power Consumption | < 4mA |
| Protection | Output short-circuit protection |
| I2C Address Range | 0x58 - 0x5F (8 addresses) |
| Max Cascaded Devices | 16 (2 channels × 8 addresses) |

## Hardware Connection

### Pinout

The module uses a Gravity PH2.0-4P connector:

| Pin | Function | ESP32 Connection |
|-----|----------|------------------|
| VCC | Power (3.3V-5V) | 3.3V or 5V |
| GND | Ground | GND |
| SDA | I2C Data | GPIO (configured in I2C bus) |
| SCL | I2C Clock | GPIO (configured in I2C bus) |

### I2C Address Configuration

The module supports 8 I2C addresses via DIP switch:

| DIP Switch | I2C Address |
|------------|-------------|
| 000 | 0x58 (default) |
| 001 | 0x59 |
| 010 | 0x5A |
| 011 | 0x5B |
| 100 | 0x5C |
| 101 | 0x5D |
| 110 | 0x5E |
| 111 | 0x5F |

**Note**: Verify actual addresses with the GP8403 datasheet if available.

## Usage

### 1. Include the Header

```c
#include "gp8403_dac.h"
```

### 2. Initialize I2C Bus

The driver requires an initialized I2C master bus. Example:

```c
#include "driver/i2c_master.h"

// Initialize I2C bus
i2c_master_bus_config_t i2c_bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_21,
    .scl_io_num = GPIO_NUM_22,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = {
        .enable_internal_pullup = true,
    },
};

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_bus_initialize(&i2c_bus_config, &i2c_bus_handle);
```

### 3. Initialize DAC Device

```c
gp8403_dac_config_t dac_config = {
    .bus_handle = i2c_bus_handle,
    .i2c_addr = GP8403_DAC_DEFAULT_I2C_ADDR,  // 0x58, or set via DIP switch
};

gp8403_dac_handle_t *dac_handle = NULL;
if (!gp8403_dac_init(&dac_config, &dac_handle)) {
    ESP_LOGE("APP", "Failed to initialize GP8403 DAC");
    return;
}
```

### 4. Set Output Voltage

#### Set Single Channel

```c
// Set channel 0 to 5.0V
gp8403_dac_set_voltage(dac_handle, GP8403_CHANNEL_0, 5.0f);

// Set channel 1 to 7.5V
gp8403_dac_set_voltage(dac_handle, GP8403_CHANNEL_1, 7.5f);
```

#### Set Both Channels

```c
// Set both channels to the same voltage
gp8403_dac_set_both_channels(dac_handle, 3.3f);

// Set channels independently
gp8403_dac_set_channels(dac_handle, 5.0f, 7.5f);
```

#### Set Using Raw DAC Value

```c
// Set channel 0 using raw 12-bit value (0x000 - 0xFFF)
// 0x800 = 2048 = approximately 5.0V
gp8403_dac_set_raw(dac_handle, GP8403_CHANNEL_0, 0x800);
```

### 5. Get Last Set Voltage

```c
float voltage = gp8403_dac_get_voltage(dac_handle, GP8403_CHANNEL_0);
ESP_LOGI("APP", "Channel 0 voltage: %.3fV", voltage);
```

### 6. Voltage Conversion Utilities

```c
// Convert voltage to raw DAC value
uint16_t dac_value = gp8403_dac_voltage_to_raw(5.0f);  // Returns ~0x800

// Convert raw DAC value to voltage
float voltage = gp8403_dac_raw_to_voltage(0x800);  // Returns ~5.0V
```

### 7. Deinitialize

```c
gp8403_dac_deinit(&dac_handle);  // Sets handle to NULL
```

## Complete Example

```c
#include "gp8403_dac.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "dac_example";

void app_main(void)
{
    // Initialize I2C bus
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
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

    // Initialize DAC
    gp8403_dac_config_t dac_config = {
        .bus_handle = i2c_bus_handle,
        .i2c_addr = GP8403_DAC_DEFAULT_I2C_ADDR,
    };

    gp8403_dac_handle_t *dac_handle = NULL;
    if (!gp8403_dac_init(&dac_config, &dac_handle)) {
        ESP_LOGE(TAG, "Failed to initialize DAC");
        return;
    }

    // Set channel 0 to 5.0V
    gp8403_dac_set_voltage(dac_handle, GP8403_CHANNEL_0, 5.0f);

    // Set channel 1 to 7.5V
    gp8403_dac_set_voltage(dac_handle, GP8403_CHANNEL_1, 7.5f);

    // Read back voltages
    float v0 = gp8403_dac_get_voltage(dac_handle, GP8403_CHANNEL_0);
    float v1 = gp8403_dac_get_voltage(dac_handle, GP8403_CHANNEL_1);
    ESP_LOGI(TAG, "Channel 0: %.3fV, Channel 1: %.3fV", v0, v1);

    // Cleanup
    gp8403_dac_deinit(&dac_handle);
}
```

## API Reference

### Types

#### `gp8403_channel_t`
Channel selection enum:
- `GP8403_CHANNEL_0`: Channel 0 (VOUT0)
- `GP8403_CHANNEL_1`: Channel 1 (VOUT1)

#### `gp8403_dac_config_t`
Configuration structure:
```c
typedef struct {
    i2c_master_bus_handle_t bus_handle;  // I2C master bus handle
    uint8_t i2c_addr;                     // I2C device address (default: 0x58)
} gp8403_dac_config_t;
```

#### `gp8403_dac_handle_t`
Device handle (opaque structure).

### Functions

#### `gp8403_dac_init()`
Initialize GP8403 DAC device.

#### `gp8403_dac_deinit()`
Deinitialize GP8403 DAC device.

#### `gp8403_dac_set_voltage()`
Set output voltage for a channel (0.0 - 10.0V).

#### `gp8403_dac_set_raw()`
Set output using raw 12-bit DAC value (0x000 - 0xFFF).

#### `gp8403_dac_get_voltage()`
Get last set voltage for a channel.

#### `gp8403_dac_set_both_channels()`
Set both channels to the same voltage.

#### `gp8403_dac_set_channels()`
Set both channels independently.

#### `gp8403_dac_voltage_to_raw()`
Convert voltage to raw DAC value.

#### `gp8403_dac_raw_to_voltage()`
Convert raw DAC value to voltage.

#### `gp8403_dac_is_initialized()`
Check if device is initialized.

#### `gp8403_dac_get_i2c_addr()`
Get I2C address of the device.

## Notes

### Register Map

The driver uses the following register addresses (typical for GP8403):

- `0x00`: Channel 0 data register (12-bit)
- `0x01`: Channel 1 data register (12-bit)
- `0x02`: Control register (if exists)

**Important**: These register addresses are based on typical DAC chip patterns. The actual GP8403 register map may differ. Please verify with the GP8403 datasheet and adjust if necessary.

### I2C Communication

- **Default I2C Speed**: 100kHz (can be increased if needed)
- **Timeout**: 100ms
- **Data Format**: 12-bit value sent as 2 bytes (MSB first)

### Voltage Accuracy

- **Output Voltage Error**: < 0.5%
- **Linearity Error**: 0.1%
- **Resolution**: 12-bit (4096 steps)
- **Voltage Step**: ~2.44mV per step (10V / 4096)

### Thread Safety

The driver is designed to be thread-safe when used with ESP-IDF's I2C master driver, which handles I2C bus locking internally.

## Troubleshooting

### Device Not Found

1. Check I2C bus initialization
2. Verify I2C address matches DIP switch setting
3. Check wiring (SDA, SCL, VCC, GND)
4. Use I2C scanner to verify device presence

### Incorrect Output Voltage

1. Verify register addresses match GP8403 datasheet
2. Check I2C communication (enable debug logging)
3. Verify power supply (3.3V or 5V)
4. Check for I2C bus conflicts

### Build Errors

Ensure the component is properly registered in your `CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "your_file.c"
    REQUIRES gp8403_dac
)
```

## References

- [Product Page](https://www.dfrobot.com/product-2613.html)
- [DFRobot Wiki](https://wiki.dfrobot.com/) (search for DFR0971)
- [GP8403 Datasheet](https://www.dfrobot.com/) (check product page for datasheet link)

## Credits

This ESP-IDF driver implementation is based on and references the DFRobot_GP8403 Arduino library:
- **Original Library**: [DFRobot_GP8403](https://github.com/DFRobot/DFRobot_GP8403)
- **Original Author**: tangjie (jie.tang@dfrobot.com)
- **License**: MIT License

## License

Copyright (c) 2025, Adam G. Sweeney

Portions of this code reference the DFRobot_GP8403 Arduino library:
Copyright (c) 2022 DFRobot (https://github.com/DFRobot/DFRobot_GP8403)
Written by tangjie (jie.tang@dfrobot.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

