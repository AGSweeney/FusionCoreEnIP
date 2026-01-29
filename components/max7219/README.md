# MAX7219 Driver

ESP-IDF driver for the **MAX7219 serial LED display driver** for driving 7-segment displays, LED matrices, and other LED displays.

## Product Information

- **Chip**: MAX7219
- **Manufacturer**: Maxim Integrated (now Analog Devices)
- **Interface**: SPI
- **Datasheet**: [MAX7219 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max7219.pdf)

## Features

- ✅ 8-digit 7-segment display support
- ✅ Cascadable (multiple chips daisy-chained)
- ✅ Brightness control (16 levels)
- ✅ Decode mode control (BCD or raw segment control)
- ✅ Scan limit control (1-8 digits)
- ✅ Shutdown mode control
- ✅ Display test mode
- ✅ ESP-IDF SPI master driver support
- ✅ Thread-safe operation

## Specifications

| Parameter | Value |
|-----------|-------|
| Supply Voltage | 4.0V - 5.5V |
| Interface | SPI (4-wire) |
| Max Clock Frequency | 10 MHz |
| Digits per Chip | 8 |
| Max Cascaded Devices | 8 (64 digits total) |
| Brightness Levels | 16 (0-15) |
| Current per Segment | 40 mA (max) |
| Operating Temperature | -40°C to +85°C |

## Hardware Connection

### Pinout

The MAX7219 uses a standard SPI interface:

| Pin | Function | ESP32 Connection |
|-----|----------|------------------|
| VCC | Power (4.0V-5.5V) | 5V (or 3.3V with level shifter) |
| GND | Ground | GND |
| DIN | Data In (MOSI) | GPIO (configured in SPI bus) |
| CS/LOAD | Chip Select | GPIO (configured in SPI device) |
| CLK | Clock (SCLK) | GPIO (configured in SPI bus) |

**Note**: The MAX7219 operates at 5V logic levels. If using 3.3V ESP32, use a level shifter or ensure the MAX7219 is tolerant of 3.3V inputs (some modules are).

### Cascading Multiple Devices

To cascade multiple MAX7219 chips:
1. Connect DOUT of the first chip to DIN of the second chip
2. Connect CS/LOAD, CLK, and VCC/GND in parallel
3. Set `num_devices` in the configuration to the number of cascaded chips

## Usage

### 1. Include the Header

```c
#include "max7219.h"
```

### 2. Initialize SPI Bus and Device

The driver requires an initialized SPI device. Example:

```c
#include "driver/spi_master.h"

// Initialize SPI bus
spi_bus_config_t bus_cfg = {
    .mosi_io_num = GPIO_NUM_23,
    .miso_io_num = GPIO_NUM_NC,  // Not used for MAX7219
    .sclk_io_num = GPIO_NUM_18,
    .quadwp_io_num = GPIO_NUM_NC,
    .quadhd_io_num = GPIO_NUM_NC,
    .max_transfer_sz = 32,  // 2 bytes × 8 devices max
};

esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus");
    return;
}

// Configure SPI device
spi_device_interface_config_t dev_cfg = {
    .clock_speed_hz = 10 * 1000 * 1000,  // 10 MHz max for MAX7219
    .mode = 0,                            // SPI mode 0
    .spics_io_num = GPIO_NUM_5,           // CS pin
    .queue_size = 1,
    .flags = 0,
    .pre_cb = NULL,
};

spi_device_handle_t spi_device;
err = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device");
    return;
}
```

### 3. Initialize MAX7219 Device

```c
max7219_config_t config = {
    .spi_device = spi_device,
    .num_devices = 1,  // Number of cascaded devices (default: 1)
};

max7219_handle_t *handle = NULL;
if (!max7219_init(&config, &handle)) {
    ESP_LOGE(TAG, "Failed to initialize MAX7219");
    return;
}
```

### 4. Basic Operations

#### Set Brightness

```c
// Set brightness level (0-15, where 0=min, 15=max)
max7219_set_intensity(handle, 8);  // Medium brightness
```

#### Display Numbers

```c
// Display a number (without leading zeros)
max7219_display_number(handle, 0, 12345, false);

// Display a number (with leading zeros)
max7219_display_number(handle, 0, 42, true);  // Shows "00000042"
```

#### Display Raw Segments

```c
// Set decode mode to none for raw segment control
max7219_set_decode_mode(handle, MAX7219_DECODE_NONE);

// Set segments for digit 0
// Segment bitmask: bit 0=DP, bit 1=G, bit 2=F, bit 3=E, bit 4=D, bit 5=C, bit 6=B, bit 7=A
max7219_set_segments(handle, 0, 0, 0x7E);  // Display "0"
max7219_set_segments(handle, 0, 1, 0x30);  // Display "1"
```

#### BCD Decode Mode

```c
// Enable BCD decode for digits 0-3
max7219_set_decode_mode(handle, MAX7219_DECODE_DIGIT0_3);

// Display BCD values (0-9, or special codes)
max7219_set_digit(handle, 0, 0, 5);  // Display "5" on digit 0
max7219_set_digit(handle, 0, 1, 9);  // Display "9" on digit 1
```

#### Control Functions

```c
// Shutdown display (power saving)
max7219_set_shutdown(handle, true);

// Enable display
max7219_set_shutdown(handle, false);

// Test mode (all segments on)
max7219_set_display_test(handle, true);
vTaskDelay(pdMS_TO_TICKS(1000));
max7219_set_display_test(handle, false);

// Clear all digits
max7219_clear_all(handle);

// Set scan limit (number of digits to display)
max7219_set_scan_limit(handle, 3);  // Display only 4 digits (0-3)
```

### 5. Cascaded Devices

When using multiple cascaded MAX7219 chips:

```c
max7219_config_t config = {
    .spi_device = spi_device,
    .num_devices = 3,  // 3 cascaded devices = 24 digits total
};

max7219_handle_t *handle = NULL;
if (!max7219_init(&config, &handle)) {
    ESP_LOGE(TAG, "Failed to initialize MAX7219");
    return;
}

// Display on first device (device_index = 0)
max7219_display_number(handle, 0, 12345678, false);

// Display on second device (device_index = 1)
max7219_display_number(handle, 1, 87654321, false);

// Display on third device (device_index = 2)
max7219_display_number(handle, 2, 99999999, false);
```

### 6. Deinitialize

```c
max7219_deinit(&handle);  // Sets handle to NULL
```

## Complete Example

```c
#include "max7219.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "max7219_example";

void app_main(void)
{
    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_NUM_23,
        .miso_io_num = GPIO_NUM_NC,
        .sclk_io_num = GPIO_NUM_18,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 32,
    };

    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = GPIO_NUM_5,
        .queue_size = 1,
    };

    spi_device_handle_t spi_device;
    err = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return;
    }

    // Initialize MAX7219
    max7219_config_t config = {
        .spi_device = spi_device,
        .num_devices = 1,
    };

    max7219_handle_t *handle = NULL;
    if (!max7219_init(&config, &handle)) {
        ESP_LOGE(TAG, "Failed to initialize MAX7219");
        return;
    }

    // Set brightness
    max7219_set_intensity(handle, 8);

    // Display test
    ESP_LOGI(TAG, "Display test mode");
    max7219_set_display_test(handle, true);
    vTaskDelay(pdMS_TO_TICKS(2000));
    max7219_set_display_test(handle, false);

    // Display numbers
    ESP_LOGI(TAG, "Displaying numbers");
    for (int i = 0; i < 10000; i++) {
        max7219_display_number(handle, 0, i, false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Clear display
    max7219_clear_all(handle);

    // Cleanup
    max7219_deinit(&handle);
    spi_bus_remove_device(spi_device);
    spi_bus_free(SPI2_HOST);
}
```

## API Reference

### Types

#### `max7219_register_t`
Register addresses enum:
- `MAX7219_REG_NOOP`: No operation
- `MAX7219_REG_DIGIT0` through `MAX7219_REG_DIGIT7`: Digit registers
- `MAX7219_REG_DECODE_MODE`: Decode mode register
- `MAX7219_REG_INTENSITY`: Intensity register
- `MAX7219_REG_SCAN_LIMIT`: Scan limit register
- `MAX7219_REG_SHUTDOWN`: Shutdown register
- `MAX7219_REG_DISPLAY_TEST`: Display test register

#### `max7219_decode_mode_t`
Decode mode enum:
- `MAX7219_DECODE_NONE`: No decode (raw segment control)
- `MAX7219_DECODE_DIGIT0`: Decode digit 0 only
- `MAX7219_DECODE_DIGIT0_3`: Decode digits 0-3
- `MAX7219_DECODE_ALL`: Decode all digits

#### `max7219_config_t`
Configuration structure:
```c
typedef struct {
    spi_device_handle_t spi_device;   // SPI device handle
    uint8_t num_devices;              // Number of cascaded devices (default: 1)
} max7219_config_t;
```

#### `max7219_handle_t`
Device handle (opaque structure).

### Functions

#### `max7219_init()`
Initialize MAX7219 device.

#### `max7219_deinit()`
Deinitialize MAX7219 device.

#### `max7219_write_register()`
Write to MAX7219 register.

#### `max7219_set_intensity()`
Set display intensity (brightness) level (0-15).

#### `max7219_set_scan_limit()`
Set scan limit (number of digits to display, 0-7).

#### `max7219_set_decode_mode()`
Set decode mode (BCD or raw segment control).

#### `max7219_set_shutdown()`
Set shutdown mode (true = off, false = on).

#### `max7219_set_display_test()`
Enable/disable display test mode (all segments on).

#### `max7219_clear_all()`
Clear all digits on all devices.

#### `max7219_set_digit()`
Set digit value (for BCD decode mode).

#### `max7219_set_segments()`
Set raw segment data (for non-decode mode).

#### `max7219_display_number()`
Display a number across multiple digits.

#### `max7219_display_float()`
Display a floating point number with decimal places.

#### `max7219_is_initialized()`
Check if device is initialized.

#### `max7219_get_intensity()`
Get current intensity level.

#### `max7219_get_scan_limit()`
Get current scan limit.

#### `max7219_get_shutdown()`
Get shutdown state.

## Register Map

The MAX7219 uses the following register addresses:

| Address | Register | Description |
|---------|----------|-------------|
| 0x00 | NOOP | No operation |
| 0x01-0x08 | DIGIT0-DIGIT7 | Digit 0-7 data |
| 0x09 | DECODE_MODE | Decode mode (BCD or raw) |
| 0x0A | INTENSITY | Brightness (0-15) |
| 0x0B | SCAN_LIMIT | Number of digits (0-7) |
| 0x0C | SHUTDOWN | Shutdown mode (0=off, 1=on) |
| 0x0F | DISPLAY_TEST | Test mode (0=off, 1=on) |

## BCD Decode Mode

When decode mode is enabled, the digit registers accept BCD values:

| Value | Display |
|-------|---------|
| 0x00-0x09 | 0-9 |
| 0x0A | - (minus sign) |
| 0x0B | E (error) |
| 0x0C | H (hex) |
| 0x0D | L (low) |
| 0x0E | P (power) |
| 0x0F | Blank |

## Segment Mapping (Raw Mode)

When decode mode is disabled, segments are controlled by bits:

| Bit | Segment | Description |
|-----|---------|-------------|
| 0 | DP | Decimal point |
| 1 | G | Middle segment |
| 2 | F | Top-right segment |
| 3 | E | Bottom-right segment |
| 4 | D | Bottom segment |
| 5 | C | Bottom-left segment |
| 6 | B | Top-left segment |
| 7 | A | Top segment |

Common segment patterns:
- `0x7E`: 0 (A, B, C, D, E, F)
- `0x30`: 1 (B, C)
- `0x6D`: 2 (A, B, D, E, G)
- `0x79`: 3 (A, B, C, D, G)
- `0x33`: 4 (B, C, F, G)
- `0x5B`: 5 (A, C, D, F, G)
- `0x5F`: 6 (A, C, D, E, F, G)
- `0x70`: 7 (A, B, C)
- `0x7F`: 8 (all segments)
- `0x7B`: 9 (A, B, C, D, F, G)

## Notes

### SPI Communication

- **SPI Mode**: Mode 0 (CPOL=0, CPHA=0)
- **Clock Speed**: Up to 10 MHz
- **Data Format**: 16 bits per register (8-bit register + 8-bit data)
- **Cascading**: Data is sent MSB first, last device first

### Power Supply

- The MAX7219 requires 4.0V-5.5V supply voltage
- If using 3.3V ESP32, ensure proper level shifting or use a 5V-tolerant module
- Typical current consumption: ~330mA with all segments on at maximum brightness

### Thread Safety

The driver is designed to be thread-safe when used with ESP-IDF's SPI master driver, which handles SPI bus locking internally.

## Troubleshooting

### Display Not Working

1. Check SPI bus initialization
2. Verify CS pin configuration
3. Check power supply (4.0V-5.5V)
4. Verify SPI mode (should be mode 0)
5. Check clock speed (should be ≤ 10 MHz)
6. Try display test mode to verify hardware

### Incorrect Display

1. Verify decode mode setting
2. Check segment mapping in raw mode
3. Verify scan limit setting
4. Check for cascading configuration errors

### Build Errors

Ensure the component is properly registered in your `CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "your_file.c"
    REQUIRES max7219
)
```

## References

- [MAX7219 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/max7219.pdf)
- [MAX7219 Application Note](https://www.analog.com/en/resources/app-notes/an-696.html)

## License

Copyright (c) 2025, Adam G. Sweeney

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
