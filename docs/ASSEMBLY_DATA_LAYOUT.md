# FusionCoreEnIP Assembly Data Layout

This document describes the byte-by-byte layout of EtherNet/IP Assembly Objects 100 (Input Assembly) and 150 (Output Assembly).

**Note:** All multi-byte values are stored in **little-endian** byte order.

---

## Input Assembly 100 (72 bytes)

The Input Assembly contains sensor readings and device feedback data. PLCs and other EtherNet/IP clients read this assembly to get current device status.

### Assembly Overview

| Byte Range | Device/Function | Size | Description |
|------------|----------------|------|-------------|
| 0-15 | VL53L1X | 16 bytes | Distance sensor data |
| 16-23 | LSM6DS3 | 8 bytes | IMU orientation data |
| 24-39 | NAU7802 | 16 bytes | Weight scale data |
| 40-55 | MCP230XX Feedback | 16 bytes | GPIO expander feedback |
| 56-60 | Device Counts | 5 bytes | Device detection counts |
| 61-71 | Reserved | 11 bytes | Reserved for future expansion |

---

### VL53L1X Distance Sensor (Bytes 0-15)

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 0-1 | Distance | uint16_t | Distance measurement in millimeters (little-endian) |
| 2 | Status | uint8_t | Status flags (bit 0 = data available) |
| 3-4 | Ambient Rate | uint16_t | Ambient light rate (little-endian) |
| 5-6 | Signal Per SPAD | uint16_t | Signal rate per SPAD (little-endian) |
| 7-8 | Number of SPADs | uint16_t | Number of SPADs used (little-endian) |
| 9-15 | Reserved | - | Reserved bytes (unused) |

**Notes:**
- Distance value represents range measurement in mm
- Status byte: bit 0 = 1 indicates valid data available
- Remaining bytes (9-15) are reserved for future use

---

### LSM6DS3 IMU (Bytes 16-23)

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 16-17 | Roll | int16_t | Roll angle scaled by 100 (-180.00° to +180.00°, little-endian) |
| 18-19 | Pitch | int16_t | Pitch angle scaled by 100 (-180.00° to +180.00°, little-endian) |
| 20-21 | Ground Angle | int16_t | Ground angle scaled by 100 (-180.00° to +180.00°, little-endian) |
| 22-23 | Reserved | - | Reserved bytes (unused) |

**Notes:**
- All angles are computed using complementary filter fusion algorithm
- Angles are stored as scaled integers (e.g., 48.66° = 4866, -123.45° = -12345)
- Actual angle range: -180.00° to +180.00°
- Ground angle is calculated from vertical using roll and pitch
- To get actual angle: divide stored value by 100.0

---

### NAU7802 Weight Scale (Bytes 24-39)

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 24-27 | Weight | int32_t | Weight value scaled by 100 in selected unit (little-endian) |
| 28-31 | Raw Reading | int32_t | Raw 24-bit ADC reading (little-endian) |
| 32 | Unit | uint8_t | Unit code: 0=grams, 1=pounds, 2=kilograms |
| 33 | Status | uint8_t | Status flags (bit 0=available, bit 1=connected, bit 2=initialized) |
| 34-39 | Reserved | - | Reserved bytes (unused) |

**Notes:**
- Weight is stored as scaled integer (e.g., 100.24 lbs = 10024)
- Raw reading is the unprocessed 24-bit ADC value
- Status byte bits:
  - Bit 0: Data available (nau7802_available)
  - Bit 1: Device connected
  - Bit 2: Device initialized

---

### MCP230XX GPIO Feedback (Bytes 40-55)

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 40-41 | Device 0x20 | uint16_t | MCP230XX at address 0x20 feedback (Port A in byte 40, Port B in byte 41) |
| 42-43 | Device 0x21 | uint16_t | MCP230XX at address 0x21 feedback (Port A in byte 42, Port B in byte 43) |
| 44-45 | Device 0x22 | uint16_t | MCP230XX at address 0x22 feedback (Port A in byte 44, Port B in byte 45) |
| 46-47 | Device 0x23 | uint16_t | MCP230XX at address 0x23 feedback (Port A in byte 46, Port B in byte 47) |
| 48-49 | Device 0x24 | uint16_t | MCP230XX at address 0x24 feedback (Port A in byte 48, Port B in byte 49) |
| 50-51 | Device 0x25 | uint16_t | MCP230XX at address 0x25 feedback (Port A in byte 50, Port B in byte 51) |
| 52-53 | Device 0x26 | uint16_t | MCP230XX at address 0x26 feedback (Port A in byte 52, Port B in byte 53) |
| 54-55 | Device 0x27 | uint16_t | MCP230XX at address 0x27 feedback (Port A in byte 54, Port B in byte 55) |

**Notes:**
- Each MCP230XX device uses 2 bytes (16 bits)
- MCP23017: Byte 0 = Port A (GPIO 0-7), Byte 1 = Port B (GPIO 8-15)
- MCP23008: Byte 0 = GPIO 0-7, Byte 1 = unused (always 0x00)
- Feedback reflects current GPIO state read from the device
- Devices are mapped by I2C address (0x20-0x27)

---

### Device Counts (Bytes 56-60)

| Byte | Device Type | Data Type | Description |
|------|-------------|-----------|-------------|
| 56 | VL53L1X Count | uint8_t | Number of VL53L1X devices detected (0 or 1) |
| 57 | LSM6DS3 Count | uint8_t | Number of LSM6DS3 devices detected (0 or 1) |
| 58 | NAU7802 Count | uint8_t | Number of NAU7802 devices detected (0 or 1) |
| 59 | MCP230XX Count | uint8_t | Number of MCP230XX devices detected (0-8) |
| 60 | GP8403 Count | uint8_t | Number of GP8403 DAC devices detected (0-4) |

**Notes:**
- Counts are set during boot-time device scanning
- Values represent the number of each device type that was detected and initialized
- Updated once at startup after all managers are initialized

---

### Reserved (Bytes 61-71)

| Byte Range | Description |
|------------|-------------|
| 61-71 | Reserved for future expansion (11 bytes) |

---

## Output Assembly 150 (40 bytes)

The Output Assembly contains control data that PLCs and other EtherNet/IP clients write to control device outputs.

### Assembly Overview

| Byte Range | Device/Function | Size | Description |
|------------|----------------|------|-------------|
| 0-15 | MCP230XX Output | 16 bytes | GPIO expander output control |
| 16-31 | GP8403 DAC | 16 bytes | DAC output values |
| 32-39 | Reserved | 8 bytes | Reserved for future expansion |

---

### MCP230XX GPIO Output (Bytes 0-15)

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 0-1 | Device 0x20 | uint16_t | MCP230XX at address 0x20 output (Port A in byte 0, Port B in byte 1) |
| 2-3 | Device 0x21 | uint16_t | MCP230XX at address 0x21 output (Port A in byte 2, Port B in byte 3) |
| 4-5 | Device 0x22 | uint16_t | MCP230XX at address 0x22 output (Port A in byte 4, Port B in byte 5) |
| 6-7 | Device 0x23 | uint16_t | MCP230XX at address 0x23 output (Port A in byte 6, Port B in byte 7) |
| 8-9 | Device 0x24 | uint16_t | MCP230XX at address 0x24 output (Port A in byte 8, Port B in byte 9) |
| 10-11 | Device 0x25 | uint16_t | MCP230XX at address 0x25 output (Port A in byte 10, Port B in byte 11) |
| 12-13 | Device 0x26 | uint16_t | MCP230XX at address 0x26 output (Port A in byte 12, Port B in byte 13) |
| 14-15 | Device 0x27 | uint16_t | MCP230XX at address 0x27 output (Port A in byte 14, Port B in byte 15) |

**Notes:**
- Each MCP230XX device uses 2 bytes (16 bits)
- MCP23017: Byte 0 = Port A output (GPIO 0-7), Byte 1 = Port B output (GPIO 8-15)
- MCP23008: Byte 0 = GPIO 0-7 output, Byte 1 = unused (ignored)
- Devices are mapped by I2C address (0x20-0x27) in order
- Output state is written to the device and mirrored to Input Assembly feedback

---

### GP8403 DAC Output (Bytes 16-31)

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 16-17 | Device 0x58 Ch0 | uint16_t | GP8403 at address 0x58, Channel 0 value (0-4095, 12-bit, little-endian) |
| 18-19 | Device 0x58 Ch1 | uint16_t | GP8403 at address 0x58, Channel 1 value (0-4095, 12-bit, little-endian) |
| 20-21 | Device 0x59 Ch0 | uint16_t | GP8403 at address 0x59, Channel 0 value (0-4095, 12-bit, little-endian) |
| 22-23 | Device 0x59 Ch1 | uint16_t | GP8403 at address 0x59, Channel 1 value (0-4095, 12-bit, little-endian) |
| 24-25 | Device 0x5A Ch0 | uint16_t | GP8403 at address 0x5A, Channel 0 value (0-4095, 12-bit, little-endian) |
| 26-27 | Device 0x5A Ch1 | uint16_t | GP8403 at address 0x5A, Channel 1 value (0-4095, 12-bit, little-endian) |
| 28-29 | Device 0x5B Ch0 | uint16_t | GP8403 at address 0x5B, Channel 0 value (0-4095, 12-bit, little-endian) |
| 30-31 | Device 0x5B Ch1 | uint16_t | GP8403 at address 0x5B, Channel 1 value (0-4095, 12-bit, little-endian) |

**Notes:**
- **Maximum devices**: 4 GP8403 devices supported (I2C addresses 0x58-0x5B)
- **Channels**: Each device has 2 independent channels (Ch0 = VOUT0, Ch1 = VOUT1)
- **Resolution**: 12-bit (0x000-0xFFF, decimal 0-4095)
- **Output range**: 0.0V to 10.0V per channel
- **Data format**: 12-bit value stored as little-endian uint16_t (2 bytes)
- **Voltage conversion**: 
  - Value to voltage: `voltage = (dac_value / 4095.0) × 10.0`
  - Voltage to value: `dac_value = (voltage / 10.0) × 4095`
- **Value clamping**: Values > 4095 (0xFFF) are automatically clamped to 4095
- **Device mapping**: Devices are mapped by I2C address in ascending order (0x58, 0x59, 0x5A, 0x5B)
- **Unused devices**: Devices not detected during initialization will have 0x0000 written (0V output)
- **Update behavior**: Values are only written to the DAC when they change (reduces I2C traffic)
- **I2C protocol**: The 12-bit value is formatted per GP8403 datasheet (shifted left 4 bits, bytes swapped) before I2C transmission

---

### Reserved (Bytes 32-39)

| Byte Range | Description |
|------------|-------------|
| 32-39 | Reserved for future expansion (8 bytes) |

---

## Byte Order and Data Format

### Multi-byte Values

All multi-byte integers are stored in **little-endian** byte order:

- **uint16_t**: Low byte first, high byte second
  - Example: Value 0x1234 stored as `[0x34, 0x12]`
  
- **int16_t**: Low byte first, high byte second (two's complement)
  - Example: Value -100 stored as `[0x9C, 0xFF]` (0xFF9C in little-endian)
  
- **uint32_t/int32_t**: Lowest byte first, highest byte last
  - Example: Value 0x12345678 stored as `[0x78, 0x56, 0x34, 0x12]`

### Bit Mapping

For GPIO and status bytes, bits are numbered 0-7 (or 0-15 for 16-bit values):
- Bit 0 = LSB (Least Significant Bit)
- Bit 7/15 = MSB (Most Significant Bit)

---

## Device Address Mapping

### MCP230XX Devices

MCP230XX devices are mapped by I2C address in the following order:

| I2C Address | Output Assembly Bytes | Input Assembly Bytes (Feedback) |
|-------------|----------------------|--------------------------------|
| 0x20 | 0-1 | 40-41 |
| 0x21 | 2-3 | 42-43 |
| 0x22 | 4-5 | 44-45 |
| 0x23 | 6-7 | 46-47 |
| 0x24 | 8-9 | 48-49 |
| 0x25 | 10-11 | 50-51 |
| 0x26 | 12-13 | 52-53 |
| 0x27 | 14-15 | 54-55 |

### GP8403 DAC Devices

GP8403 devices are mapped by I2C address in ascending order:

| I2C Address | Assembly Bytes | Description |
|-------------|---------------|-------------|
| 0x58 | 16-19 | Channel 0 (16-17), Channel 1 (18-19) |
| 0x59 | 20-23 | Channel 0 (20-21), Channel 1 (22-23) |
| 0x5A | 24-27 | Channel 0 (24-25), Channel 1 (26-27) |
| 0x5B | 28-31 | Channel 0 (28-29), Channel 1 (30-31) |

**Notes:**
- Only the first 4 detected devices (I2C addresses 0x58-0x5B) are supported
- Each device requires a separate 9-36V VIN power supply (in addition to I2C VCC) for output generation
- Output voltage range: 0.0V to 10.0V (configured at initialization)
- Resolution: 12-bit (4096 steps, ~2.44 mV per step)
- Output accuracy: < 0.5% voltage error, < 0.1% linearity error

---

## Assembly Size Summary

| Assembly | Total Size | Used Bytes | Reserved Bytes |
|----------|-----------|------------|----------------|
| Input Assembly 100 | 72 bytes | 61 bytes | 11 bytes |
| Output Assembly 150 | 40 bytes | 32 bytes | 8 bytes |

---

## Implementation Notes

1. **Thread Safety**: All assembly data access is protected by a mutex (`fusion_core_get_assembly_mutex()`)

2. **Update Frequency**: 
   - VL53L1X: ~10 Hz (100 ms interval)
   - LSM6DS3: ~10 Hz (100 ms interval)
   - NAU7802: ~10 Hz (100 ms interval)
   - MCP230XX: ~50 Hz (20 ms interval)
   - GP8403: ~20 Hz (50 ms polling interval, writes only on value change)

3. **Device Detection**: Device counts are set during boot-time I2C scanning and updated once after all managers initialize

4. **Disabled Devices**: If a device is disabled via NVS or not detected, its assembly bytes remain at 0x00

5. **Reserved Bytes**: Reserved bytes are initialized to 0x00 and should not be used by external systems

---

## References

- `components/opener/src/ports/ESP32/fusion_core/include/fusion_core_assembly.h` - Assembly byte offset definitions
- `components/opener/src/ports/ESP32/fusion_core/fusioncore.c` - Assembly data arrays
- Individual manager components for data update implementations

