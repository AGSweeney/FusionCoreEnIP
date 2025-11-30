# FusionCoreEnIP REST API Manual

Complete reference documentation for all REST API endpoints available on the FusionCoreEnIP device.

**Base URL**: `http://<device-ip>/api`

All endpoints return JSON responses. Error responses include a `status` field set to `"error"` and a `message` field describing the error.

---

## Table of Contents

- [System Endpoints](#system-endpoints)
- [Assembly Endpoints](#assembly-endpoints)
- [Network Configuration](#network-configuration)
- [I2C Configuration](#i2c-configuration)
- [OTA Firmware Updates](#ota-firmware-updates)
- [NAU7802 Weight Scale](#nau7802-weight-scale)
- [VL53L1X Distance Sensor](#vl53l1x-distance-sensor)
- [LSM6DS3 IMU Sensor](#lsm6ds3-imu-sensor)
- [GP8403 DAC Output](#gp8403-dac-output)
- [MCP230XX GPIO Expander](#mcp230xx-gpio-expander)
- [System Logs](#system-logs)

---

## System Endpoints

### POST `/api/reboot`

Reboot the device. The device will restart after a 2-second delay.

**Request Body**: None required

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "Device rebooting..."
}
```

**Example**:
```bash
curl -X POST http://192.168.1.100/api/reboot
```

---

## Assembly Endpoints

### GET `/api/assemblies/sizes`

Get the size of EtherNet/IP input and output assemblies.

**Response** (200 OK):
```json
{
  "input_assembly_size": 72,
  "output_assembly_size": 40
}
```

**Example**:
```bash
curl http://192.168.1.100/api/assemblies/sizes
```

---

### GET `/api/status`

Get comprehensive device status including all sensor data and assembly contents.

**Response** (200 OK):
```json
{
  "vl53l1x": {
    "distance_mm": 1234,
    "status": 0,
    "ambient": 500,
    "sig_per_spad": 250,
    "num_spads": 256,
    "data_valid": true,
    "initialized": true
  },
  "lsm6ds3": {
    "roll": 45.66,
    "pitch": -12.34,
    "ground_angle": 90.00,
    "enabled": true,
    "initialized": true
  },
  "nau7802": {
    "weight": 123.45,
    "unit": "lbs",
    "raw_reading": 12345678,
    "available": true,
    "connected": true,
    "initialized": true
  },
  "mcp230xx": {
    "output": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "feedback": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  },
  "gp8403": {
    "devices": [
      {
        "address": 88,
        "channel_0": 2048,
        "channel_1": 4095
      }
    ]
  }
}
```

**Example**:
```bash
curl http://192.168.1.100/api/status
```

---

## Network Configuration

### GET `/api/ipconfig`

Get current network configuration.

**Response** (200 OK):
```json
{
  "use_dhcp": false,
  "ip_address": "192.168.1.100",
  "netmask": "255.255.255.0",
  "gateway": "192.168.1.1",
  "dns1": "8.8.8.8",
  "dns2": "8.8.4.4"
}
```

**Example**:
```bash
curl http://192.168.1.100/api/ipconfig
```

---

### POST `/api/ipconfig`

Update network configuration. Reboot required after changes.

**Request Body**:
```json
{
  "use_dhcp": false,
  "ip_address": "192.168.1.100",
  "netmask": "255.255.255.0",
  "gateway": "192.168.1.1",
  "dns1": "8.8.8.8",
  "dns2": "8.8.4.4"
}
```

**Fields**:
- `use_dhcp` (boolean): Enable DHCP if true, static IP if false
- `ip_address` (string): Static IP address (required if `use_dhcp` is false)
- `netmask` (string): Network mask (required if `use_dhcp` is false)
- `gateway` (string): Gateway address (optional)
- `dns1` (string): Primary DNS server (optional)
- `dns2` (string): Secondary DNS server (optional)

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "IP configuration saved successfully. Reboot required to apply changes."
}
```

**Error Response** (400 Bad Request):
```json
{
  "status": "error",
  "message": "Invalid IP address format"
}
```

**Example**:
```bash
curl -X POST http://192.168.1.100/api/ipconfig \
  -H "Content-Type: application/json" \
  -d '{
    "use_dhcp": false,
    "ip_address": "192.168.1.100",
    "netmask": "255.255.255.0",
    "gateway": "192.168.1.1"
  }'
```

---

## I2C Configuration

### GET `/api/i2c/pullup`

Get I2C pull-up resistor enabled state (legacy endpoint, applies to both buses).

**Response** (200 OK):
```json
{
  "enabled": true
}
```

---

### POST `/api/i2c/pullup`

Set I2C pull-up resistor enabled state (legacy endpoint). Restart required.

**Request Body**:
```json
{
  "enabled": true
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "enabled": true,
  "message": "I2C pull-up setting saved. Restart required for changes to take effect."
}
```

---

### GET `/api/i2c/pullup/primary`

Get primary I2C bus pull-up resistor enabled state.

**Response** (200 OK):
```json
{
  "enabled": true,
  "bus": "primary",
  "sda_gpio": 7,
  "scl_gpio": 8
}
```

---

### POST `/api/i2c/pullup/primary`

Set primary I2C bus pull-up resistor enabled state. Restart required.

**Request Body**:
```json
{
  "enabled": true
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "enabled": true,
  "message": "Primary I2C pull-up setting saved. Restart required."
}
```

---

### GET `/api/i2c/pullup/secondary`

Get secondary I2C bus pull-up resistor enabled state.

**Response** (200 OK):
```json
{
  "enabled": false,
  "bus": "secondary",
  "sda_gpio": 9,
  "scl_gpio": 10
}
```

---

### POST `/api/i2c/pullup/secondary`

Set secondary I2C bus pull-up resistor enabled state. Restart required.

**Request Body**:
```json
{
  "enabled": false
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "enabled": false,
  "message": "Secondary I2C pull-up setting saved. Restart required."
}
```

---

## OTA Firmware Updates

### POST `/api/ota/update`

Start an over-the-air firmware update. Supports two methods:

1. **File Upload**: Multipart form data with firmware binary
2. **URL Download**: JSON with firmware URL

**Method 1: File Upload**

**Content-Type**: `multipart/form-data`

**Form Field**: `firmware` (binary file)

**Example**:
```bash
curl -X POST http://192.168.1.100/api/ota/update \
  -F "firmware=@firmware.bin"
```

**Method 2: URL Download**

**Content-Type**: `application/json`

**Request Body**:
```json
{
  "url": "http://example.com/firmware.bin"
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "OTA update started"
}
```

**Note**: File uploads will automatically finish and reboot. URL-based updates start asynchronously.

---

### GET `/api/ota/status`

Get current OTA update status and progress.

**Response** (200 OK):
```json
{
  "status": "idle",
  "progress": 0,
  "message": "Ready"
}
```

**Status Values**:
- `"idle"`: No update in progress
- `"in_progress"`: Update currently running
- `"complete"`: Update completed successfully
- `"error"`: Update failed

**Progress**: Integer from 0-100 representing completion percentage

**Example**:
```bash
curl http://192.168.1.100/api/ota/status
```

---

## NAU7802 Weight Scale

### GET `/api/nau7802`

Get NAU7802 weight scale status, configuration, and current reading.

**Response** (200 OK):
```json
{
  "enabled": true,
  "initialized": true,
  "connected": true,
  "weight": 123.45,
  "unit": "lbs",
  "raw_reading": 12345678,
  "byte_offset": 24,
  "gain": 7,
  "sample_rate": 3,
  "channel": 0,
  "ldo": 4,
  "average": 5,
  "zero_offset": 0.0,
  "calibration_factor": 1.0,
  "ch1_offset": 0,
  "ch1_gain": 8388608
}
```

**Fields**:
- `enabled`: Device enabled state
- `initialized`: Device initialized status
- `connected`: Device connection status
- `weight`: Current weight reading (in configured unit)
- `unit`: Unit of measurement ("g", "lbs", or "kg")
- `raw_reading`: Raw 24-bit ADC reading
- `byte_offset`: Assembly byte offset (24)
- `gain`: PGA gain setting (0=x1, 1=x2, ..., 7=x128)
- `sample_rate`: Sample rate setting (0=10 SPS, 1=20, 2=40, 3=80, 4=320)
- `channel`: Active channel (0=Channel 1, 1=Channel 2)
- `ldo`: LDO voltage setting (0=4.5V, ..., 4=3.3V, ..., 7=2.4V)
- `average`: Number of samples to average (1-50)
- `zero_offset`: Calibration zero offset
- `calibration_factor`: Calibration factor
- `ch1_offset`: Channel 1 hardware offset
- `ch1_gain`: Channel 1 hardware gain

---

### POST `/api/nau7802`

Update NAU7802 configuration. Most settings require reboot to take effect.

**Request Body**:
```json
{
  "enabled": true,
  "byte_offset": 24,
  "unit": 1,
  "average": 5,
  "gain": 7,
  "sample_rate": 3,
  "channel": 0,
  "ldo": 4
}
```

**Fields**:
- `enabled` (boolean): Enable/disable device
- `byte_offset` (integer): Assembly byte offset (default: 24)
- `unit` (integer): Unit code (0=grams, 1=pounds, 2=kilograms)
- `average` (integer): Sample averaging count (1-50, immediate effect)
- `gain` (integer): PGA gain (0-7, requires reboot)
- `sample_rate` (integer): Sample rate (0-4, requires reboot)
- `channel` (integer): Active channel (0 or 1, requires reboot)
- `ldo` (integer): LDO voltage (0-7, requires reboot)

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "NAU7802 configuration saved. Some settings require reboot."
}
```

---

### POST `/api/nau7802/calibrate`

Perform NAU7802 calibration operations.

**Request Body**:
```json
{
  "operation": "tare",
  "known_weight": 0.0,
  "sample_count": 64,
  "timeout_ms": 2000
}
```

**Operations**:
- `"tare"`: Calculate zero offset (tare). Remove all weight first.
- `"calibrate"`: Calculate calibration factor using known weight. Requires `known_weight` field.
- `"afe"`: Perform AFE (Analog Front End) hardware calibration.

**Fields**:
- `operation` (string): Calibration operation type
- `known_weight` (float): Known weight value for calibration (required for "calibrate")
- `sample_count` (integer): Number of samples to average (default: 64)
- `timeout_ms` (integer): Timeout in milliseconds (default: 2000)

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "Tare calibration completed successfully",
  "zero_offset": 1234.56
}
```

**Example - Tare**:
```bash
curl -X POST http://192.168.1.100/api/nau7802/calibrate \
  -H "Content-Type: application/json" \
  -d '{"operation": "tare", "sample_count": 64}'
```

**Example - Calibrate with Known Weight**:
```bash
curl -X POST http://192.168.1.100/api/nau7802/calibrate \
  -H "Content-Type: application/json" \
  -d '{"operation": "calibrate", "known_weight": 100.0, "sample_count": 64}'
```

---

## VL53L1X Distance Sensor

### GET `/api/vl53l1x`

Get VL53L1X distance sensor status and current reading.

**Response** (200 OK):
```json
{
  "enabled": true,
  "initialized": true,
  "distance_mm": 1234,
  "status": 0,
  "ambient": 500,
  "sig_per_spad": 250,
  "num_spads": 256,
  "byte_offset": 0
}
```

**Fields**:
- `enabled`: Device enabled state
- `initialized`: Device initialized status
- `distance_mm`: Distance measurement in millimeters
- `status`: Range status code (0=valid)
- `ambient`: Ambient light rate (kcps)
- `sig_per_spad`: Signal rate per SPAD (kcps/SPAD)
- `num_spads`: Number of enabled SPADs
- `byte_offset`: Assembly byte offset (0)

---

### POST `/api/vl53l1x`

Update VL53L1X enabled state and assembly byte offset.

**Request Body**:
```json
{
  "enabled": true,
  "byte_offset": 0
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "VL53L1X configuration saved. Reboot required."
}
```

---

### GET `/api/vl53l1x/config`

Get VL53L1X sensor configuration parameters.

**Response** (200 OK):
```json
{
  "distance_mode": 1,
  "timing_budget_ms": 50,
  "inter_measurement_ms": 0,
  "roi_width": 16,
  "roi_height": 16,
  "roi_center_x": 8,
  "roi_center_y": 8
}
```

**Fields**:
- `distance_mode`: Distance mode (0=Short, 1=Long)
- `timing_budget_ms`: Timing budget in milliseconds (15-500)
- `inter_measurement_ms`: Inter-measurement period (0=continuous)
- `roi_width`: Region of Interest width (4-16)
- `roi_height`: Region of Interest height (4-16)
- `roi_center_x`: ROI center X coordinate
- `roi_center_y`: ROI center Y coordinate

---

### POST `/api/vl53l1x/config`

Update VL53L1X sensor configuration.

**Request Body**:
```json
{
  "distance_mode": 1,
  "timing_budget_ms": 50,
  "inter_measurement_ms": 0,
  "roi_width": 16,
  "roi_height": 16,
  "roi_center_x": 8,
  "roi_center_y": 8
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "VL53L1X configuration updated"
}
```

---

### POST `/api/vl53l1x/calibrate/offset`

Perform VL53L1X offset calibration at a known distance.

**Request Body**:
```json
{
  "distance_mm": 100
}
```

**Fields**:
- `distance_mm` (integer): Known distance in millimeters (50-4000mm for Long mode)

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "Offset calibration completed",
  "offset": 1234
}
```

---

### POST `/api/vl53l1x/calibrate/xtalk`

Perform VL53L1X crosstalk calibration with a cover at close distance.

**Request Body**:
```json
{
  "distance_mm": 50
}
```

**Fields**:
- `distance_mm` (integer): Distance to cover during calibration (typically 50-100mm)

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "Crosstalk calibration completed",
  "xtalk": 123
}
```

---

## LSM6DS3 IMU Sensor

### GET `/api/lsm6ds3`

Get LSM6DS3 IMU sensor status and current orientation.

**Response** (200 OK):
```json
{
  "enabled": true,
  "initialized": true,
  "roll": 45.66,
  "pitch": -12.34,
  "ground_angle": 90.00,
  "byte_offset": 16
}
```

**Fields**:
- `enabled`: Device enabled state
- `initialized`: Device initialized status
- `roll`: Roll angle in degrees (-180.00 to +180.00)
- `pitch`: Pitch angle in degrees (-180.00 to +180.00)
- `ground_angle`: Ground angle in degrees
- `byte_offset`: Assembly byte offset (16)

---

### POST `/api/lsm6ds3`

Update LSM6DS3 enabled state and assembly byte offset.

**Request Body**:
```json
{
  "enabled": true,
  "byte_offset": 16
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "LSM6DS3 configuration saved. Reboot required."
}
```

---

### GET `/api/lsm6ds3/calibrate`

Get LSM6DS3 calibration status.

**Response** (200 OK):
```json
{
  "accelerometer_calibrated": true,
  "gyroscope_calibrated": false,
  "accelerometer_offsets": {
    "x": 0.01,
    "y": -0.02,
    "z": 0.03
  },
  "gyroscope_offsets": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  }
}
```

---

### POST `/api/lsm6ds3/calibrate`

Start LSM6DS3 calibration process. Device must be kept completely still during calibration.

**Request Body**:
```json
{
  "calibrate_accelerometer": true,
  "calibrate_gyroscope": true,
  "sample_count": 100,
  "interval_ms": 10
}
```

**Fields**:
- `calibrate_accelerometer` (boolean): Calibrate accelerometer
- `calibrate_gyroscope` (boolean): Calibrate gyroscope
- `sample_count` (integer): Number of samples to collect (default: 100)
- `interval_ms` (integer): Delay between samples in milliseconds (default: 10)

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "Calibration started. Keep device still.",
  "estimated_duration_ms": 1000
}
```

---

### POST `/api/lsm6ds3/calibrate/preview`

Preview calibration offsets without saving them.

**Request Body**: Same as `/api/lsm6ds3/calibrate`

**Response** (200 OK):
```json
{
  "status": "ok",
  "accelerometer_offsets": {
    "x": 0.01,
    "y": -0.02,
    "z": 0.03
  },
  "gyroscope_offsets": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "message": "Preview calibration completed. Call /api/lsm6ds3/calibrate to apply."
}
```

---

### POST `/api/lsm6ds3/zero`

Set angle zero offsets for roll, pitch, and yaw.

**Request Body**:
```json
{
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0
}
```

**Or to clear zero offsets**:
```json
{
  "clear": true
}
```

**Fields**:
- `roll` (float, optional): Roll zero offset in degrees
- `pitch` (float, optional): Pitch zero offset in degrees
- `yaw` (float, optional): Yaw zero offset in degrees
- `clear` (boolean, optional): Clear all zero offsets

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "Angle zero offsets saved successfully",
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0
}
```

---

### GET `/api/lsm6ds3/nvs-calibration`

Get stored calibration values from NVS (Non-Volatile Storage).

**Response** (200 OK):
```json
{
  "accelerometer_offsets": {
    "x": 0.01,
    "y": -0.02,
    "z": 0.03
  },
  "gyroscope_offsets": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "angle_zeros": {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0
  }
}
```

---

## GP8403 DAC Output

### GET `/api/gp8403`

Get GP8403 DAC status and current output values.

**Response** (200 OK):
```json
{
  "enabled": true,
  "initialized": true,
  "device_count": 2,
  "devices": [
    {
      "address": 88,
      "channel_0": 2048,
      "channel_1": 4095
    },
    {
      "address": 89,
      "channel_0": 0,
      "channel_1": 1024
    }
  ]
}
```

**Fields**:
- `enabled`: Device enabled state
- `initialized`: Manager initialized status
- `device_count`: Number of detected devices (0-4)
- `devices`: Array of device objects with:
  - `address`: I2C address (88-91, or 0x58-0x5B)
  - `channel_0`: Channel 0 DAC value (0-4095, 12-bit)
  - `channel_1`: Channel 1 DAC value (0-4095, 12-bit)

**Note**: DAC values represent 0-10V output (0 = 0V, 4095 = 10V)

---

### POST `/api/gp8403`

Enable or disable GP8403 DAC devices. Reboot required.

**Request Body**:
```json
{
  "enabled": true
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "enabled": true,
  "message": "GP8403 configuration saved. Reboot required."
}
```

**Note**: To control DAC output values, write to the EtherNet/IP Output Assembly (bytes 16-31).

---

## MCP230XX GPIO Expander

### GET `/api/mcp230xx`

Get MCP230XX GPIO expander status and current I/O states.

**Response** (200 OK):
```json
{
  "initialized": true,
  "output": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  "feedback": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
}
```

**Fields**:
- `initialized`: Manager initialized status
- `output`: Array of 16 output bytes (one per GPIO expander device)
- `feedback`: Array of 16 feedback bytes (current GPIO states)

**Note**: Each byte represents one device's GPIO state (2 bytes per device for MCP23017, 1 byte for MCP23008).

---

### POST `/api/mcp230xx`

Set MCP230XX GPIO output states.

**Request Body**:
```json
{
  "output": [0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
}
```

**Fields**:
- `output` (array): Array of up to 16 bytes representing output states for each device

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "MCP230XX output updated"
}
```

---

### GET `/api/mcp230xx/enabled`

Get global MCP230XX enabled state.

**Response** (200 OK):
```json
{
  "enabled": true
}
```

---

### POST `/api/mcp230xx/enabled`

Enable or disable MCP230XX GPIO expanders. Reboot required.

**Request Body**:
```json
{
  "enabled": true
}
```

**Response** (200 OK):
```json
{
  "status": "ok",
  "enabled": true,
  "message": "MCP230XX enabled. Reboot required."
}
```

---

### GET `/api/mcp230xx/devices`

Get detected and configured MCP230XX devices.

**Response** (200 OK):
```json
{
  "devices": [
    {
      "address": "0x20",
      "device_type": 0,
      "enabled": true,
      "detected": true,
      "gpio_count": 16
    },
    {
      "address": "0x21",
      "device_type": 1,
      "enabled": false,
      "detected": true,
      "gpio_count": 8
    }
  ],
  "global_enabled": true
}
```

**Fields**:
- `devices`: Array of device configuration objects:
  - `address`: I2C address as hex string (e.g., "0x20")
  - `device_type`: Device type (0=MCP23017, 1=MCP23008)
  - `enabled`: Device enabled state
  - `detected`: Device detected on I2C bus
  - `gpio_count`: Number of GPIO pins (16 for MCP23017, 8 for MCP23008)
- `global_enabled`: Global enable/disable state

---

### POST `/api/mcp230xx/config`

Configure MCP230XX device type and enabled state.

**Request Body**:
```json
{
  "address": "0x20",
  "device_type": 0,
  "enabled": true
}
```

**Fields**:
- `address` (string): I2C address as hex string (e.g., "0x20")
- `device_type` (integer): Device type (0=MCP23017, 1=MCP23008)
- `enabled` (boolean): Enable/disable this device

**Response** (200 OK):
```json
{
  "status": "ok",
  "message": "Device configuration saved. Reboot required."
}
```

**Error Response** (400 Bad Request):
```json
{
  "status": "error",
  "message": "Address must be between 0x20 and 0x27"
}
```

---

## System Logs

### GET `/api/logs`

Get system logs from circular buffer.

**Response** (200 OK):
```json
{
  "logs": "Log entry 1\nLog entry 2\n...",
  "size": 32768
}
```

**Fields**:
- `logs`: Log entries as newline-separated string
- `size`: Buffer size in bytes

**Example**:
```bash
curl http://192.168.1.100/api/logs
```

---

## Error Responses

All endpoints may return error responses in the following format:

**400 Bad Request**:
```json
{
  "status": "error",
  "message": "Description of the error"
}
```

**500 Internal Server Error**:
```json
{
  "status": "error",
  "message": "Internal server error description"
}
```

---

## Notes

- **Base URL**: All endpoints are relative to `http://<device-ip>/api`
- **Content-Type**: POST requests should use `Content-Type: application/json` header (unless specified otherwise)
- **Reboot Required**: Some configuration changes require a device reboot to take effect. The response will indicate when a reboot is needed.
- **Thread Safety**: All assembly data access is thread-safe and protected by mutexes
- **Rate Limiting**: No rate limiting is currently implemented. Avoid excessive rapid requests.
- **Timeout**: Default HTTP request timeout applies. Long-running operations (e.g., calibration) may take several seconds.

---

## Examples

### Complete Device Status Check

```bash
curl http://192.168.1.100/api/status
```

### Update Network Configuration

```bash
curl -X POST http://192.168.1.100/api/ipconfig \
  -H "Content-Type: application/json" \
  -d '{
    "use_dhcp": false,
    "ip_address": "192.168.1.100",
    "netmask": "255.255.255.0",
    "gateway": "192.168.1.1"
  }'
```

### Calibrate NAU7802 Scale

```bash
# Step 1: Tare (zero offset)
curl -X POST http://192.168.1.100/api/nau7802/calibrate \
  -H "Content-Type: application/json" \
  -d '{"operation": "tare", "sample_count": 64}'

# Step 2: Calibrate with known weight (100 lbs)
curl -X POST http://192.168.1.100/api/nau7802/calibrate \
  -H "Content-Type: application/json" \
  -d '{"operation": "calibrate", "known_weight": 100.0, "sample_count": 64}'
```

### Set GPIO Outputs

```bash
# Set device at address 0x20 to output 0xFF on Port A
curl -X POST http://192.168.1.100/api/mcp230xx \
  -H "Content-Type: application/json" \
  -d '{"output": [255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
```

### OTA Firmware Update via URL

```bash
curl -X POST http://192.168.1.100/api/ota/update \
  -H "Content-Type: application/json" \
  -d '{"url": "http://example.com/firmware.bin"}'
```

### OTA Firmware Update via File Upload

```bash
curl -X POST http://192.168.1.100/api/ota/update \
  -F "firmware=@FusionCoreEnIP.bin"
```

---

## See Also

- [Assembly Data Layout](ASSEMBLY_DATA_LAYOUT.md) - Complete byte-by-byte assembly layout
- [README](../README.md) - Project overview and features

