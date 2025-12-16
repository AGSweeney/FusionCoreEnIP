# FusionCoreEnIP

**EtherNet/IP Industrial I/O Adapter Stack for ESP32-P4**

FusionCoreEnIP is a comprehensive EtherNet/IP adapter device built on the ESP32-P4 platform, providing industrial-grade connectivity for multiple sensors and I/O modules. The device integrates seamlessly with PLCs and industrial automation systems through standard EtherNet/IP protocols, while offering a modern web interface and REST API for configuration and monitoring.

---

## Table of Contents

- [Hardware Platform](#hardware-platform)
- [Device Testing Status](#device-testing-status)
- [EtherNet/IP Protocol Support](#ethernetip-protocol-support)
- [Sensors and Input Devices](#sensors-and-input-devices)
- [Output Devices](#output-devices)
- [Network Features](#network-features)
- [Web Interface and REST API](#web-interface-and-rest-api)
- [Firmware Management](#firmware-management)
- [System Features](#system-features)
- [Assembly Data Layout](#assembly-data-layout)
- [Project Structure](#project-structure)
- [Building](#building)
- [Documentation](#documentation)
- [License](#license)

---

## Hardware Platform

- **Microcontroller**: ESP32-P4
- **Development Board**: [Waveshare ESP32-P4 WiFi6 Dev Kit](https://www.waveshare.com/product/arduino/boards-kits/esp32-p4/esp32-p4-wifi6-dev-kit.htm) (used for development and testing)
- **Ethernet PHY**: IP101 (10/100 Mbps)
- **Interface**: I2C bus for sensor and I/O communication
- **User LED**: GPIO27 for status indication
- **Flash**: Partition-based firmware storage with OTA support

**Note on Hardware Support**: This project includes implementations for the hardware components listed in the Sensors and Output Devices sections. However, **use of any specific hardware component is entirely optional** - you can configure the device to use none, some, or all of the supported hardware based on your application needs. The modular component architecture makes it straightforward to **add support for custom hardware** by implementing additional components following the existing patterns. Additionally, **any hardware supported by ESP32/ESP-IDF can be integrated** with some development effort, leveraging the full capabilities of the ESP32 platform. All hardware components can be individually enabled or disabled via configuration.

---

## Device Testing Status

**All Supported Devices Validated and Confirmed Working**

All supported sensors, input devices, and output devices have been validated and confirmed working:

**Sensors and Input Devices:**
- ✅ **VL53L1X** Time-of-Flight Distance Sensor - Validated and confirmed working (max: 1 device)
- ✅ **LSM6DS3** 6-DOF IMU - Validated and confirmed working (max: 1 device)
- ✅ **NAU7802** 24-Bit Weight Scale ADC - Validated and confirmed working (max: 1 device)

**Output Devices:**
- ✅ **MCP230XX** GPIO Expanders - Validated and confirmed working (max: 8 devices)
- ✅ **GP8403** DAC (Digital-to-Analog Converter) - Validated and confirmed working (max: 4 devices)

**Hardware Verification Testing:**
During validation testing, all devices were tested simultaneously with the following configuration:

| Device | QTY Tested | Max Supported |
|--------|------------|---------------|
| VL53L1X | 1 | 1 |
| LSM6DS3 | 1 | 1 |
| NAU7802 | 1 | 1 |
| MCP23008 | 2 | 8 |
| GP8403 | 1 | 4 |

All devices operated correctly when connected simultaneously on the I2C bus.

---

## EtherNet/IP Protocol Support

### Core Functionality

- **OpENer EtherNet/IP Stack**: Full-featured EtherNet/IP adapter implementation
- **Input Assembly (100)**: 72-byte assembly containing sensor readings and device feedback
- **Output Assembly (150)**: 40-byte assembly for controlling device outputs
- **Explicit Messaging**: Support for explicit (request/response) messaging
- **Implicit I/O Messaging**: Cyclic data exchange via Input/Output assemblies
- **Connection Management**: Multiple simultaneous connections (Exclusive Owner, Input Only, Listen Only)
  - Up to 1 Exclusive Owner connection
  - Up to 1 Input Only connection (with 3 connection paths)
  - Up to 1 Listen Only connection (with 3 connection paths)
  - Up to 6 explicit connections
  - Maximum 20 simultaneous sessions
- **Configurable RPI**: Requested Packet Interval configuration per connection type
- **Port**: Standard EtherNet/IP port 0xAF12 (44818)

### Implemented CIP Objects

The device implements the following CIP objects:

- **Identity Object (Class 0x01)**: Device identification and status
- **Message Router Object (Class 0x02)**: CIP message routing and object discovery
- **DeviceNet Object (Class 0x03)**: DeviceNet protocol support
- **Assembly Object (Class 0x04)**: Input Assembly 100 and Output Assembly 150
- **Connection Manager Object (Class 0x06)**: Connection establishment and management
- **TCP/IP Interface Object (Class 0xF5)**: Network configuration with ACD support
- **Ethernet Link Object (Class 0xF6)**: Ethernet interface status and statistics
  - **Note**: MAC-level hardware statistics are limited due to ESP32-P4 hardware (MMC/RMON module not enabled)
  - PHY-level counters (IP101) are available: RX CRC errors, RX symbol errors
  - See [EMAC Statistics Limitation Documentation](docs/EMAC_STATISTICS_LIMITATION.md) for details
- **QoS Object (Class 0x48)**: Quality of Service configuration
  - All 8 DSCP attributes are readable (GetAttributeSingle service supported)
  - Attributes 4-8 are configurable (SetAttributeSingle service supported): Urgent, Scheduled, High, Low, and Explicit DSCP values
  - Attributes 1-3 are read-only: 802.1Q Tag Enable, DSCP PTP Event, DSCP PTP General
- **File Object (Class 0x37)**: EDS file and icon serving
- **Parameter Object (Class 0x0F)**: Device configuration parameters
- **Port Object (Class 0xF4)**: Communication port information
- **LLDP Management Object (Class 0x109)**: LLDP configuration and control
- **LLDP Data Table Object (Class 0x10A)**: LLDP neighbor information

### EDS File Support

- **EDS File Support**: Embedded Electronic Data Sheet for device identification
- **CIP File Object (Class 0x37)**: On-device EDS file and icon serving
  - Instance 200: Embedded EDS file ("EDS.txt") available for download via RSLinx and EtherNet/IP tools
  - Instance 201: Device icon file ("EDSCollection.gz") for visualization in configuration tools
  - Automatic EDS file installation: RSLinx can automatically download and install EDS file from device
  - Compatible with Rockwell Studio 5000, RSLinx, and EtherNet/IP Explorer
  - See [File Object Integration Documentation](docs/FILE_OBJECT_INTEGRATION.md) for implementation details

### LLDP Support

- **IEEE 802.1AB Compliant Implementation**: Full Link Layer Discovery Protocol support
- **LLDP Management Object (Class 0x109)**: Configuration and status management
  - Enable/disable LLDP per Ethernet interface
  - Message transmission interval configuration (default: 30 seconds)
  - Transmission hold multiplier (default: 4)
  - Last change timestamp tracking
- **LLDP Data Table Object (Class 0x10A)**: Neighbor information storage
  - Stores discovered neighbor device information
  - TLV (Type-Length-Value) data: Chassis ID, Port ID, System Name, System Description
  - System capabilities and management address information
  - **CIP Identification (Attribute 7)**: Stores CIP Identity information from neighbor devices (Vendor ID, Device Type, Product Code, Revision, Serial Number) when provided in LLDP frames
- **CIP Identification TLV Transmission**: Device transmits CIP Identification in outgoing LLDP frames
  - Organization-Specific TLV (Type 127) with ODVA OUI (0x001B1E)
  - CIP Identification subtype (0x0E) containing device identity (Vendor ID, Device Type, Product Code, Revision, Serial Number)
  - Enables other EtherNet/IP devices to discover this device's CIP Identity via LLDP
- **Raw Ethernet Frame Transmission/Reception**: Uses ESP-NETIF L2 TAP for direct Ethernet frame handling at Layer 2
- **Periodic LLDP Frame Transmission**: Configurable transmission intervals
- **Neighbor Discovery**: Automatic topology mapping of neighboring network devices
- **EtherNet/IP Configuration**: LLDP can be configured via EtherNet/IP CIP objects using explicit messaging (Get/Set Attribute services)
- **Compliance**: Supports EtherNet/IP compliance requirements for network discovery and topology information
- See [LLDP Component Documentation](components/lldp/README.md) for implementation details

### CIP Parameter Object (Class 0x0F)

- **Standard CIP Parameter Object**: Provides standardized access to device configuration parameters via EtherNet/IP
- **60 Parameter Instances**: Comprehensive device configuration and monitoring
  - Network configuration parameters (IP address, subnet, gateway, DNS, DHCP, hostname, domain, multicast TTL, ACD settings)
  - NAU7802 scale configuration (enabled, unit, gain, sample rate, channel, LDO, averaging)
  - VL53L1X sensor configuration (enabled, distance mode, timing budget, ROI settings, thresholds, calibration)
  - Connection parameters (default RPI, max connections, assembly sizes)
  - Security/management parameters (Web API enable/disable)
- **Services**: GetAttributeSingle, GetAttributeAll, SetAttributeSingle (for writable parameters)
- **NVS Persistence**: Automatic persistence of configuration changes to Non-Volatile Storage
- **Parameter Metadata**: Each parameter includes name, units, help string, min/max values, default values, and data type codes
- **Web API Control**: Parameter instance #60 allows enabling/disabling the HTTP REST API (requires reboot, defaults to enabled)
- See [Parameter Object Map Documentation](docs/PARAMETER_OBJECT_MAP.md) for complete parameter reference

### CIP Port Object (Class 0xF4)

- **Port Object**: Represents physical communication interface (port) on EtherNet/IP device
- **Port Information**: Provides port-level information and diagnostics
  - Port type (TCP/IP)
  - Port number
  - Link path to associated TCP/IP Interface Object
  - Port name and type name
- **Services**: GetAttributeSingle, GetAttributeAll

### Enhanced CIP Objects

- **TCP/IP Interface Object**: Enhanced with ACD control (Attribute #10) and conflict reporting (Attribute #11)
- **Message Router Object**: Modified to advertise supported CIP objects for automatic discovery

---

## Sensors and Input Devices

### VL53L1X Time-of-Flight Distance Sensor

- Distance measurement up to 4 meters
- Status flags and diagnostic data
- Ambient light and signal rate information
- SPAD count reporting
- Update rate: ~10 Hz

### LSM6DS3 6-DOF IMU (Inertial Measurement Unit)

- Roll, Pitch, and Ground Angle orientation calculations
- Sensor fusion using complementary filter (96% gyroscope, 4% accelerometer)
- Ground truth calculation: Angle from vertical using 3D angle formula
- Calibration support: Accelerometer and gyroscope offset calibration
- Configurable accelerometer and gyroscope settings
- Zero offset support: Adjustable roll/pitch/yaw zero points
- Update rate: ~10 Hz (sensor samples at 104 Hz internally)

### NAU7802 24-Bit Weight Scale ADC

- High-precision load cell interface
- Programmable gain (x1 to x128)
- Multiple sample rates (10-320 SPS)
- Dual-channel support
- Comprehensive calibration (tare, known-weight, AFE)
- Unit selection (grams, pounds, kilograms)
- Update rate: ~10 Hz

---

## Output Devices

### MCP230XX GPIO Expanders

- Support for MCP23017 (16-bit) and MCP23008 (8-bit) devices
- Up to 8 devices on I2C addresses 0x20-0x27
- Real-time input reading and output control
- Feedback loop for output verification
- Digital output control via EtherNet/IP
- Individual pin control with port-based or pin-based access
- Update rate: ~50 Hz

### GP8403 DAC (Digital-to-Analog Converter)

- 12-bit resolution (0-10V output per channel)
- Up to 4 devices on I2C addresses 0x58-0x5B
- 2 independent channels per device
- Real-time voltage control
- Continuous write on value change

---

## Network Features

### Ethernet Connectivity

- 10/100 Mbps Ethernet interface
- IP101 PHY support with media counters (RX CRC errors, RX symbol errors)
- **Hardware Limitation**: MAC-level statistics not available (MMC/RMON module not enabled in ESP32-P4)
- Link status monitoring
- MAC address configuration

### IP Configuration

- DHCP client support
- Static IP configuration
- Gateway and DNS configuration
- Hostname configuration
- Persistent configuration in NVS

### Address Conflict Detection (ACD)

- **RFC 5227 Compliant Implementation**: Full IPv4 Address Conflict Detection support
- **Enable/Disable Control**: ACD can be enabled or disabled for both Static IP and DHCP configurations via EtherNet/IP TCP/IP Interface Object Attribute #10 (`select_acd`)
- **Pre-assignment Detection**: ARP probe sequence (3 probes, ~200ms intervals) before IP assignment (when ACD is enabled)
- **EtherNet/IP Integration**: 
  - Control via TCP/IP Interface Object Attribute #10 (`select_acd`) - applies to both Static IP and DHCP
  - Conflict data available via Attribute #11 (conflicting MAC, ARP frame, activity status)
  - Persistent configuration in NVS (setting persists across reboots for both configuration types)
- **Configurable Timing**: Fully configurable timing parameters optimized for EtherNet/IP (currently set to: 200ms probe intervals, 2000ms announce intervals, 90s defensive probes). All timing values can be adjusted via ESP-IDF Kconfig menu
- **Automatic Retry Logic**: Configurable retry attempts with delay intervals
- **Visual Status Indication**: GPIO27 LED solid ON during conflicts, blinking during normal operation
- **Active IP Defense**: Periodic defensive ARP probes every 90 seconds (matches Rockwell PLC behavior)
- **Conflict Handling**: RFC 5227 option (b) - defends first conflict, retreats on second conflict within 10 seconds
- **Behavior When Disabled**: IP addresses are assigned immediately without conflict detection (both Static IP and DHCP)
- See [ACD Conflict Reporting Documentation](docs/ACD_CONFLICT_REPORTING.md) for complete details

---

## Web Interface and REST API

### Web Interface

Access the web interface at `http://<device-ip>` on port 80.

- **Port 80 HTTP server**
- **Responsive design** for desktop and mobile
- **Self-contained** (no external dependencies/CDN)

**Pages:**
- `/` - Network configuration page (IP settings, DHCP/Static configuration)
- `/ota` - Firmware update (OTA) interface
- `/nau7802` - NAU7802 scale configuration and monitoring
- `/vl53l1x` - VL53L1X distance sensor configuration and monitoring
- `/lsm6ds3` - LSM6DS3 IMU configuration, calibration, and angle monitoring
- `/gp8403` - GP8403 DAC configuration and output control
- `/mcp230xx` - MCP230XX GPIO expander configuration and control
- `/i2c` - I2C bus configuration (pull-up resistors and secondary bus enable/disable)

### REST API

Complete REST API with comprehensive JSON endpoints. All API endpoints return JSON responses.

**Key Endpoints:**
- `/api/status` - Device status and assembly data
- `/api/assemblies/*` - Assembly data access
- `/api/ipconfig` - Network configuration management (GET/POST)
- `/api/ota/*` - OTA update control
- `/api/logs` - System logs retrieval
- `/api/reboot` - Device reboot control
- `/api/i2c/*` - I2C bus configuration
- Sensor-specific endpoints for configuration, status, and calibration (VL53L1X, LSM6DS3, NAU7802)
- GPIO and DAC control (`/api/mcp230xx`, `/api/gp8403`)

See [Complete API Documentation](docs/API_Endpoints.md) for detailed endpoint reference.

---

## Firmware Management

### Over-the-Air (OTA) Updates

- **HTTP/HTTPS URL-based firmware updates**: Download and update from remote server
- **Direct binary upload**: Via web interface
- **Streaming update support**
- **Progress tracking**: Real-time update progress and status reporting
- **Error handling**: Automatic rollback on failure
- **Boot partition management**

### Firmware Versioning

- Timestamped firmware builds
- Git commit hash in build artifacts
- Automatic firmware image archival to `FirmwareImages/` directory

---

## System Features

### I2C Bus Management

- Centralized I2C bus manager
- Automatic device scanning and detection during boot
- Configurable pull-up resistors (primary and secondary buses)
- Secondary I2C bus enable/disable configuration (default: disabled)
  - Prevents initialization of secondary bus when not in use
  - Reduces power consumption and avoids issues on unpopulated buses
  - Configurable via web UI or NVS
- Improved error handling for empty/unpopulated buses
- Thread-safe I2C operations with mutex protection
- Device count tracking (reported in Assembly data)
- Support for multiple I2C buses

### Configuration Persistence

- **Non-Volatile Storage (NVS)**: Persistent configuration storage for all settings
- Network settings persistence (IP, DHCP, ACD settings)
- Sensor calibration data storage (LSM6DS3 offsets, NAU7802 calibration)
- Device-specific settings (MCP230XX, I2C pull-ups)
- ACD enable/disable state persistence
- Sensor enable/disable flags

### Logging System

- Circular log buffer (32KB default)
- Boot sequence capture
- Runtime log access via REST API (`/api/logs`)
- ESP-IDF logging framework integration
- Thread-safe log buffering

### System Configuration

- Device-specific configuration management
- MCP230XX device configuration
- I2C bus configuration
- Sensor enable/disable controls

### User LED Indication

- GPIO27 status LED
- Blinking during normal operation
- Solid on during network conflicts
- ACD status visualization

### Network Stack Optimizations

- **LWIP stack optimized for EtherNet/IP**:
  - Increased socket limits (64 sockets, 128 UDP PCBs)
  - Larger TCP buffers (32KB send/receive windows)
  - IRAM optimizations for real-time performance
  - Task affinity configured for optimal core assignment:
    - **Core 0 (pinned)**: Network services (LWIP TCP/IP task, OpENer EtherNet/IP task, Web server)
    - **Core 0 & 1 (unpinned)**: I/O services (sensor reading tasks: VL53L1X, LSM6DS3, NAU7802, GP8403, MCP230XX)
    - Network tasks are pinned to Core 0 for predictable, low-latency performance
    - I/O tasks are distributed across both cores by the FreeRTOS scheduler, reducing I2C bus contention
- See [LWIP Modifications Documentation](docs/LWIP_MODIFICATIONS.md) for complete details

---

## Assembly Data Layout

This section describes the byte-by-byte layout of EtherNet/IP Assembly Objects 100 (Input Assembly) and 150 (Output Assembly).

**Note:** All multi-byte values are stored in **little-endian** byte order.

### Input Assembly 100 (72 bytes)

The Input Assembly contains sensor readings and device feedback data. PLCs and other EtherNet/IP clients read this assembly to get current device status.

**Assembly Overview:**

| Byte Range | Device/Function | Size | Description |
|------------|----------------|------|-------------|
| 0-15 | VL53L1X | 16 bytes | Distance sensor data |
| 16-23 | LSM6DS3 | 8 bytes | IMU orientation data |
| 24-39 | NAU7802 | 16 bytes | Weight scale data |
| 40-55 | MCP230XX Feedback | 16 bytes | GPIO expander feedback |
| 56-60 | Device Counts | 5 bytes | Device detection counts |
| 61-71 | Reserved | 11 bytes | Reserved for future expansion |

**VL53L1X Distance Sensor (Bytes 0-15):**

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 0-1 | Distance | uint16_t | Distance measurement in millimeters (little-endian) |
| 2 | Status | uint8_t | Status flags (bit 0 = data available) |
| 3-4 | Ambient Rate | uint16_t | Ambient light rate (little-endian) |
| 5-6 | Signal Per SPAD | uint16_t | Signal rate per SPAD (little-endian) |
| 7-8 | Number of SPADs | uint16_t | Number of SPADs used (little-endian) |
| 9-15 | Reserved | - | Reserved bytes (unused) |

**LSM6DS3 IMU (Bytes 16-23):**

| Byte | Field Name | Data Type | Description |
|------|------------|-----------|-------------|
| 16-17 | Roll | int16_t | Roll angle scaled by 100 (-180.00° to +180.00°, little-endian) |
| 18-19 | Pitch | int16_t | Pitch angle scaled by 100 (-180.00° to +180.00°, little-endian) |
| 20-21 | Ground Angle | int16_t | Ground angle scaled by 100 (-180.00° to +180.00°, little-endian) |
| 22-23 | Reserved | - | Reserved bytes (unused) |

**Notes:**
- All angles are computed using complementary filter fusion algorithm
- Angles are stored as scaled integers (e.g., 48.66° = 4866, -123.45° = -12345)
- To get actual angle: divide stored value by 100.0

**NAU7802 Weight Scale (Bytes 24-39):**

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

**MCP230XX GPIO Feedback (Bytes 40-55):**

| Byte Range | Field Name | Data Type | Description |
|------------|------------|-----------|-------------|
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
- Devices are mapped by I2C address (0x20-0x27)

**Device Counts (Bytes 56-60):**

| Byte | Device Type | Data Type | Description |
|------|-------------|-----------|-------------|
| 56 | VL53L1X Count | uint8_t | Number of VL53L1X devices detected (0 or 1) |
| 57 | LSM6DS3 Count | uint8_t | Number of LSM6DS3 devices detected (0 or 1) |
| 58 | NAU7802 Count | uint8_t | Number of NAU7802 devices detected (0 or 1) |
| 59 | MCP230XX Count | uint8_t | Number of MCP230XX devices detected (0-8) |
| 60 | GP8403 Count | uint8_t | Number of GP8403 DAC devices detected (0-4) |

### Output Assembly 150 (40 bytes)

The Output Assembly contains control data that PLCs and other EtherNet/IP clients write to control device outputs.

**Assembly Overview:**

| Byte Range | Device/Function | Size | Description |
|------------|----------------|------|-------------|
| 0-15 | MCP230XX Output | 16 bytes | GPIO expander output control |
| 16-31 | GP8403 DAC | 16 bytes | DAC output values |
| 32-39 | Reserved | 8 bytes | Reserved for future expansion |

**MCP230XX GPIO Output (Bytes 0-15):**

| Byte Range | Field Name | Data Type | Description |
|------------|------------|-----------|-------------|
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

**GP8403 DAC Output (Bytes 16-31):**

| Byte Range | Field Name | Data Type | Description |
|------------|------------|-----------|-------------|
| 16-17 | Device 0x58 Ch0 | uint16_t | GP8403 at address 0x58, Channel 0 value (0-4095, 12-bit, little-endian) |
| 18-19 | Device 0x58 Ch1 | uint16_t | GP8403 at address 0x58, Channel 1 value (0-4095, 12-bit, little-endian) |
| 20-21 | Device 0x59 Ch0 | uint16_t | GP8403 at address 0x59, Channel 0 value (0-4095, 12-bit, little-endian) |
| 22-23 | Device 0x59 Ch1 | uint16_t | GP8403 at address 0x59, Channel 1 value (0-4095, 12-bit, little-endian) |
| 24-25 | Device 0x5A Ch0 | uint16_t | GP8403 at address 0x5A, Channel 0 value (0-4095, 12-bit, little-endian) |
| 26-27 | Device 0x5A Ch1 | uint16_t | GP8403 at address 0x5A, Channel 1 value (0-4095, 12-bit, little-endian) |
| 28-29 | Device 0x5B Ch0 | uint16_t | GP8403 at address 0x5B, Channel 0 value (0-4095, 12-bit, little-endian) |
| 30-31 | Device 0x5B Ch1 | uint16_t | GP8403 at address 0x5B, Channel 1 value (0-4095, 12-bit, little-endian) |

**Notes:**
- Maximum of 4 GP8403 devices supported (addresses 0x58-0x5B)
- Each device has 2 channels (Ch0 and Ch1)
- Each channel value is 12-bit (0-4095) stored in 2 bytes (little-endian)
- Devices are mapped by I2C address in ascending order

### Byte Order and Data Format

**Multi-byte Values:**

All multi-byte integers are stored in **little-endian** byte order:
- **uint16_t**: Low byte first, high byte second
  - Example: Value 0x1234 stored as `[0x34, 0x12]`
- **int16_t**: Low byte first, high byte second (two's complement)
  - Example: Value -100 stored as `[0x9C, 0xFF]` (0xFF9C in little-endian)
- **uint32_t/int32_t**: Lowest byte first, highest byte last
  - Example: Value 0x12345678 stored as `[0x78, 0x56, 0x34, 0x12]`

**Device Address Mapping:**

**MCP230XX Devices:**
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

**GP8403 DAC Devices:**
| I2C Address | Assembly Bytes | Description |
|-------------|---------------|-------------|
| 0x58 | 16-19 | Channel 0 (16-17), Channel 1 (18-19) |
| 0x59 | 20-23 | Channel 0 (20-21), Channel 1 (22-23) |
| 0x5A | 24-27 | Channel 0 (24-25), Channel 1 (26-27) |
| 0x5B | 28-31 | Channel 0 (28-29), Channel 1 (30-31) |

**Assembly Size Summary:**
| Assembly | Total Size | Used Bytes | Reserved Bytes |
|----------|-----------|------------|----------------|
| Input Assembly 100 | 72 bytes | 61 bytes | 11 bytes |
| Output Assembly 150 | 40 bytes | 32 bytes | 8 bytes |

**Implementation Notes:**
- **Thread Safety**: All assembly data access is protected by a mutex
- **Update Frequency**: 
  - VL53L1X: ~10 Hz (100 ms interval)
  - LSM6DS3: ~10 Hz (100 ms interval)
  - NAU7802: ~10 Hz (100 ms interval)
  - MCP230XX: ~50 Hz (20 ms interval)
  - GP8403: Continuous (writes on value change)
- **Device Detection**: Device counts are set during boot-time I2C scanning
- **Disabled Devices**: If a device is disabled via NVS or not detected, its assembly bytes remain at 0x00

---

## Project Structure

```
FusionCoreEnIP/
├── components/           # ESP-IDF components
│   ├── opener/          # OpENer EtherNet/IP stack
│   ├── webui/           # Web interface and REST API
│   ├── vl53l1x_manager/ # VL53L1X sensor manager
│   ├── lsm6ds3_manager/ # LSM6DS3 IMU manager
│   ├── nau7802_manager/ # NAU7802 scale manager
│   ├── gp8403_dac_manager/ # GP8403 DAC manager
│   ├── mcp230xx_manager/   # MCP230XX GPIO manager
│   ├── i2c_bus_manager/    # I2C bus management
│   ├── acd_manager/        # Address Conflict Detection
│   ├── ota_manager/        # OTA firmware updates
│   ├── system_config/      # System configuration
│   └── ...                 # Additional components
├── main/              # Main application code
├── docs/              # Documentation
├── eds/               # Electronic Data Sheet files
├── FirmwareImages/    # Build artifact storage
└── scripts/           # Build and utility scripts
```

---

## Building

### Prerequisites

- ESP-IDF v5.0 or later
- Python 3.x
- CMake 3.16 or later

### Build Steps

```bash
# Set up ESP-IDF environment
. $IDF_PATH/export.sh

# Configure project (optional)
idf.py menuconfig

# Build project
idf.py build

# Flash to device
idf.py flash

# Monitor serial output
idf.py monitor
```

### Configuration

Key configuration options available via `idf.py menuconfig`:

- **Ethernet Configuration**: 
  - PHY address configuration
  - MDC, MDIO, reset pin assignments
- **ACD Configuration** (OpENer → ACD Configuration):
  - Probe wait interval (default: 200ms)
  - Probe interval (default: 200ms)
  - Number of probe packets (default: 3)
  - Announce wait interval (default: 2000ms)
  - Number of announcement packets (default: 4)
  - Periodic defend interval (default: 90000ms)
  - Retry enable/disable and retry limits
- **I2C Bus Configuration**: 
  - Primary and secondary bus SDA/SCL pin assignments
  - Pull-up resistor enable/disable per bus
  - Secondary bus enable/disable (default: disabled, requires restart)
- **Sensor Configuration**: 
  - Per-sensor enable/disable controls
  - VL53L1X, LSM6DS3, NAU7802 individual enable flags
- **LWIP Stack Configuration**:
  - Socket limits, buffer sizes, task priorities
  - IRAM optimization options

---

## Documentation

### Technical Documentation

- [API Endpoints](docs/API_Endpoints.md) - Complete REST API reference with examples
- [ACD Conflict Reporting](docs/ACD_CONFLICT_REPORTING.md) - Address Conflict Detection implementation and EtherNet/IP integration
- [Assembly Data Layout](docs/ASSEMBLY_DATA_LAYOUT.md) - Complete byte-by-byte assembly layout
- [Parameter Object Map](docs/PARAMETER_OBJECT_MAP.md) - Complete reference for CIP Parameter Object instances and attributes
- [File Object Integration](docs/FILE_OBJECT_INTEGRATION.md) - CIP File Object implementation for EDS file serving
- [LWIP Modifications](docs/LWIP_MODIFICATIONS.md) - LWIP stack optimizations and RFC 5227 ACD implementation

### Component READMEs

- `components/webui/README.md` - Web interface and REST API documentation
- `components/lldp/README.md` - LLDP protocol implementation
- `components/vl53l1x_uld/README.md` - VL53L1X sensor driver
- `components/lsm6ds3/README.md` - LSM6DS3 IMU driver
- `components/nau7802/README.md` - NAU7802 scale driver
- `components/gp8403_dac/README.md` - GP8403 DAC driver
- `components/mcp23017/README.md` - MCP23017 GPIO expander
- `components/mcp23008/README.md` - MCP23008 GPIO expander

### Tools Documentation

- `tools/README.md` - Development and testing tools

---

## License

MIT License

Copyright (c) 2025 Adam G. Sweeney

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

## Contact

**Author**: Adam G. Sweeney  
**Email**: agsweeney@gmail.com  
**Website**: http://www.AGSweeney.net

---

## Acknowledgments

This project uses the OpENer EtherNet/IP stack, an open-source implementation of the EtherNet/IP protocol. Additional credits and acknowledgments can be found in individual component documentation.
