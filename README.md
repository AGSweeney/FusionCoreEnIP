# FusionCoreEnIP

**EtherNet/IP Industrial I/O Adapter Stack for ESP32-P4**

FusionCoreEnIP is a comprehensive EtherNet/IP adapter device built on the ESP32-P4 platform, providing industrial-grade connectivity for multiple sensors and I/O modules. The device integrates seamlessly with PLCs and industrial automation systems through standard EtherNet/IP protocols, while offering a modern web interface and REST API for configuration and monitoring.

---

## Table of Contents

- [Features](#features)
- [Hardware Platform](#hardware-platform)
- [EtherNet/IP Protocol Support](#ethernetip-protocol-support)
- [Sensors and Input Devices](#sensors-and-input-devices)
- [Output Devices](#output-devices)
- [Network Features](#network-features)
- [Web Interface and REST API](#web-interface-and-rest-api)
- [Firmware Management](#firmware-management)
- [System Features](#system-features)
- [Project Structure](#project-structure)
- [Building](#building)
- [Documentation](#documentation)
- [License](#license)

---

## Features

### Core EtherNet/IP Functionality

- **OpENer EtherNet/IP Stack**: Full-featured EtherNet/IP adapter implementation
- **Input Assembly (100)**: 72-byte assembly containing sensor readings and device feedback
- **Output Assembly (150)**: 40-byte assembly for controlling device outputs
- **Explicit Messaging**: Support for explicit (request/response) messaging
- **Implicit I/O Messaging**: Cyclic data exchange via Input/Output assemblies
- **Connection Management**: Multiple simultaneous connections (Exclusive Owner, Input Only, Listen Only)
- **Configurable RPI**: Requested Packet Interval configuration per connection type
- **EDS File Support**: Embedded Electronic Data Sheet for device identification
- **CIP File Object (Class 0x37)**: On-device EDS file and icon serving
  - Instance 200: Embedded EDS file ("EDS.txt") available for download via RSLinx and EtherNet/IP tools
  - Instance 201: Device icon file ("EDSCollection.gz") for visualization in configuration tools
  - Automatic EDS file installation: RSLinx can automatically download and install EDS file from device
  - Compatible with Rockwell Studio 5000, RSLinx, and EtherNet/IP Explorer

### Sensor Integration

- **VL53L1X Time-of-Flight Distance Sensor**
  - Distance measurement up to 4 meters
  - Status flags and diagnostic data
  - Ambient light and signal rate information
  - SPAD count reporting
  - Update rate: ~10 Hz

- **LSM6DS3 6-DOF IMU (Inertial Measurement Unit)**
  - Roll, Pitch, and Ground Angle orientation calculations
  - Sensor fusion using complementary filter (96% gyroscope, 4% accelerometer)
  - Ground truth calculation: Angle from vertical using 3D angle formula
  - Calibration support: Accelerometer and gyroscope offset calibration
  - Configurable accelerometer and gyroscope settings
  - Zero offset support: Adjustable roll/pitch/yaw zero points
  - Update rate: ~10 Hz (sensor samples at 104 Hz internally)
  - See [LSM6DS3 Angle Calculation Documentation](docs/LSM6DS3_ANGLE_CALCULATION.md) for detailed implementation

- **NAU7802 24-Bit Weight Scale ADC**
  - High-precision load cell interface
  - Programmable gain (x1 to x128)
  - Multiple sample rates (10-320 SPS)
  - Dual-channel support
  - Comprehensive calibration (tare, known-weight, AFE)
  - Unit selection (grams, pounds, kilograms)
  - Update rate: ~10 Hz

### Input/Output Capabilities

- **MCP230XX GPIO Expanders**
  - Support for MCP23017 (16-bit) and MCP23008 (8-bit) devices
  - Up to 8 devices on I2C addresses 0x20-0x27
  - Real-time input reading and output control
  - Feedback loop for output verification
  - Update rate: ~50 Hz

- **GP8403 DAC (Digital-to-Analog Converter)**
  - 12-bit resolution (0-10V output per channel)
  - Up to 4 devices on I2C addresses 0x58-0x5B
  - 2 independent channels per device
  - Real-time voltage control
  - Continuous write on value change

### Network Features

- **Ethernet Connectivity**
  - 10/100 Mbps Ethernet interface
  - IP101 PHY support with media counters
  - Link status monitoring
  - MAC address configuration

- **IP Configuration**
  - DHCP client support
  - Static IP configuration
  - Gateway and DNS configuration
  - Hostname configuration
  - Persistent configuration in NVS

- **Address Conflict Detection (ACD)**
  - RFC 5227 compliant ACD can be enabled or disabled for both Static IP and DHCP configurations
  - ARP probe sequence before IP assignment (3 probes, ~200ms intervals)
  - Conflict detection with automatic retry logic (configurable attempts)
  - EtherNet/IP integration: TCP/IP Interface Object Attribute #10 (`select_acd`) for enable/disable control (applies to both Static IP and DHCP)
  - Conflict reporting: Attribute #11 provides conflicting device MAC address and ARP frame data
  - Persistent ACD setting: Configuration saved in NVS, persists across reboots
  - Visual LED indication: GPIO27 solid ON during conflicts, blinking during normal operation
  - Ongoing defensive ARP probes: Periodic probes every 90 seconds to actively defend assigned IP
  - RFC 5227 option (b) defense: First conflict defended, second conflict within 10 seconds triggers retreat

- **LLDP (Link Layer Discovery Protocol)**
  - IEEE 802.1AB compliant Link Layer Discovery Protocol implementation
  - LLDP Management Object (Class 0x109): Configuration and status management
    - Enable/disable LLDP per Ethernet interface
    - Message transmission interval configuration (default: 30 seconds)
    - Transmission hold multiplier (default: 4)
    - Last change timestamp tracking
  - LLDP Data Table Object (Class 0x10A): Neighbor information storage
    - Stores discovered neighbor device information
    - TLV (Type-Length-Value) data: Chassis ID, Port ID, System Name, System Description
    - System capabilities and management address information
  - Raw Ethernet frame transmission/reception using ESP-NETIF L2 TAP
  - Periodic LLDP frame transmission with configurable intervals
  - Neighbor discovery and topology mapping
  - Configuration via EtherNet/IP CIP objects using explicit messaging (Get/Set Attribute services)
  - **ODVA Compliance**: Implementation in progress to achieve ODVA EtherNet/IP compliance for network discovery and topology information

### Web Interface and REST API

- **HTTP Web Server**
  - Port 80 HTTP server
  - Responsive design for desktop and mobile
  - Self-contained (no external dependencies/CDN)

- **Web Pages**
  - `/` - Network configuration page (IP settings, DHCP/Static configuration)
  - `/ota` - Firmware update (OTA) interface
  - `/nau7802` - NAU7802 scale configuration and monitoring
  - `/vl53l1x` - VL53L1X distance sensor configuration and monitoring
  - `/lsm6ds3` - LSM6DS3 IMU configuration, calibration, and angle monitoring
  - `/gp8403` - GP8403 DAC configuration and output control
  - `/mcp230xx` - MCP230XX GPIO expander configuration and control
  - `/i2c` - I2C bus pull-up resistor configuration

- **REST API Endpoints**
  - Complete REST API with comprehensive JSON endpoints
  - Assembly data access (`/api/assemblies/*`, `/api/status`)
  - Sensor configuration and status (VL53L1X, LSM6DS3, NAU7802)
  - Sensor calibration endpoints (tare, offset, crosstalk calibration)
  - Network configuration management (`/api/ipconfig`)
  - OTA update control (`/api/ota/*`)
  - System logs retrieval (`/api/logs`)
  - I2C bus configuration (`/api/i2c/*`)
  - GPIO and DAC control (`/api/mcp230xx`, `/api/gp8403`)
  - Device reboot control (`/api/reboot`)
  - See [Complete API Documentation](docs/API_Endpoints.md) for detailed endpoint reference

### Firmware Management

- **Over-the-Air (OTA) Updates**
  - HTTP/HTTPS URL-based firmware updates
  - Direct binary upload via web interface
  - Streaming update support
  - Progress tracking and status reporting
  - Automatic rollback on failure
  - Boot partition management

- **Firmware Versioning**
  - Timestamped firmware builds
  - Git commit hash in build artifacts
  - Automatic firmware image archival

### System Features

- **I2C Bus Management**
  - Centralized I2C bus manager
  - Automatic device scanning and detection
  - Configurable pull-up resistors
  - Thread-safe I2C operations
  - Device count tracking

- **Non-Volatile Storage (NVS)**
  - Persistent configuration storage
  - Network settings persistence
  - Sensor calibration data storage
  - System configuration storage

- **Logging System**
  - Circular log buffer (32KB)
  - Boot sequence capture
  - Runtime log access via REST API
  - ESP-IDF logging framework integration

- **System Configuration**
  - Device-specific configuration management
  - MCP230XX device configuration
  - I2C bus configuration
  - Sensor enable/disable controls

- **User LED Indication**
  - GPIO27 status LED
  - Blinking during normal operation
  - Solid on during network conflicts
  - ACD status visualization

### Assembly Data Layout

This section describes the byte-by-byte layout of EtherNet/IP Assembly Objects 100 (Input Assembly) and 150 (Output Assembly).

**Note:** All multi-byte values are stored in **little-endian** byte order.

#### Input Assembly 100 (72 bytes)

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

#### Output Assembly 150 (40 bytes)

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

#### Byte Order and Data Format

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

## Hardware Platform

- **Microcontroller**: ESP32-P4
- **Development Board**: [Waveshare ESP32-P4 WiFi6 Dev Kit](https://www.waveshare.com/product/arduino/boards-kits/esp32-p4/esp32-p4-wifi6-dev-kit.htm) (used for development and testing)
- **Ethernet PHY**: IP101 (10/100 Mbps)
- **Interface**: I2C bus for sensor and I/O communication
- **User LED**: GPIO27 for status indication
- **Flash**: Partition-based firmware storage with OTA support

**Note on Hardware Support**: This project includes implementations for the hardware components listed in the Sensors and Output Devices sections above. However, **use of any specific hardware component is entirely optional** - you can configure the device to use none, some, or all of the supported hardware based on your application needs. The modular component architecture makes it straightforward to **add support for custom hardware** by implementing additional components following the existing patterns. Additionally, **any hardware supported by ESP32/ESP-IDF can be integrated** with some development effort, leveraging the full capabilities of the ESP32 platform. All hardware components can be individually enabled or disabled via configuration.

---

## EtherNet/IP Protocol Support

- **Protocol Stack**: OpENer EtherNet/IP implementation
- **Port**: Standard EtherNet/IP port 0xAF12 (44818)
- **Connection Types**: Exclusive Owner, Input Only, Listen Only
- **CIP Objects**: Standard EtherNet/IP CIP objects plus custom objects
- **File Object (Class 0x37)**: On-device EDS file and icon serving for automatic device configuration
  - Compatible with RSLinx automatic EDS file download
  - Supports Rockwell Studio 5000 and RSLinx (others untested)
  - Embedded EDS file accessible via CIP File Object services
- **LLDP Objects**: LLDP Management (Class 0x109) and Data Table (Class 0x10A) objects for network discovery
  - **LLDP Management Object (Class 0x109)**: Configure LLDP operation via EtherNet/IP explicit messaging
    - Attribute 1: `lldp_enable` - Enable/disable LLDP per interface (array of BOOL)
    - Attribute 2: `msg_tx_interval` - Transmission interval in seconds (UINT)
    - Attribute 3: `msg_tx_hold` - Transmission hold multiplier (USINT, typically 4)
    - Attribute 4: `lldp_datastore` - Data store identifier (WORD)
    - Attribute 5: `last_change` - Timestamp of last configuration change (UDINT)
  - **LLDP Data Table Object (Class 0x10A)**: Read neighbor information discovered via LLDP
    - Stores neighbor entries with Chassis ID, Port ID, TTL, and optional TLVs
    - Accessible via GetAttributeSingle and GetAttributeAll services
  - Configuration can be set via EtherNet/IP CIP explicit messaging to the LLDP Management Object attributes
  - **ODVA Compliance**: Active development to achieve full ODVA EtherNet/IP specification compliance for network topology discovery
- **TCP/IP Interface Object**: Enhanced with ACD control (Attribute #10) and conflict reporting (Attribute #11)
- **Message Router Object**: Modified to advertise supported CIP objects for automatic discovery

---

## Sensors and Input Devices

### VL53L1X Distance Sensor

- Long-range ToF sensor (up to 4 meters)
- Distance measurement in millimeters
- Status flags and diagnostic information
- Ambient light rate reporting
- Signal per SPAD measurement
- SPAD count information

### LSM6DS3 IMU

- 6-axis accelerometer and gyroscope
- Sensor fusion for orientation calculation
- Roll, Pitch, and Ground Angle output
- Configurable filter algorithms (Complementary/Madgwick)
- Full calibration support
- Configurable output data rates

### NAU7802 Weight Scale

- 24-bit ADC for load cells
- High precision weight measurement
- Multiple gain settings (x1 to x128)
- Configurable sample rates
- Dual-channel support
- Comprehensive calibration system

---

## Output Devices

### MCP230XX GPIO Expanders

- Digital output control via EtherNet/IP
- Real-time feedback of output states
- Support for multiple devices on I2C bus
- Individual pin control
- Port-based or pin-based access

### GP8403 DAC

- 0-10V analog output per channel
- 12-bit resolution (4096 steps)
- Independent channel control
- Multiple devices support
- Real-time voltage updates

---

## Network Features

### Address Conflict Detection (ACD)

- **RFC 5227 Compliant Implementation**: Full IPv4 Address Conflict Detection support
- **Enable/Disable Control**: ACD can be enabled or disabled for both Static IP and DHCP configurations via EtherNet/IP TCP/IP Interface Object Attribute #10 (`select_acd`)
- **Pre-assignment Detection**: ARP probe sequence (3 probes) before IP assignment (when ACD is enabled)
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

### LLDP Support

- **IEEE 802.1AB Compliant Implementation**: Full Link Layer Discovery Protocol support
- **Network Topology Discovery**: Automatic discovery of neighboring network devices (switches, routers, other EtherNet/IP devices)
- **Neighbor Information Storage**: LLDP Data Table Object stores discovered neighbor data including:
  - Chassis ID and Port ID for neighbor identification
  - System Name and System Description
  - System Capabilities (Station, Bridge, Router, etc.)
  - Management IP address
  - Time To Live (TTL) information
- **Periodic Frame Transmission**: Configurable LLDP frame transmission interval (default: 30 seconds)
- **Raw Ethernet Frame Handling**: Uses ESP-NETIF L2 TAP for direct Ethernet frame transmission/reception at Layer 2
- **EtherNet/IP Configuration**: LLDP can be configured via EtherNet/IP CIP objects using explicit messaging:
  - LLDP Management Object (Class 0x109) - Set transmission interval, enable/disable per interface
  - Configuration accessible via GetAttributeSingle and SetAttributeSingle services
  - Settings persist in NVS storage
- **ODVA Compliance**: Implementation in progress to achieve full ODVA EtherNet/IP specification compliance for network topology discovery and management information
- See [LLDP Component Documentation](components/lldp/README.md) for implementation details

---

## Web Interface and REST API

### Web Interface

Access the web interface at `http://<device-ip>` on port 80.

**Pages:**
- `/` - Network configuration page
- `/ota` - Firmware update interface
- `/nau7802` - Scale configuration and monitoring

### REST API

All API endpoints return JSON responses. See component documentation for complete API reference.

**Key Endpoints:**
- `/api/status` - Device status and assembly data
- `/api/ipconfig` - Network configuration (GET/POST)
- `/api/ota/*` - OTA update control
- `/api/logs` - System log retrieval
- `/api/reboot` - Device reboot control
- Sensor-specific endpoints for configuration and monitoring

---

## Firmware Management

### OTA Updates

- **Web Interface**: Upload firmware via web page
- **HTTP/HTTPS URL**: Download and update from remote server
- **Progress Tracking**: Real-time update progress
- **Error Handling**: Automatic rollback on failure
- **Status API**: Query update status and progress

### Build Artifacts

- Timestamped firmware binaries
- Automatic archival to `FirmwareImages/` directory
- Git commit hash in build metadata

---

## System Features

### I2C Bus Management

- Centralized bus initialization and management
- Automatic device discovery during boot
- Device count tracking (reported in Assembly data)
- Configurable pull-up resistor control (primary and secondary buses)
- Thread-safe operations with mutex protection
- Support for multiple I2C buses

### Configuration Persistence

- Non-volatile storage (NVS) for all settings
- Network configuration persistence (IP, DHCP, ACD settings)
- Sensor calibration data storage (LSM6DS3 offsets, NAU7802 calibration)
- Device-specific settings (MCP230XX, I2C pull-ups)
- ACD enable/disable state persistence
- Sensor enable/disable flags

### Logging

- Circular buffer for boot and runtime logs (32KB default)
- REST API access to log data (`/api/logs`)
- ESP-IDF logging framework integration
- Boot sequence capture for troubleshooting
- Thread-safe log buffering

### Network Stack Optimizations

- LWIP stack optimized for EtherNet/IP:
  - Increased socket limits (64 sockets, 128 UDP PCBs)
  - Larger TCP buffers (32KB send/receive windows)
  - IRAM optimizations for real-time performance
  - Task affinity configured for optimal core assignment:
    - **Core 0**: Network services (LWIP TCP/IP task, OpENer EtherNet/IP task, Web server)
    - **Core 1**: I/O services (sensor reading tasks: VL53L1X, LSM6DS3, NAU7802, GP8403, MCP230XX)
    - This separation provides dedicated CPU resources for real-time network communication while isolating I/O operations
- See [LWIP Modifications Documentation](docs/LWIP_MODIFICATIONS.md) for complete details

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
│   └── ...
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
- [LSM6DS3 Angle Calculation](docs/LSM6DS3_ANGLE_CALCULATION.md) - IMU sensor fusion algorithm details
- [File Object Integration](docs/FILE_OBJECT_INTEGRATION.md) - CIP File Object implementation for EDS file serving
- [LWIP Modifications](docs/LWIP_MODIFICATIONS.md) - LWIP stack optimizations and RFC 5227 ACD implementation
- [NAU7802 API Enhancements](docs/NAU7802_API_ENHANCEMENTS.md) - Weight scale API features and configuration

### Component READMEs

- `components/webui/README.md` - Web interface and REST API documentation
- `components/lldp/README.md` - LLDP protocol implementation
- `components/vl53l1x_uld/README.md` - VL53L1X sensor driver
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

