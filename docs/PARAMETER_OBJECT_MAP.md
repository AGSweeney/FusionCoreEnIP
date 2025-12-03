# Parameter Object Instance Map

## Overview

The Parameter Object (Class 0x0F) provides standardized access to device configuration parameters via EtherNet/IP. This document maps all parameter instances and their attributes.

**Total Instances:** 60 (39 active, 21 unused/reserved)  
**Highest Instance ID:** 60

---

## Parameter Instance Attributes

Each parameter instance has the following attributes:

- **Attribute 1:** Parameter Name (SHORT_STRING) - Read-only
- **Attribute 2:** Parameter Value (varies by data type) - Read/Write (if settable)
- **Attribute 3:** Parameter Units (SHORT_STRING) - Read-only
- **Attribute 4:** Help String (SHORT_STRING) - Read-only
- **Attribute 5:** Minimum Value (varies by data type) - Read-only (optional)
- **Attribute 6:** Maximum Value (varies by data type) - Read-only (optional)
- **Attribute 7:** Default Value (varies by data type) - Read-only (optional)
- **Attribute 8:** Data Type Code (USINT) - Read-only

---

## Network Configuration Parameters (Instances 1-9, 20)

| Instance ID | Parameter Name | Data Type | Units | Settable | Min | Max | Default | Description |
|------------|----------------|-----------|-------|----------|-----|-----|---------|-------------|
| 1 | IP Address | DWORD | - | Yes | - | - | - | Device IP address in network byte order |
| 2 | Subnet Mask | DWORD | - | Yes | - | - | - | Network subnet mask in network byte order |
| 3 | Gateway | DWORD | - | Yes | - | - | - | Default gateway IP address in network byte order |
| 4 | DNS Server 1 | DWORD | - | Yes | - | - | - | Primary DNS server IP address in network byte order |
| 5 | DNS Server 2 | DWORD | - | Yes | - | - | - | Secondary DNS server IP address in network byte order |
| 6 | DHCP Enabled | BOOL | - | Yes | False | True | True | Enable DHCP for automatic IP configuration (1=enabled, 0=static) |
| 7 | Hostname | STRING | - | Yes | - | - | - | Device hostname (max 64 characters) |
| 8 | Domain Name | STRING | - | Yes | - | - | - | Network domain name (max 48 characters) |
| 9 | Multicast TTL | USINT | - | Yes | 0 | 255 | 1 | Time-to-live value for multicast connections |
| 20 | ACD Enabled | BOOL | - | Yes | False | True | False | Enable Address Conflict Detection (1=enabled, 0=disabled) |
| 21 | ACD Timeout | UINT | seconds | Yes | 1 | 3600 | 10 | ACD timeout in seconds (1-3600, default: 10). Timeout for ACD conflict detection operations. |

**Note:** 
- Network parameters (IP, Subnet, Gateway, DNS, Hostname, Domain) require a reboot to take effect. Changes are persisted to NVS.
- Multicast TTL changes take effect on the next multicast connection.
- ACD Enabled changes take effect on the next network configuration.

---

## NAU7802 Scale Parameters (Instances 10-19)

| Instance ID | Parameter Name | Data Type | Units | Settable | Min | Max | Default | Description |
|------------|----------------|-----------|-------|----------|-----|-----|---------|-------------|
| 10 | NAU7802 Enabled | BOOL | - | Yes | False | True | True | Enable/disable NAU7802 weight scale sensor |
| 11 | NAU7802 Unit | USINT | - | Yes | 0 | 2 | - | Weight unit: 0=grams, 1=pounds, 2=kilograms |
| 12 | NAU7802 Gain | USINT | - | Yes | 0 | 7 | - | PGA gain setting: 0=x1, 1=x2, ..., 7=x128 |
| 13 | NAU7802 Sample Rate | USINT | SPS | Yes | 0 | 7 | - | Sample rate: 0=10, 1=20, 2=40, 3=80, 7=320 samples per second |
| 14 | NAU7802 Channel | USINT | - | Yes | 0 | 1 | - | Active channel: 0=Channel 1, 1=Channel 2 |
| 15 | NAU7802 LDO | USINT | V | Yes | 0 | 7 | - | LDO voltage setting: 0=4.5V, ..., 4=3.3V, ..., 7=2.4V |
| 16 | NAU7802 Average | USINT | samples | Yes | 1 | 50 | - | Number of samples to average for readings (1-50) |
| 17 | NAU7802 Cal Factor | USINT | - | - | - | - | - | **Reserved** - Calibration factor (not yet implemented) |
| 18 | NAU7802 Zero Offset | USINT | - | - | - | - | - | **Reserved** - Zero offset (not yet implemented) |
| 19 | *(Unused)* | - | - | - | - | - | - | *(Reserved - previously NAU7802 Byte Offset, removed as byte offsets are statically mapped)* |

**Note:** 
- NAU7802 parameters are persisted to NVS immediately when changed.
- Gain, Sample Rate, Channel, and LDO changes require a reboot for AFE recalibration.
- Enabled, Unit, and Average take effect immediately.
- Byte offsets in assembly data are statically mapped (NAU7802 at bytes 24-39) and cannot be configured.

---

## VL53L1X Time-of-Flight Sensor Parameters (Instances 22-37)

| Instance ID | Parameter Name | Data Type | Units | Settable | Min | Max | Default | Description |
|------------|----------------|-----------|-------|----------|-----|-----|---------|-------------|
| 22 | VL53L1X Enabled | BOOL | - | Yes | False | True | True | Enable/disable VL53L1X time-of-flight sensor |
| 23 | VL53L1X Distance Mode | UINT | - | Yes | 1 | 2 | 2 | Distance mode: 1=Short (<1.3m), 2=Long (<4m) |
| 24 | VL53L1X Timing Budget | UINT | ms | Yes | 15 | 500 | 100 | Timing budget in milliseconds (15, 20, 33, 50, 100, 200, 500) |
| 25 | VL53L1X Inter-Measurement | UDINT | ms | Yes | 15 | 10000 | 100 | Inter-measurement period in milliseconds (must be >= timing budget) |
| 26 | VL53L1X ROI X Size | UINT | - | Yes | 4 | 16 | 16 | Region of Interest width (4-16) |
| 27 | VL53L1X ROI Y Size | UINT | - | Yes | 4 | 16 | 16 | Region of Interest height (4-16) |
| 28 | VL53L1X ROI Center SPAD | USINT | - | Yes | 0 | 199 | 199 | ROI center SPAD index (0-199, 199=center) |
| 29 | VL53L1X Offset | INT | mm | Yes | -128 | 127 | 0 | Range offset in millimeters |
| 30 | VL53L1X Crosstalk | UINT | cps | Yes | 0 | 65535 | 0 | Crosstalk compensation in counts per second |
| 31 | VL53L1X Signal Threshold | UINT | kcps | Yes | 0 | 65535 | 1024 | Signal threshold in kilocounts per second |
| 32 | VL53L1X Sigma Threshold | UINT | mm | Yes | 0 | 65535 | 15 | Sigma threshold in millimeters |
| 33 | VL53L1X Threshold Low | UINT | mm | Yes | 0 | 4000 | 0 | Distance threshold low in millimeters (0=disabled) |
| 34 | VL53L1X Threshold High | UINT | mm | Yes | 0 | 4000 | 0 | Distance threshold high in millimeters (0=disabled) |
| 35 | VL53L1X Threshold Window | USINT | - | Yes | 0 | 3 | 0 | Threshold window: 0=Below, 1=Above, 2=Out, 3=In |
| 36 | VL53L1X Interrupt Polarity | USINT | - | Yes | 0 | 1 | 1 | Interrupt polarity: 0=Active Low, 1=Active High |
| 37 | VL53L1X I2C Address | USINT | - | Yes | 0x29 | 0x7F | 0x29 | I2C address (0x29-0x7F) |

**Note:** 
- VL53L1X parameters are persisted to NVS immediately when changed.
- All configuration changes require a reboot for sensor reconfiguration.
- Enabled state takes effect immediately (if sensor is initialized).

---

## Reserved Instances (38-49, 55-59)

| Instance ID | Status |
|------------|--------|
| 38-49 | Reserved for future use (e.g., LSM6DS3, MCP230XX, GP8403 parameters) |
| 55-59 | Reserved for future security/management parameters |

---

## Connection/Communication Parameters (Instances 50-54)

| Instance ID | Parameter Name | Data Type | Units | Settable | Min | Max | Default | Description |
|------------|----------------|-----------|-------|----------|-----|-----|---------|-------------|
| 50 | Default RPI | UINT | ms | Yes | 1 | 65535 | 1000 | Default Requested Packet Interval in milliseconds |
| 51 | Max Connections | USINT | - | No | 1 | 24 | 24 | Maximum number of simultaneous EtherNet/IP connections (read-only) |
| 52 | - | - | - | - | - | - | - | **Reserved** |
| 53 | Assembly 100 Size | USINT | bytes | No | - | - | 72 | Size of Input Assembly 100 in bytes (read-only) |
| 54 | Assembly 150 Size | USINT | bytes | No | - | - | 40 | Size of Output Assembly 150 in bytes (read-only) |

**Note:**
- Default RPI changes apply to new connections (no reboot required).
- Max Connections, Assembly 100 Size, and Assembly 150 Size are read-only system parameters.

---

## Security/Management Parameters (Instance 60)

| Instance ID | Parameter Name | Data Type | Units | Settable | Min | Max | Default | Description |
|------------|----------------|-----------|-------|----------|-----|-----|---------|-------------|
| 60 | Web API Enabled | BOOL | - | Yes | False | True | True | Enable/disable HTTP REST API (requires reboot to take effect) |

**Note:**
- **Default Behavior**: Web API is **enabled by default** for ease of initial setup and development
- **Security**: Can be disabled for production deployments to reduce attack surface
- **Requires Reboot**: Changes take effect only after device reboot
- **EtherNet/IP Always Available**: Disabling Web API does NOT affect EtherNet/IP functionality (port 44818)
- **Recovery**: If Web API is disabled, it can be re-enabled via EtherNet/IP (Parameter #60 = 1) and reboot
- **Persistence**: Setting is saved to NVS and persists across reboots
- **Use Case**: Disable Web API in locked-down production environments where only EtherNet/IP access is required

**Security Impact:**
- When **Enabled (1)**: HTTP server runs on port 80, REST API accessible, OTA updates available
- When **Disabled (0)**: HTTP server does not start, port 80 closed, only EtherNet/IP accessible

---

## Data Type Codes

| Data Type | CIP Code | Size (bytes) | Description |
|-----------|----------|--------------|-------------|
| BOOL | 0xC1 | 1 | Boolean (0 or 1) |
| STRING | 0xD0 | Variable | Character string (1 byte per character, 2-byte length) |
| USINT | 0xC6 | 1 | Unsigned 8-bit integer |
| UINT | 0xC7 | 2 | Unsigned 16-bit integer |
| UDINT | 0xC8 | 4 | Unsigned 32-bit integer |
| INT | 0xC3 | 2 | Signed 16-bit integer |
| DWORD | 0xD3 | 4 | 32-bit bit string (used for IP addresses) |

---

## Access Methods

### Read Parameter Value
- **Service:** GetAttributeSingle
- **Path:** `Class 0x0F, Instance <ID>, Attribute 2`
- **Example:** Read IP Address → `0x0F 01 02`

### Write Parameter Value (if settable)
- **Service:** SetAttributeSingle
- **Path:** `Class 0x0F, Instance <ID>, Attribute 2`
- **Example:** Set DHCP Enabled → `0x0F 06 02` with BOOL value

### Read All Attributes
- **Service:** GetAttributeAll
- **Path:** `Class 0x0F, Instance <ID>`
- **Returns:** All 8 attributes in sequence

### Read Parameter Metadata
- **Service:** GetAttributeSingle
- **Attributes:**
  - Attribute 1: Parameter Name
  - Attribute 3: Units
  - Attribute 4: Help String
  - Attribute 5: Minimum Value (if defined)
  - Attribute 6: Maximum Value (if defined)
  - Attribute 7: Default Value (if defined)
  - Attribute 8: Data Type Code

---

## NVS Persistence

The following parameters are automatically saved to Non-Volatile Storage (NVS) when changed:

### Network Parameters (require reboot)
- IP Address
- Subnet Mask
- Gateway
- DNS Server 1
- DNS Server 2
- DHCP Enabled
- Hostname
- Domain Name
- Multicast TTL (takes effect on next multicast connection)
- ACD Enabled (takes effect on next network configuration)

### NAU7802 Parameters (immediate or reboot required)
- Enabled (immediate)
- Unit (immediate)
- Gain (reboot required)
- Sample Rate (reboot required)
- Channel (reboot required)
- LDO (reboot required)
- Average (immediate)

### VL53L1X Parameters (reboot required)
- Enabled (immediate if sensor initialized)
- Distance Mode
- Timing Budget
- Inter-Measurement
- ROI X Size
- ROI Y Size
- ROI Center SPAD
- Offset
- Crosstalk
- Signal Threshold
- Sigma Threshold
- Threshold Low
- Threshold High
- Threshold Window
- Interrupt Polarity
- I2C Address

### Connection Parameters
- Default RPI (no persistence, applies to new connections only)

### Security/Management Parameters (reboot required)
- Web API Enabled (parameter instance #60, defaults to enabled)

---

## Implementation Notes

1. **Instance Allocation:** The Parameter Object allocates 60 instances to support non-sequential instance IDs. Unused instances (7-8, 38-49, 52, 55-59) are reserved for future expansion.

2. **Optional Attributes:** Attributes 5 (Min), 6 (Max), and 7 (Default) are always registered but return zero values when not defined for a parameter.

3. **PostSetCallback:** Writable parameters use a PostSetCallback to trigger NVS persistence and other side effects after a successful write.

4. **Validation:** Min/Max values are enforced for parameters that define them (e.g., NAU7802 Unit: 0-2, NAU7802 Gain: 0-7).

5. **Data Type Handling:** Each parameter's data type determines the encode/decode functions used for Attribute 2 (Parameter Value).

---

## Future Expansion

Reserved instance ranges for potential future parameters:

- **Instances 7-8:** Additional network configuration
- **Instances 20-21:** ACD configuration (implemented)
- **Instances 22-37:** VL53L1X time-of-flight sensor parameters (implemented)
- **Instances 38-49:** Reserved for future sensor and peripheral configuration
  - LSM6DS3 IMU sensor parameters
  - MCP230XX GPIO expander parameters
  - GP8403 DAC parameters
- **Instance 52:** Additional connection parameter
- **Instances 55-59:** Reserved for future security/management parameters
- **Instance 60:** Web API enable/disable (implemented)

---

## Revision History

- **v1.0** (2025-01): Initial implementation with 18 parameter instances
  - Network configuration (6 instances)
  - NAU7802 scale configuration (8 instances)
  - Connection parameters (4 instances)
- **v1.1** (2025-01): Added VL53L1X sensor parameters (instances 22-37)
  - 16 VL53L1X configuration parameters
  - Network configuration expanded (ACD settings added)
  - Total: 38 active parameter instances
- **v1.2** (2025-12): Security and quality improvements
  - Added Web API Enabled parameter (instance #60)
  - Fixed boolean parameter min/max values (instances 6, 10, 20, 22)
  - Increased total instances to 60
  - Total: 39 active parameter instances

