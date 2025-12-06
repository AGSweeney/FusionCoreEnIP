# OpENer - EtherNet/IP Communication Stack for ESP-IDF

OpENer is an open-source EtherNet/IP communication stack ported for ESP-IDF. It provides a full implementation of EtherNet/IP adapter functionality, supporting explicit messaging, implicit I/O connections, and CIP objects.

## Features

- Full EtherNet/IP adapter implementation
- Explicit messaging (CIP)
- Implicit I/O connections (Class 3)
- Multiple CIP objects:
  - Identity Object
  - Message Router Object
  - Assembly Object
  - Connection Manager Object
  - Connection Object
  - TCP/IP Interface Object
  - Ethernet Link Object
  - Parameter Object (optional, requires system_config)
  - File Object (for EDS/icon serving)
- ESP32 port with FreeRTOS integration
- NVS-based non-volatile data storage

## Requirements

### Required Components

- ESP-IDF v4.1 or later
- lwip (ESP-IDF component)
- freertos (ESP-IDF component)
- esp_eth (ESP-IDF component)
- esp_netif (ESP-IDF component)
- driver (ESP-IDF component)
- nvs_flash (ESP-IDF component)

### Optional Components

- **system_config**: Required for Parameter Object functionality (device-specific configuration parameters). Currently listed in REQUIRES - remove from CMakeLists.txt if not needed and set `OPENER_SYSTEM_CONFIG_ENABLED=0`.
- **lldp**: Optional LLDP (Link Layer Discovery Protocol) support. Currently listed in REQUIRES - remove from CMakeLists.txt if not needed and set `OPENER_LLDP_ENABLED=0`.

## Installation

### Method 1: Copy Component Folder

1. Copy the `opener` folder to your project's `components` directory
2. Add `opener` to your component's `REQUIRES` in `CMakeLists.txt`:

```cmake
idf_component_register(
    ...
    REQUIRES
        opener
        lwip
        esp_netif
        ...
)
```

### Method 2: Component Manager

Add to your project's `idf_component.yml`:

```yaml
dependencies:
  opener:
    git: "https://github.com/yourusername/opener-esp-idf.git"
    # or
    path: "../path/to/opener"
```

## Configuration

### Basic Usage

The component is ready to use after installation. Basic initialization:

```c
#include "opener.h"
#include "networkconfig.h"

// Initialize OpENer
OpenerInit();
```

### Optional Dependencies

#### System Config (for Parameter Object)

If you need the Parameter Object with device-specific configuration parameters, add `system_config` to your component's `REQUIRES`:

```cmake
idf_component_register(
    ...
    REQUIRES
        opener
        system_config
        ...
)
```

To disable system_config functionality even if the component is present:

```cmake
target_compile_definitions(main PRIVATE OPENER_SYSTEM_CONFIG_ENABLED=0)
```

#### LLDP Support

LLDP support is enabled by default if the `lldp` component is found. To disable:

```cmake
target_compile_definitions(main PRIVATE OPENER_LLDP_ENABLED=0)
```

Or set it in `opener_user_conf.h`:

```c
#define OPENER_LLDP_ENABLED 0
```

### File Object Configuration

The File Object allows serving EDS and icon files directly from the device. To enable:

1. Provide EDS file path in your project's `CMakeLists.txt`:

```cmake
set(OPENER_EDS_FILE_PATH "${CMAKE_SOURCE_DIR}/eds/your_device.eds")
```

2. Optionally provide icon file:

```cmake
set(OPENER_ICON_FILE_PATH "${CMAKE_SOURCE_DIR}/eds/favicon.ico")
```

These files will be automatically embedded into the firmware during build.

**Note**: If not provided, empty placeholders will be generated. The File Object instances will still be created but will serve empty data.

## Usage Example

```c
#include "opener.h"
#include "networkconfig.h"
#include "opener_api.h"

void app_main(void)
{
    // Configure network interface
    ConfigureNetworkInterface("192.168.1.100", "255.255.255.0", "192.168.1.1");
    ConfigureMACAddress((uint8_t[]){0x00, 0x15, 0xC5, 0xBF, 0xD0, 0x87});
    ConfigureHostName("my-eip-device");
    ConfigureDomainName("local");

    // Set device serial number (unique per device)
    setDeviceSerialNumber(0x12345678);

    // Initialize OpENer stack
    CipStackInit(0x1234);

    // Initialize ESP32 port
    OpenerInit();

    // Your application code...
}
```

## API Documentation

The main API is defined in `opener_api.h`. Key functions:

- `CipStackInit()` - Initialize the CIP stack
- `ManageConnections()` - Call periodically (every 10ms recommended)
- `HandleReceivedExplictTCPData()` - Handle explicit TCP messages
- `HandleReceivedExplictUDPData()` - Handle explicit UDP messages
- `HandleReceivedConnectedData()` - Handle implicit I/O data

See the OpENer documentation for complete API reference.

## Component Structure

```
opener/
├── CMakeLists.txt          # Component build configuration
├── idf_component.yml       # Component manifest
├── LICENSE.txt             # BSD-style license
├── README.md               # This file
└── src/
    ├── cip/                # CIP layer implementation
    ├── cip_objects/        # CIP object implementations
    │   └── OpENerFileObject/
    ├── enet_encap/         # Ethernet encapsulation layer
    ├── ports/              # Platform-specific ports
    │   └── ESP32/          # ESP32-specific implementation
    ├── utils/              # Utility functions
    └── opener_api.h        # Main API header
```

## Building

The component integrates with ESP-IDF's build system automatically. No special build steps required:

```bash
idf.py build
```

## Troubleshooting

### Missing system_config errors

If you see errors about `system_config.h` not found:
- Add `system_config` to your component's `REQUIRES`, or
- Set `OPENER_SYSTEM_CONFIG_ENABLED=0` to disable Parameter Object features

### Missing lldp errors

If you see errors about `lldp.h` not found:
- Add `lldp` to your component's `REQUIRES`, or
- Set `OPENER_LLDP_ENABLED=0` to disable LLDP support

### File Object not serving files

Ensure `OPENER_EDS_FILE_PATH` and optionally `OPENER_ICON_FILE_PATH` are set in your `CMakeLists.txt` before the opener component is processed.

## License

OpENer is licensed under an adapted BSD-style license. See `LICENSE.txt` for details.

**Important**: EtherNet/IP is a trademark of ODVA, Inc. Use of this software may require a license from ODVA through its Terms of Usage Agreement. See `LICENSE.txt` for complete license terms.

## References

- [OpENer Official Repository](https://github.com/EIPStackGroup/OpENer)
- [ODVA EtherNet/IP Technology](https://www.odva.org/)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)

