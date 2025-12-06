# LLDP Component for EtherNet/IP

This component provides IEEE 802.1AB compliant LLDP (Link Layer Discovery Protocol) support for EtherNet/IP adapter devices, including CIP Identification TLV transmission and reception for EtherNet/IP device discovery.

## Components

### CIP Objects
- **LLDP Management Object** (Class 0x109): Manages LLDP configuration and status
- **LLDP Data Table Object** (Class 0x10A): Stores LLDP neighbor information

### Platform Implementation
- **ESP32 Raw Socket Layer**: Raw Ethernet frame transmission/reception using ESP-NETIF L2 TAP
- **Frame Building Utilities**: Helper functions for constructing LLDP Ethernet frames

## Files

```
components/lldp/
├── CMakeLists.txt           # Component build configuration
├── README.md                # This file
├── include/
│   ├── lldp.h              # Public component interface
│   ├── ciplldpmanagement.h # LLDP Management Object
│   ├── ciplldpdatatable.h  # LLDP Data Table Object
│   ├── lldp_raw_socket.h   # ESP32 raw socket interface
│   └── lldp_frame_builder.h # Frame building utilities
└── src/
    ├── lldp_component.c    # Component initialization/deinitialization
    ├── ciplldpmanagement.c # LLDP Management Object implementation
    ├── ciplldpdatatable.c  # LLDP Data Table Object implementation
    └── esp32/
        ├── lldp_raw_socket.c      # ESP32 L2 TAP raw socket implementation
        └── lldp_frame_builder.c   # Ethernet frame building functions
```

## Usage

### Initialization

Initialize the LLDP component in your application (typically in `opener_init()`):

```c
#include "lldp.h"

void opener_init(struct netif *netif) {
    // ... other initialization ...
    
    #if OPENER_LLDP_ENABLED
    EipStatus status = LldpComponentInit(netif);
    if (status != kEipStatusOk) {
        // Handle error
    }
    #endif
}
```

### Cleanup

Deinitialize when shutting down:

```c
void cleanup() {
    #if OPENER_LLDP_ENABLED
    LldpComponentDeinit();
    #endif
}
```

## Configuration

LLDP can be enabled/disabled via `opener_user_conf.h`:

```c
#define OPENER_LLDP_ENABLED 1
#define OPENER_LLDP_TX_INTERVAL_MS 30000  // 30 seconds
```

## Dependencies

- **opener**: OpENer EtherNet/IP stack
- **lwip**: Network stack
- **freertos**: RTOS
- **esp_netif**: ESP-IDF network interface (ESP-IDF v5.0+)

## Status

### Fully Implemented

- ✅ Component structure and initialization
- ✅ ESP32 raw socket layer (L2 TAP) for Ethernet frame transmission/reception
- ✅ LLDP frame building (all mandatory and optional TLVs)
- ✅ LLDP frame reception and parsing (all TLVs)
- ✅ Periodic LLDP frame transmission with configurable interval
- ✅ Neighbor discovery and database management
- ✅ Automatic neighbor cleanup (expired entries)
- ✅ CIP Identification TLV transmission (ODVA OUI 0x001B1E, subtype 0x0E)
- ✅ CIP Identification TLV reception and storage
- ✅ LLDP Management Object (Class 0x109) with full attribute support
- ✅ LLDP Data Table Object (Class 0x10A) with CIP Identification (Attribute 7)
- ✅ Integration with OpENer EtherNet/IP stack
- ✅ NVS persistence for LLDP Management Object configuration

### Implemented TLV Types

**Mandatory TLVs:**
- Chassis ID (Type 1)
- Port ID (Type 2)
- TTL (Type 3)
- End of LLDPDU (Type 0)

**Optional TLVs:**
- System Name (Type 5)
- System Description (Type 6)
- System Capabilities (Type 7)
- Management Address (Type 8)
- CIP Identification (Type 127, Organization-Specific with ODVA OUI)

