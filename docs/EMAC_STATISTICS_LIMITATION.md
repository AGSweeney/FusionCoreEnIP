# ESP32-P4 EMAC Hardware Statistics Limitation

## Official Espressif Confirmation

**Date:** 2025-12-02  
**Status:** Confirmed Hardware Limitation

Espressif has officially confirmed that **ESP32 and ESP32-P4 series chips do not enable the MMC/RMON module**, and therefore **do not provide hardware Ethernet statistics registers** based on MMC/RMON.

## What is MMC/RMON?

**MMC/RMON** = Media Independent Interface Management Counters / Remote Network Monitoring

- Standard Ethernet MAC statistics modules (IEEE 802.3)
- Provides hardware-level counters for:
  - Alignment errors
  - FCS/CRC errors
  - Collision counts (single, multiple, late, excessive)
  - Deferred transmissions
  - Carrier sense errors
  - Frame length errors
  - And many other MAC-level statistics

## Impact on EtherNet/IP Implementation

The EtherNet/IP **Ethernet Link Object (CIP Class 0xF6, Attribute #5)** requires media statistics counters for network diagnostics and compliance. 

### Available Counters

✅ **PHY-Level Counters (IP101 PHY):**
- RX CRC Error Counter (via MDIO Page 1, Register 18)
- RX Symbol Error Counter (via MDIO Page 11, Register 18)

❌ **MAC-Level Counters (ESP32-P4 EMAC):**
- **NOT AVAILABLE** - MMC/RMON module not enabled in hardware
- All MAC-level counters return zero:
  - Alignment errors
  - FCS errors (from MAC)
  - Frames too long
  - Single/multiple/late/excessive collisions
  - Deferred transmissions
  - Carrier sense errors
  - SQE test errors
  - MAC TX/RX error counts

## Technical Details

### Hardware Architecture

The ESP32-P4 EMAC IP core does not include or enable the MMC/RMON statistics module. This is a **hardware limitation**, not a software or driver issue.

### ESP-IDF API Status

- **ESP-IDF v4.x**: Had `soc/emac_reg.h` headers, but registers were not functional (MMC/RMON not enabled)
- **ESP-IDF v5.x**: Register headers removed from public API
- **Current (v5.5.1)**: No API available for hardware statistics

### Why This Matters

For industrial EtherNet/IP applications:
- ⚠️ **Partial compliance**: Can provide PHY-level statistics only
- ⚠️ **Limited diagnostics**: MAC-level network issues may not be detectable
- ⚠️ **Certification impact**: May affect EtherNet/IP conformance testing

## Workarounds

### Option 1: Software-Maintained Counters

Implement software-level packet monitoring:
- Use lwIP/netif callbacks to track packet statistics
- Count errors from packet processing
- Less accurate than hardware, but functional

**Status:** Not currently implemented

### Option 2: Accept Limitation

- Document that MAC-level counters return zero
- Use available PHY counters (IP101)
- Note limitation in EtherNet/IP compliance documentation

**Status:** Current approach

### Option 3: Request Future Support

- Request MMC/RMON enablement in future ESP32-P4 silicon revisions
- Request software-maintained counter APIs via `esp_eth_ioctl()`

**Status:** Inquiry submitted to Espressif

## Code Implementation

The codebase correctly handles this limitation:

```c
// ESP32-P4 EMAC: MMC/RMON module is NOT enabled in hardware
// Confirmed by Espressif: No hardware Ethernet statistics registers available
// Only PHY counters (IP101) are accessible via MDIO
#define EMAC_REGISTERS_NOT_AVAILABLE
```

All ESP32 variants (ESP32, ESP32-S2, ESP32-S3, ESP32-P4) are affected by this limitation.

## References

- **Espressif Response**: Confirmed MMC/RMON not enabled in ESP32/ESP32-P4 series
- **Inquiry Date**: 2025-12-02
- **Related Files**:
  - `components/opener/src/ports/ESP32/eth_media_counters.c`
  - `components/opener/src/ports/ESP32/eth_media_counters.h`

## Conclusion

This is a **confirmed hardware limitation** that affects all ESP32 series chips. The implementation correctly handles this by:
1. Using available PHY counters (IP101)
2. Returning zero for unavailable MAC counters
3. Documenting the limitation

For full EtherNet/IP compliance with complete statistics, alternative hardware platforms with MMC/RMON support would be required.

---

*Last Updated: 2025-12-02*  
*Status: Confirmed by Espressif Support*

