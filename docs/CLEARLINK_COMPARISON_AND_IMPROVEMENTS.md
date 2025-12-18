# ClearLink EtherNet/IP Comparison and Improvement Suggestions

## Overview

This document compares our current ClearPath servo implementation with Teknic ClearLink EtherNet/IP functionality and suggests improvements for servo control features (excluding GPIO/I/O).

**Reference:** [ClearLink EtherNet/IP Object Reference](https://teknic.com/files/downloads/clearlink_ethernet-ip_object_reference.pdf)

## Current Implementation Summary

### Output Assembly 150 (Commands)
- **Command Types:** Stop, Move Absolute, Move Relative, Move Velocity, Enable, Disable, Home
- **Data Format:** 1 byte command type + 4 bytes int32_t value
- **Limitations:** 
  - No per-move acceleration/deceleration override
  - No motion profile selection
  - No response type specification
  - Limited status feedback

### Input Assembly 100 (Feedback)
- **Current Fields:** Position (int32_t), Velocity (int16_t), Status (uint8_t)
- **Status Bits:** Move complete, Enabled, Fault, In position
- **Limitations:**
  - No "in motion" status
  - No "homed" status bit
  - No current direction indication
  - No executing command/block number
  - No motion state details

## ClearLink-Style Improvements

### 1. Enhanced Status Feedback (Input Assembly 100)

#### Current Status Byte (Byte 67)
```
Bit 0: Move complete
Bit 1: Enabled
Bit 2: Fault
Bit 3: In position
Bits 4-7: Reserved
```

#### Proposed Enhanced Status Word 1 (Byte 67)
```
Bit 0: In Motion (1 = actively moving, 0 = stopped)
Bit 1: Move Complete (1 = move finished, 0 = moving)
Bit 2: In Position (1 = within position window, 0 = not in position)
Bit 3: General Fault (1 = fault detected, 0 = no fault)
Bit 4: Current Direction (1 = forward/positive, 0 = reverse/negative)
Bit 5: Homed (1 = homing complete, 0 = not homed)
Bit 6: Enabled (1 = enabled, 0 = disabled)
Bit 7: Reserved
```

#### Proposed Additional Feedback Fields
- **Status Word 2 (Byte 68):** Extended status
  - Bit 0: Velocity mode active
  - Bit 1: Homing in progress
  - Bit 2: Decelerating
  - Bit 3: Accelerating
  - Bit 4: Constant velocity phase
  - Bits 5-7: Reserved

- **Motion State (Byte 69):** Current motion state enumeration
  - 0: Idle
  - 1: Moving (position mode)
  - 2: Velocity mode
  - 3: Homing
  - 4: Fault
  - 5-255: Reserved

- **Current Command Type (Byte 70):** Echo of last executed command
  - Mirrors Output Assembly command type for verification

- **Position Window Status (Byte 71):** Position tolerance status
  - Distance from target position (scaled)

### 2. Enhanced Command Structure (Output Assembly 150)

#### Current: Simple Command + Value
```
Byte 0: Command Type
Bytes 1-4: Value (int32_t)
```

#### Proposed: Extended Command Structure (8 bytes per servo)
```
Byte 0: Command Type
Byte 1: Response Type (what feedback to return)
Byte 2: Motion Profile (0=trapezoidal, 1=S-curve, 2=constant velocity)
Byte 3: Reserved
Bytes 4-7: Value (int32_t) - Position/Velocity/Acceleration
```

#### New Command Types
- **7: Move with Acceleration Override** - Override default acceleration for this move
- **8: Move with Deceleration Override** - Override default deceleration for this move
- **9: Move with Profile** - Specify motion profile (trapezoidal/S-curve)
- **10: Set Position** - Set current position without moving (for offset)
- **11: Clear Fault** - Clear fault condition
- **12: Get Status** - Request detailed status update

#### Response Type Field
Allows PLC to request specific feedback:
- 0: Standard feedback (position, velocity, status)
- 1: Extended feedback (add motion state, profile phase)
- 2: Diagnostic feedback (add error codes, internal state)

### 3. Configuration Assembly (New Assembly 200)

ClearLink uses a Configuration Assembly sent once at connection. We should add:

#### Configuration Assembly 200 Structure
```
Bytes 0-3:   Servo 0: Max Velocity (int32_t)
Bytes 4-7:   Servo 0: Max Acceleration (int32_t)
Bytes 8-11:  Servo 0: Max Deceleration (int32_t)
Bytes 12-15: Servo 0: Position Window (int32_t) - tolerance for "in position"
Bytes 16-19: Servo 0: Homing Velocity (int32_t)
Bytes 20-23: Servo 0: Homing Acceleration (int32_t)
Bytes 24-27: Servo 0: Reserved
Bytes 28-31: Servo 0: Reserved
[Repeat for Servos 1-3]
```

**Benefits:**
- Runtime configuration without code changes
- Per-servo parameter tuning
- PLC can adjust motion parameters dynamically

### 4. Motion Profile Enhancements

#### Current: Trapezoidal Only
- Acceleration → Constant Velocity → Deceleration

#### Proposed: Multiple Profiles
- **Trapezoidal** (current): Linear acceleration/deceleration
- **S-Curve**: Smooth acceleration/deceleration (sine/cosine based)
- **Constant Velocity**: No acceleration phase (instant start)
- **Jerk-Limited**: Control rate of change of acceleration

### 5. Position Window/Tolerance

#### Current: Binary "in position" based on exact match
- Only true when position == target_position

#### Proposed: Configurable Position Window
- **Position Window Parameter**: Tolerance in steps
- **In Position Status**: True when |position - target_position| <= window
- **Configurable per servo**: Different tolerances for different applications
- **Feedback**: Current distance from target in status byte

### 6. Enhanced Homing Feedback

#### Current: Basic homing with timeout
- Homing state, sensor status, completion flag

#### Proposed: Extended Homing Status
- **Homing State Bits:**
  - Searching for sensor
  - Sensor found, decelerating
  - At home position
  - Homing complete
- **Homing Error Codes:**
  - 0: Success
  - 1: Timeout
  - 2: Sensor not found
  - 3: Motion error during homing
- **Homing Position Offset**: Configurable offset from sensor trigger point

### 7. Command Verification and Echo

#### Current: Fire-and-forget commands
- No verification that command was received/executed

#### Proposed: Command Echo in Feedback
- **Current Command Type**: Echo of last command received
- **Command Sequence Number**: Incrementing counter for command tracking
- **Command Acknowledgment**: Status bit indicating command received

### 8. Velocity Feedback Enhancement

#### Current: int16_t velocity (±32,767 steps/second)
- May be limiting for high-speed applications

#### Proposed: int32_t velocity (±2,147,483,647 steps/second)
- Full 32-bit range matching position range
- Consistent with position data width

### 9. Acceleration/Deceleration Feedback

#### Proposed: Add to Input Assembly
- **Current Acceleration** (int32_t): Current acceleration rate
- **Current Deceleration** (int32_t): Current deceleration rate
- Useful for monitoring motion profile execution

### 10. Error Codes and Diagnostics

#### Current: Simple fault bit
- No detail about fault type

#### Proposed: Error Code Field
- **Error Code** (uint16_t): Specific error identifier
  - 0: No error
  - 1: Position error (following error)
  - 2: Velocity error
  - 3: Acceleration error
  - 4: Communication timeout
  - 5: Homing timeout
  - 6: Sensor error
  - 7-65535: Reserved/Extended errors

## Implementation Priority

### High Priority (Core Functionality)
1. ✅ Enhanced Status Word 1 (In Motion, Homed, Direction bits)
2. ✅ Motion State enumeration
3. ✅ Position Window/Tolerance support
4. ✅ Extended velocity feedback (int32_t)
5. ✅ Command echo/verification

### Medium Priority (Enhanced Features)
6. Configuration Assembly support
7. Per-move acceleration override
8. Motion profile selection (S-curve)
9. Enhanced homing feedback
10. Error codes

### Low Priority (Advanced Features)
11. Jerk-limited profiles
12. Diagnostic feedback modes
13. Response type selection
14. Command sequence numbers

## Recommended Assembly Layout Changes

### Input Assembly 100 - Enhanced (Expanded to 80+ bytes)

```
Bytes 61-64:   Position (int32_t) - Servo 0
Bytes 65-68:   Velocity (int32_t) - Servo 0 [CHANGED from int16_t]
Bytes 69:      Status Word 1 - Servo 0 [ENHANCED]
Bytes 70:      Status Word 2 - Servo 0 [NEW]
Bytes 71:      Motion State - Servo 0 [NEW]
Bytes 72:      Current Command Type - Servo 0 [NEW]
Bytes 73-74:   Error Code - Servo 0 [NEW]
Bytes 75-76:   Position Window Status - Servo 0 [NEW]
Bytes 77-80:   Reserved for Servo 0
Bytes 81-96:   Servo 1 (repeat structure)
Bytes 97-112:  Servo 2 (repeat structure)
Bytes 113-128: Servo 3 (repeat structure)
```

### Output Assembly 150 - Enhanced (Expanded to 64+ bytes)

```
Bytes 32-35:   Servo 0: Command Type + Extended Fields (4 bytes)
Bytes 36-39:   Servo 0: Value (int32_t)
Bytes 40-43:   Servo 0: Acceleration Override (int32_t, optional)
Bytes 44-47:   Servo 0: Deceleration Override (int32_t, optional)
[Repeat for Servos 1-3]
```

### Configuration Assembly 200 - New (128 bytes)

```
Bytes 0-127:   Configuration parameters for 4 servos (32 bytes each)
```

## Code Changes Required

### 1. Status Word Enhancement
- Update `clearpath_servo_manager_get_status()` to return enhanced status word
- Add helper functions for individual status bits
- Update assembly write logic

### 2. Motion State Tracking
- Add `clearpath_servo_motion_state_t` enum
- Track current motion state in servo handle
- Update state machine transitions

### 3. Position Window
- Add `position_window` to servo config
- Implement `clearpath_servo_is_in_position()` with window tolerance
- Update status bit calculation

### 4. Command Echo
- Store last command type in servo handle
- Write to Input Assembly feedback

### 5. Configuration Assembly
- Create new assembly handler
- Implement parameter getters/setters
- Add NVS storage for configuration

## Benefits of These Improvements

1. **Better PLC Integration**: Matches industry-standard EtherNet/IP motion control patterns
2. **Enhanced Diagnostics**: Detailed status and error codes improve troubleshooting
3. **Flexible Configuration**: Runtime parameter adjustment without code changes
4. **Improved Feedback**: More detailed status information for better control
5. **Professional Features**: S-curve profiles, position windows, command verification
6. **Compatibility**: Easier integration with existing PLC programs expecting ClearLink-style data

## Migration Path

1. **Phase 1**: Add enhanced status bits (backward compatible)
2. **Phase 2**: Expand velocity to int32_t (may require PLC updates)
3. **Phase 3**: Add Configuration Assembly
4. **Phase 4**: Add extended command types
5. **Phase 5**: Add motion profiles (S-curve)

Each phase can be implemented incrementally without breaking existing functionality.
