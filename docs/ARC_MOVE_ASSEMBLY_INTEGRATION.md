# Arc Move Assembly Integration

This document describes how to integrate coordinated motion commands (linear interpolation, circular interpolation, and helical arcs) into the EtherNet/IP Output Assembly 150 structure.

## Overview

The current Output Assembly 150 structure supports single-axis servo commands (bytes 32-51, 5 bytes per servo). Arc moves and other coordinated motion operations require multiple parameters and involve multiple axes simultaneously, so they need a dedicated command section.

## Recommended Approach

**Option 1: Dedicated Coordinated Motion Section with Integer-Scaled Parameters**

- Expand Output Assembly 150 to 128 bytes
- Add coordinated motion command block at bytes 52-95 (44 bytes)
- Use integer-scaled parameters (no floating-point serialization)
- Command types 7-11 for coordinated motion
- Manager task processes coordinated motion commands before single-axis commands

## Output Assembly 150 Layout

### Current Structure (Bytes 0-51)

| Byte Range | Description | Size |
|------------|-------------|------|
| 0-31 | Other device commands | 32 bytes |
| 32-51 | Single-axis servo commands | 20 bytes (5 bytes per servo × 4 servos) |

### Extended Structure (Bytes 0-127)

| Byte Range | Description | Size |
|------------|-------------|------|
| 0-31 | Other device commands | 32 bytes |
| 32-51 | Single-axis servo commands | 20 bytes |
| 52-95 | Coordinated motion commands | 44 bytes |
| 96-127 | Reserved for future expansion | 32 bytes |

## Coordinated Motion Command Block (Bytes 52-95)

### Command Structure

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 52 | Command Type | uint8_t | See command types below |
| 53 | Axis Count | uint8_t | Number of axes (2-4) |
| 54 | Axis X Index | uint8_t | Servo index for X axis (0-3) |
| 55 | Axis Y Index | uint8_t | Servo index for Y axis (0-3) |
| 56 | Axis Z Index | uint8_t | Servo index for Z axis (0-3, 0xFF if not used) |
| 57 | Flags | uint8_t | Bit flags (see below) |
| 58-61 | Param 1 | int32_t | See parameter mapping below |
| 62-65 | Param 2 | int32_t | See parameter mapping below |
| 66-69 | Param 3 | int32_t | See parameter mapping below |
| 70-73 | Param 4 | int32_t | See parameter mapping below |
| 74-77 | Param 5 | int32_t | See parameter mapping below |
| 78-81 | Param 6 | int32_t | See parameter mapping below |
| 82-85 | Param 7 | int32_t | See parameter mapping below |
| 86-89 | Param 8 | int32_t | See parameter mapping below |
| 90-93 | Reserved | - | Reserved for future use |
| 94-95 | Reserved | - | Reserved for future use |

### Command Types

| Value | Command | Description |
|-------|---------|-------------|
| 0 | None/Idle | No coordinated motion command (default) |
| 7 | Arc Center | 2-axis circular arc using center point |
| 8 | Arc To Point | 2-axis circular arc using end point + radius |
| 9 | Arc Helical Center | 3-axis helical arc using center point |
| 10 | Arc Helical To Point | 3-axis helical arc using end point + radius |
| 11 | Linear | Multi-axis linear interpolation |

### Flags Byte (Byte 57)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | Clockwise | 1 = clockwise, 0 = counter-clockwise (for arc commands) |
| 1 | Absolute | 1 = absolute coordinates, 0 = relative (for _to_point commands) |
| 2-7 | Reserved | Reserved for future use |

## Parameter Mapping by Command Type

### Command Type 7: Arc Center (2-axis)

Moves X and Y axes in a circular arc using center point specification.

**Parameters:**
- Param 1 (bytes 58-61): Center X (int32_t, scaled by 1000: 100.5mm = 100500)
- Param 2 (bytes 62-65): Center Y (int32_t, scaled by 1000: 50.25mm = 50250)
- Param 3 (bytes 66-69): Radius (int32_t, scaled by 1000: 25.0mm = 25000)
- Param 4 (bytes 70-73): Start Angle (int32_t, in 0.1 degree units: 90.5° = 905)
- Param 5 (bytes 74-77): End Angle (int32_t, in 0.1 degree units: 180.0° = 1800)
- Param 6 (bytes 78-81): Feedrate (int32_t, scaled by 100: 1000.5 mm/min = 100050)
- Param 7-8: Unused

**Flags:**
- Bit 0: Clockwise (1 = clockwise, 0 = counter-clockwise)

**Example:**
```
Byte 52: 7 (arc_center)
Byte 53: 2 (axis count)
Byte 54: 0 (X axis servo index)
Byte 55: 1 (Y axis servo index)
Byte 56: 0xFF (Z axis not used)
Byte 57: 0x01 (clockwise)
Bytes 58-61: 100000 (center X = 100.000 mm)
Bytes 62-65: 100000 (center Y = 100.000 mm)
Bytes 66-69: 50000 (radius = 50.000 mm)
Bytes 70-73: 0 (start angle = 0.0°)
Bytes 74-77: 3600 (end angle = 360.0°)
Bytes 78-81: 100000 (feedrate = 1000.00 mm/min)
```

### Command Type 8: Arc To Point (2-axis)

Moves X and Y axes in a circular arc from current position to end point using radius.

**Parameters:**
- Param 1 (bytes 58-61): End X (int32_t, scaled by 1000)
- Param 2 (bytes 62-65): End Y (int32_t, scaled by 1000)
- Param 3 (bytes 66-69): Radius (int32_t, scaled by 1000)
- Param 4 (bytes 70-73): Feedrate (int32_t, scaled by 100)
- Param 5-8: Unused

**Flags:**
- Bit 0: Clockwise (1 = clockwise, 0 = counter-clockwise)
- Bit 1: Absolute (1 = absolute coordinates, 0 = relative)

**Example:**
```
Byte 52: 8 (arc_to_point)
Byte 53: 2 (axis count)
Byte 54: 0 (X axis servo index)
Byte 55: 1 (Y axis servo index)
Byte 56: 0xFF (Z axis not used)
Byte 57: 0x03 (clockwise + absolute)
Bytes 58-61: 150000 (end X = 150.000 mm, absolute)
Bytes 62-65: 100000 (end Y = 100.000 mm, absolute)
Bytes 66-69: 50000 (radius = 50.000 mm)
Bytes 70-73: 100000 (feedrate = 1000.00 mm/min)
```

### Command Type 9: Arc Helical Center (3-axis)

Moves X and Y axes in a circular arc while simultaneously moving Z axis linearly (helical motion).

**Parameters:**
- Param 1 (bytes 58-61): Center X (int32_t, scaled by 1000)
- Param 2 (bytes 62-65): Center Y (int32_t, scaled by 1000)
- Param 3 (bytes 66-69): Radius (int32_t, scaled by 1000)
- Param 4 (bytes 70-73): Start Angle (int32_t, in 0.1 degree units)
- Param 5 (bytes 74-77): End Angle (int32_t, in 0.1 degree units)
- Param 6 (bytes 78-81): Z Start (int32_t, scaled by 1000)
- Param 7 (bytes 82-85): Z End (int32_t, scaled by 1000)
- Param 8 (bytes 86-89): Feedrate (int32_t, scaled by 100)

**Flags:**
- Bit 0: Clockwise (1 = clockwise, 0 = counter-clockwise)

**Example:**
```
Byte 52: 9 (arc_helical_center)
Byte 53: 3 (axis count)
Byte 54: 0 (X axis servo index)
Byte 55: 1 (Y axis servo index)
Byte 56: 2 (Z axis servo index)
Byte 57: 0x01 (clockwise)
Bytes 58-61: 100000 (center X = 100.000 mm)
Bytes 62-65: 100000 (center Y = 100.000 mm)
Bytes 66-69: 50000 (radius = 50.000 mm)
Bytes 70-73: 0 (start angle = 0.0°)
Bytes 74-77: 900 (end angle = 90.0°)
Bytes 78-81: 0 (Z start = 0.000 mm)
Bytes 82-85: 10000 (Z end = 10.000 mm)
Bytes 86-89: 100000 (feedrate = 1000.00 mm/min)
```

### Command Type 10: Arc Helical To Point (3-axis)

Moves X and Y axes in a circular arc while simultaneously moving Z axis linearly, from current position to end point.

**Parameters:**
- Param 1 (bytes 58-61): End X (int32_t, scaled by 1000)
- Param 2 (bytes 62-65): End Y (int32_t, scaled by 1000)
- Param 3 (bytes 66-69): End Z (int32_t, scaled by 1000)
- Param 4 (bytes 70-73): Radius (int32_t, scaled by 1000)
- Param 5 (bytes 74-77): Feedrate (int32_t, scaled by 100)
- Param 6-8: Unused

**Flags:**
- Bit 0: Clockwise (1 = clockwise, 0 = counter-clockwise)
- Bit 1: Absolute (1 = absolute coordinates, 0 = relative)

**Example:**
```
Byte 52: 10 (arc_helical_to_point)
Byte 53: 3 (axis count)
Byte 54: 0 (X axis servo index)
Byte 55: 1 (Y axis servo index)
Byte 56: 2 (Z axis servo index)
Byte 57: 0x03 (clockwise + absolute)
Bytes 58-61: 150000 (end X = 150.000 mm, absolute)
Bytes 62-65: 100000 (end Y = 100.000 mm, absolute)
Bytes 66-69: 20000 (end Z = 20.000 mm, absolute)
Bytes 70-73: 50000 (radius = 50.000 mm)
Bytes 74-77: 100000 (feedrate = 1000.00 mm/min)
```

### Command Type 11: Linear (Multi-axis)

Moves multiple axes in a straight line to target positions.

**Parameters:**
- Param 1 (bytes 58-61): Target X (int32_t, scaled by 1000)
- Param 2 (bytes 62-65): Target Y (int32_t, scaled by 1000)
- Param 3 (bytes 66-69): Target Z (int32_t, scaled by 1000, 0 if not used)
- Param 4 (bytes 70-73): Target A (int32_t, scaled by 1000, 0 if not used)
- Param 5 (bytes 74-77): Feedrate (int32_t, scaled by 100)
- Param 6-8: Unused

**Flags:**
- Bit 1: Absolute (1 = absolute coordinates, 0 = relative)

**Example (2-axis):**
```
Byte 52: 11 (linear)
Byte 53: 2 (axis count)
Byte 54: 0 (X axis servo index)
Byte 55: 1 (Y axis servo index)
Byte 56: 0xFF (Z axis not used)
Byte 57: 0x02 (absolute)
Bytes 58-61: 100000 (target X = 100.000 mm)
Bytes 62-65: 50000 (target Y = 50.000 mm)
Bytes 66-69: 0 (Z not used)
Bytes 70-73: 0 (A not used)
Bytes 74-77: 100000 (feedrate = 1000.00 mm/min)
```

**Example (3-axis):**
```
Byte 52: 11 (linear)
Byte 53: 3 (axis count)
Byte 54: 0 (X axis servo index)
Byte 55: 1 (Y axis servo index)
Byte 56: 2 (Z axis servo index)
Byte 57: 0x02 (absolute)
Bytes 58-61: 100000 (target X = 100.000 mm)
Bytes 62-65: 50000 (target Y = 50.000 mm)
Bytes 66-69: 25000 (target Z = 25.000 mm)
Bytes 70-73: 0 (A not used)
Bytes 74-77: 100000 (feedrate = 1000.00 mm/min)
```

## Parameter Scaling

All position and distance parameters use **scaled integers** to avoid floating-point serialization issues:

| Parameter Type | Scaling Factor | Example |
|----------------|----------------|---------|
| Position (X, Y, Z) | × 1000 | 100.5 mm = 100500 |
| Radius | × 1000 | 25.0 mm = 25000 |
| Angle | × 10 (0.1 degree units) | 90.5° = 905 |
| Feedrate | × 100 (0.01 units/min) | 1000.5 mm/min = 100050 |

**Conversion Formulas:**
- Position to scaled: `scaled = (int32_t)(position * 1000.0f)`
- Scaled to position: `position = (float)scaled / 1000.0f`
- Angle to scaled: `scaled = (int32_t)(angle * 10.0f)`
- Scaled to angle: `angle = (float)scaled / 10.0f`
- Feedrate to scaled: `scaled = (int32_t)(feedrate * 100.0f)`
- Scaled to feedrate: `feedrate = (float)scaled / 100.0f`

## Manager Implementation

### Command Processing Flow

1. **Check Coordinated Motion Command First** (byte 52)
   - If command type is 7-11, process coordinated motion
   - Otherwise, process single-axis commands (bytes 32-51)

2. **Parse Command Block**
   - Read command type, axis count, axis indices, flags
   - Read parameters based on command type
   - Validate all parameters (axis indices, scaling, etc.)

3. **Convert Parameters**
   - Convert scaled integers to floats
   - Apply scaling factors

4. **Execute Command**
   - Call appropriate `clearpath_servo_manager_move_*` function
   - Handle errors and log warnings

5. **Clear Command**
   - Set command type to 0 after successful execution
   - This signals completion to the PLC

### Pseudo-Code Example

```c
// In step_generation_task, before processing single-axis commands:

// Check for coordinated motion command
uint8_t coord_cmd_type = OUTPUT_ASSEMBLY_150[52];
if (coord_cmd_type >= 7 && coord_cmd_type <= 11) {
    // Parse coordinated motion command
    uint8_t axis_count = OUTPUT_ASSEMBLY_150[53];
    uint8_t axis_x = OUTPUT_ASSEMBLY_150[54];
    uint8_t axis_y = OUTPUT_ASSEMBLY_150[55];
    uint8_t axis_z = OUTPUT_ASSEMBLY_150[56];
    uint8_t flags = OUTPUT_ASSEMBLY_150[57];
    
    bool clockwise = (flags & 0x01) != 0;
    bool absolute = (flags & 0x02) != 0;
    
    // Read parameters as int32_t
    int32_t param1, param2, param3, param4, param5, param6, param7, param8;
    memcpy(&param1, &OUTPUT_ASSEMBLY_150[58], sizeof(int32_t));
    memcpy(&param2, &OUTPUT_ASSEMBLY_150[62], sizeof(int32_t));
    memcpy(&param3, &OUTPUT_ASSEMBLY_150[66], sizeof(int32_t));
    memcpy(&param4, &OUTPUT_ASSEMBLY_150[70], sizeof(int32_t));
    memcpy(&param5, &OUTPUT_ASSEMBLY_150[74], sizeof(int32_t));
    memcpy(&param6, &OUTPUT_ASSEMBLY_150[78], sizeof(int32_t));
    memcpy(&param7, &OUTPUT_ASSEMBLY_150[82], sizeof(int32_t));
    memcpy(&param8, &OUTPUT_ASSEMBLY_150[86], sizeof(int32_t));
    
    esp_err_t ret = ESP_OK;
    
    switch (coord_cmd_type) {
        case 7: // Arc Center
            {
                float center_x = param1 / 1000.0f;
                float center_y = param2 / 1000.0f;
                float radius = param3 / 1000.0f;
                int32_t start_angle = param4;  // Already in 0.1 degree units
                int32_t end_angle = param5;
                float feedrate = param6 / 100.0f;
                
                ret = clearpath_servo_manager_move_arc(
                    axis_x, axis_y,
                    center_x, center_y, radius,
                    start_angle, end_angle,
                    clockwise, feedrate
                );
            }
            break;
            
        case 8: // Arc To Point
            {
                float end_x = param1 / 1000.0f;
                float end_y = param2 / 1000.0f;
                float radius = param3 / 1000.0f;
                float feedrate = param4 / 100.0f;
                
                ret = clearpath_servo_manager_move_arc_to_point(
                    axis_x, axis_y,
                    end_x, end_y, absolute,
                    radius, clockwise, feedrate
                );
            }
            break;
            
        case 9: // Arc Helical Center
            {
                float center_x = param1 / 1000.0f;
                float center_y = param2 / 1000.0f;
                float radius = param3 / 1000.0f;
                int32_t start_angle = param4;
                int32_t end_angle = param5;
                float z_start = param6 / 1000.0f;
                float z_end = param7 / 1000.0f;
                float feedrate = param8 / 100.0f;
                
                ret = clearpath_servo_manager_move_arc_helical(
                    axis_x, axis_y, axis_z,
                    center_x, center_y, radius,
                    start_angle, end_angle,
                    z_start, z_end,
                    clockwise, feedrate
                );
            }
            break;
            
        case 10: // Arc Helical To Point
            {
                float end_x = param1 / 1000.0f;
                float end_y = param2 / 1000.0f;
                float end_z = param3 / 1000.0f;
                float radius = param4 / 1000.0f;
                float feedrate = param5 / 100.0f;
                
                ret = clearpath_servo_manager_move_arc_helical_to_point(
                    axis_x, axis_y, axis_z,
                    end_x, end_y, end_z, absolute,
                    radius, clockwise, feedrate
                );
            }
            break;
            
        case 11: // Linear
            {
                uint8_t axes[4];
                float targets[4];
                int axis_idx = 0;
                
                if (axis_count >= 2) {
                    axes[axis_idx] = axis_x;
                    targets[axis_idx++] = param1 / 1000.0f;
                }
                if (axis_count >= 2) {
                    axes[axis_idx] = axis_y;
                    targets[axis_idx++] = param2 / 1000.0f;
                }
                if (axis_count >= 3 && axis_z != 0xFF) {
                    axes[axis_idx] = axis_z;
                    targets[axis_idx++] = param3 / 1000.0f;
                }
                if (axis_count >= 4) {
                    // For 4-axis, would need to add axis A index and param4
                    // This is a simplified example
                }
                
                float feedrate = param5 / 100.0f;
                
                ret = clearpath_servo_manager_move_linear(
                    axis_count, axes, targets, feedrate
                );
            }
            break;
    }
    
    // Clear command after execution (success or failure)
    if (ret == ESP_OK) {
        OUTPUT_ASSEMBLY_150[52] = 0;  // Set to idle
    } else {
        // Could set error code in reserved bytes or log error
        OUTPUT_ASSEMBLY_150[52] = 0;  // Clear command on error too
    }
}
```

## PLC Programming Examples

### Example 1: 2-Axis Arc (Center Point)

Draw a full circle, center at (100mm, 100mm), radius 50mm, clockwise, 1000 mm/min.

```
// Write to Output Assembly 150
Assembly[52] := 7;                    // Command: arc_center
Assembly[53] := 2;                    // Axis count: 2
Assembly[54] := 0;                    // X axis servo index
Assembly[55] := 1;                    // Y axis servo index
Assembly[56] := $FF;                  // Z axis not used
Assembly[57] := 1;                    // Flags: clockwise

// Center X = 100.000 mm (scaled: 100000)
Assembly[58] := $A0;                  // Low byte
Assembly[59] := $86;                  // 
Assembly[60] := $01;                  // 
Assembly[61] := $00;                  // High byte

// Center Y = 100.000 mm (scaled: 100000)
Assembly[62] := $A0;
Assembly[63] := $86;
Assembly[64] := $01;
Assembly[65] := $00;

// Radius = 50.000 mm (scaled: 50000)
Assembly[66] := $50;
Assembly[67] := $C3;
Assembly[68] := $00;
Assembly[69] := $00;

// Start Angle = 0.0° (scaled: 0)
Assembly[70] := $00;
Assembly[71] := $00;
Assembly[72] := $00;
Assembly[73] := $00;

// End Angle = 360.0° (scaled: 3600)
Assembly[74] := $10;
Assembly[75] := $0E;
Assembly[76] := $00;
Assembly[77] := $00;

// Feedrate = 1000.00 mm/min (scaled: 100000)
Assembly[78] := $A0;
Assembly[79] := $86;
Assembly[80] := $01;
Assembly[81] := $00;

// Wait for command to clear (completion)
WHILE Assembly[52] <> 0 DO
    // Wait
END_WHILE
```

### Example 2: 2-Axis Arc (End Point + Radius)

Move from current position to (150mm, 100mm) with 50mm radius, clockwise, absolute coordinates, 1000 mm/min.

```
Assembly[52] := 8;                    // Command: arc_to_point
Assembly[53] := 2;                    // Axis count: 2
Assembly[54] := 0;                    // X axis servo index
Assembly[55] := 1;                    // Y axis servo index
Assembly[56] := $FF;                  // Z axis not used
Assembly[57] := 3;                    // Flags: clockwise + absolute

// End X = 150.000 mm (scaled: 150000)
Assembly[58] := $70;
Assembly[59] := $49;
Assembly[60] := $02;
Assembly[61] := $00;

// End Y = 100.000 mm (scaled: 100000)
Assembly[62] := $A0;
Assembly[63] := $86;
Assembly[64] := $01;
Assembly[65] := $00;

// Radius = 50.000 mm (scaled: 50000)
Assembly[66] := $50;
Assembly[67] := $C3;
Assembly[68] := $00;
Assembly[69] := $00;

// Feedrate = 1000.00 mm/min (scaled: 100000)
Assembly[70] := $A0;
Assembly[71] := $86;
Assembly[72] := $01;
Assembly[73] := $00;

// Wait for completion
WHILE Assembly[52] <> 0 DO
    // Wait
END_WHILE
```

### Example 3: 3-Axis Helical Arc (Center Point)

90-degree arc in XY plane while moving 10mm in Z, center at (100mm, 100mm), radius 50mm, clockwise, 1000 mm/min.

```
Assembly[52] := 9;                    // Command: arc_helical_center
Assembly[53] := 3;                    // Axis count: 3
Assembly[54] := 0;                    // X axis servo index
Assembly[55] := 1;                    // Y axis servo index
Assembly[56] := 2;                    // Z axis servo index
Assembly[57] := 1;                    // Flags: clockwise

// Center X = 100.000 mm (scaled: 100000)
Assembly[58] := $A0;
Assembly[59] := $86;
Assembly[60] := $01;
Assembly[61] := $00;

// Center Y = 100.000 mm (scaled: 100000)
Assembly[62] := $A0;
Assembly[63] := $86;
Assembly[64] := $01;
Assembly[65] := $00;

// Radius = 50.000 mm (scaled: 50000)
Assembly[66] := $50;
Assembly[67] := $C3;
Assembly[68] := $00;
Assembly[69] := $00;

// Start Angle = 0.0° (scaled: 0)
Assembly[70] := $00;
Assembly[71] := $00;
Assembly[72] := $00;
Assembly[73] := $00;

// End Angle = 90.0° (scaled: 900)
Assembly[74] := $84;
Assembly[75] := $03;
Assembly[76] := $00;
Assembly[77] := $00;

// Z Start = 0.000 mm (scaled: 0)
Assembly[78] := $00;
Assembly[79] := $00;
Assembly[80] := $00;
Assembly[81] := $00;

// Z End = 10.000 mm (scaled: 10000)
Assembly[82] := $10;
Assembly[83] := $27;
Assembly[84] := $00;
Assembly[85] := $00;

// Feedrate = 1000.00 mm/min (scaled: 100000)
Assembly[86] := $A0;
Assembly[87] := $86;
Assembly[88] := $01;
Assembly[89] := $00;

// Wait for completion
WHILE Assembly[52] <> 0 DO
    // Wait
END_WHILE
```

## Error Handling

### Command Validation

The manager should validate:
- Command type is 7-11 (valid coordinated motion command)
- Axis count matches command type (2 for arc, 2-3 for helical, 2-4 for linear)
- Axis indices are valid (0-3) and initialized
- Z axis index is valid or 0xFF (for 2-axis commands)
- Parameters are within valid ranges (positive radius, valid angles, positive feedrate)
- Scaled integers don't overflow when converted to floats

### Error Reporting

Options for error reporting:
1. **Status Byte in Input Assembly**: Add a status byte at byte 72 that indicates command execution status
2. **Command Echo**: Echo the command type in Input Assembly to show last executed command
3. **Error Code**: Use reserved bytes (90-93) to store error codes
4. **Logging**: Log errors via ESP_LOG and rely on PLC timeout/retry logic

**Recommended**: Add status byte in Input Assembly 100:
- Byte 72: Coordinated Motion Status
  - Bit 0: Command in progress (1 = executing, 0 = idle)
  - Bit 1: Command complete (1 = completed successfully)
  - Bit 2: Command error (1 = error occurred)
  - Bits 3-7: Reserved

## Assembly Size Requirements

### Current State
- Output Assembly 150: 40 bytes (supports 1 servo fully)
- Input Assembly 100: 72 bytes

### Required Changes
- **Output Assembly 150**: Expand to 128 bytes (add 88 bytes)
- **Input Assembly 100**: Expand to 80 bytes (add 8 bytes for coordinated motion status)

### EDS File Updates

The EDS file (`FusionCoreEnIP.eds`) needs to be updated to reflect:
1. Output Assembly 150 size change: 40 → 128 bytes
2. Input Assembly 100 size change: 72 → 80 bytes
3. Documentation of new coordinated motion command structure

## Migration Path

1. **Phase 1**: Implement manager code to parse coordinated motion commands
2. **Phase 2**: Expand assembly sizes in OpENer configuration
3. **Phase 3**: Update EDS file with new assembly sizes and documentation
4. **Phase 4**: Test with PLC to verify command execution
5. **Phase 5**: Add error reporting and status feedback

## Benefits

1. **Standardized Interface**: PLCs can send complex motion commands via EtherNet/IP
2. **No Code Changes Required**: PLCs don't need custom drivers, just assembly writes
3. **Deterministic**: Integer scaling avoids floating-point serialization issues
4. **Extensible**: Easy to add new command types in the future
5. **Backward Compatible**: Single-axis commands (bytes 32-51) remain unchanged

## Limitations

1. **Single Command Queue**: Only one coordinated motion command can be queued at a time
2. **No Command Cancellation**: Once started, command must complete (use emergency stop for abort)
3. **Assembly Size**: Requires expanding assemblies, which may affect other components
4. **Parameter Precision**: Scaling factors limit precision (usually sufficient for motion control)

## Future Enhancements

1. **Command Queue**: Add support for queuing multiple coordinated motion commands
2. **Command Cancellation**: Add command type to cancel current coordinated motion
3. **Status Feedback**: Enhanced status reporting (progress, estimated time remaining)
4. **4-Axis Support**: Full support for 4-axis linear interpolation
5. **Spline Interpolation**: Add spline/cubic interpolation commands
6. **Feedrate Override**: Add feedrate override parameter for real-time speed adjustment
