# LSM6DS3 Assembly Data Format

## Input Assembly 100 — LSM6DS3 Data (8 bytes)

The LSM6DS3 IMU sensor writes data to EtherNet/IP Input Assembly 100 at a **fixed** byte offset (bytes 16-23). The data consists of 3 signed 16-bit integer values representing orientation angles.

### Data Structure

| Offset | Field | Type | Size | Description |
|--------|-------|------|------|-------------|
| 0 | Roll | INT16 | 2 bytes | Roll angle scaled by 100 (-180.00° to +180.00°) |
| 2 | Pitch | INT16 | 2 bytes | Pitch angle scaled by 100 (-180.00° to +180.00°) |
| 4 | GroundAngle | INT16 | 2 bytes | Computed ground angle scaled by 100 (-180.00° to +180.00°) |
| 6 | Reserved | - | 2 bytes | Reserved for future expansion |

**Total Size:** 8 bytes (3 × INT16 + 2 bytes reserved)

**Fixed Byte Location:** Bytes 16-23 in Input Assembly 100

### Byte Order

All values are stored as **little-endian** signed 16-bit integers (`int16_t`).

### Data Format Details

#### Orientation Angles (Roll, Pitch, GroundAngle)

- **Type:** `int16_t` (signed 16-bit integer)
- **Range:** -180.00° to +180.00° (actual angle)
- **Precision:** 0.01 degrees (when scaled by 100)
- **Scaling:** Values are stored scaled by 100 (fixed-point format, like NAU7802 weight)

**Example:**
- `45.66°` → `4566` (stored as `0x11D6`)
- `-90.50°` → `-9050` (stored as `0xDD96`)
- `180.00°` → `18000` (stored as `0x4650`)

**Conversion Formula:**
```c
// Writing to assembly (float to scaled int16_t):
float angle_float = 45.66f;  // degrees
int16_t angle_scaled = (int16_t)(angle_float * 100.0f + 0.5f);  // Scale by 100 with rounding

// Reading from assembly (scaled int16_t to float):
int16_t angle_scaled = 4566;  // from assembly
float angle_float = (float)angle_scaled / 100.0f;  // Unscale: divide by 100
```

### Sensor Fusion

The orientation angles (Roll, Pitch, GroundAngle) are calculated using **sensor fusion**:

1. **Accelerometer Data:** Used to determine gravity vector and calculate pitch/roll
2. **Gyroscope Data:** Used for high-frequency orientation tracking
3. **Complementary Filter or Madgwick Filter:** Combines accelerometer and gyroscope data to provide stable, drift-free orientation

**Filter Parameters:**
- **Filter Type:** Configurable (Complementary Filter or Madgwick Filter)
- **Alpha (α):** Configurable for Complementary Filter (default: 0.96)
- **Beta (β):** Configurable for Madgwick Filter
- **Sample Rate:** Configurable filter update rate in Hz
- **ODR:** Configurable Output Data Rate for accelerometer and gyroscope
- **Full-Scale Range:** Configurable for accelerometer (2g, 4g, 8g, 16g) and gyroscope (125dps, 250dps, 500dps, 1000dps, 2000dps)

### Fixed Byte Offset

The starting byte offset for LSM6DS3 data in Input Assembly 100 is **fixed** at bytes 16-23 (not configurable). This ensures consistent mapping across all devices.

**Byte Allocation:**
- Bytes 16-17: Roll (int16_t)
- Bytes 18-19: Pitch (int16_t)
- Bytes 20-21: GroundAngle (int16_t)
- Bytes 22-23: Reserved

### Memory Layout Example

```
Input Assembly 100 (72 bytes total)
┌─────────────────────────────────────────────────────────┐
│ Byte  │ Field          │ Value (hex)    │ Value (dec)   │
├───────┼────────────────┼────────────────┼──────────────┤
│ 0-15  │ VL53L1X        │ ...            │ ...           │
│ 16-17 │ Roll           │ 0x11D6         │ 4566 (45.66°) │
│ 18-19 │ Pitch          │ 0xDD96         │ -9050 (-90.50°) │
│ 20-21 │ GroundAngle    │ 0x4650         │ 18000 (180.00°) │
│ 22-23 │ Reserved       │ 0x0000         │ 0             │
│ 24-39 │ NAU7802        │ ...            │ ...           │
│ ...   │ ...            │ ...            │ ...           │
└─────────────────────────────────────────────────────────┘
```

### Calibration

The LSM6DS3 supports calibration for improved accuracy:

**Calibration Types:**
- **Accelerometer Calibration:** Offset values to compensate for sensor bias
- **Gyroscope Calibration:** Offset values to remove drift
- **Angle Zero Offsets:** Roll, Pitch, and Yaw zero offsets for reference frame adjustment

**Calibration Process:**
- Device must be kept **completely still** during gyroscope calibration
- Default: 100 samples at configurable intervals
- Calibration values are stored in NVS and persist across reboots

**API Endpoints:**
- `GET /api/lsm6ds3/config` - Get current IMU configuration
- `POST /api/lsm6ds3/config` - Set IMU configuration
- `GET /api/lsm6ds3/calibrate` - Get calibration status
- `POST /api/lsm6ds3/calibrate` - Trigger calibration
- `POST /api/lsm6ds3/zero` - Set angle zero offsets

### Data Flow

```
LSM6DS3 Sensor (I2C)
    ↓
Raw Accelerometer/Gyroscope Readings
    ↓
Calibration Applied (offsets subtracted)
    ↓
Sensor Fusion (Complementary Filter or Madgwick Filter)
    ↓
Roll, Pitch, GroundAngle (degrees, -180.00° to +180.00°)
    ↓
Scale by 100 and convert to int16_t (fixed-point format)
    ↓
Assembly Buffer (little-endian bytes)
    ↓
EtherNet/IP Input Assembly 100 (bytes 16-23)
```

### Configuration

The LSM6DS3 supports extensive configuration via NVS and Web API:

**Sensor Configuration:**
- Accelerometer ODR: 12.5Hz, 26Hz, 52Hz, 104Hz, 208Hz, 416Hz, 833Hz, 1.66kHz, 3.33kHz, 6.66kHz
- Accelerometer Full-Scale Range: 2g, 4g, 8g, 16g
- Gyroscope ODR: 12.5Hz, 26Hz, 52Hz, 104Hz, 208Hz, 416Hz, 833Hz, 1.66kHz
- Gyroscope Full-Scale Range: 125dps, 250dps, 500dps, 1000dps, 2000dps

**Filter Configuration:**
- Filter Type: Complementary Filter or Madgwick Filter
- Filter Parameters: Alpha (α) for Complementary, Beta (β) for Madgwick
- Sample Rate: Filter update rate in Hz

**Calibration Configuration:**
- Accelerometer calibration offsets
- Gyroscope calibration offsets
- Angle zero offsets (roll, pitch, yaw)

### Notes

- **Fixed Offset:** LSM6DS3 data is always at bytes 16-23 (not configurable)
- **Data Format:** int16_t values scaled by 100 (fixed-point format), range -180.00° to +180.00°
- **Scaling:** Angles are scaled by 100 before storage (e.g., 45.66° = 4566), similar to NAU7802 weight scaling
- **Thread Safety:** Assembly writes are protected by mutex
- **Update Rate:** Configurable (default: 50 Hz)
- **Endianness:** All values are little-endian (LSB first)
- **Removed:** BottomPressure and TopPressure fields (cylinder bore calculations not used)

### Related Documentation

- [LSM6DS3 Driver README](README.md)
- [API Endpoints Documentation](../../docs/API_Endpoints.md)
- [Assembly Data Layout](../../docs/ASSEMBLY_DATA_LAYOUT.md)
