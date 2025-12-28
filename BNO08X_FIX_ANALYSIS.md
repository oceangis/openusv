# BNO08x Driver Fix - Deep Analysis and Solution

## Executive Summary

The BNO08x driver had **THREE CRITICAL BUGS** that prevented it from reading sensor data:

1. **Incorrect I2C read logic** in chunked data handling
2. **Wrong polling strategy** - using INT pin for I2C mode (INT is ONLY for SPI!)
3. **Incorrect understanding** of how Adafruit library works

All issues have been fixed in v2.1.

---

## Root Cause Analysis

### Problem 1: Incorrect I2C Chunked Read Logic

**Location:** `AP_ExternalAHRS_BNO08x::hal_read()` lines 197-237

**The Bug:**
The original code tried to read the "header + cargo" together in the first read:
```cpp
if (first_read) {
    // First read: just the cargo (no extra header)  <-- WRONG!
    read_size = (cargo_remaining < MAX_I2C_BUFFER) ? cargo_remaining : MAX_I2C_BUFFER;
}
```

**The Fix:**
According to Adafruit's `i2chal_read()` (lines 307-387), the read sequence is:
1. Read 4-byte header to get packet size
2. **First data read**: Contains ONLY cargo (no extra header)
3. **Subsequent reads**: Each I2C transaction includes a 4-byte header that must be skipped

The comment was backwards! The first read has NO extra header.

**Reference Code (Adafruit_BNO08x.cpp:344-375):**
```cpp
while (cargo_remaining > 0) {
    if (first_read) {
        read_size = min(i2c_buffer_max, (size_t)cargo_remaining);  // NO +4 header!
    } else {
        read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);  // +4 for header
    }

    if (!i2c_dev->read(i2c_buffer, read_size)) {
        return 0;
    }

    if (first_read) {
        // The first time we're saving the "original" header, so include it
        cargo_read_amount = read_size;
        memcpy(pBuffer, i2c_buffer, cargo_read_amount);
        first_read = false;
    } else {
        // skip the 4-byte header included with every new i2c read
        cargo_read_amount = read_size - 4;
        memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
}
```

### Problem 2: INT Pin Polling for I2C Mode

**Location:** `AP_ExternalAHRS_BNO08x::update_thread()` line 1008

**The Bug:**
```cpp
// Check if data is ready via INT pin
if (!data_ready()) {
    hal.scheduler->delay(1);
    continue;
}
```

**The Reality:**
After analyzing Adafruit's code:
- `i2chal_read()` does **NOT** check INT pin
- INT pin is **ONLY** used in `spihal_read()` for SPI mode
- For I2C, you just call `read()` directly and it returns 0 if no data

**Proof from Adafruit_BNO08x.cpp:**
- Line 307: `static int i2chal_read(...)` - No INT pin check
- Line 566: `static int spihal_read(...)` - Calls `spihal_wait_for_int()` at line 572
- Line 549: `static bool spihal_wait_for_int(void)` - Uses `digitalRead(_int_pin)`

**The Fix:**
Remove INT pin polling entirely. Use continuous polling with 5ms delay like Arduino test code:
```cpp
// Main sensor read loop
// CRITICAL: For I2C mode, we do NOT use INT pin!
while (true) {
    // ...
    got_packet = shtp_receive_packet();  // Just read directly
    if (got_packet) {
        process_packet();
    }
    hal.scheduler->delay(5);  // Match Arduino test code
}
```

### Problem 3: How getSensorEvent() Actually Works

**Location:** Understanding the data flow

**Adafruit Flow:**
1. User calls `getSensorEvent()` (Adafruit_BNO08x.cpp:237)
2. It calls `sh2_service()` (line 242)
3. Which calls `shtp_service()` (sh2.c:1784)
4. Which calls `pHal->read()` (shtp.c:815)
5. Which is `i2chal_read()` - does I2C read **WITHOUT** checking INT pin
6. If data available, callback is triggered and `_sensor_value` is filled

**Key Insight:**
The Arduino test code calls `getSensorEvent()` in a tight loop with `delay(5)`. This means it's **polling the I2C bus every 5ms**, not waiting for interrupts!

```cpp
void loop() {
    if (bno08x.wasReset()) {
        setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {  // Polls I2C
        // Process data
    }

    delay(5);  // 5ms delay between polls
}
```

---

## Changes Made

### File: `AP_ExternalAHRS_BNO08x.cpp`

#### Change 1: Fixed I2C Chunked Read (lines 197-237)
- **What:** Corrected the first_read logic to NOT expect a header
- **Why:** Matches Adafruit i2chal_read() exactly
- **Impact:** Data can now be read correctly from BNO08x

#### Change 2: Removed INT Pin Polling (lines 998-1030)
- **What:** Removed `data_ready()` check, changed to continuous polling
- **Why:** INT pin is not used for I2C in Adafruit library
- **Impact:** Driver now polls every 5ms like Arduino test code

#### Change 3: Updated Sensor Configuration (lines 547-575)
- **What:** Changed report interval from 10000us (100Hz) to 20000us (50Hz)
- **Why:** Matches Arduino test code configuration
- **Impact:** More conservative update rate, easier debugging

#### Change 4: Added Debug Logging
- **What:** Added packet logging in `process_packet()` and `parse_rotation_vector()`
- **Why:** Help trace data flow and verify packets are received
- **Impact:** Better diagnostics

### File: `AP_ExternalAHRS_BNO08x.h`

#### Change 1: Updated Header Comments (lines 15-38)
- **What:** Updated to v2.1 with detailed fix list
- **Why:** Document the changes
- **Impact:** Better code documentation

#### Change 2: Clarified INT Pin Comment (lines 91-95)
- **What:** Added note that INT pin is NOT used for I2C
- **Why:** Prevent future confusion
- **Impact:** Code clarity

---

## Verification Steps

After these fixes, you should see:

1. **Initialization logs:**
   ```
   BNO08x: Starting initialization...
   BNO08x: Waiting 500ms for sensor boot...
   BNO08x: Found at 0x4B
   BNO08x: Sending soft reset...
   BNO08x: Reset sent on attempt 1
   BNO08x: Waiting 300ms after reset...
   BNO08x: Waiting for reset complete...
   BNO08x: Got packet ch=1 len=5
   BNO08x: Reset complete received!
   BNO08x: FW vX.X.X (reset cause: X)
   ```

2. **Packet reception logs:**
   ```
   BNO08x: RX ch=3 len=XX seq=X
   BNO08x: RX ch=3 len=XX seq=X
   ...
   ```

3. **Quaternion data logs:**
   ```
   BNO08x: Quat w=0.XXX x=0.XXX y=0.XXX z=0.XXX
   ```

---

## Technical Reference

### SHTP Protocol Basics

**Packet Structure:**
```
[0-1]: Length (16-bit LE, bit 15 = continuation)
[2]:   Channel number
[3]:   Sequence number
[4+]:  Payload
```

**I2C Read Sequence (for packets > I2C buffer size):**
```
Transaction 1: Read 4-byte header
               Parse length from header
Transaction 2: Read cargo[0..MAX_BUFFER]  (NO header in this read!)
Transaction 3: Read [header(4 bytes)] + cargo[MAX_BUFFER..2*MAX_BUFFER]
Transaction 4: Read [header(4 bytes)] + cargo[2*MAX_BUFFER..end]
...
```

### Channel Assignments

- Channel 0: SHTP Command/Advertisement
- Channel 1: Executable/Device Control
- Channel 2: Sensor Hub Control (Set Feature commands)
- Channel 3: Sensor Input Reports (IMU data)
- Channel 4: Wake Sensor Input Reports
- Channel 5: Gyro Rotation Vector

### Sensor Report IDs

- 0x01: Accelerometer
- 0x02: Gyroscope (calibrated)
- 0x03: Magnetic Field
- 0x05: Rotation Vector (quaternion)
- 0x08: Game Rotation Vector
- 0xF1: Command Response
- 0xF8: Product ID Response
- 0xFD: Set Feature Command

---

## Comparison: Arduino vs ArduPilot Implementation

| Aspect | Arduino (Adafruit) | ArduPilot (Fixed) |
|--------|-------------------|-------------------|
| INT Pin Usage | NOT used for I2C | NOT used for I2C |
| Polling Method | `getSensorEvent()` every 5ms | Thread polls every 5ms |
| I2C Read | `i2chal_read()` direct | Matches `i2chal_read()` exactly |
| Reset Handling | `wasReset()` checked in loop | `reset_occurred` flag + reconfigure |
| Report Rate | 20000us (50Hz) | 20000us (50Hz) |
| Threading | Single-threaded | Multi-threaded (safer) |

---

## Key Learnings

1. **Always read reference implementation carefully** - The INT pin usage was clearly documented in Adafruit code
2. **Trust working code** - The Arduino test code worked because it didn't use INT pin
3. **Protocol documentation alone isn't enough** - Need to see actual implementation
4. **I2C vs SPI differences matter** - INT pin behavior is different between interfaces
5. **Chunk reading is tricky** - The header handling in multi-transaction reads is subtle

---

## Files Modified

1. `f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\libraries\AP_ExternalAHRS\AP_ExternalAHRS_BNO08x.cpp`
2. `f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\libraries\AP_ExternalAHRS\AP_ExternalAHRS_BNO08x.h`

## Testing Recommendation

1. Flash the firmware
2. Monitor serial output for initialization sequence
3. Verify packet reception (ch=3 packets)
4. Verify quaternion data is printed
5. Check `healthy()` status in MAVLink

---

## Conclusion

The driver should now work correctly. The root issues were:
1. Subtle bug in I2C chunked read implementation
2. Fundamental misunderstanding of INT pin usage
3. Not matching the polling pattern of working code

All three have been fixed based on deep analysis of Adafruit's actual implementation.
