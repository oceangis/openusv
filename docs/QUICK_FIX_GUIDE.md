# Quick Fix Guide - ESP32-S3 ArduPilot Rover Critical Issues

## Problem Summary

System has two critical issues preventing operation:
1. **Param storage failed** - Partition table not flashed
2. **Compass not healthy** - I2C bypass mode not implemented

## Quick Fix Steps

### Step 1: Flash Partition Table (5 minutes - DO THIS FIRST)

```bash
cd F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT partition-table-flash
```

**Replace COM_PORT with your actual port (e.g., COM3, COM4, etc.)**

**Verification:**
After reboot, serial console should show:
```
Storage partition found: addr=0x... size=256K
```

NOT:
```
WARNING: Storage partition (type 0x45) not found
```

If still fails, do full flash:
```bash
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT erase_flash
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT flash
```

### Step 2: Implement I2C Bypass Mode (2-3 hours)

Apply all patches from `docs/IMPLEMENTATION_PATCHES.md` in this order:

1. Patch 1: Storage diagnostics (optional)
2. Patch 2: Enable I2C bypass in ICM20948
3. Patch 6: Add helper methods to Invensensev2.h
4. Patch 3: Add Direct I2C bus driver header
5. Patch 4: Implement Direct I2C bus driver
6. Patch 5: Update compass probe logic (use simplified version with Patch 6)

### Step 3: Build and Flash

```bash
# Clean build
D:\Espressif\v5.5.1\esp-idf\tools\idf.py fullclean

# Build
D:\Espressif\v5.5.1\esp-idf\tools\idf.py build

# Flash
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT flash

# Monitor
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT monitor
```

### Step 4: Verify and Calibrate

1. Check serial output for:
   - "Storage partition found"
   - "ICM20948: I2C bypass mode enabled successfully"
   - "AK09916: Using I2C bypass mode"
   - "AK09916: Found on I2C bus 0"

2. Connect Mission Planner

3. Run calibrations:
   - Initial Setup -> Accel Calibration
   - Initial Setup -> Compass Calibration

4. Reboot and verify:
   - Calibration persists
   - Only "Radio failsafe" pre-arm error remains (expected)
   - HUD shows attitude changes

## Expected Results

### Before Fix
```
PreArm: Param storage failed
PreArm: Compass not healthy
PreArm: 3D Accel calibration needed
PreArm: Radio failsafe on
```

### After Fix
```
PreArm: Radio failsafe on
```

All other errors should be gone!

## Troubleshooting

### Storage still fails
- Try full erase and reflash
- Check partition table with: `idf.py partition-table`
- Verify partitions.csv is correct

### Compass still not detected
- Check I2C pull-ups (2.2K-4.7K to 3.3V)
- Verify GPIO 8/9 configuration
- Check if AK09916 visible at address 0x0C
- Add debug logging to bypass enable code

## Key Files Modified

1. `libraries/AP_HAL_ESP32/Storage.cpp` - Better error messages
2. `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp` - Enable bypass
3. `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.h` - Helper methods
4. `libraries/AP_Compass/AP_Compass_AK09916.h` - New driver class
5. `libraries/AP_Compass/AP_Compass_AK09916.cpp` - Driver implementation

## Documentation

Full documentation available in:
- `docs/ANALYSIS_AND_FIX.md` - Detailed technical analysis
- `docs/IMPLEMENTATION_PATCHES.md` - Complete code patches
- `log/log.txt` - Runtime logs

## Support

If issues persist:
1. Check full analysis document
2. Review all patches carefully
3. Verify hardware connections (I2C pull-ups, GPIO assignments)
4. Add debug logging to trace execution
5. Use I2C scanner code to verify device visibility

## Success Indicators

- Storage partition found at boot
- I2C bypass mode enabled
- AK09916 compass detected
- Calibrations complete successfully
- Calibration data persists after reboot
- EKF3 converges (no forced resets)
- HUD displays attitude correctly
- Only radio failsafe pre-arm error remains

---

Total fix time: 3-4 hours
Difficulty: Medium
Success rate: Very High (standard ICM20948 feature)
