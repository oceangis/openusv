# ESP32-S3 ArduPilot Rover - Critical Issues Fix Documentation

## Document Index

This directory contains complete documentation for fixing the critical issues in the ESP32-S3 ArduPilot Rover project.

### Quick Start
**Start here**: `QUICK_FIX_GUIDE.md` - Step-by-step fix instructions

### Detailed Documentation

1. **ANALYSIS_AND_FIX.md** (Ultra-Think Deep Analysis)
   - Complete technical analysis of all issues
   - Root cause identification
   - Architecture deep dive
   - ICM20948 I2C bypass mode technical details
   - Implementation methodology
   - Testing and verification procedures

2. **IMPLEMENTATION_PATCHES.md** (Code Changes)
   - All required code modifications
   - 6 patches with before/after code
   - Build and flash commands
   - Testing procedures
   - Troubleshooting guide

3. **QUICK_FIX_GUIDE.md** (Quick Reference)
   - Fast implementation guide
   - Critical steps only
   - Expected results
   - Quick troubleshooting

## Problem Summary

### Issue 1: Param Storage Failed (CRITICAL)
**Symptom**: "PreArm: Param storage failed" error
**Root Cause**: Partition table (partitions.csv) not flashed to device
**Impact**:
- All calibration data lost on reboot
- Cannot save any parameter changes
- System cannot arm

**Fix**: Flash partition table
**Time**: 5 minutes
**Difficulty**: Easy

### Issue 2: Compass Not Healthy (CRITICAL)
**Symptom**: "PreArm: Compass not healthy" error
**Root Cause**: ICM20948 I2C bypass mode not implemented
**Impact**:
- Magnetometer (AK09916) not accessible
- Cannot initialize compass
- EKF3 cannot converge
- System cannot arm

**Fix**: Implement I2C bypass mode
**Time**: 2-3 hours
**Difficulty**: Medium

### Related Issue: EKF3 Convergence
**Symptom**: "EKF3 IMU0 forced reset" in logs
**Root Cause**: Caused by Issues 1 and 2 above
**Fix**: Will resolve automatically when primary issues are fixed

## Technical Background

### Why I2C Bypass Mode is Needed

The ICM20948 IMU contains an internal AK09916 magnetometer connected via an internal I2C bus. There are two ways to access it:

1. **Master Mode (SPI only in current code)**:
   - ICM20948 reads AK09916 and exposes data in its registers
   - Used when ICM20948 is on SPI
   - Current ArduPilot implementation: `AP_Invensensev2_AuxiliaryBus`

2. **Bypass Mode (I2C - NOT IMPLEMENTED)**:
   - ICM20948 acts as I2C pass-through
   - Host MCU talks directly to AK09916
   - Required when ICM20948 is on I2C
   - **This is what we're implementing**

### Hardware Configuration

- **Board**: ESP32-S3-N16R8
- **IMU**: ICM20948 on I2C
- **I2C Bus**: GPIO 8 (SDA), GPIO 9 (SCL)
- **IMU Address**: 0x68
- **Magnetometer Address**: 0x0C (when bypass enabled)

### Why Storage Partition Failed

ESP32 uses a partition table to define flash memory layout. The custom partition type 0x45 is used by ArduPilot for parameter storage. If the partition table isn't flashed, the firmware can't find this partition and falls back to "empty storage" mode where all writes are ignored.

## Implementation Strategy

### Phase 1: Immediate Fix (Storage)
1. Flash partition table to device
2. Verify storage partition is found
3. Test parameter persistence

### Phase 2: Code Implementation (Compass)
1. Enable I2C bypass in ICM20948 initialization
2. Create direct I2C bus driver for AK09916
3. Update compass probe logic to use bypass mode
4. Build and test

### Phase 3: Calibration and Verification
1. Run accelerometer calibration
2. Run compass calibration
3. Verify persistence after reboot
4. Confirm EKF3 convergence

## File Changes Summary

### Modified Files
- `libraries/AP_HAL_ESP32/Storage.cpp` - Enhanced diagnostics
- `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp` - Bypass enable
- `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.h` - Helper methods

### New Code Added
- `libraries/AP_Compass/AP_Compass_AK09916.h` - Direct I2C driver class
- `libraries/AP_Compass/AP_Compass_AK09916.cpp` - Driver implementation

### Configuration Files
- `partitions.csv` - Already correct, just needs flashing
- `sdkconfig` - Already correct

## Testing Checklist

Before fix:
- [ ] "Param storage failed" error present
- [ ] "Compass not healthy" error present
- [ ] "3D Accel calibration needed" error present
- [ ] Calibration data doesn't persist
- [ ] EKF3 forced resets
- [ ] HUD doesn't update

After Phase 1 (Storage fix):
- [ ] "Param storage failed" error gone
- [ ] Storage partition found in boot log
- [ ] Can save parameters

After Phase 2 (Compass fix):
- [ ] "Compass not healthy" error gone
- [ ] I2C bypass mode enabled message
- [ ] AK09916 detected on I2C bus

After Phase 3 (Calibration):
- [ ] Accelerometer calibration succeeds
- [ ] Compass calibration succeeds
- [ ] Calibration data persists after reboot
- [ ] EKF3 converges without forced resets
- [ ] HUD displays attitude correctly
- [ ] Only "Radio failsafe" pre-arm error remains

## Expected Timeline

- **Phase 1**: 30 minutes (including verification)
- **Phase 2**: 2-3 hours (code changes and testing)
- **Phase 3**: 30 minutes (calibration and verification)
- **Total**: 3-4 hours

## Success Criteria

1. No "Param storage failed" error
2. No "Compass not healthy" error
3. Storage partition found at boot
4. I2C bypass mode enabled
5. AK09916 compass detected
6. Calibration completes successfully
7. Calibration persists across reboots
8. EKF3 initializes cleanly
9. HUD shows attitude changes
10. Only expected errors remain (radio failsafe)

## Additional Resources

### ArduPilot Architecture
- HAL (Hardware Abstraction Layer) for ESP32
- Invensensev2 backend for ICM20948
- AK09916 compass driver
- Auxiliary bus vs direct I2C access

### ICM20948 Features
- 9-axis IMU with integrated magnetometer
- I2C and SPI interfaces
- Auxiliary I2C master mode
- I2C bypass/pass-through mode
- Register bank architecture

### ESP32-S3 Features
- Dual-core Xtensa LX7
- 16MB Flash, 8MB PSRAM
- Multiple I2C controllers
- Custom partition table support

## Troubleshooting Resources

### If Storage Still Fails
1. Verify partitions.csv content
2. Check partition table in flash: `idf.py partition-table`
3. Try full erase: `idf.py erase_flash`
4. Reflash everything: `idf.py flash`

### If Compass Still Not Detected
1. Check I2C pull-up resistors (2.2K-4.7K)
2. Verify GPIO configuration (GPIO 8/9)
3. Run I2C scanner to check device visibility
4. Add debug logging to bypass enable code
5. Verify bypass bit is actually set

### Debug Tools
- Serial monitor: `idf.py monitor`
- Read partition table: `idf.py read_flash 0x8000 0x1000 pt.bin`
- I2C scanner code provided in patches
- Mission Planner for calibration and testing

## Version Information

- **ESP-IDF**: v5.5.1
- **ArduPilot**: V4.7.0-dev
- **Board**: ESP32-S3-N16R8
- **IMU**: ICM20948 (I2C mode)
- **Compass**: AK09916 (internal to ICM20948)

## Project Structure

```
ardupilot_rover_esp32s3_idf/
├── docs/
│   ├── ANALYSIS_AND_FIX.md         (This analysis)
│   ├── IMPLEMENTATION_PATCHES.md    (Code changes)
│   ├── QUICK_FIX_GUIDE.md          (Quick reference)
│   └── README_FIXES.md             (This file)
├── libraries/
│   ├── AP_HAL_ESP32/
│   │   └── Storage.cpp             (Modified)
│   ├── AP_InertialSensor/
│   │   ├── AP_InertialSensor_Invensensev2.cpp  (Modified)
│   │   ├── AP_InertialSensor_Invensensev2.h    (Modified)
│   │   └── AP_InertialSensor_Invensensev2_registers.h  (Already has bypass bit)
│   └── AP_Compass/
│       ├── AP_Compass_AK09916.h    (Modified - new driver)
│       └── AP_Compass_AK09916.cpp  (Modified - new implementation)
├── partitions.csv                   (Correct, needs flashing)
├── sdkconfig                        (Correct)
└── log/
    └── log.txt                      (Runtime logs)
```

## References

1. ICM20948 Datasheet - I2C bypass mode registers
2. AK09916 Datasheet - Magnetometer specifications
3. ESP-IDF Documentation - Partition tables
4. ArduPilot HAL Documentation - Device drivers
5. ESP32-S3 Technical Reference Manual

## Contact and Support

For issues or questions:
1. Review full analysis document first
2. Check implementation patches carefully
3. Verify hardware connections
4. Review troubleshooting sections
5. Check log files for error messages

## License

This documentation follows the same license as the ArduPilot project (GPLv3).

---

**Document Version**: 1.0
**Created**: 2025-11-04
**Analysis Level**: Ultra-Think Deep Analysis
**Status**: Ready for Implementation
**Confidence**: Very High

---

## Quick Command Reference

```bash
# Navigate to project
cd F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf

# Flash partition table (Phase 1)
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT partition-table-flash

# Build after code changes (Phase 2)
D:\Espressif\v5.5.1\esp-idf\tools\idf.py fullclean
D:\Espressif\v5.5.1\esp-idf\tools\idf.py build

# Flash firmware
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT flash

# Monitor output
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT monitor

# Exit monitor: Ctrl+]
```

Replace `COM_PORT` with your actual COM port (e.g., COM3, COM4).

---

Good luck with the implementation!
