# Implementation Patches for ICM20948 I2C Bypass Mode

This document contains all code changes needed to implement I2C bypass mode for ICM20948 with internal AK09916 magnetometer.

---

## Patch 1: Enhanced Storage Diagnostics

**File**: `libraries/AP_HAL_ESP32/Storage.cpp`

**Location**: Lines 39-47 (function `_storage_open`)

**Change Type**: Enhancement (optional but recommended)

```cpp
// BEFORE:
void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }
#ifdef STORAGEDEBUG
    printf("%s:%d _storage_open \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _dirty_mask.clearall();

    // Initialize _last_empty_ms to current time to make healthy() work
    _last_empty_ms = AP_HAL::millis();

    p = esp_partition_find_first((esp_partition_type_t)0x45, ESP_PARTITION_SUBTYPE_ANY, nullptr);

    // Check if partition was found
    if (p == nullptr) {
        printf("WARNING: Storage partition (type 0x45) not found, using empty storage\n");
        _use_empty_storage = true;
        _initialised = true;
        return;
    }

    // load from storage backend
    _flash_load();
    _initialised = true;
}

// AFTER:
void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }
#ifdef STORAGEDEBUG
    printf("%s:%d _storage_open \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _dirty_mask.clearall();

    // Initialize _last_empty_ms to current time to make healthy() work
    _last_empty_ms = AP_HAL::millis();

    p = esp_partition_find_first((esp_partition_type_t)0x45, ESP_PARTITION_SUBTYPE_ANY, nullptr);

    // Check if partition was found
    if (p == nullptr) {
        printf("ERROR: Storage partition (type 0x45) not found!\n");
        printf("  This indicates the partition table was not properly flashed.\n");
        printf("  Please run: idf.py partition-table-flash\n");
        printf("  Falling back to empty storage (calibration data will NOT persist)\n");
        _use_empty_storage = true;
        _initialised = true;
        return;
    }

    printf("Storage partition found: addr=0x%08x, size=%u bytes\n",
           (unsigned)p->address, (unsigned)p->size);

    // load from storage backend
    _flash_load();
    _initialised = true;
}
```

---

## Patch 2: Enable ICM20948 I2C Bypass Mode

**File**: `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.cpp`

**Location**: Function `_hardware_init()` (around line 350-400, exact location depends on existing code)

**Change Type**: Critical - Required for compass functionality

**Instructions**:
Find the `_hardware_init()` function and add the bypass mode initialization AFTER the successful chip initialization but BEFORE the function returns true.

```cpp
// Add this code near the end of _hardware_init(), after all register initialization
// but before the final "return true;"

    // Enable I2C bypass mode for I2C-connected ICM20948
    // This allows direct access to internal AK09916 magnetometer
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C &&
        _inv2_type == Invensensev2_ICM20948) {

        // Read current INT_PIN_CFG value
        uint8_t int_pin_cfg = _register_read(INV2REG_INT_PIN_CFG);

        debug("ICM20948: Current INT_PIN_CFG = 0x%02x", int_pin_cfg);

        // Set BYPASS_EN bit (bit 1)
        int_pin_cfg |= BIT_BYPASS_EN;
        _register_write(INV2REG_INT_PIN_CFG, int_pin_cfg, true);

        hal.scheduler->delay(10);  // Allow bypass mode to stabilize

        // Verify bypass mode was enabled
        uint8_t verify = _register_read(INV2REG_INT_PIN_CFG);
        if (verify & BIT_BYPASS_EN) {
            debug("ICM20948: I2C bypass mode enabled successfully (0x%02x)", verify);
        } else {
            debug("ICM20948: WARNING - Failed to enable I2C bypass mode");
        }
    }

    return true;
```

**Exact Location** (use this to find where to insert):
Look for the end of `_hardware_init()` where it returns `true`. Insert the bypass code just before the final `return true;` statement.

---

## Patch 3: Add Direct I2C Bus Driver - Header

**File**: `libraries/AP_Compass/AP_Compass_AK09916.h`

**Location**: Add new class declaration after existing bus driver classes

**Change Type**: New code addition

```cpp
// Add this class declaration in the header file
// Place it after the existing AP_AK09916_BusDriver classes
// but before the final #endif

/**
 * Direct I2C bus driver for AK09916 when ICM20948 is in bypass mode
 * This allows direct I2C communication with the magnetometer without
 * using the ICM20948's auxiliary bus (master mode)
 */
class AP_AK09916_BusDriver_Direct_I2C : public AP_AK09916_BusDriver
{
public:
    AP_AK09916_BusDriver_Direct_I2C(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool block_read(uint8_t reg, uint8_t *buf, uint32_t size) override;
    bool register_read(uint8_t reg, uint8_t *val) override;
    bool register_write(uint8_t reg, uint8_t val) override;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec,
        AP_HAL::Device::PeriodicCb cb) override;

    AP_HAL::Semaphore *get_semaphore() override;

    bool configure() override {
        // No special configuration needed for direct I2C access
        return true;
    }

    bool start_measurements() override {
        // Measurement start is handled by AK09916 init sequence
        return true;
    }

    AP_HAL::Device::PeriodicHandle get_periodic() override {
        return _periodic;
    }

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    AP_HAL::Device::PeriodicHandle _periodic;
};
```

---

## Patch 4: Implement Direct I2C Bus Driver

**File**: `libraries/AP_Compass/AP_Compass_AK09916.cpp`

**Location**: Add implementation after existing bus driver implementations

**Change Type**: New code addition

```cpp
// Add this implementation in the .cpp file
// Place it after the existing bus driver implementations

/*
 * Direct I2C Bus Driver Implementation
 * Used when ICM20948 is in I2C bypass mode
 */

AP_AK09916_BusDriver_Direct_I2C::AP_AK09916_BusDriver_Direct_I2C(
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
{
    if (_dev) {
        // Set I2C speed for magnetometer (typically 400kHz works well)
        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    }
}

bool AP_AK09916_BusDriver_Direct_I2C::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    if (!_dev) {
        return false;
    }
    return _dev->read_registers(reg, buf, size);
}

bool AP_AK09916_BusDriver_Direct_I2C::register_read(uint8_t reg, uint8_t *val)
{
    if (!_dev) {
        return false;
    }
    return _dev->read_registers(reg, val, 1);
}

bool AP_AK09916_BusDriver_Direct_I2C::register_write(uint8_t reg, uint8_t val)
{
    if (!_dev) {
        return false;
    }
    return _dev->write_register(reg, val);
}

AP_HAL::Device::PeriodicHandle AP_AK09916_BusDriver_Direct_I2C::register_periodic_callback(
    uint32_t period_usec,
    AP_HAL::Device::PeriodicCb cb)
{
    if (!_dev) {
        return nullptr;
    }
    _periodic = _dev->register_periodic_callback(period_usec, cb);
    return _periodic;
}

AP_HAL::Semaphore *AP_AK09916_BusDriver_Direct_I2C::get_semaphore()
{
    if (!_dev) {
        return nullptr;
    }
    return _dev->get_semaphore();
}
```

---

## Patch 5: Update Compass Probe Logic

**File**: `libraries/AP_Compass/AP_Compass_AK09916.cpp`

**Location**: Function `probe_ICM20948_I2C()` (around lines 195-213)

**Change Type**: Critical - Modify existing function

```cpp
// BEFORE:
AP_Compass_Backend *AP_Compass_AK09916::probe_ICM20948_I2C(uint8_t inv2_instance,
                                                     enum Rotation rotation)
{
    AP_InertialSensor &ins = AP::ins();

    AP_AK09916_BusDriver *bus =
        NEW_NOTHROW AP_AK09916_BusDriver_Auxiliary(ins, HAL_INS_INV2_I2C, inv2_instance, HAL_COMPASS_AK09916_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK09916 *sensor = NEW_NOTHROW AP_Compass_AK09916(bus, false, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// AFTER:
AP_Compass_Backend *AP_Compass_AK09916::probe_ICM20948_I2C(uint8_t inv2_instance,
                                                     enum Rotation rotation)
{
    AP_InertialSensor &ins = AP::ins();

    // First, check if the ICM20948 is on I2C bus
    // If so, we must use direct I2C access via bypass mode
    AP_InertialSensor_Backend *backend = ins.get_backend(inv2_instance);

    if (backend != nullptr) {
        // Try to cast to Invensensev2 backend
        // This is safe because we know the backend ID is HAL_INS_INV2_I2C
        auto *inv2_backend = dynamic_cast<AP_InertialSensor_Invensensev2*>(backend);

        if (inv2_backend != nullptr) {
            // Access the device to check bus type
            // Note: We need to add a public accessor for this, or use friend class
            // For now, we'll use a different approach based on backend detection

            // Try to get the auxiliary bus - if it returns nullptr for I2C, we know
            // we need to use direct I2C access
            AuxiliaryBus *aux_bus = backend->get_auxiliary_bus();

            if (aux_bus == nullptr) {
                // ICM20948 is on I2C bus - use direct I2C access (bypass mode)
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AK09916: Using direct I2C (bypass mode)");

                // Get the I2C bus number from the backend
                // We'll probe on the standard ESP32 I2C bus with AK09916 address
                // The bypass mode makes AK09916 visible at address 0x0C on same bus

                // Try common I2C bus numbers for ESP32
                for (uint8_t bus_num = 0; bus_num < 2; bus_num++) {
                    auto dev = hal.i2c_mgr->get_device(bus_num, HAL_COMPASS_AK09916_I2C_ADDR);
                    if (!dev) {
                        continue;
                    }

                    // Try to communicate with the device
                    dev->get_semaphore()->take_blocking();
                    uint8_t whoami = 0;
                    bool ok = dev->read_registers(REG_COMPANY_ID, &whoami, 1);
                    dev->get_semaphore()->give();

                    if (ok && whoami == 0x48) {  // AK09916 company ID
                        // Found the magnetometer!
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AK09916: Found on I2C bus %u", bus_num);

                        AP_AK09916_BusDriver *bus =
                            NEW_NOTHROW AP_AK09916_BusDriver_Direct_I2C(std::move(dev));
                        if (!bus) {
                            return nullptr;
                        }

                        AP_Compass_AK09916 *sensor =
                            NEW_NOTHROW AP_Compass_AK09916(bus, false, rotation);
                        if (!sensor || !sensor->init()) {
                            delete sensor;
                            return nullptr;
                        }

                        return sensor;
                    }
                }

                // If we get here, direct I2C access failed
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AK09916: Direct I2C probe failed");
                return nullptr;
            }
        }
    }

    // Fall back to auxiliary bus method (for SPI-connected ICM20948)
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AK09916: Using auxiliary bus mode");

    AP_AK09916_BusDriver *bus =
        NEW_NOTHROW AP_AK09916_BusDriver_Auxiliary(ins, HAL_INS_INV2_I2C,
                                                    inv2_instance, HAL_COMPASS_AK09916_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK09916 *sensor = NEW_NOTHROW AP_Compass_AK09916(bus, false, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}
```

---

## Patch 6: Make Device Accessible (Alternative Approach)

If the dynamic_cast approach doesn't work well, we can add a helper method to the backend.

**File**: `libraries/AP_InertialSensor/AP_InertialSensor_Invensensev2.h`

**Location**: Add new public method in class definition

```cpp
// Add to public section of AP_InertialSensor_Invensensev2 class:

    /**
     * Check if this IMU is on I2C bus
     * Used by compass probe to determine if bypass mode is needed
     */
    bool is_on_i2c_bus() const {
        return _dev && _dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C;
    }

    /**
     * Get I2C bus number for this device (if on I2C)
     * Returns 0xFF if not on I2C
     */
    uint8_t get_i2c_bus_number() const {
        if (_dev && _dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C) {
            return _dev->bus_num();
        }
        return 0xFF;
    }
```

**Then update Patch 5 to use this simpler approach**:

```cpp
// Simplified probe_ICM20948_I2C using the new helper methods:

AP_Compass_Backend *AP_Compass_AK09916::probe_ICM20948_I2C(uint8_t inv2_instance,
                                                     enum Rotation rotation)
{
    AP_InertialSensor &ins = AP::ins();
    AP_InertialSensor_Backend *backend = ins.get_backend(inv2_instance);

    if (backend != nullptr) {
        // Check if auxiliary bus is available
        AuxiliaryBus *aux_bus = backend->get_auxiliary_bus();

        if (aux_bus == nullptr) {
            // No auxiliary bus - ICM20948 must be on I2C, use bypass mode
            AP_InertialSensor_Invensensev2 &inv2 =
                AP_InertialSensor_Invensensev2::from(*backend);

            if (inv2.is_on_i2c_bus()) {
                uint8_t bus_num = inv2.get_i2c_bus_number();

                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                    "AK09916: Using I2C bypass mode on bus %u", bus_num);

                auto dev = hal.i2c_mgr->get_device(bus_num, HAL_COMPASS_AK09916_I2C_ADDR);
                if (!dev) {
                    return nullptr;
                }

                AP_AK09916_BusDriver *bus =
                    NEW_NOTHROW AP_AK09916_BusDriver_Direct_I2C(std::move(dev));
                if (!bus) {
                    return nullptr;
                }

                AP_Compass_AK09916 *sensor =
                    NEW_NOTHROW AP_Compass_AK09916(bus, false, rotation);
                if (!sensor || !sensor->init()) {
                    delete sensor;
                    return nullptr;
                }

                return sensor;
            }
        }
    }

    // Fall back to auxiliary bus method (for SPI-connected ICM20948)
    AP_AK09916_BusDriver *bus =
        NEW_NOTHROW AP_AK09916_BusDriver_Auxiliary(ins, HAL_INS_INV2_I2C,
                                                    inv2_instance, HAL_COMPASS_AK09916_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK09916 *sensor = NEW_NOTHROW AP_Compass_AK09916(bus, false, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}
```

---

## Build and Flash Commands

After applying all patches:

```bash
# Navigate to project directory
cd F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf

# Clean build (recommended after major changes)
D:\Espressif\v5.5.1\esp-idf\tools\idf.py fullclean

# Build
D:\Espressif\v5.5.1\esp-idf\tools\idf.py build

# Flash partition table first
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT partition-table-flash

# Flash firmware
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT app-flash

# Or flash everything
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT flash

# Monitor output
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT monitor
```

---

## Testing Procedure

1. **After flashing, check serial output for:**
   ```
   Storage partition found: addr=0x...
   ICM20948: I2C bypass mode enabled successfully
   AK09916: Using I2C bypass mode on bus 0
   AK09916: Found on I2C bus 0
   ```

2. **Connect to Mission Planner**

3. **Run calibrations:**
   - Initial Setup -> Accel Calibration
   - Initial Setup -> Compass Calibration

4. **Reboot and verify:**
   - Calibration data persists
   - Pre-arm checks pass (except Radio)
   - EKF3 converges
   - HUD displays correct attitude

5. **Check log file:**
   ```
   F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\log\log.txt
   ```

   Should see:
   - No "Param storage failed"
   - No "Compass not healthy"
   - Only "Radio failsafe on" (expected without RC)

---

## Troubleshooting

### If storage still fails:
```bash
# Erase entire flash and reflash everything
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT erase_flash
D:\Espressif\v5.5.1\esp-idf\tools\idf.py -p COM_PORT flash
```

### If compass still not detected:
1. Check I2C pull-up resistors (2.2K-4.7K to 3.3V)
2. Verify GPIO 8/9 are correctly configured
3. Use I2C scanner to verify AK09916 is visible at 0x0C
4. Add more debug logging to bypass enable code

### I2C Scanner Code (for debugging):
```cpp
// Add to startup code temporarily:
for (uint8_t addr = 1; addr < 127; addr++) {
    auto dev = hal.i2c_mgr->get_device(0, addr);
    if (dev) {
        dev->get_semaphore()->take_blocking();
        uint8_t tmp;
        bool found = dev->read_registers(0, &tmp, 1);
        dev->get_semaphore()->give();
        if (found) {
            printf("I2C device found at 0x%02X\n", addr);
        }
    }
}
```

---

## Summary

Total files modified: 5-6
- Storage.cpp (optional diagnostic enhancement)
- AP_InertialSensor_Invensensev2.cpp (bypass enable - CRITICAL)
- AP_InertialSensor_Invensensev2.h (helper methods - RECOMMENDED)
- AP_Compass_AK09916.h (new driver class - CRITICAL)
- AP_Compass_AK09916.cpp (driver implementation and probe logic - CRITICAL)

Implementation time: 2-3 hours
Testing time: 30 minutes

Success indicators:
- Storage partition found
- Bypass mode enabled
- Compass detected and healthy
- All calibrations work and persist
- EKF3 converges properly

---

*Patches Created: 2025-11-04*
*Target: ESP32-S3 ArduPilot Rover with ICM20948 on I2C*
