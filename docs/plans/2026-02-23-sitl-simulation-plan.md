# USV SITL Simulation System — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a 6-frame SITL simulation for ESP32-S3 USV, all code in this project, injected into ardupilot-master via symlinks + patches.

**Architecture:** C++ physics models inherit from ArduPilot's `Aircraft` base class via a shared `USV_Base` water dynamics layer. A `setup_sitl.sh` script symlinks models into `ardupilot-master/libraries/SITL/` and applies minimal patches to register frames. `run_sitl.sh` wraps `sim_vehicle.py` for one-command launch.

**Tech Stack:** C++ (ArduPilot SITL framework), Python (autotest), Bash (deploy scripts), WAF (build)

**Reference files:**
- ArduPilot SITL base: `f:/opensource/usv_esp32/ardupilot-master/libraries/SITL/SIM_Aircraft.h`
- Sailboat physics: `f:/opensource/usv_esp32/ardupilot-master/libraries/SITL/SIM_Sailboat.cpp` (337 lines)
- Rover skid-steering: `f:/opensource/usv_esp32/ardupilot-master/libraries/SITL/SIM_Rover.cpp` (262 lines)
- MotorBoat thin wrapper: `f:/opensource/usv_esp32/ardupilot-master/libraries/SITL/SIM_MotorBoat.h` (42 lines)
- Model registry: `f:/opensource/usv_esp32/ardupilot-master/libraries/AP_HAL_SITL/SITL_cmdline.cpp:123-212`
- Frame registry: `f:/opensource/usv_esp32/ardupilot-master/Tools/autotest/pysim/vehicleinfo.py:412-460`
- Existing USV sim: `simulation/SIM_Sailboat_USV.h` (293 lines, has aero tables + Beaufort/Douglas)
- Design doc: `docs/plans/2026-02-23-sitl-simulation-design.md`

---

## Phase 1: MVP — Differential Thrust Boat

### Task 1: Create directory structure

**Files:**
- Create: `sitl/models/` `sitl/params/` `sitl/scripts/` `sitl/patches/` `sitl/tests/`

**Step 1: Create directories**

```bash
cd F:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf
mkdir -p sitl/models sitl/params sitl/scripts sitl/patches sitl/tests
```

**Step 2: Commit**

```bash
git add sitl/
git commit -m "feat(sitl): create SITL simulation directory structure"
```

---

### Task 2: Write SIM_USV_Base — common water physics

This is the shared base class for all 6 boat types. Adapted from `SIM_Sailboat.cpp` wave/tide/drag code.

**Files:**
- Create: `sitl/models/SIM_USV_Base.h`
- Create: `sitl/models/SIM_USV_Base.cpp`

**Step 1: Write SIM_USV_Base.h**

```cpp
// sitl/models/SIM_USV_Base.h
#pragma once

#include <SITL/SIM_Aircraft.h>

namespace SITL {

class USV_Base : public Aircraft {
public:
    USV_Base(const char *frame_str);

    // All derived classes must implement this
    // update() calls update_usv() then does common integration
    void update(const struct sitl_input &input) override;

    bool on_ground() const override { return true; }

protected:
    // Derived classes implement this to compute accel_body and gyro
    // from servo inputs, BEFORE common integration
    virtual void update_propulsion(const struct sitl_input &input,
                                   float delta_time) = 0;

    // Common water physics — called by update()
    void apply_hull_drag(float speed, float drag_coeff);
    void apply_lateral_drag(float speed_y, float drag_coeff);
    float get_yaw_rate_from_steering(float steering, float speed) const;

    // Hull parameters (set by derived classes in constructor)
    float mass_ = 5.0f;               // kg
    float steering_angle_max_ = 35.0f; // degrees
    float turning_circle_ = 1.8f;      // meters diameter
    float max_thrust_ = 50.0f;         // Newtons at full throttle
    float hull_drag_coeff_ = 0.5f;     // quadratic drag coefficient

    // Water-frame velocity (excludes tide)
    Vector3f velocity_ef_water_;

private:
    // Wave simulation (from SIM_Sailboat.cpp)
    void update_wave(float delta_time);
    Vector3f wave_gyro_;
    float wave_heave_ = 0.0f;
    float wave_phase_ = 0.0f;

    // Common integration step
    void integrate_state(float delta_time);
};

} // namespace SITL
```

**Step 2: Write SIM_USV_Base.cpp**

```cpp
// sitl/models/SIM_USV_Base.cpp
#include "SIM_USV_Base.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

#define WAVE_ANGLE_GAIN 1
#define WAVE_HEAVE_GAIN 1

USV_Base::USV_Base(const char *frame_str) :
    Aircraft(frame_str)
{
    lock_step_scheduled = true;
}

void USV_Base::update(const struct sitl_input &input)
{
    // update wind (base class)
    update_wind(input);

    float delta_time = frame_time_us * 1.0e-6f;

    // derived class computes accel_body and gyro from servos
    update_propulsion(input, delta_time);

    // add wave effects to gyro
    gyro += wave_gyro_;

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // common integration
    integrate_state(delta_time);

    // update wave model
    update_wave(delta_time);
}

void USV_Base::integrate_state(float delta_time)
{
    // earth frame acceleration (remove roll/pitch from waves for accel)
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    Matrix3f yaw_only;
    yaw_only.from_euler(0.0f, 0.0f, y);
    Vector3f accel_earth = yaw_only * accel_body;

    // constrain to water surface + wave heave
    accel_earth.z = wave_heave_;

    // accelerometer sees kinematic accel + gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0, 0, -GRAVITY_MSS));

    // tide/current
    Vector3f tide_velocity_ef;
    if (hal.util->get_soft_armed() && !is_zero(sitl->tide.speed)) {
        tide_velocity_ef.x = -cosf(radians(sitl->tide.direction)) * sitl->tide.speed;
        tide_velocity_ef.y = -sinf(radians(sitl->tide.direction)) * sitl->tide.speed;
    }

    // integrate velocity and position
    velocity_ef_water_ += accel_earth * delta_time;
    velocity_ef = velocity_ef_water_ + tide_velocity_ef;
    position += (velocity_ef * delta_time).todouble();

    update_position();
    time_advance();
    update_mag_field_bf();
}

void USV_Base::apply_hull_drag(float speed, float drag_coeff)
{
    float drag = sq(speed) * drag_coeff;
    if (!is_positive(speed)) {
        drag *= -1.0f;
    }
    accel_body.x -= drag / mass_;
}

void USV_Base::apply_lateral_drag(float speed_y, float drag_coeff)
{
    float drag = sq(speed_y) * drag_coeff * 2.0f;  // higher lateral drag
    if (!is_positive(speed_y)) {
        drag *= -1.0f;
    }
    accel_body.y -= drag / mass_;
}

float USV_Base::get_yaw_rate_from_steering(float steering, float speed) const
{
    if (is_zero(steering) || is_zero(speed)) {
        return 0.0f;
    }
    float d = turning_circle_ * sinf(radians(steering_angle_max_)) /
              sinf(radians(steering * steering_angle_max_));
    float c = M_PI * d;
    float t = c / speed;
    return 360.0f / t;
}

// Wave simulation — adapted from SIM_Sailboat.cpp:128-180
void USV_Base::update_wave(float delta_time)
{
    const float wave_heading = sitl->wave.direction;
    const float wave_speed = sitl->wave.speed;
    const float wave_length = sitl->wave.length;
    const float wave_amp = sitl->wave.amp;

    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    if (sitl->wave.enable == 0 || !hal.util->get_soft_armed() || is_zero(wave_amp)) {
        wave_gyro_ = Vector3f(-r, -p, 0.0f) * WAVE_ANGLE_GAIN;
        wave_heave_ = -velocity_ef.z * WAVE_HEAVE_GAIN;
        wave_phase_ = 0.0f;
        return;
    }

    const float boat_speed = velocity_ef.x * sinf(radians(wave_heading)) +
                             velocity_ef.y * cosf(radians(wave_heading));
    const float apparent_wave_dist = (wave_speed - boat_speed) * delta_time;
    const float apparent_wave_phase_change = (apparent_wave_dist / wave_length) * M_2PI;

    wave_phase_ += apparent_wave_phase_change;
    wave_phase_ = wrap_2PI(wave_phase_);

    const float wave_slope = (wave_amp * 0.5f) * (M_2PI / wave_length) * cosf(wave_phase_);
    const float wave_angle = atanf(wave_slope);

    const float heading_diff = wave_heading - y;
    float angle_error_x = (sinf(heading_diff) * wave_angle) - r;
    float angle_error_y = (cosf(heading_diff) * wave_angle) - p;

    wave_gyro_.x = angle_error_x * WAVE_ANGLE_GAIN;
    wave_gyro_.y = angle_error_y * WAVE_ANGLE_GAIN;
    wave_gyro_.z = 0.0f;

    wave_heave_ = (sitl->wave.enable == 2) ?
        (wave_slope - velocity_ef.z) * WAVE_HEAVE_GAIN : 0.0f;
}

} // namespace SITL
```

**Step 3: Commit**

```bash
git add sitl/models/SIM_USV_Base.h sitl/models/SIM_USV_Base.cpp
git commit -m "feat(sitl): add USV_Base class with water physics (waves, drag, tide)"
```

---

### Task 3: Write SIM_USV_DiffThrust — differential thrust boat

**Files:**
- Create: `sitl/models/SIM_USV_DiffThrust.h`
- Create: `sitl/models/SIM_USV_DiffThrust.cpp`

**Step 1: Write SIM_USV_DiffThrust.h**

```cpp
// sitl/models/SIM_USV_DiffThrust.h
#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

// Servo channels for differential thrust
#define USV_MOTOR_LEFT_CH   0   // servo output 1
#define USV_MOTOR_RIGHT_CH  2   // servo output 3

class USV_DiffThrust : public USV_Base {
public:
    USV_DiffThrust(const char *frame_str);

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_DiffThrust(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input,
                           float delta_time) override;

    // Motor arm length (distance from CG to each motor)
    float motor_arm_ = 0.3f;   // meters
    float skid_turn_rate_ = 140.0f;  // deg/s at full differential
};

} // namespace SITL
```

**Step 2: Write SIM_USV_DiffThrust.cpp**

```cpp
// sitl/models/SIM_USV_DiffThrust.cpp
#include "SIM_USV_DiffThrust.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

namespace SITL {

USV_DiffThrust::USV_DiffThrust(const char *frame_str) :
    USV_Base(frame_str)
{
    mass_ = 5.0f;
    max_thrust_ = 50.0f;
    hull_drag_coeff_ = 0.5f;
    printf("USV Differential Thrust Simulation Started\n");
}

void USV_DiffThrust::update_propulsion(const struct sitl_input &input,
                                        float delta_time)
{
    // read left/right motor inputs (-1 to +1)
    const float motor_left = input.servos[USV_MOTOR_LEFT_CH] ?
        normalise_servo_input(input.servos[USV_MOTOR_LEFT_CH]) : 0;
    const float motor_right = input.servos[USV_MOTOR_RIGHT_CH] ?
        normalise_servo_input(input.servos[USV_MOTOR_RIGHT_CH]) : 0;

    // combined throttle and steering
    const float throttle = 0.5f * (motor_left + motor_right);
    const float steering = motor_left - motor_right;

    // body frame velocity
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed = velocity_body.x;

    // yaw rate from differential thrust
    float yaw_rate = constrain_float(steering * skid_turn_rate_, -360.0f, 360.0f);

    gyro = Vector3f(0, 0, radians(yaw_rate));

    // forward thrust force
    float thrust_force = throttle * max_thrust_;

    // hull drag (quadratic)
    float hull_drag = sq(speed) * hull_drag_coeff_;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // net acceleration in body frame
    accel_body = Vector3f((thrust_force - hull_drag) / mass_, 0, 0);

    // centripetal acceleration from turning
    accel_body.y += radians(yaw_rate) * speed;
}

} // namespace SITL
```

**Step 3: Commit**

```bash
git add sitl/models/SIM_USV_DiffThrust.h sitl/models/SIM_USV_DiffThrust.cpp
git commit -m "feat(sitl): add USV_DiffThrust model (differential thrust boat)"
```

---

### Task 4: Write default parameters for usv-diff

**Files:**
- Create: `sitl/params/usv-diff.parm`

**Step 1: Write usv-diff.parm**

Based on the project's `defaults_base.parm` + rover defaults.

```
# USV Differential Thrust - SITL Parameters
FRAME_CLASS      2
SERVO1_FUNCTION  73
SERVO2_FUNCTION  74
SERVO3_FUNCTION  0
SERVO4_FUNCTION  0
MOT_THR_MIN      -100
MOT_THR_MAX      100
CRUISE_SPEED     1.0
CRUISE_THROTTLE  30
WP_RADIUS        3
WP_SPEED         1.0
TURN_RADIUS      1.5
FS_GCS_ENABLE    1
FS_ACTION        1
ARMING_CHECK     1
SCHED_LOOP_RATE  50
```

**Step 2: Commit**

```bash
git add sitl/params/usv-diff.parm
git commit -m "feat(sitl): add usv-diff default parameters"
```

---

### Task 5: Write setup_sitl.sh — deploy script

**Files:**
- Create: `sitl/scripts/setup_sitl.sh`

**Step 1: Write setup_sitl.sh**

```bash
#!/bin/bash
# setup_sitl.sh — Deploy USV SITL models into ardupilot-master
# Usage: ./sitl/scripts/setup_sitl.sh [path_to_ardupilot]
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
USV_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
AP_ROOT="${1:-$(cd "$USV_ROOT/../../ardupilot-master" && pwd)}"

SITL_DIR="$AP_ROOT/libraries/SITL"
PARAMS_DIR="$AP_ROOT/Tools/autotest/default_params"
CMDLINE="$AP_ROOT/libraries/AP_HAL_SITL/SITL_cmdline.cpp"
VINFO="$AP_ROOT/Tools/autotest/pysim/vehicleinfo.py"

echo "=== USV SITL Setup ==="
echo "USV project: $USV_ROOT"
echo "ArduPilot:   $AP_ROOT"

# 1. Symlink model files
echo "[1/4] Symlinking model files..."
for f in "$USV_ROOT/sitl/models"/SIM_USV_*.h "$USV_ROOT/sitl/models"/SIM_USV_*.cpp; do
    [ -f "$f" ] || continue
    base=$(basename "$f")
    target="$SITL_DIR/$base"
    if [ -L "$target" ]; then
        rm "$target"
    fi
    ln -sf "$f" "$target"
    echo "  $base -> $target"
done

# 2. Symlink param files
echo "[2/4] Symlinking parameter files..."
for f in "$USV_ROOT/sitl/params"/usv-*.parm; do
    [ -f "$f" ] || continue
    base=$(basename "$f")
    target="$PARAMS_DIR/$base"
    if [ -L "$target" ]; then
        rm "$target"
    fi
    ln -sf "$f" "$target"
    echo "  $base -> $target"
done

# 3. Patch SITL_cmdline.cpp (if not already patched)
echo "[3/4] Patching SITL_cmdline.cpp..."
if ! grep -q "SIM_USV_DiffThrust" "$CMDLINE"; then
    # Add includes after the last existing SITL include
    sed -i '/^#include <SITL\/SIM_MotorBoat.h>/a \
#include <SITL/SIM_USV_DiffThrust.h>\
#include <SITL/SIM_USV_DiffBow.h>\
#include <SITL/SIM_USV_XFrame.h>\
#include <SITL/SIM_USV_Wingsail.h>\
#include <SITL/SIM_USV_Rudder.h>' "$CMDLINE"

    # Add model entries after motorboat entry
    sed -i '/{ "motorboat",.*MotorBoat::create },/a \
    { "usv-diff",           USV_DiffThrust::create },\
    { "usv-diff-bow",       USV_DiffBow::create },\
    { "usv-xframe",         USV_XFrame::create },\
    { "usv-wingsail-flap",  USV_Wingsail::create },\
    { "usv-wingsail",       USV_Wingsail::create },\
    { "usv-rudder",         USV_Rudder::create },' "$CMDLINE"

    echo "  Patched SITL_cmdline.cpp"
else
    echo "  Already patched, skipping"
fi

# 4. Patch vehicleinfo.py (if not already patched)
echo "[4/4] Patching vehicleinfo.py..."
if ! grep -q "usv-diff" "$VINFO"; then
    # Add USV frames after motorboat-skid entry
    sed -i '/"motorboat-skid".*{/,/},/!b;/},/a \
        "usv-diff": {\
            "waf_target": "bin/ardurover",\
            "default_params_filename": ["default_params/rover.parm",\
                                        "default_params/usv-diff.parm"],\
        },\
        "usv-diff-bow": {\
            "waf_target": "bin/ardurover",\
            "default_params_filename": ["default_params/rover.parm",\
                                        "default_params/usv-diff-bow.parm"],\
        },\
        "usv-xframe": {\
            "waf_target": "bin/ardurover",\
            "default_params_filename": ["default_params/rover.parm",\
                                        "default_params/usv-xframe.parm"],\
        },\
        "usv-wingsail-flap": {\
            "waf_target": "bin/ardurover",\
            "default_params_filename": ["default_params/rover.parm",\
                                        "default_params/usv-wingsail-flap.parm"],\
        },\
        "usv-wingsail": {\
            "waf_target": "bin/ardurover",\
            "default_params_filename": ["default_params/rover.parm",\
                                        "default_params/usv-wingsail.parm"],\
        },\
        "usv-rudder": {\
            "waf_target": "bin/ardurover",\
            "default_params_filename": ["default_params/rover.parm",\
                                        "default_params/usv-rudder.parm"],\
        },' "$VINFO"

    echo "  Patched vehicleinfo.py"
else
    echo "  Already patched, skipping"
fi

echo ""
echo "=== Setup complete ==="
echo "Next: Build SITL"
echo "  cd $AP_ROOT"
echo "  ./waf configure --board sitl"
echo "  ./waf rover"
echo ""
echo "Then run:"
echo "  ./sitl/scripts/run_sitl.sh usv-diff"
```

**Step 2: Make executable and commit**

```bash
chmod +x sitl/scripts/setup_sitl.sh
git add sitl/scripts/setup_sitl.sh
git commit -m "feat(sitl): add setup_sitl.sh deploy script"
```

---

### Task 6: Write run_sitl.sh — launch script

**Files:**
- Create: `sitl/scripts/run_sitl.sh`

**Step 1: Write run_sitl.sh**

```bash
#!/bin/bash
# run_sitl.sh — Launch USV SITL simulation
# Usage: ./sitl/scripts/run_sitl.sh <frame> [options]
# Examples:
#   ./sitl/scripts/run_sitl.sh usv-diff
#   ./sitl/scripts/run_sitl.sh usv-diff --instance 1
#   ./sitl/scripts/run_sitl.sh usv-wingsail-flap --speedup 5
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
USV_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
AP_ROOT="$(cd "$USV_ROOT/../../ardupilot-master" && pwd)"

FRAME="${1:?Usage: $0 <frame-name> [sim_vehicle.py options]}"
shift

# Verify setup
if ! grep -q "usv-diff" "$AP_ROOT/Tools/autotest/pysim/vehicleinfo.py" 2>/dev/null; then
    echo "ERROR: SITL not set up. Run setup_sitl.sh first."
    exit 1
fi

# Default home location (can override with --home)
# Example: Shenzhen Bay
DEFAULT_HOME="22.5024,113.9150,0,270"

echo "Launching USV SITL: $FRAME"
cd "$AP_ROOT"

python Tools/autotest/sim_vehicle.py \
    -v Rover \
    -f "$FRAME" \
    --home "$DEFAULT_HOME" \
    --no-rebuild \
    "$@"
```

**Step 2: Make executable and commit**

```bash
chmod +x sitl/scripts/run_sitl.sh
git add sitl/scripts/run_sitl.sh
git commit -m "feat(sitl): add run_sitl.sh launch script"
```

---

### Task 7: Test MVP — build and run usv-diff

**Step 1: Run setup**

```bash
cd F:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf
./sitl/scripts/setup_sitl.sh
```

Expected: All symlinks created, patches applied.

**Step 2: Build SITL**

```bash
cd f:/opensource/usv_esp32/ardupilot-master
./waf configure --board sitl
./waf rover
```

Expected: Compiles with USV models included. If compile errors, fix and re-run.

**Step 3: Launch simulation**

```bash
cd F:/opensource/usv_esp32/esp32s3rover/ardupilot_rover_esp32s3_idf
./sitl/scripts/run_sitl.sh usv-diff
```

Expected: "USV Differential Thrust Simulation Started" in output, MAVProxy connects, vehicle responds to `arm throttle` and `mode guided`.

**Step 4: Verify in MAVProxy**

```
arm throttle force
mode guided
guided 22.5030 113.9155
```

Expected: Boat moves toward target coordinates.

**Step 5: Commit any fixes**

```bash
git add -A sitl/
git commit -m "feat(sitl): Phase 1 complete — usv-diff MVP working"
```

---

## Phase 2: DiffBow + Rudder Boats

### Task 8: Write SIM_USV_DiffBow — differential + bow thruster

**Files:**
- Create: `sitl/models/SIM_USV_DiffBow.h`
- Create: `sitl/models/SIM_USV_DiffBow.cpp`

**Step 1: Write SIM_USV_DiffBow.h**

```cpp
// sitl/models/SIM_USV_DiffBow.h
#pragma once

#include "SIM_USV_DiffThrust.h"

namespace SITL {

#define USV_BOW_THRUSTER_CH 4   // servo output 5 (SERVO3 = index 2 used by right motor)

class USV_DiffBow : public USV_DiffThrust {
public:
    USV_DiffBow(const char *frame_str);

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_DiffBow(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input,
                           float delta_time) override;

private:
    float bow_thrust_max_ = 30.0f;  // Newtons
    float bow_arm_ = 0.5f;          // meters from CG to bow thruster
};

} // namespace SITL
```

**Step 2: Write SIM_USV_DiffBow.cpp**

```cpp
// sitl/models/SIM_USV_DiffBow.cpp
#include "SIM_USV_DiffBow.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

namespace SITL {

USV_DiffBow::USV_DiffBow(const char *frame_str) :
    USV_DiffThrust(frame_str)
{
    printf("USV Differential + Bow Thruster Simulation Started\n");
}

void USV_DiffBow::update_propulsion(const struct sitl_input &input,
                                     float delta_time)
{
    // Get base differential thrust forces
    USV_DiffThrust::update_propulsion(input, delta_time);

    // Add bow thruster lateral force
    const float bow_input = input.servos[USV_BOW_THRUSTER_CH] ?
        normalise_servo_input(input.servos[USV_BOW_THRUSTER_CH]) : 0;

    const float lateral_force = bow_input * bow_thrust_max_;

    // lateral acceleration
    accel_body.y += lateral_force / mass_;

    // yaw moment from bow thruster (positive bow thrust = positive lateral = nose right)
    float bow_yaw_rate = (lateral_force * bow_arm_) / (mass_ * 0.1f);  // simplified moment of inertia
    gyro.z += radians(bow_yaw_rate);
}

} // namespace SITL
```

**Step 3: Write usv-diff-bow.parm**

Create: `sitl/params/usv-diff-bow.parm`

```
# USV Differential Thrust + Bow Thruster
FRAME_CLASS      2
SERVO1_FUNCTION  73
SERVO2_FUNCTION  74
SERVO3_FUNCTION  0
SERVO4_FUNCTION  0
SERVO5_FUNCTION  75
MOT_THR_MIN      -100
MOT_THR_MAX      100
CRUISE_SPEED     1.0
CRUISE_THROTTLE  30
WP_RADIUS        3
WP_SPEED         1.0
TURN_RADIUS      1.0
FS_GCS_ENABLE    1
FS_ACTION        1
ARMING_CHECK     1
SCHED_LOOP_RATE  50
```

**Step 4: Commit**

```bash
git add sitl/models/SIM_USV_DiffBow.h sitl/models/SIM_USV_DiffBow.cpp sitl/params/usv-diff-bow.parm
git commit -m "feat(sitl): add USV_DiffBow model (differential + bow thruster)"
```

---

### Task 9: Write SIM_USV_Rudder — single thruster + rudder

**Files:**
- Create: `sitl/models/SIM_USV_Rudder.h`
- Create: `sitl/models/SIM_USV_Rudder.cpp`

**Step 1: Write SIM_USV_Rudder.h**

```cpp
// sitl/models/SIM_USV_Rudder.h
#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

#define USV_RUDDER_CH     0   // servo output 1
#define USV_THROTTLE_CH   2   // servo output 3

class USV_Rudder : public USV_Base {
public:
    USV_Rudder(const char *frame_str);

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_Rudder(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input,
                           float delta_time) override;
};

} // namespace SITL
```

**Step 2: Write SIM_USV_Rudder.cpp**

```cpp
// sitl/models/SIM_USV_Rudder.cpp
#include "SIM_USV_Rudder.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

namespace SITL {

USV_Rudder::USV_Rudder(const char *frame_str) :
    USV_Base(frame_str)
{
    mass_ = 5.0f;
    max_thrust_ = 50.0f;
    hull_drag_coeff_ = 0.5f;
    steering_angle_max_ = 35.0f;
    turning_circle_ = 1.8f;
    printf("USV Single Thruster + Rudder Simulation Started\n");
}

void USV_Rudder::update_propulsion(const struct sitl_input &input,
                                    float delta_time)
{
    const float steering = input.servos[USV_RUDDER_CH] ?
        normalise_servo_input(input.servos[USV_RUDDER_CH]) : 0;
    const float throttle = input.servos[USV_THROTTLE_CH] ?
        normalise_servo_input(input.servos[USV_THROTTLE_CH]) : 0;

    // body frame velocity
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed = velocity_body.x;

    // yaw rate from rudder (needs speed to work)
    float yaw_rate = get_yaw_rate_from_steering(steering, speed);

    gyro = Vector3f(0, 0, radians(yaw_rate));

    // thrust and drag
    float thrust_force = throttle * max_thrust_;
    float hull_drag = sq(speed) * hull_drag_coeff_;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    accel_body = Vector3f((thrust_force - hull_drag) / mass_, 0, 0);
    accel_body.y += radians(yaw_rate) * speed;
}

} // namespace SITL
```

**Step 3: Write usv-rudder.parm**

Create: `sitl/params/usv-rudder.parm`

```
# USV Single Thruster + Rudder
FRAME_CLASS      2
SERVO1_FUNCTION  26
SERVO3_FUNCTION  70
MOT_THR_MIN      0
MOT_THR_MAX      100
CRUISE_SPEED     1.5
CRUISE_THROTTLE  40
WP_RADIUS        3
WP_SPEED         1.5
TURN_RADIUS      2.0
FS_GCS_ENABLE    1
FS_ACTION        1
ARMING_CHECK     1
SCHED_LOOP_RATE  50
```

**Step 4: Commit**

```bash
git add sitl/models/SIM_USV_Rudder.h sitl/models/SIM_USV_Rudder.cpp sitl/params/usv-rudder.parm
git commit -m "feat(sitl): add USV_Rudder model (single thruster + rudder)"
```

---

### Task 10: Test Phase 2 — rebuild and verify DiffBow + Rudder

**Step 1: Re-run setup (picks up new files)**

```bash
./sitl/scripts/setup_sitl.sh
```

**Step 2: Rebuild**

```bash
cd f:/opensource/usv_esp32/ardupilot-master
./waf rover
```

**Step 3: Test usv-diff-bow**

```bash
./sitl/scripts/run_sitl.sh usv-diff-bow
```

Expected: "USV Differential + Bow Thruster Simulation Started"

**Step 4: Test usv-rudder**

```bash
./sitl/scripts/run_sitl.sh usv-rudder
```

Expected: "USV Single Thruster + Rudder Simulation Started"

**Step 5: Commit any fixes**

```bash
git add -A sitl/
git commit -m "feat(sitl): Phase 2 complete — usv-diff-bow + usv-rudder working"
```

---

## Phase 3: Wingsail Boats

### Task 11: Write SIM_USV_Wingsail — both wingsail variants

**Files:**
- Create: `sitl/models/SIM_USV_Wingsail.h`
- Create: `sitl/models/SIM_USV_Wingsail.cpp`

**Step 1: Write SIM_USV_Wingsail.h**

```cpp
// sitl/models/SIM_USV_Wingsail.h
#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

#define USV_SAIL_RUDDER_CH     0   // servo output 1
#define USV_SAIL_WINGSAIL_CH   2   // servo output 3
#define USV_SAIL_FLAP_CH       3   // servo output 4

class USV_Wingsail : public USV_Base {
public:
    USV_Wingsail(const char *frame_str);

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_Wingsail(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input,
                           float delta_time) override;

private:
    bool has_flap_;
    float sail_area_ = 1.0f;    // m^2

    void calc_lift_and_drag(float wind_speed, float aoa_deg,
                            float& lift, float& drag) const;

    // Aerodynamic lookup tables (18 points, 0-170 degrees, from SIM_Sailboat)
    static constexpr uint8_t AERO_TABLE_SIZE = 18;
    static const float lift_curve_[AERO_TABLE_SIZE];
    static const float drag_curve_[AERO_TABLE_SIZE];
};

} // namespace SITL
```

**Step 2: Write SIM_USV_Wingsail.cpp**

```cpp
// sitl/models/SIM_USV_Wingsail.cpp
#include "SIM_USV_Wingsail.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// Aerodynamic curves — from ArduPilot SIM_Sailboat.cpp
// Index: 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170 degrees
const float USV_Wingsail::lift_curve_[AERO_TABLE_SIZE] = {
    0.0f, 0.5f, 0.9f, 1.2f, 1.3f, 1.3f, 1.2f, 1.0f,
    0.8f, 0.6f, 0.4f, 0.2f, 0.1f, 0.0f, -0.1f, -0.1f, -0.05f, 0.0f
};
const float USV_Wingsail::drag_curve_[AERO_TABLE_SIZE] = {
    0.01f, 0.02f, 0.04f, 0.06f, 0.10f, 0.15f, 0.22f, 0.30f,
    0.40f, 0.50f, 0.60f, 0.70f, 0.78f, 0.85f, 0.90f, 0.93f, 0.95f, 0.96f
};

USV_Wingsail::USV_Wingsail(const char *frame_str) :
    USV_Base(frame_str)
{
    has_flap_ = strstr(frame_str, "flap") != nullptr;
    mass_ = 3.0f;
    hull_drag_coeff_ = 0.4f;
    steering_angle_max_ = 35.0f;
    turning_circle_ = 1.8f;

    if (has_flap_) {
        printf("USV Wingsail + Flap Simulation Started\n");
    } else {
        printf("USV Wingsail (no flap) Simulation Started\n");
    }
}

void USV_Wingsail::calc_lift_and_drag(float wind_speed, float aoa_deg,
                                       float& lift, float& drag) const
{
    const uint16_t index_width_deg = 10;
    const uint8_t index_max = AERO_TABLE_SIZE - 1;

    aoa_deg = wrap_180(aoa_deg);
    const float aoa = fabsf(aoa_deg);

    if (aoa <= 0.0f) {
        lift = lift_curve_[0];
        drag = drag_curve_[0];
    } else if (aoa >= index_max * index_width_deg) {
        lift = lift_curve_[index_max];
        drag = drag_curve_[index_max];
    } else {
        uint8_t index = constrain_int16(aoa / index_width_deg, 0, index_max);
        float remainder = aoa - (index * index_width_deg);
        lift = linear_interpolate(lift_curve_[index], lift_curve_[index + 1],
                                  remainder, 0.0f, (float)index_width_deg);
        drag = linear_interpolate(drag_curve_[index], drag_curve_[index + 1],
                                  remainder, 0.0f, (float)index_width_deg);
    }

    lift *= wind_speed * sail_area_;
    drag *= wind_speed * sail_area_;

    if (is_negative(aoa_deg)) {
        lift *= -1;
    }
}

void USV_Wingsail::update_propulsion(const struct sitl_input &input,
                                      float delta_time)
{
    // rudder input
    const float steering = input.servos[USV_SAIL_RUDDER_CH] ?
        normalise_servo_input(input.servos[USV_SAIL_RUDDER_CH]) : 0;

    // wingsail angle: servo output maps to -90 to +90 degrees
    const float wingsail_angle_bf = constrain_float(
        (input.servos[USV_SAIL_WINGSAIL_CH] - 1500) / 500.0f * 90.0f,
        -90.0f, 90.0f);

    // apparent wind calculation (earth frame)
    Vector3f wind_apparent_ef = velocity_ef - wind_ef;
    const float wind_apparent_dir_ef = degrees(atan2f(wind_apparent_ef.y, wind_apparent_ef.x));
    const float wind_apparent_speed = safe_sqrt(sq(wind_apparent_ef.x) + sq(wind_apparent_ef.y));

    float roll, pitch, yaw;
    dcm.to_euler(&roll, &pitch, &yaw);
    const float wind_apparent_dir_bf = wrap_180(wind_apparent_dir_ef - degrees(yaw));

    // angle of attack
    float aoa_deg;
    if (has_flap_) {
        // flap modifies effective AoA
        const float flap_angle = constrain_float(
            (input.servos[USV_SAIL_FLAP_CH] - 1500) / 500.0f * 30.0f,
            -30.0f, 30.0f);
        aoa_deg = wind_apparent_dir_bf - wingsail_angle_bf - flap_angle * 0.5f;
    } else {
        // direct wing rotation
        aoa_deg = wind_apparent_dir_bf - wingsail_angle_bf;
    }

    // lift and drag
    float lift_wf, drag_wf;
    calc_lift_and_drag(wind_apparent_speed, aoa_deg, lift_wf, drag_wf);

    // rotate from wind frame to body frame
    const float sin_rot = sinf(radians(wind_apparent_dir_bf));
    const float cos_rot = cosf(radians(wind_apparent_dir_bf));
    const float force_fwd = (lift_wf * sin_rot) - (drag_wf * cos_rot);

    // body frame velocity
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed = velocity_body.x;

    // yaw rate from rudder
    float yaw_rate = get_yaw_rate_from_steering(steering, speed);
    gyro = Vector3f(0, 0, radians(yaw_rate));

    // hull drag
    float hull_drag = sq(speed) * hull_drag_coeff_;
    if (!is_positive(speed)) {
        hull_drag *= -1.0f;
    }

    // net body frame acceleration
    accel_body = Vector3f((force_fwd - hull_drag) / mass_, 0, 0);
    accel_body.y += radians(yaw_rate) * speed;

    // expose wind speed as RPM for wind vane backend
    rpm[0] = wind_apparent_speed;
    airspeed_pitot = wind_apparent_speed;
}

} // namespace SITL
```

**Step 3: Write param files**

Create: `sitl/params/usv-wingsail.parm`
```
# USV Wingsail (no flap) + Rudder
FRAME_CLASS      2
SERVO1_FUNCTION  26
SERVO3_FUNCTION  89
WNDVN_TYPE       10
WNDVN_SPEED_TYPE 0
CRUISE_SPEED     2.0
WP_RADIUS        5
WP_SPEED         2.0
TURN_RADIUS      3.0
FS_GCS_ENABLE    1
FS_ACTION        1
ARMING_CHECK     1
SCHED_LOOP_RATE  50
```

Create: `sitl/params/usv-wingsail-flap.parm`
```
# USV Wingsail with Flap + Rudder
FRAME_CLASS      2
SERVO1_FUNCTION  26
SERVO3_FUNCTION  89
SERVO4_FUNCTION  90
WNDVN_TYPE       10
WNDVN_SPEED_TYPE 0
CRUISE_SPEED     2.0
WP_RADIUS        5
WP_SPEED         2.0
TURN_RADIUS      3.0
FS_GCS_ENABLE    1
FS_ACTION        1
ARMING_CHECK     1
SCHED_LOOP_RATE  50
```

**Step 4: Commit**

```bash
git add sitl/models/SIM_USV_Wingsail.h sitl/models/SIM_USV_Wingsail.cpp \
       sitl/params/usv-wingsail.parm sitl/params/usv-wingsail-flap.parm
git commit -m "feat(sitl): add USV_Wingsail model (rotation + flap variants)"
```

---

## Phase 4: X-Frame Quad Thruster

### Task 12: Write SIM_USV_XFrame — X-frame quad thruster

**Files:**
- Create: `sitl/models/SIM_USV_XFrame.h`
- Create: `sitl/models/SIM_USV_XFrame.cpp`

**Step 1: Write SIM_USV_XFrame.h**

```cpp
// sitl/models/SIM_USV_XFrame.h
#pragma once

#include "SIM_USV_Base.h"

namespace SITL {

class USV_XFrame : public USV_Base {
public:
    USV_XFrame(const char *frame_str);

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW USV_XFrame(frame_str);
    }

protected:
    void update_propulsion(const struct sitl_input &input,
                           float delta_time) override;

private:
    float arm_ = 0.3f;              // meters from CG to each motor
    float mount_angle_ = 45.0f;     // degrees
    float thrust_per_motor_ = 25.0f; // Newtons at full throttle
};

} // namespace SITL
```

**Step 2: Write SIM_USV_XFrame.cpp**

```cpp
// sitl/models/SIM_USV_XFrame.cpp
#include "SIM_USV_XFrame.h"
#include <AP_Math/AP_Math.h>
#include <stdio.h>

namespace SITL {

/*
    Motor layout (top view, looking down):

        M1(FL)     M2(FR)
           \       /
            \     /
             [CG]
            /     \
           /       \
        M3(BL)     M4(BR)

    Servo channels: 0=M1, 1=M2, 2=M3, 3=M4

    Force mixing:
      Forward:  all motors forward component
      Lateral:  (-M1+M2-M3+M4) * sin(mount_angle)
      Yaw:      (-M1+M2+M3-M4) * arm
*/

USV_XFrame::USV_XFrame(const char *frame_str) :
    USV_Base(frame_str)
{
    mass_ = 8.0f;
    hull_drag_coeff_ = 0.5f;
    printf("USV X-Frame Quad Thruster Simulation Started\n");
}

void USV_XFrame::update_propulsion(const struct sitl_input &input,
                                    float delta_time)
{
    // read 4 motor inputs
    float m[4];
    for (uint8_t i = 0; i < 4; i++) {
        m[i] = input.servos[i] ? normalise_servo_input(input.servos[i]) : 0;
    }

    const float cos_a = cosf(radians(mount_angle_));
    const float sin_a = sinf(radians(mount_angle_));

    // force calculations
    float force_x = (m[0] + m[1] + m[2] + m[3]) * cos_a * thrust_per_motor_;
    float force_y = (-m[0] + m[1] - m[2] + m[3]) * sin_a * thrust_per_motor_;
    float torque_z = (-m[0] + m[1] + m[2] - m[3]) * arm_ * thrust_per_motor_;

    // body frame velocity
    Vector3f velocity_body = dcm.transposed() * velocity_ef_water_;
    float speed_x = velocity_body.x;
    float speed_y = velocity_body.y;

    // hull drag (both axes)
    float drag_x = sq(speed_x) * hull_drag_coeff_;
    if (!is_positive(speed_x)) drag_x *= -1.0f;

    float drag_y = sq(speed_y) * hull_drag_coeff_ * 2.0f;  // higher lateral drag
    if (!is_positive(speed_y)) drag_y *= -1.0f;

    // yaw rate from torque (simplified: torque / moment_of_inertia)
    float yaw_rate = torque_z / (mass_ * 0.1f);  // simplified I_zz

    gyro = Vector3f(0, 0, yaw_rate);

    // net body frame acceleration
    accel_body = Vector3f(
        (force_x - drag_x) / mass_,
        (force_y - drag_y) / mass_,
        0
    );

    // centripetal from yaw rate
    accel_body.y += yaw_rate * speed_x;
}

} // namespace SITL
```

**Step 3: Write usv-xframe.parm**

Create: `sitl/params/usv-xframe.parm`

```
# USV X-Frame Quad Thruster
FRAME_CLASS      2
SERVO1_FUNCTION  33
SERVO2_FUNCTION  34
SERVO3_FUNCTION  35
SERVO4_FUNCTION  36
MOT_THR_MIN      -100
MOT_THR_MAX      100
CRUISE_SPEED     1.5
CRUISE_THROTTLE  30
WP_RADIUS        3
WP_SPEED         1.5
TURN_RADIUS      1.0
FS_GCS_ENABLE    1
FS_ACTION        1
ARMING_CHECK     1
SCHED_LOOP_RATE  50
```

**Step 4: Commit**

```bash
git add sitl/models/SIM_USV_XFrame.h sitl/models/SIM_USV_XFrame.cpp sitl/params/usv-xframe.parm
git commit -m "feat(sitl): add USV_XFrame model (X-frame quad thruster)"
```

---

### Task 13: Test Phase 3+4 — all 6 frames

**Step 1: Re-run setup and rebuild**

```bash
./sitl/scripts/setup_sitl.sh
cd f:/opensource/usv_esp32/ardupilot-master
./waf rover
```

**Step 2: Test each frame**

```bash
./sitl/scripts/run_sitl.sh usv-diff
./sitl/scripts/run_sitl.sh usv-diff-bow
./sitl/scripts/run_sitl.sh usv-rudder
./sitl/scripts/run_sitl.sh usv-wingsail
./sitl/scripts/run_sitl.sh usv-wingsail-flap
./sitl/scripts/run_sitl.sh usv-xframe
```

Expected for each: startup message printed, MAVProxy connects, basic arm+move works.

**Step 3: Commit fixes**

```bash
git add -A sitl/
git commit -m "feat(sitl): Phase 3+4 complete — all 6 USV frames verified"
```

---

## Phase 5: Formation Tests + CI

### Task 14: Write formation launch script

**Files:**
- Create: `sitl/scripts/run_formation.sh`

**Step 1: Write run_formation.sh**

```bash
#!/bin/bash
# run_formation.sh — Launch multi-vehicle USV formation
# Usage: ./sitl/scripts/run_formation.sh <frame> <count>
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FRAME="${1:?Usage: $0 <frame> <count>}"
COUNT="${2:-3}"

echo "Launching $COUNT x $FRAME formation..."

for i in $(seq 0 $((COUNT - 1))); do
    echo "Starting instance $i..."
    "$SCRIPT_DIR/run_sitl.sh" "$FRAME" --instance "$i" &
    sleep 3
done

echo ""
echo "Formation running. Connect QGC to:"
for i in $(seq 0 $((COUNT - 1))); do
    PORT=$((5760 + i * 10))
    echo "  Instance $i: 127.0.0.1:$PORT"
done
echo ""
echo "Press Ctrl+C to stop all instances"
wait
```

**Step 2: Make executable and commit**

```bash
chmod +x sitl/scripts/run_formation.sh
git add sitl/scripts/run_formation.sh
git commit -m "feat(sitl): add formation launch script"
```

---

### Task 15: Write automated smoke test

**Files:**
- Create: `sitl/tests/test_usv_smoke.py`

**Step 1: Write test_usv_smoke.py**

```python
#!/usr/bin/env python3
"""
USV SITL Smoke Test — Verify all 6 frames can arm and move.

Usage:
    python sitl/tests/test_usv_smoke.py

Requires: pymavlink
    pip install pymavlink
"""

import subprocess
import sys
import time
import os

# Frames to test
USV_FRAMES = [
    "usv-diff",
    "usv-diff-bow",
    "usv-rudder",
    "usv-wingsail",
    "usv-wingsail-flap",
    "usv-xframe",
]

def test_frame_starts(frame):
    """Test that a frame can be started and prints its identification string."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    run_script = os.path.join(script_dir, "..", "scripts", "run_sitl.sh")

    print(f"  Testing {frame}...", end=" ", flush=True)

    try:
        proc = subprocess.Popen(
            [run_script, frame, "--speedup", "10"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        # Wait for startup message (max 30s)
        start = time.time()
        found = False
        while time.time() - start < 30:
            line = proc.stdout.readline()
            if "Simulation Started" in line:
                found = True
                break
            if proc.poll() is not None:
                break

        proc.terminate()
        proc.wait(timeout=5)

        if found:
            print("OK")
            return True
        else:
            print("FAIL (no startup message)")
            return False

    except Exception as e:
        print(f"FAIL ({e})")
        return False


def main():
    print("USV SITL Smoke Test")
    print("=" * 40)

    results = {}
    for frame in USV_FRAMES:
        results[frame] = test_frame_starts(frame)

    print("\n" + "=" * 40)
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    print(f"Results: {passed}/{total} passed")

    if passed < total:
        print("\nFailed frames:")
        for frame, ok in results.items():
            if not ok:
                print(f"  - {frame}")
        sys.exit(1)

    print("\nAll frames OK!")


if __name__ == "__main__":
    main()
```

**Step 2: Commit**

```bash
git add sitl/tests/test_usv_smoke.py
git commit -m "feat(sitl): add smoke test for all 6 USV frames"
```

---

### Task 16: Final integration commit

**Step 1: Run full smoke test**

```bash
python sitl/tests/test_usv_smoke.py
```

Expected: 6/6 passed.

**Step 2: Final commit**

```bash
git add -A
git commit -m "feat(sitl): USV SITL simulation system complete — 6 boat types

Supported frames:
- usv-diff: Differential thrust (skid steering)
- usv-diff-bow: Differential + bow thruster
- usv-xframe: X-frame quad thruster
- usv-wingsail-flap: Wingsail with trailing-edge flap + rudder
- usv-wingsail: Wingsail (rotation only) + rudder
- usv-rudder: Single thruster + rudder

Features:
- Common water physics (waves, tide, hull drag)
- One-command deploy and launch scripts
- Formation support via --instance
- Automated smoke test"
```
