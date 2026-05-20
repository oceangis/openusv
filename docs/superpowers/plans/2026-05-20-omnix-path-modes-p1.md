# OMNIX Path Modes P1 Implementation Plan — AR_OmniControl + ModeDP Refactor

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract the verified 3-DOF position+heading PID kernel from `Rover/mode_dp.cpp` into a reusable `libraries/AR_OmniControl/` library, then refactor `ModeDP` to use it. P1 produces zero behavioral change — its success criterion is the existing `dp_*.py` bench scripts all passing regression. P1 unblocks P2 (AUTO/GUIDED OMNIX branches) and beyond.

**Architecture:** New library exposes `AR_OmniControl` class owning all PID parameters (`DP_POS_P`, `DP_YAW_P`, etc. — names preserved so EEPROM keys stay stable). Class is instantiated once inside `ParametersG2` as `g2.omni_ctrl`. `ModeDP` strips its `var_info` and PID state, becoming a thin wrapper that captures target on entry and calls `g2.omni_ctrl.update()/get_outputs()`.

**Tech Stack:** ESP-IDF 5.5.1, ArduPilot HAL, `AP_Param` subgroup pattern, existing `bench/verify/dp_*.py` scripts as regression harness.

**Spec:** `docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md`

---

## File Structure

**New files:**
- `libraries/AR_OmniControl/AR_OmniControl.h` — controller class declaration
- `libraries/AR_OmniControl/AR_OmniControl.cpp` — PID kernel implementation + `var_info`
- `libraries/AR_OmniControl/CMakeLists.txt` — component stub (vestigial, but matches sibling libs)

**Modified files:**
- `Rover/mode.h` — strip migrated members from `ModeDP` class; drop `var_info`
- `Rover/mode_dp.cpp` — drop `var_info` body; rewrite `_enter`/`_exit`/`update` as thin wrapper around `g2.omni_ctrl`
- `Rover/Parameters.h` — add `AR_OmniControl omni_ctrl;` to `ParametersG2`; add `_omni_yaw_mode` + `_omni_los_look` declarations
- `Rover/Parameters.cpp` — replace `mode_dp` subgroup at index 60 with `omni_ctrl` (same prefix `"DP"`); add `OMNI_YAW_MODE` and `OMNI_LOS_LOOK` as new G2 params

**Unchanged (verify):**
- `bench/verify/dp_arm_diag.py`, `dp_final_test.py`, `dp_mode_test.py`, `omnix_mix_test.py` — used as regression harness, must pass post-refactor

---

## Task 1: Create AR_OmniControl library skeleton

**Files:**
- Create: `libraries/AR_OmniControl/AR_OmniControl.h`
- Create: `libraries/AR_OmniControl/AR_OmniControl.cpp`
- Create: `libraries/AR_OmniControl/CMakeLists.txt`

- [ ] **Step 1: Create the header file**

```cpp
// libraries/AR_OmniControl/AR_OmniControl.h
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// Shared 3-DOF position + heading controller for OMNIX (X-config 4-thruster
// holonomic) boats. Used by ModeDP, ModeAuto (OMNIX branch), ModeGuided, etc.
// Owns PID parameters under prefix "DP_" (names preserved from original
// ModeDP for EEPROM key stability).
class AR_OmniControl {
public:
    AR_OmniControl();

    CLASS_NO_COPY(AR_OmniControl);

    // Set target NED position (m, from EKF origin), target heading (rad),
    // optional NED velocity feed-forward (m/s, reserved for path-follower
    // modes — unused in P1).
    void set_target(const Vector2f& pos_ned,
                    float yaw_rad,
                    const Vector2f& vel_ff_ned = Vector2f{0.0f, 0.0f});

    // Run PID + R(psi)^T rotation. Reads AHRS internally.
    // dt: loop period (s). Caller passes rover.G_Dt.
    void update(float dt);

    // Fetch most-recent outputs in body frame, normalized [-1, +1].
    // Returns false if AHRS position estimate was unavailable in last update;
    // caller should stop motors and trigger HOLD failsafe.
    bool get_outputs(float& fwd, float& lat, float& steer_norm) const;

    // Zero integrators and previous-error history. Call on mode entry.
    void reset();

    // Parameter table (DP_POS_P, DP_YAW_P, ... — see .cpp for full list).
    static const struct AP_Param::GroupInfo var_info[];

private:
    // --- parameters (registered in var_info) ---
    AP_Float _pos_p;
    AP_Float _pos_i;
    AP_Float _pos_d;
    AP_Float _yaw_p;
    AP_Float _yaw_i;
    AP_Float _yaw_d;
    AP_Float _pos_db;
    AP_Float _yaw_db;
    AP_Float _speed_max;
    AP_Float _imax;
    AP_Int16 _options;

    // --- target ---
    Vector2f _target_pos;
    float    _target_yaw;
    Vector2f _vel_ff;

    // --- PID state ---
    Vector2f _pos_integ;
    Vector2f _prev_pos_err;
    float    _yaw_integ;
    float    _prev_yaw_err;

    // --- last outputs ---
    float _out_fwd;
    float _out_lat;
    float _out_steer;
    bool  _outputs_valid;
};
```

- [ ] **Step 2: Create the .cpp skeleton with empty bodies**

```cpp
// libraries/AR_OmniControl/AR_OmniControl.cpp
#include "AR_OmniControl.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

const AP_Param::GroupInfo AR_OmniControl::var_info[] = {
    // Populated in Task 2.
    AP_GROUPEND
};

AR_OmniControl::AR_OmniControl()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AR_OmniControl::set_target(const Vector2f& pos_ned, float yaw_rad,
                                const Vector2f& vel_ff_ned)
{
    _target_pos = pos_ned;
    _target_yaw = yaw_rad;
    _vel_ff = vel_ff_ned;
}

void AR_OmniControl::update(float dt)
{
    // Implemented in Task 4.
    (void)dt;
    _outputs_valid = false;
}

bool AR_OmniControl::get_outputs(float& fwd, float& lat, float& steer_norm) const
{
    fwd = _out_fwd;
    lat = _out_lat;
    steer_norm = _out_steer;
    return _outputs_valid;
}

void AR_OmniControl::reset()
{
    _pos_integ.zero();
    _prev_pos_err.zero();
    _yaw_integ = 0.0f;
    _prev_yaw_err = 0.0f;
    _out_fwd = _out_lat = _out_steer = 0.0f;
    _outputs_valid = false;
}
```

- [ ] **Step 3: Create CMakeLists.txt (vestigial — matches sibling libs)**

```cmake
# ArduPilot AR_OmniControl 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AR_OmniControl.cpp"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_timer driver
)

set(COMPONENT_SRCS
    "AR_OmniControl.cpp"
)

set(COMPONENT_ADD_INCLUDEDIRS
    "."
)
```

Note: the project's actual build uses `GLOB_RECURSE` in `components/ardupilot/CMakeLists.txt` to pick up all `libraries/**/*.cpp`. This CMakeLists exists for consistency only.

- [ ] **Step 4: Touch the top CMakeLists so glob re-scans on next build**

Run:
```bash
touch components/ardupilot/CMakeLists.txt
```

- [ ] **Step 5: Build to verify nothing is broken**

Run:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: `_build_log.txt` ends with `Project build complete.` (or equivalent IDF success message), no errors from AR_OmniControl files. Existing ModeDP code is unaffected since nothing imports the new class yet.

- [ ] **Step 6: Commit**

```bash
git add libraries/AR_OmniControl/ components/ardupilot/CMakeLists.txt
git commit -m "AR_OmniControl: library skeleton (no logic yet)

Empty class wired into project tree via GLOB_RECURSE. Bodies filled
in subsequent tasks. Builds clean; no callers."
```

---

## Task 2: Populate AR_OmniControl::var_info with migrated DP_* parameters

**Files:**
- Modify: `libraries/AR_OmniControl/AR_OmniControl.cpp:5-7` (the `var_info[]` body)

- [ ] **Step 1: Fill in var_info with all 11 parameters at original indices**

Replace the empty `var_info` from Task 1 with this exact block (indices 1-11 match what `ModeDP` currently uses, so EEPROM keys for `DP_POS_P` etc. resolve unchanged):

```cpp
const AP_Param::GroupInfo AR_OmniControl::var_info[] = {

    // @Param: _POS_P
    // @DisplayName: OMNIX position P gain
    // @Description: Proportional gain, NED position error (m) to force
    // @User: Standard
    AP_GROUPINFO("_POS_P", 1, AR_OmniControl, _pos_p, 0.20f),

    // @Param: _POS_I
    // @DisplayName: OMNIX position I gain
    // @User: Standard
    AP_GROUPINFO("_POS_I", 2, AR_OmniControl, _pos_i, 0.05f),

    // @Param: _POS_D
    // @DisplayName: OMNIX position D gain
    // @User: Standard
    AP_GROUPINFO("_POS_D", 3, AR_OmniControl, _pos_d, 0.0f),

    // @Param: _YAW_P
    // @DisplayName: OMNIX heading P gain
    // @User: Standard
    AP_GROUPINFO("_YAW_P", 4, AR_OmniControl, _yaw_p, 2.0f),

    // @Param: _YAW_I
    // @DisplayName: OMNIX heading I gain
    // @User: Standard
    AP_GROUPINFO("_YAW_I", 5, AR_OmniControl, _yaw_i, 0.10f),

    // @Param: _YAW_D
    // @DisplayName: OMNIX heading D gain
    // @User: Standard
    AP_GROUPINFO("_YAW_D", 6, AR_OmniControl, _yaw_d, 0.0f),

    // @Param: _POS_DB
    // @DisplayName: OMNIX position deadband
    // @Description: Position error below this is ignored (GPS noise suppression)
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("_POS_DB", 7, AR_OmniControl, _pos_db, 1.5f),

    // @Param: _YAW_DB
    // @DisplayName: OMNIX heading deadband
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("_YAW_DB", 8, AR_OmniControl, _yaw_db, 5.0f),

    // @Param: _SPEED
    // @DisplayName: OMNIX max correction speed
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("_SPEED", 9, AR_OmniControl, _speed_max, 1.0f),

    // @Param: _IMAX
    // @DisplayName: OMNIX integrator limit
    // @User: Standard
    AP_GROUPINFO("_IMAX", 10, AR_OmniControl, _imax, 0.5f),

    // @Param: _OPTIONS
    // @DisplayName: OMNIX options bitmask
    // @Description: reserved for stick-nudge and future options
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 11, AR_OmniControl, _options, 0),

    AP_GROUPEND
};
```

- [ ] **Step 2: Build to verify**

Run:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: build succeeds. No new parameters appear at runtime yet because nobody calls `AP_SUBGROUPINFO(omni_ctrl, ...)` — that's Task 6.

- [ ] **Step 3: Commit**

```bash
git add libraries/AR_OmniControl/AR_OmniControl.cpp
git commit -m "AR_OmniControl: declare DP_* parameter table

Names + indices + defaults match existing ModeDP::var_info exactly so
EEPROM keys remain stable after migration. Display names rebranded
DP -> OMNIX to reflect new ownership."
```

---

## Task 3: Implement AR_OmniControl::update() — PID kernel + R(psi)^T

**Files:**
- Modify: `libraries/AR_OmniControl/AR_OmniControl.cpp:32-36` (the `update()` body)

This is a direct port of the working loop from `Rover/mode_dp.cpp:116-185`. No algorithmic changes — only refactored to live inside the controller class.

- [ ] **Step 1: Implement update() body**

Replace the `update()` stub with:

```cpp
void AR_OmniControl::update(float dt)
{
    AP_AHRS& ahrs = AP::ahrs();

    // Position estimate required
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        _out_fwd = _out_lat = _out_steer = 0.0f;
        _outputs_valid = false;
        return;
    }

    const float yaw = radians(ahrs.yaw_sensor * 0.01f);   // yaw_sensor in cdeg
    const Vector2f pos_now(pos_ned.x, pos_ned.y);

    // --- Position error + deadband ---
    Vector2f e = _target_pos - pos_now;
    if (e.length() < _pos_db) {
        e.zero();
        _pos_integ.zero();
    }

    // --- Position PID (NED frame vector) ---
    if (is_positive(dt)) {
        _pos_integ += e * (_pos_i * dt);
    }
    _pos_integ.x = constrain_float(_pos_integ.x, -_imax, _imax);
    _pos_integ.y = constrain_float(_pos_integ.y, -_imax, _imax);
    Vector2f pos_deriv;
    if (is_positive(dt)) {
        pos_deriv = (e - _prev_pos_err) * (1.0f / dt);
    }
    _prev_pos_err = e;
    const Vector2f f_ned = e * _pos_p + _pos_integ + pos_deriv * _pos_d;

    // --- Heading error + deadband ---
    float ey = wrap_PI(_target_yaw - yaw);
    if (fabsf(ey) < radians(_yaw_db)) {
        ey = 0.0f;
        _yaw_integ = 0.0f;
    }

    // --- Heading PID ---
    if (is_positive(dt)) {
        _yaw_integ += ey * (_yaw_i * dt);
    }
    _yaw_integ = constrain_float(_yaw_integ, -_imax, _imax);
    float yaw_deriv = 0.0f;
    if (is_positive(dt)) {
        yaw_deriv = (ey - _prev_yaw_err) * (1.0f / dt);
    }
    _prev_yaw_err = ey;
    float m_yaw = ey * _yaw_p + _yaw_integ + yaw_deriv * _yaw_d;

    // --- R(psi)^T: rotate NED force into body frame ---
    const float cy = cosf(yaw);
    const float sy = sinf(yaw);
    float forward =  cy * f_ned.x + sy * f_ned.y;
    float lateral = -sy * f_ned.x + cy * f_ned.y;

    // --- Saturate ---
    _out_fwd   = constrain_float(forward, -1.0f, 1.0f);
    _out_lat   = constrain_float(lateral, -1.0f, 1.0f);
    _out_steer = constrain_float(m_yaw,   -1.0f, 1.0f);
    _outputs_valid = true;
}
```

- [ ] **Step 2: Build to verify**

Run:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: build succeeds. Still no behavioral change — class is never instantiated.

- [ ] **Step 3: Commit**

```bash
git add libraries/AR_OmniControl/AR_OmniControl.cpp
git commit -m "AR_OmniControl: port PID kernel from ModeDP

Direct port of the loop body from mode_dp.cpp:116-185. NED position
PID + heading PID + R(psi)^T rotation to body frame, saturation to
[-1, +1]. Returns _outputs_valid=false on missing position estimate."
```

---

## Task 4: Wire AR_OmniControl into ParametersG2 (subgroup prefix "DP", index 60)

**Files:**
- Modify: `Rover/Parameters.h:435` — replace `ModeDP mode_dp;` with `AR_OmniControl omni_ctrl;`
- Modify: `Rover/Parameters.cpp:647-649` — replace `mode_dp` subgroup with `omni_ctrl` subgroup at same index/prefix

The critical invariant: subgroup **prefix `"DP"` and index `60`** must remain identical. EEPROM stores params keyed by `(group_index, sub_index)` — changing index 60 to anything else would orphan all saved `DP_POS_P` etc. on existing boards.

- [ ] **Step 1: Add the include + member to Parameters.h**

At the top of `Rover/Parameters.h`, find the existing AR_* / AP_* include block and add:
```cpp
#include <AR_OmniControl/AR_OmniControl.h>
```

(Place it alphabetically near `#include <AR_WPNav/...>` if such an include exists; otherwise group with other library includes near the top.)

In `Rover/Parameters.h:435`, replace:
```cpp
    class ModeDP mode_dp;
```
with:
```cpp
    AR_OmniControl omni_ctrl;
```

- [ ] **Step 2: Replace the subgroup registration in Parameters.cpp**

In `Rover/Parameters.cpp:647-649`, replace this block:
```cpp
    // @Group: DP
    // @Path: mode_dp.cpp
    AP_SUBGROUPINFO(mode_dp, "DP", 60, ParametersG2, ModeDP),
```
with:
```cpp
    // @Group: DP
    // @Path: ../libraries/AR_OmniControl/AR_OmniControl.cpp
    AP_SUBGROUPINFO(omni_ctrl, "DP", 60, ParametersG2, AR_OmniControl),
```

- [ ] **Step 3: Build to verify**

Run:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: **build will FAIL** because `ModeDP` still declares its own `var_info` and member parameters; both `ModeDP` and `AR_OmniControl` are now competing to own `DP_*`. This is intentional — Task 5 strips ModeDP. Verify the failure is from duplicate `DP_*` parameters or missing `mode_dp` reference (not from any other issue).

- [ ] **Step 4: Commit (build broken — declared intentional in commit message)**

```bash
git add Rover/Parameters.h Rover/Parameters.cpp
git commit -m "Parameters: route DP_* subgroup to g2.omni_ctrl (build broken)

Replaces g2.mode_dp at subgroup index 60 prefix \"DP\" with
g2.omni_ctrl. EEPROM keys for DP_POS_P etc. remain unchanged. Build
will fail until Task 5 strips ModeDP's own var_info — intentional
mid-refactor checkpoint."
```

---

## Task 5: Strip ModeDP — remove migrated params/state, refactor to thin wrapper

**Files:**
- Modify: `Rover/mode.h:619-667` — strip migrated parameter members and target state from ModeDP class
- Modify: `Rover/mode_dp.cpp` — full rewrite as thin wrapper

- [ ] **Step 1: Strip ModeDP class declaration**

In `Rover/mode.h`, replace the entire `class ModeDP` block (lines 619-667) with:

```cpp
// Dynamic Positioning mode — X-config 4-thruster holonomic boats only.
// Holds GPS position and heading simultaneously. PID kernel lives in
// AR_OmniControl (g2.omni_ctrl); ModeDP is a thin wrapper that captures
// the target on entry and dispatches to the controller.
class ModeDP : public Mode
{
public:

    ModeDP() {}

    CLASS_NO_COPY(ModeDP);

    Number mode_number() const override { return Number::DP; }
    const char *name4() const override { return "DP"; }

    void update() override;

    bool is_autopilot_mode() const override { return true; }

protected:

    bool _enter() override;
    void _exit() override;

private:
    bool _have_target = false;
};
```

Note: `var_info` declaration and all `AP_Float`/`AP_Int16` members are gone. PID state (integrators, prev errors) is gone — lives in `AR_OmniControl`. Only `_have_target` remains.

- [ ] **Step 2: Rewrite mode_dp.cpp as wrapper**

Replace the entire contents of `Rover/mode_dp.cpp` with:

```cpp
#include "Rover.h"

/*
  ModeDP — Dynamic Positioning (X-config 4-thruster holonomic boats only).

  Holds GPS position and heading. PID/rotation kernel lives in
  AR_OmniControl (g2.omni_ctrl). This class only:
    - gates entry on FRAME_TYPE_OMNIX + position estimate
    - snapshots target on entry
    - dispatches update() to the shared controller
    - degrades to HOLD on lost position estimate

  Design: docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md
*/

bool ModeDP::_enter()
{
    // Gate: OMNIX frame only
    if (g2.motors.get_frame_type() != AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DP: requires OMNIX frame");
        return false;
    }

    // Gate: need a position estimate to snapshot the target
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "DP: no position estimate");
        return false;
    }

    // Snapshot current position + heading as the hold target
    const Vector2f target_pos(pos_ned.x, pos_ned.y);
    const float target_yaw = radians(ahrs.yaw_sensor * 0.01f);
    g2.omni_ctrl.reset();
    g2.omni_ctrl.set_target(target_pos, target_yaw);
    _have_target = true;

    gcs().send_text(MAV_SEVERITY_INFO, "DP: holding position");
    return true;
}

void ModeDP::_exit()
{
    _have_target = false;
    g2.omni_ctrl.reset();
}

void ModeDP::update()
{
    if (!_have_target) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    g2.omni_ctrl.update(rover.G_Dt);

    float fwd, lat, steer_norm;
    if (!g2.omni_ctrl.get_outputs(fwd, lat, steer_norm)) {
        // Lost position estimate — stop motors, degrade to HOLD
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    g2.motors.set_throttle(fwd * 100.0f);
    g2.motors.set_lateral(lat * 100.0f);
    g2.motors.set_steering(steer_norm * 4500.0f, false);
}
```

- [ ] **Step 3: Touch top CMakeLists to force CMake reconfigure**

Run:
```bash
touch components/ardupilot/CMakeLists.txt
```

- [ ] **Step 4: Build**

Run:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: build SUCCEEDS. If it fails, common causes:
- `rover.mode_hold` not visible — check it's still public in `Rover.h`
- `g2.omni_ctrl` not found — check Task 4 changes to Parameters.h
- Missing include of `AR_OmniControl.h` somewhere — `Rover.h` should already pull `Parameters.h` which includes it

- [ ] **Step 5: Commit**

```bash
git add Rover/mode.h Rover/mode_dp.cpp components/ardupilot/CMakeLists.txt
git commit -m "ModeDP: refactor to thin wrapper over g2.omni_ctrl

PID kernel and DP_* parameters have moved to AR_OmniControl. ModeDP
keeps only entry gating (OMNIX frame + position estimate), target
snapshot on _enter, dispatch to g2.omni_ctrl.update/get_outputs in
update(), and failsafe to HOLD on lost position estimate.

Behavioral equivalence to pre-refactor ModeDP — verified by
bench/verify/dp_*.py regression in Task 8."
```

---

## Task 6: Add OMNI_YAW_MODE + OMNI_LOS_LOOK to ParametersG2 (skeleton for P2)

These two params are declared now so the EEPROM layout is set before P2 starts using them. ModeDP doesn't consume them in P1.

**Files:**
- Modify: `Rover/Parameters.h` — add two `AP_*` members near the other `g2` scalars
- Modify: `Rover/Parameters.cpp` — register the two params at fresh subgroup indices

- [ ] **Step 1: Add members to ParametersG2**

In `Rover/Parameters.h`, find a clean spot near the other top-level `g2` `AP_Float`/`AP_Int*` members (e.g., adjacent to `manual_steering_expo`, around line 426). Add:

```cpp
    // OMNIX path-mode yaw strategy (consumed by AUTO/GUIDED/RTL/etc. in P2+)
    // 0=LOCK_INITIAL, 1=TANGENT, 2=POINT_NEXT_WP, 3=MANUAL_RC
    AP_Int8  _omni_yaw_mode;

    // Look-ahead distance (m) for TANGENT mode (consumed in P2+)
    AP_Float _omni_los_look;
```

- [ ] **Step 2: Register in var_info**

In `Rover/Parameters.cpp`, find the highest currently-used index (61 is free — 60 is `DP`, 59 is LORA_). Add to the `ParametersG2::var_info[]` block, before `AP_GROUPEND`:

```cpp
    // @Param: OMNI_YAW_MODE
    // @DisplayName: OMNIX path-mode yaw strategy
    // @Description: 0=LockInitial 1=Tangent 2=PointNextWP 3=ManualRC. Used by AUTO/GUIDED/RTL/etc. when frame type is OMNIX.
    // @Values: 0:LockInitial,1:Tangent,2:PointNextWP,3:ManualRC
    // @User: Standard
    AP_GROUPINFO("OMNI_YAW_MODE", 61, ParametersG2, _omni_yaw_mode, 1),

    // @Param: OMNI_LOS_LOOK
    // @DisplayName: OMNIX line-of-sight look-ahead distance
    // @Description: Look-ahead distance for tangent heading mode
    // @Units: m
    // @Range: 0.5 10.0
    // @User: Standard
    AP_GROUPINFO("OMNI_LOS_LOOK", 62, ParametersG2, _omni_los_look, 2.0f),
```

- [ ] **Step 3: Build**

Run:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: build succeeds. Two new params (`OMNI_YAW_MODE`, `OMNI_LOS_LOOK`) appear at runtime but are not yet consumed by any code path.

- [ ] **Step 4: Commit**

```bash
git add Rover/Parameters.h Rover/Parameters.cpp
git commit -m "Parameters: add OMNI_YAW_MODE and OMNI_LOS_LOOK (P2 placeholders)

Skeleton params declared at g2 indices 61/62. P1 does not consume
them — ModeDP uses fixed snapshot-at-entry behavior. P2 will read
these in update_omnix() of AUTO/GUIDED/RTL/etc."
```

---

## Task 7: Flash and run dp_arm_diag.py regression

**Files:** none modified. Pure verification.

- [ ] **Step 1: Confirm COM port**

Run:
```bash
powershell.exe -NoProfile -Command "Get-CimInstance Win32_PnPEntity | Where { $_.Name -match 'COM\d+' } | Select-Object Name"
```

Expected: ESP32-S3 device on COM10 (or another COM number — if not COM10, update `_flash_dp.bat` before next step). The CLAUDE.md note "板子 USB COM 口会漂" applies.

- [ ] **Step 2: Flash via _flash_dp.bat**

Run:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_dp.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: `_flash_log.txt` ends with `Hash of data verified.` and `Leaving... Hard resetting via RTS pin...`

- [ ] **Step 3: Run dp_arm_diag.py**

Run:
```bash
python bench/verify/dp_arm_diag.py --port COM10
```

Expected output ends with `总体: PASS`. Key sub-results from the prior baseline:
- `dp_mode_registered = True` (mode 17 in HEARTBEAT after `MAV_CMD_DO_SET_MODE 17`)
- `dp_gate_omnix = True` (entry rejected on non-OMNIX frame)
- `dp_param_count >= 11` (DP_POS_P through DP_OPTIONS all visible — must still see DP_OPTIONS even though OMNIX-rebranded display names show OMNIX_* in description)

If `dp_param_count < 11`: the prefix `"DP"` subgroup wiring (Task 4 step 2) is wrong. EEPROM keys won't match and saved values will be lost.

- [ ] **Step 4: Commit log on PASS**

If the regression passes, no code commit needed. Just note in conversation: "Task 7 PASS, baseline behavior preserved through refactor."

If it FAILS: do NOT proceed. Diagnose. Most likely culprits:
- Param prefix changed accidentally (Task 4 step 2 wrong)
- ModeDP entry gate broken (Task 5 step 2 — check `_enter()`)
- AHRS yaw read changed (`ahrs.yaw_sensor` units — confirm `*0.01f` for cdeg → deg, then `radians()`)

---

## Task 8: Run dp_final_test.py + omnix_mix_test.py regression

**Files:** none modified. Pure verification.

- [ ] **Step 1: Run dp_final_test.py (full DP arm + drive)**

Pre-requisite: GPS must have FIX_3D for DP entry. Confirm with:
```bash
python bench/verify/gps_check.py --port COM10
```
If `sats < 8` or `fix < 3`: move the board near the window or to the balcony; wait until `sats >= 8` and `hdop < 2.0` before proceeding.

Then run:
```bash
python bench/verify/dp_final_test.py --port COM10
```

Expected output: `总体: PASS`. Key sub-results:
- `dp_entered = True` (mode 17 accepted)
- `dp_armed = True` (force-arm under DP succeeds)
- `dp_drives_motors = True` (SERVO_OUTPUT_RAW deviates >5 from 1500 within 20s with DP_POS_DB=0)
- `dp_stable_armed = True` (still in mode 17 after 20s)

If `dp_drives_motors = False`: the PID kernel port in Task 3 is wrong somewhere. Compare `AR_OmniControl::update()` line-by-line against the original `ModeDP::update()` in `git show 218f09ac24:Rover/mode_dp.cpp`.

- [ ] **Step 2: Run omnix_mix_test.py (4-thruster mixing unchanged)**

Run:
```bash
python bench/verify/omnix_mix_test.py --port COM10
```

Expected: `总体: PASS`. Sub-results:
- `no_motor_setup_error = True`
- `motor_test_all_respond = True`
- `manual_armed = True`
- `mix_forward = True`, `mix_lateral = True`, `mix_yaw = True`

This validates that the mixer layer (`AP_MotorsUGV::output_omni`) was not touched by the refactor — the refactor only moved the PID layer above it.

- [ ] **Step 3: Capture regression evidence**

After both pass, append a summary line to memory:
```
P1 regression 2026-05-20: dp_arm_diag PASS / dp_final_test PASS / omnix_mix_test PASS
```

If both pass: P1 is done. Proceed to Task 9.

If either fails: stop, diagnose, fix, re-run from Task 7 step 2 (re-flash).

---

## Task 9: Final commit + memory update

**Files:**
- Modify: `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\omnix_dp_mode.md` — note P1 refactor done
- Modify: `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\MEMORY.md` — add pointer line for AR_OmniControl

- [ ] **Step 1: Update omnix_dp_mode memory note**

Append a section to `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\omnix_dp_mode.md`:

```markdown

## P1 完成 (2026-05-20):AR_OmniControl 抽出 + ModeDP 重构成薄壳
- PID 内核 + DP_* 参数搬到 `libraries/AR_OmniControl/`
- `g2.omni_ctrl` 单例,EEPROM 键名(DP_*)不变
- ModeDP 只剩入口门禁 + target snapshot + dispatch + 失位降级 HOLD
- 新增 G2 params `OMNI_YAW_MODE`(default 1=TANGENT)、`OMNI_LOS_LOOK`(default 2.0m) —— P1 不消费,P2 用
- 回归: dp_arm_diag PASS, dp_final_test PASS, omnix_mix_test PASS
- 下一步 (P2):ModeAuto::update_omnix() —— 复用 AR_WPNav 目标 + 走 g2.omni_ctrl
```

- [ ] **Step 2: Add new memory file for AR_OmniControl shared controller**

Create `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\ar_omnicontrol_library.md`:

```markdown
---
name: ar-omnicontrol-library
description: "AR_OmniControl — 3-DOF position+heading PID library shared by all OMNIX path modes"
metadata: 
  type: project
---

OMNIX 全驱船共享 3-DOF 控制器,放在 `libraries/AR_OmniControl/`。一个 g2.omni_ctrl 单例,
所有 OMNIX-aware 模式(ModeDP/AUTO/GUIDED/RTL/SMART_RTL/LOITER/DOCK)走同一份内核。

**参数(prefix DP_,sub-group index 60)**: DP_POS_P/I/D, DP_YAW_P/I/D, DP_POS_DB,
DP_YAW_DB, DP_SPEED, DP_IMAX, DP_OPTIONS。名字保留 DP_ 是为了 EEPROM 键不变。

**API**:
- `set_target(pos_ned, yaw_rad, vel_ff_ned=0)` —— mode 在 _enter()/update_omnix() 调
- `update(dt)` —— 主循环每次调,dt 用 `rover.G_Dt`
- `get_outputs(fwd, lat, steer_norm)` —— 取 body-frame 输出 (-1..+1),
  返回 false → 失位 → 调用方应触发 HOLD failsafe
- `reset()` —— 切模式时清积分器

**控制律**: NED 位置 PID(死区) + 航向 PID(死区) → R(ψ)^T 旋到船体系 → 限幅。
直接从 ModeDP::update() 抽出,行为 1:1 等价。

**给 P2+ 的接入点**:
```cpp
if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
    update_omnix();  // 走 g2.omni_ctrl
    return;
}
// 原差速逻辑
```
```

- [ ] **Step 3: Add memory index entry**

In `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\MEMORY.md`, add a line near the existing OMNIX DP entry:

```markdown
- [AR_OmniControl library](ar_omnicontrol_library.md) — 3-DOF 位置/航向 PID 共享库，所有 OMNIX 路径模式走 g2.omni_ctrl
```

- [ ] **Step 4: Done — no git commit needed for memory updates (memory is outside repo)**

P1 complete. Source repo is at a clean state with all changes committed. Next iteration: write P2 plan for ModeAuto OMNIX branch using `AR_WPNav` targets + `OMNI_YAW_MODE` consumption.

---

## Self-Review

**Spec coverage:**

- ✅ `AR_OmniControl` library (spec §1) → Tasks 1-3
- ✅ ModeDP薄壳重构 (spec §2) → Task 5
- ✅ 参数归宿: DP_* 名字保留 + `g2.omni_ctrl` 单例 (spec §1 "参数归宿") → Task 4
- ✅ `OMNI_YAW_MODE` + `OMNI_LOS_LOOK` 添加在 g2 (spec §3) → Task 6
- ✅ 失位降级 HOLD (spec §4) → Task 5 step 2 (in `ModeDP::update()`)
- ✅ Phase P1 验证 (spec §6) → Tasks 7-8 (existing `dp_*.py` + `omnix_mix_test.py` regression)
- ❓ Visualization / spec §5 "验证策略 / 出水验证" — out of P1 scope, deferred to P6 plan

**Placeholder scan:** No TBDs. Every code block is concrete. Every command has expected output.

**Type/name consistency:**
- `AR_OmniControl::set_target(Vector2f, float, Vector2f)` consistent in header (Task 1) and caller (Task 5).
- `g2.omni_ctrl` reference consistent across Parameters.h (Task 4), mode_dp.cpp wrapper (Task 5).
- `_omni_yaw_mode` / `_omni_los_look` are P2 placeholders — declared in Task 6, no P1 consumer. P2 plan will need to reference them.
- `ModeReason::EKF_FAILSAFE` used in Task 5 — verify this enum value exists in `Rover/mode.h` enum (commonly present in ArduPilot Rover; if missing, use `ModeReason::FAILSAFE` or `ModeReason::UNKNOWN`).

**Known risk noted:** Task 4 produces an intentionally broken build (mid-refactor) before Task 5 fixes it. Commit message flags this. If using subagent-driven execution, ensure each subagent reads the full task context including this caveat.
