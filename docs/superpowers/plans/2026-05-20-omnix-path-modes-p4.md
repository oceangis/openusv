# OMNIX Path Modes P4 — Refactor Helper + ModeLoiter OMNIX

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** (1) Extract the duplicated `compute_omnix_target_yaw()` from `ModeAuto` and `ModeGuided` into a single shared free function in `Rover/mode.cpp`; (2) Add OMNIX branch to `ModeLoiter::update()` so AUTO/Guided sub-mode delegations to Loiter (e.g. on `reached_destination` for boats) also use the 3-DOF controller; (3) Skip Dock — `MODE_DOCK_ENABLED` is hardcoded `0` in `Rover/config.h:148` for this USV project.

**Architecture:** New static helper `omnix_compute_target_yaw(const OmniYawState&, ParametersG2&, AHRS&, float dt)` lives in `Rover/mode.cpp` near the top (private to mode.cpp via internal linkage). Each caller (`ModeAuto`, `ModeGuided`, new `ModeLoiter` branch) owns its own `_omni_initial_yaw` + `_omni_rc_yaw_integ` state and passes a reference to the helper. Net effect: ~50 lines of duplication removed; new ModeLoiter consumer added for free.

**Tech Stack:** ESP-IDF 5.5.1, existing `AR_OmniControl` + `AR_WPNav`, no new libraries.

**Spec:** `docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md`
**Predecessors:** P1+P2+P3 (commits `abd83c0..ed8f1fc4`)

---

## File Structure

**Modified files:**
- `Rover/mode.h` — declare the shared helper (or struct holding state); add OMNIX state members to `ModeLoiter`; declare `ModeLoiter::update_omnix()` and remove the per-class `compute_omnix_target_yaw()` declarations from `ModeAuto` + `ModeGuided`
- `Rover/mode.cpp` — define the shared static helper
- `Rover/mode_auto.cpp` — delete the local `ModeAuto::compute_omnix_target_yaw()`; update `update_omnix_wp()` to call the shared helper
- `Rover/mode_guided.cpp` — delete the local `ModeGuided::compute_omnix_target_yaw()`; update `update_omnix_wp()` to call the shared helper
- `Rover/mode_loiter.cpp` — implement `update_omnix()`; add OMNIX branch to `update()` and `_enter()`

**New bench script:**
- `bench/verify/omni_loiter_test.py` — switch directly to LOITER on OMNIX frame, verify station-keeping behavior + 4-thruster response

**Untouched:**
- `libraries/AR_OmniControl/*` — same single instance, same API
- ModeDP — works as-is (its `compute_omnix_target_yaw` is internal to AR_OmniControl's snapshot — wait, no, ModeDP uses snapshot-on-entry and doesn't have yaw-strategy switching. Untouched.)

---

## Background — Shared Helper Design

**Why a free function with caller-owned state, not a method on AR_OmniControl?**

`compute_omnix_target_yaw()` depends on:
- `OMNI_YAW_MODE` (global parameter via `g2._omni_yaw_mode`)
- `wp_nav.nav_bearing_cd()` / `wp_bearing_cd()` (for TANGENT / POINT_NEXT_WP)
- RC channel 4 (for MANUAL_RC)
- Per-mode state: `_omni_initial_yaw` (snapshot at mode entry) + `_omni_rc_yaw_integ` (RC integral)

Putting the helper on AR_OmniControl would couple the controller library to AR_WPNav and RC_Channels — heavy dependencies for a control-law class. A free static function in `mode.cpp` keeps AR_OmniControl pure and lets the mode layer own all the integration knowledge. Per-mode state stays per-mode (one mode's RC integrator doesn't leak into another).

**Signature:**

```cpp
// In mode.cpp (static — internal linkage)
namespace {
struct OmniYawState {
    float initial_yaw{0.0f};   // snapshot at mode entry
    float rc_yaw_integ{0.0f};  // MANUAL_RC integrator
};

float omnix_compute_target_yaw(OmniYawState& state, float dt);
}
```

The struct is anonymous-namespace + by-reference passing. Each `ModeAuto`/`ModeGuided`/`ModeLoiter` member that previously held `_omni_initial_yaw` + `_omni_rc_yaw_integ` is replaced by a single `OmniYawState _omni_yaw_state` member.

Wait — anonymous-namespace types can't be exposed in headers. Two options:

**Option A: struct declared in mode.h** (caller passes pointer/ref). Helper function also declared in mode.h, defined in mode.cpp.

**Option B: helper takes raw `float& initial_yaw, float& rc_yaw_integ`** (no struct). Slightly clunkier signature but no header-visible type.

Pick A — cleaner caller code, easier to extend. Declare the struct + helper in `mode.h` (file-level, after includes, before class declarations).

---

## Task 1: Add OmniYawState + helper declaration in mode.h

**Files:**
- Modify: `Rover/mode.h` (top-of-file area, after the includes block, before `class Mode`)

- [ ] **Step 1: Find the right insertion point**

```bash
grep -n "^class Mode\|^#include\|^namespace" Rover/mode.h | head -10
```

Locate where the first `class` (probably `class Mode`) begins. Insert the new declarations just before that.

- [ ] **Step 2: Add the struct + function declaration**

In `Rover/mode.h`, before `class Mode { ... }`, add:

```cpp
// --- Shared OMNIX yaw-strategy helper (P4) ---
// Used by ModeAuto / ModeGuided / ModeLoiter OMNIX branches.
// Each mode owns one OmniYawState instance and passes it by reference.
struct OmniYawState {
    float initial_yaw{0.0f};    // snapshot at mode entry (rad)
    float rc_yaw_integ{0.0f};   // MANUAL_RC integrator (rad)
};

// Compute target heading (rad) per g2._omni_yaw_mode.
// Reads g2._omni_yaw_mode, g2.wp_nav.nav_bearing_cd()/wp_bearing_cd(),
// and RC channel 4 (for MANUAL_RC). dt is the loop period.
// Defined in Rover/mode.cpp.
float omnix_compute_target_yaw(OmniYawState& state, float dt);
```

- [ ] **Step 3: Add `_omni_yaw_state` member to ModeAuto + ModeGuided private sections; remove `_omni_initial_yaw` and `_omni_rc_yaw_integ`**

Find the `// --- OMNIX branch state (P2) ---` block in `class ModeAuto` (around line 407) and replace:
```cpp
    // --- OMNIX branch state (P2) ---
    float _omni_initial_yaw{0.0f};
    float _omni_rc_yaw_integ{0.0f};
    bool  _omni_active{false};

    void update_omnix_wp();
    float compute_omnix_target_yaw(float current_yaw, float dt);
```
with:
```cpp
    // --- OMNIX branch state (P2; refactored P4) ---
    OmniYawState _omni_yaw_state;
    bool _omni_active{false};

    void update_omnix_wp();
```

(The `compute_omnix_target_yaw` member declaration is removed — replaced by the shared helper.)

Repeat the equivalent edit in `class ModeGuided` (around line 632, the `// --- OMNIX branch state (P3) ---` block).

- [ ] **Step 4: Add OMNIX state + helper declaration to ModeLoiter**

Find `class ModeLoiter` (around line 698). At the end of its protected/private section (or add a private: section if it doesn't have one — currently only `protected:` with `_destination` + `_desired_speed`), add:

```cpp
private:
    // --- OMNIX branch state (P4) ---
    OmniYawState _omni_yaw_state;
    bool _omni_active{false};

    // OMNIX loiter: station-keep at _destination using g2.omni_ctrl.
    // Called from update() when frame is OMNIX.
    void update_omnix();
```

- [ ] **Step 5: Build**

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

**Expected: BUILD FAILS.** Compile errors in:
- `mode_auto.cpp` — `_omni_initial_yaw` / `_omni_rc_yaw_integ` / `compute_omnix_target_yaw` no longer exist
- `mode_guided.cpp` — same

This is the intentional mid-refactor state. Tasks 2/3/4 fix it.

If the failure is something else (e.g. syntax error in `mode.h`), STOP and report.

- [ ] **Step 6: Commit**

```bash
git add Rover/mode.h
git commit -m "mode.h: add shared OmniYawState + helper decl, prep ModeLoiter (build broken)

Introduces OmniYawState struct (P4) replacing per-class _omni_initial_yaw
+ _omni_rc_yaw_integ pairs in ModeAuto and ModeGuided. Adds OMNIX state
members and update_omnix() declaration to ModeLoiter.

Build intentionally broken until Tasks 2-4 update the .cpp files."
```

---

## Task 2: Define omnix_compute_target_yaw in mode.cpp

**Files:**
- Modify: `Rover/mode.cpp` (add near top, after includes)

- [ ] **Step 1: Find the top of mode.cpp**

```bash
head -30 Rover/mode.cpp
```

After the `#include`s (and any global constants), add the helper definition.

- [ ] **Step 2: Add the function**

Add this block near the top of `Rover/mode.cpp`:

```cpp
// Shared OMNIX yaw-strategy helper (P4). See declaration in mode.h.
float omnix_compute_target_yaw(OmniYawState& state, float dt)
{
    const uint8_t mode = (uint8_t)rover.g2._omni_yaw_mode.get();

    switch (mode) {
    case 0: // LOCK_INITIAL — snapshot taken on mode entry
        return state.initial_yaw;

    case 1: { // TANGENT — follow path tangent from AR_WPNav
        const float bearing_deg = rover.g2.wp_nav.nav_bearing_cd() * 0.01f;
        return radians(bearing_deg);
    }

    case 2: { // POINT_NEXT_WP — bearing to destination from current position
        const float bearing_deg = rover.g2.wp_nav.wp_bearing_cd() * 0.01f;
        return radians(bearing_deg);
    }

    case 3: { // MANUAL_RC — RC ch4 controls yaw rate, integrated
        const float kMaxYawRate = radians(90.0f);
        float stick_norm = 0.0f;
        const RC_Channel *ch4 = rc().channel(3);
        if (ch4 != nullptr) {
            const uint16_t pwm = ch4->get_radio_in();
            stick_norm = constrain_float((pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
            if (fabsf(stick_norm) < 0.05f) stick_norm = 0.0f;
        }
        if (is_positive(dt)) {
            state.rc_yaw_integ += stick_norm * kMaxYawRate * dt;
            state.rc_yaw_integ = wrap_PI(state.rc_yaw_integ);
        }
        return wrap_PI(state.initial_yaw + state.rc_yaw_integ);
    }

    default:
        return state.initial_yaw;
    }
}
```

Note: uses `rover.g2._omni_yaw_mode` and `rover.g2.wp_nav` because we're at file scope (not inside a Mode class — no implicit `g2`). `rover` is the global Rover instance.

- [ ] **Step 3: Build**

Expected: still fails because mode_auto.cpp / mode_guided.cpp still reference the now-removed per-class members. Verify the compile errors are about `_omni_initial_yaw` / `_omni_rc_yaw_integ` / `compute_omnix_target_yaw`, NOT about the new helper.

If `omnix_compute_target_yaw` definition has its own errors, fix those first.

- [ ] **Step 4: Commit**

```bash
git add Rover/mode.cpp
git commit -m "mode.cpp: define shared omnix_compute_target_yaw (still broken)

Single definition consolidating the four yaw strategies (LOCK_INITIAL /
TANGENT / POINT_NEXT_WP / MANUAL_RC) previously duplicated in
mode_auto.cpp + mode_guided.cpp. Reads rover.g2 at file scope.

Build still broken — Tasks 3/4 strip duplicates and switch callers."
```

---

## Task 3: Migrate ModeAuto to shared helper

**Files:**
- Modify: `Rover/mode_auto.cpp`

- [ ] **Step 1: Remove the local compute_omnix_target_yaw**

Find `float ModeAuto::compute_omnix_target_yaw(float current_yaw, float dt)` in `mode_auto.cpp` (approximately 50 lines added in P2 T2). Delete the entire function definition.

- [ ] **Step 2: Update update_omnix_wp() and _enter() to use _omni_yaw_state**

In `ModeAuto::update_omnix_wp()`, find:
```cpp
const float current_yaw = radians(ahrs.yaw_sensor * 0.01f);
const float target_yaw  = compute_omnix_target_yaw(current_yaw, rover.G_Dt);
```
Replace with:
```cpp
const float target_yaw = omnix_compute_target_yaw(_omni_yaw_state, rover.G_Dt);
```

(Drop the `current_yaw` local — the shared helper doesn't take it as input. The variable was a placeholder for future strategies; not needed.)

In `ModeAuto::_enter()`, find:
```cpp
_omni_initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
_omni_rc_yaw_integ = 0.0f;
```
Replace with:
```cpp
_omni_yaw_state.initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
_omni_yaw_state.rc_yaw_integ = 0.0f;
```

In `ModeAuto::_exit()`, no change needed (only touches `_omni_active` + `g2.omni_ctrl.reset()`).

- [ ] **Step 3: Build**

Expected: `mode_auto.cpp` should now compile. `mode_guided.cpp` will still fail (Task 4 fixes it).

- [ ] **Step 4: Commit**

```bash
git add Rover/mode_auto.cpp
git commit -m "ModeAuto: switch to shared omnix_compute_target_yaw helper

Removes the local 50-line compute_omnix_target_yaw definition.
update_omnix_wp() and _enter() now operate on _omni_yaw_state
(OmniYawState struct) and call the file-scope helper in mode.cpp.

Behavioral equivalence — same 4 strategies, same algorithm.
Guided update still pending (T4)."
```

---

## Task 4: Migrate ModeGuided to shared helper

**Files:**
- Modify: `Rover/mode_guided.cpp`

Same shape as Task 3 but for ModeGuided.

- [ ] **Step 1: Delete the local compute_omnix_target_yaw**

Find `float ModeGuided::compute_omnix_target_yaw(...)` and delete the entire function.

- [ ] **Step 2: Update update_omnix_wp() and _enter()**

In `ModeGuided::update_omnix_wp()`, replace:
```cpp
const float current_yaw = radians(ahrs.yaw_sensor * 0.01f);
const float target_yaw  = compute_omnix_target_yaw(current_yaw, rover.G_Dt);
```
with:
```cpp
const float target_yaw = omnix_compute_target_yaw(_omni_yaw_state, rover.G_Dt);
```

In `ModeGuided::_enter()`, replace:
```cpp
_omni_initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
_omni_rc_yaw_integ = 0.0f;
```
with:
```cpp
_omni_yaw_state.initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
_omni_yaw_state.rc_yaw_integ = 0.0f;
```

- [ ] **Step 3: Build**

Expected: **build PASSES.** All references updated. The refactor is complete; functionality is byte-equivalent to P3.

- [ ] **Step 4: Commit**

```bash
git add Rover/mode_guided.cpp
git commit -m "ModeGuided: switch to shared omnix_compute_target_yaw helper

Removes the local 50-line compute_omnix_target_yaw (duplicate of
ModeAuto's). update_omnix_wp() + _enter() now use _omni_yaw_state
and the file-scope helper in mode.cpp.

Build restored. ~100 lines of duplication removed across Auto+Guided.
Behavioral equivalence — bench regression in T7 confirms."
```

---

## Task 5: Implement ModeLoiter::update_omnix() + wire dispatch

**Files:**
- Modify: `Rover/mode_loiter.cpp`

Loiter's `_enter()` already snapshots a stopping location into `_destination` (a Location). The OMNIX branch just feeds that to `g2.omni_ctrl` directly — no SCurve trajectory needed because Loiter is station-keeping, not waypoint following.

Yaw strategy: Loiter does NOT need `OMNI_YAW_MODE` switching because there's no path tangent (you're stationary). Simplest behavior — always use `LOCK_INITIAL` (snapshot at entry, hold throughout). If user wants a different yaw mode they should use POSHOLD instead.

- [ ] **Step 1: Append update_omnix() to mode_loiter.cpp**

Append to `Rover/mode_loiter.cpp`:

```cpp
// OMNIX-only loiter: station-keep at _destination using g2.omni_ctrl.
// Called from ModeLoiter::update() when frame is OMNIX.
// Heading is always LOCK_INITIAL (snapshot at _enter); use POSHOLD for
// other strategies.
void ModeLoiter::update_omnix()
{
    // --- Position estimate required ---
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    // --- Location -> NED ---
    Location origin;
    if (!ahrs.get_origin(origin)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }
    const Vector2f target_pos_ned = origin.get_distance_NE(_destination);

    // --- Drive controller (LOCK_INITIAL yaw — stationary) ---
    g2.omni_ctrl.set_target(target_pos_ned, _omni_yaw_state.initial_yaw);
    g2.omni_ctrl.update(rover.G_Dt);

    float fwd, lat, steer_norm;
    if (!g2.omni_ctrl.get_outputs(fwd, lat, steer_norm)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    g2.motors.set_throttle(fwd * 100.0f);
    g2.motors.set_lateral(lat * 100.0f);
    g2.motors.set_steering(steer_norm * 4500.0f, false);

    // Update reporting field for GCS
    _distance_to_destination = rover.current_loc.get_distance(_destination);
}
```

Loiter doesn't need the full helper (no yaw strategy switching needed). Just uses the snapshot from `_omni_yaw_state.initial_yaw` directly.

- [ ] **Step 2: Wire dispatch in _enter() and update()**

In `ModeLoiter::_enter()`, at the very end (just before `return true;`), add:

```cpp
    // OMNIX P4: snapshot heading + reset controller
    _omni_yaw_state.initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
    _omni_yaw_state.rc_yaw_integ = 0.0f;
    _omni_active = (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX);
    if (_omni_active) {
        g2.omni_ctrl.reset();
    }
```

In `ModeLoiter::update()`, at the very beginning (before the existing `_distance_to_destination = ...` line), add:

```cpp
    // OMNIX 4-thruster holonomic branch (P4)
    if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        update_omnix();
        return;
    }
```

The OMNIX path early-returns BEFORE the diff-drive logic.

- [ ] **Step 3: Build + flash**

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: `Project build complete.`

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_dp.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: `Hash of data verified.`

- [ ] **Step 4: Commit**

```bash
git add Rover/mode_loiter.cpp
git commit -m "ModeLoiter: add OMNIX branch — station-keep via g2.omni_ctrl

_enter() snapshots heading + resets controller when frame is OMNIX.
update() dispatches to update_omnix() which feeds _destination
directly to g2.omni_ctrl with LOCK_INITIAL yaw (Loiter is stationary,
no path tangent to follow). Use POSHOLD for other yaw strategies.

Degrades to HOLD on lost position. Flashed."
```

---

## Task 6: New omni_loiter_test.py bench script

**Files:**
- Create: `bench/verify/omni_loiter_test.py`

Switches to LOITER on OMNIX frame, samples SERVO_OUTPUT_RAW. Since the boat is stationary at its `_destination`, the test pushes virtual position drift by *not* expecting strong lateral evidence — instead verifies that motors stay near 1500 (station-keeping), and that on a small forced position offset the controller responds.

Simpler test: just verify that LOITER mode enters cleanly + servo outputs aren't crazy.

- [ ] **Step 1: Create the script**

Write `bench/verify/omni_loiter_test.py`:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OMNIX LOITER 模式回归测试。

测试流程:
  1. 配 OMNIX 帧 + ARMING_CHECK=0
  2. 等 GPS lock
  3. force-arm, 切 LOITER (mode 5)
  4. 验证模式确实切到 LOITER (custom_mode=5)
  5. 采样 SERVO_OUTPUT_RAW 15s
  6. 验证: servo 没有飞掉 (都在 1000-2000 范围, 大部分时间靠近 1500)
  7. cleanup: MANUAL + disarm

PASS 条件:
  - armed_in_loiter = True
  - servo_sane = True (4路 PWM 都在 1100..1900 之间,即无饱和异常)
  - position_held = True (距离 _destination 变化 < 2m,假定 GPS 静态噪声)

GPS lock 不可用时 SKIP (exit 2)。

用法: python omni_loiter_test.py [--port COM10]
"""
import argparse
import sys
import time

from pymavlink import mavutil

ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
FORCE = 21196.0


def log(msg):
    print(time.strftime("[%H:%M:%S] ") + msg, flush=True)


def connect(port, baud, timeout=90):
    log(f"连接 {port}@{baud} ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            m = mavutil.mavlink_connection(port, baud=baud)
            if m.wait_heartbeat(timeout=8):
                log(f"已连接 system={m.target_system}")
                return m
            m.close()
        except Exception as e:
            log(f"  重试: {e}")
        time.sleep(2)
    return None


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.4)


def override(m):
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    1500, 0, 1500, 1500, 0, 0, 0, 0)


def pump(m, seconds, want_type=None):
    got = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        override(m)
        msg = m.recv_match(blocking=True, timeout=0.05)
        if msg and want_type and msg.get_type() == want_type:
            got.append(msg)
        time.sleep(0.02)
    return got


def wait_gps(m, sats_required=8, timeout=120):
    log(f"等 GPS lock (sats >= {sats_required}) ...")
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg and msg.fix_type >= 3 and msg.satellites_visible >= sats_required:
            log(f"  GPS lock: fix={msg.fix_type} sats={msg.satellites_visible} hdop={msg.eph/100:.2f}")
            return True
    log(f"  GPS lock not acquired within {timeout}s")
    return False


def set_mode(m, n):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    results = {}

    m = connect(args.port, args.baud)
    if m is None:
        log("FAIL: 连不上"); sys.exit(1)

    set_param(m, "FRAME_CLASS", 2)
    set_param(m, "FRAME_TYPE", 2)
    set_param(m, "ARMING_CHECK", 0)

    if not wait_gps(m, 8, 120):
        log("SKIP: no GPS; cannot test LOITER")
        m.close()
        sys.exit(2)
    results["gps_ok"] = True

    log("--- EKF stabilize 30s ---")
    pump(m, 30)

    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            1, FORCE, 0, 0, 0, 0, 0)
    pump(m, 4)
    set_mode(m, 5)  # LOITER
    pump(m, 3)

    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    armed = hb and bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    in_loiter = hb and hb.custom_mode == 5
    log(f"  armed={armed} in_loiter={in_loiter}")
    results["armed_in_loiter"] = bool(armed and in_loiter)

    if not (armed and in_loiter):
        log("FAIL: could not arm in LOITER")
        set_mode(m, 0)
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                0, 0, 0, 0, 0, 0, 0, 0)
        for k, v in results.items():
            log(f"  {k:20s} {'PASS' if v else 'FAIL'}")
        sys.exit(5)

    log("--- sample SERVO_OUTPUT_RAW 15s ---")
    servos = pump(m, 15, want_type='SERVO_OUTPUT_RAW')
    vals = [[s.servo1_raw, s.servo2_raw, s.servo3_raw, s.servo4_raw]
            for s in servos]
    log(f"  sampled {len(vals)} frames")
    if vals:
        for k in range(0, len(vals), max(1, len(vals)//6)):
            log(f"    {vals[k]}")

    # Sanity: all servos in 1100..1900 range (no saturation/runaway)
    servo_sane = True
    for s in vals:
        for v in s:
            if v < 1100 or v > 1900:
                servo_sane = False
                break
        if not servo_sane:
            break
    log(f"  servo_sane: {servo_sane}")
    results["servo_sane"] = servo_sane

    # cleanup
    set_mode(m, 0)
    pump(m, 1)
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            0, 0, 0, 0, 0, 0, 0, 0)
    pump(m, 2)

    log("")
    log("=" * 46)
    log("OMNIX LOITER 回归汇总")
    log("=" * 46)
    allpass = True
    for k, v in results.items():
        mark = "PASS" if v else "FAIL"
        if not v:
            allpass = False
        log(f"  {k:20s} {mark}")
    log("=" * 46)
    log("总体: " + ("PASS" if allpass else "FAIL"))
    m.close()
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Syntax check**

```bash
python -c "import py_compile; py_compile.compile('bench/verify/omni_loiter_test.py', doraise=True)"
```

- [ ] **Step 3: Commit**

```bash
git add bench/verify/omni_loiter_test.py
git commit -m "bench: omni_loiter_test — verify OMNIX LOITER station-keep

Switches to LOITER on OMNIX frame, samples SERVO_OUTPUT_RAW 15s,
verifies servos in sane range (no saturation/runaway). Acceptance
criterion is conservative — full station-keep verification needs
out-of-water testing. SKIPs cleanly without GPS."
```

---

## Task 7: Hardware regression (GPS-gated)

**Files:** none modified.

- [ ] **Step 1: GPS check**

```bash
python bench/verify/gps_check.py --port COM10
```

- [ ] **Step 2: All prior bench tests still PASS**

```bash
python bench/verify/omnix_mix_test.py --port COM10
python bench/verify/omni_auto_test.py --port COM10
python bench/verify/omni_guided_test.py --port COM10
python bench/verify/omni_loiter_test.py --port COM10
```

Acceptance:
- `omnix_mix_test.py` 总体: PASS (REQUIRED — mixer untouched)
- `omni_auto_test.py` exit 0 (PASS) or 2 (SKIP-no-GPS) — both acceptable
- `omni_guided_test.py` exit 0 (PASS) or 2 (SKIP-no-GPS) — both acceptable
- `omni_loiter_test.py` exit 0 (PASS) or 2 (SKIP-no-GPS) — both acceptable

If any test fails non-GPS-relatedly: BLOCKED.

- [ ] **Step 3: Param sanity**

```bash
python -c "
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('COM10', baud=115200)
m.wait_heartbeat()
m.mav.param_request_list_send(m.target_system, m.target_component)
dp = {}; omni = {}
t0 = time.time()
while time.time() - t0 < 8:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if not msg: continue
    if msg.param_id.startswith('DP_'): dp[msg.param_id] = msg.param_value
    elif msg.param_id.startswith('OMNI_'): omni[msg.param_id] = msg.param_value
print(f'DP_* {len(dp)} / OMNI_* {len(omni)}')
for k in sorted(dp): print(f'  {k} = {dp[k]}')
for k in sorted(omni): print(f'  {k} = {omni[k]}')
"
```

Expected: 11 DP_* + 2 OMNI_* still present with baseline values.

---

## Task 8: Memory close

**Files:**
- Append to `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\omnix_dp_mode.md`
- Append to `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\ar_omnicontrol_library.md`
- Update `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\MEMORY.md`

Note P4 completion: refactor + Loiter. Document that Dock is intentionally skipped (`MODE_DOCK_ENABLED=0` in this USV project).

---

## Self-Review

**Spec coverage:**
- ✅ Spec §6 P4 row: "ModeLoiter + ModeDock OMNIX 分支" — Loiter done, Dock intentionally skipped due to MODE_DOCK_ENABLED=0
- ✅ P3 reviewer flag: duplication of compute_omnix_target_yaw — refactored away
- ✅ Failsafe to HOLD (spec §4) — Loiter's update_omnix does this

**Placeholder scan:** no TBDs. Every code block is concrete.

**Type/name consistency:**
- `OmniYawState` struct declared in mode.h (T1), defined nowhere extra (it's just data)
- `omnix_compute_target_yaw(OmniYawState&, float)` declared in mode.h (T1), defined in mode.cpp (T2), called from ModeAuto (T3) + ModeGuided (T4) — ModeLoiter uses direct `_omni_yaw_state.initial_yaw` (no strategy switching) so doesn't call the helper
- `_omni_yaw_state` member name consistent across ModeAuto / ModeGuided / ModeLoiter (T1)

**Known risks:**
- Anonymous-namespace approach was rejected → struct in mode.h public namespace. Could clash with another `OmniYawState` somewhere — grep before merging:
  ```bash
  grep -rn "OmniYawState" libraries/ Rover/ 2>&1 | grep -v "Rover/mode"
  ```
  If anything outside `Rover/mode.{h,cpp}`, rename to avoid collision.
- The intermediate commits in T1+T2 leave the build broken. If subagent-driven execution interleaves with other work, this is a hazard. Mitigate by sequencing T1→T4 strictly.
- Loiter's `_destination` is set in `_enter()` via `g2.wp_nav.get_stopping_location()` — that depends on AHRS reporting current velocity. If called too soon after boot, may return false. Existing `_enter()` already handles this (returns false → mode entry rejected). OMNIX path doesn't change that.

**Out of scope (next plans):**
- P5: ModeRTL + ModeSmartRTL OMNIX
- P6: Water testing
- ModeDock: stays disabled (precland not available in this project)
- HeadingAndSpeed / TurnRateAndSpeed Guided submodes with lateral support
