# OMNIX Path Modes P2 — ModeAuto OMNIX Branch + OMNI_YAW_MODE

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `ModeAuto` use OMNIX 3-DOF control when `FRAME_TYPE == OMNIX`. Boat follows waypoint missions using `g2.omni_ctrl` (the shared controller from P1) so it can crab/translate without rotating, with selectable yaw strategies via `OMNI_YAW_MODE`. Differential-drive AUTO path is untouched.

**Architecture:** Each `ModeAuto::update()` SubMode::WP tick dispatches to a new private `ModeAuto::update_omnix_wp()` when the frame is OMNIX. The new path: (a) extract `target_pos_ned` from `g2.wp_nav.get_destination()` after converting Location→NED via `AHRS::get_origin()` + `Location::get_distance_NE_float`; (b) compute `target_yaw` per `OMNI_YAW_MODE`; (c) feed `g2.omni_ctrl.set_target() / update() / get_outputs()`; (d) on lost position degrade to HOLD. `g2.wp_nav.update()` still runs to manage waypoint progression (reached_destination, mission advancement). All four other SubModes (HeadingAndSpeed/RTL/Loiter/Guided/etc.) remain on the differential path — only WP gets the OMNIX branch in P2.

**Tech Stack:** ESP-IDF 5.5.1, existing `AR_OmniControl` + `AR_WPNav`, no new libraries.

**Spec:** `docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md`
**Predecessor plan:** `docs/superpowers/plans/2026-05-20-omnix-path-modes-p1.md` (all P1 commits landed up to `ed44edb321`)

---

## File Structure

**Modified files:**
- `Rover/mode.h` — add `update_omnix_wp()` private method to `ModeAuto`; add private state members for OMNIX target tracking
- `Rover/mode_auto.cpp` — implement `update_omnix_wp()` + yaw-strategy helper; dispatch from `update()` SubMode::WP

**Unchanged but referenced:**
- `libraries/AR_OmniControl/AR_OmniControl.{h,cpp}` — used as-is (no changes)
- `libraries/AR_WPNav/AR_WPNav.h` — read-only via existing public getters
- `Rover/Parameters.h/cpp` — `OMNI_YAW_MODE` + `OMNI_LOS_LOOK` already declared in P1 (T6)

**Regression scripts (consumed in T8):**
- `bench/verify/omnix_mix_test.py` (P1 — should still PASS, OMNIX MANUAL unchanged)
- `bench/verify/dp_final_test.py` (P1 — should still PASS once GPS available)
- New: `bench/verify/omni_auto_test.py`

---

## Background — What target does AR_WPNav expose?

`AR_WPNav` public API gives us:

```cpp
const Location &get_destination() const;          // current target Location (lat/lon)
bool   is_destination_valid() const;              // true once a mission set destination
bool   reached_destination() const;               // for mission advancement
float  get_distance_to_destination() const;       // meters
float  wp_bearing_cd() const;                     // bearing to dest, cdeg
float  nav_bearing_cd() const;                    // "desired heading" along track, cdeg
float  crosstrack_error() const;                  // cross-track error, m
float  get_speed() const;                         // desired speed m/s (diff-drive use)
float  get_turn_rate_rads() const;                // desired turn rate (diff-drive use)
```

For OMNIX we use: `get_destination()` (Location → NED), `is_destination_valid()`, `nav_bearing_cd()` (for TANGENT yaw), `wp_bearing_cd()` (for POINT_NEXT_WP yaw). `reached_destination()` is consulted by the existing diff-drive code path for mission advancement; OMNIX uses `g2.wp_nav.update()` to drive the same internal state, so mission stepping continues to work.

**Target NED conversion:** `Location` is lat/lon; `AR_OmniControl::set_target` wants `Vector2f` NED meters from EKF origin. The standard pattern:

```cpp
Location origin;
if (!ahrs.get_origin(origin)) { /* no origin yet */ return failsafe; }
Location dest = g2.wp_nav.get_destination();
Vector2f offset_NE_m = origin.get_distance_NE_float(dest);   // m, NED (N,E)
```

`get_distance_NE_float()` returns `(north_m, east_m)` from `origin` to `dest`. This matches AR_OmniControl's NED convention (x=north, y=east).

---

## Background — Yaw strategies

`OMNI_YAW_MODE` (g2 AP_Int8, default 1):

| Value | Name | Behavior |
|---|---|---|
| 0 | `LOCK_INITIAL` | Snapshot heading at `_enter()`, hold throughout mission (crab/translate without rotating) |
| 1 | `TANGENT` | Target heading = `g2.wp_nav.nav_bearing_cd() * 0.01f * DEG_TO_RAD`. Looks like a traditional boat. |
| 2 | `POINT_NEXT_WP` | Target heading = bearing from current pos to `wp_nav.get_destination()` (i.e. `wp_bearing_cd()`). Same as TANGENT in straight legs; diverges at corners. Useful for sensors that want to look at the target. |
| 3 | `MANUAL_RC` | Read RC channel 4 (yaw stick): map RC PWM 1000-2000 → ±target heading rate; integrate to derive `target_yaw`. Mission moves autonomously, operator steers heading freely. |

Each strategy is a single helper function `compute_target_yaw(SubMode, current_yaw, dt)` returning a rad value.

---

## Task 1: Add OMNIX state members + helper declaration to ModeAuto

**Files:**
- Modify: `Rover/mode.h` (the `class ModeAuto` body, around lines 246-410)

- [ ] **Step 1: Read the existing ModeAuto class**

```bash
grep -n "class ModeAuto" Rover/mode.h
```
Then read ~150 lines from that point. Identify the existing `private:` section.

- [ ] **Step 2: Add private members + helper declaration**

In `Rover/mode.h`, in the `private:` section of `class ModeAuto`, near the bottom (before the closing brace), add:

```cpp
    // --- OMNIX branch state (P2) ---
    // initial yaw captured at mode entry, used by OMNI_YAW_MODE=0 (LOCK_INITIAL)
    float _omni_initial_yaw{0.0f};
    // integrator for OMNI_YAW_MODE=3 (MANUAL_RC) — accumulates target yaw from RC stick rate
    float _omni_rc_yaw_integ{0.0f};
    // last omnix tick (for derivative / RC integration)
    bool  _omni_active{false};

    // OMNIX-only WP control. Replaces navigate_to_waypoint() when FRAME_TYPE==OMNIX.
    // Reads target from g2.wp_nav (destination + nav_bearing), yaw per OMNI_YAW_MODE,
    // dispatches to g2.omni_ctrl. Triggers HOLD failsafe on lost position.
    void update_omnix_wp();

    // Computes target heading (rad) per OMNI_YAW_MODE for the current tick.
    // dt is the loop period; current_yaw is from AHRS in radians.
    float compute_omnix_target_yaw(float current_yaw, float dt);
```

The state members get in-class initializers so they're safe on construction.

- [ ] **Step 3: Build to confirm the header parses**

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: build FAILS with linker errors about `ModeAuto::update_omnix_wp` and `ModeAuto::compute_omnix_target_yaw` being unresolved (declared but not defined). That's the next task's job. If the build instead fails with a compile error in `mode.h`, that's a syntax issue — STOP and report.

- [ ] **Step 4: Commit (intentionally broken — declared, not defined)**

```bash
git add Rover/mode.h
git commit -m "ModeAuto: declare update_omnix_wp + compute_omnix_target_yaw (linker broken)

Adds private state for OMNIX yaw tracking (LOCK_INITIAL snapshot,
MANUAL_RC integrator, active flag) and two helper method
declarations. Linker error expected until Task 2 defines the
methods -- intentional mid-refactor checkpoint."
```

---

## Task 2: Implement compute_omnix_target_yaw() — all 4 strategies

**Files:**
- Modify: `Rover/mode_auto.cpp` (append the new method near the bottom of the file, before any closing namespace if present)

- [ ] **Step 1: Add the implementation**

Append to `Rover/mode_auto.cpp`:

```cpp
// Compute target heading for the OMNIX AUTO branch.
// Strategy chosen by g2._omni_yaw_mode parameter.
// All return values are radians.
float ModeAuto::compute_omnix_target_yaw(float current_yaw, float dt)
{
    const uint8_t mode = (uint8_t)g2._omni_yaw_mode.get();

    switch (mode) {
    case 0: // LOCK_INITIAL — snapshot taken on entry
        return _omni_initial_yaw;

    case 1: { // TANGENT — follow path tangent from AR_WPNav
        // nav_bearing_cd is the SCurve-shaped desired heading along the track
        const float bearing_deg = g2.wp_nav.nav_bearing_cd() * 0.01f;
        return radians(bearing_deg);
    }

    case 2: { // POINT_NEXT_WP — bearing to destination from current position
        const float bearing_deg = g2.wp_nav.wp_bearing_cd() * 0.01f;
        return radians(bearing_deg);
    }

    case 3: { // MANUAL_RC — RC ch4 controls yaw rate, we integrate
        // RC channel 4 is the yaw stick. Read PWM, map 1000..2000 → -1..+1.
        // Scale to a yaw rate (rad/s) and integrate over dt.
        // Default max rate: 90 deg/s = 1.57 rad/s at full stick.
        const float kMaxYawRate = radians(90.0f);
        float stick_norm = 0.0f;
        const RC_Channel *ch4 = rc().channel(3);   // 0-indexed -> channel 4
        if (ch4 != nullptr) {
            const uint16_t pwm = ch4->get_radio_in();
            // map 1000..2000 → -1..+1 (1500 is centered)
            stick_norm = constrain_float((pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
            // small deadband
            if (fabsf(stick_norm) < 0.05f) stick_norm = 0.0f;
        }
        if (is_positive(dt)) {
            _omni_rc_yaw_integ += stick_norm * kMaxYawRate * dt;
            _omni_rc_yaw_integ = wrap_PI(_omni_rc_yaw_integ);
        }
        // Initial yaw + accumulated RC delta
        return wrap_PI(_omni_initial_yaw + _omni_rc_yaw_integ);
    }

    default:
        // Unknown mode: fall back to LOCK_INITIAL behavior
        return _omni_initial_yaw;
    }
    (void)current_yaw;  // currently unused; kept for future strategies (e.g. rate-limited yaw)
}
```

- [ ] **Step 2: Build to verify**

Same build command as Task 1. Expected: **still fails** because `update_omnix_wp()` isn't defined yet. The `compute_omnix_target_yaw` symbol will now be defined though, so look at the linker error — it should only complain about `update_omnix_wp`.

- [ ] **Step 3: Commit**

```bash
git add Rover/mode_auto.cpp
git commit -m "ModeAuto: implement compute_omnix_target_yaw for all 4 OMNI_YAW_MODE values

LOCK_INITIAL (0): returns _omni_initial_yaw snapshot from _enter().
TANGENT (1): radians(wp_nav.nav_bearing_cd() * 0.01).
POINT_NEXT_WP (2): radians(wp_nav.wp_bearing_cd() * 0.01).
MANUAL_RC (3): integrates RC ch4 stick rate (90 deg/s @ full) into
_omni_rc_yaw_integ, applied as delta on _omni_initial_yaw.

Build still broken until update_omnix_wp() lands in Task 3."
```

---

## Task 3: Implement update_omnix_wp() — the OMNIX WP control loop

**Files:**
- Modify: `Rover/mode_auto.cpp` (add the new method after `compute_omnix_target_yaw`)

- [ ] **Step 1: Add the implementation**

Append to `Rover/mode_auto.cpp`:

```cpp
// OMNIX-only WP control. Called from ModeAuto::update() SubMode::WP branch
// when g2.motors.get_frame_type() == FRAME_TYPE_OMNIX.
//
// Responsibilities:
//   - keep g2.wp_nav.update() ticking so it advances waypoints / reached flags
//   - extract destination Location → NED target offset
//   - compute target yaw per OMNI_YAW_MODE
//   - feed g2.omni_ctrl + push outputs to motors (forward + lateral + steering)
//   - degrade to HOLD on lost position or invalid destination
void ModeAuto::update_omnix_wp()
{
    // Tick AR_WPNav so it maintains _destination / nav_bearing / reached state.
    // Its forward/steering outputs are ignored — we're in OMNIX path.
    g2.wp_nav.update(rover.G_Dt);

    // Boats stop on reached + non-fast WP (matches diff-drive boat behavior)
    if (g2.wp_nav.reached_destination() && !g2.wp_nav.is_fast_waypoint()) {
        if (!start_loiter()) {
            start_stop();
        }
        return;
    }

    // --- Position estimate required ---
    Vector3f pos_ned;
    if (!ahrs.get_relative_position_NED_origin_float(pos_ned)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        rover.set_mode(rover.mode_hold, ModeReason::EKF_FAILSAFE);
        return;
    }

    // --- Destination validity ---
    if (!g2.wp_nav.is_destination_valid()) {
        // No mission target yet — just hold position
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    // --- Convert destination Location to NED offset from EKF origin ---
    Location origin;
    if (!ahrs.get_origin(origin)) {
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }
    const Location& dest = g2.wp_nav.get_destination();
    const Vector2f target_pos_ned = origin.get_distance_NE_float(dest);

    // --- Compute target yaw per OMNI_YAW_MODE ---
    const float current_yaw = radians(ahrs.yaw_sensor * 0.01f);
    const float target_yaw  = compute_omnix_target_yaw(current_yaw, rover.G_Dt);

    // --- Drive g2.omni_ctrl ---
    g2.omni_ctrl.set_target(target_pos_ned, target_yaw);
    g2.omni_ctrl.update(rover.G_Dt);

    float fwd, lat, steer_norm;
    if (!g2.omni_ctrl.get_outputs(fwd, lat, steer_norm)) {
        // Lost position mid-update — same degrade
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

- [ ] **Step 2: Build** — same command. Should now succeed because both helpers are defined.

If it fails with `rc()`, `RC_Channel`, `radians`, etc. not found — add appropriate includes at the top of `mode_auto.cpp` (likely already pulled in via `Rover.h`).

If it fails with `g2.omni_ctrl` not found — the `AR_OmniControl` header isn't visible. `Rover.h` should transitively pull `Parameters.h` which includes it. Verify with `grep -rn "include.*AR_OmniControl" Rover/`.

- [ ] **Step 3: Commit**

```bash
git add Rover/mode_auto.cpp
git commit -m "ModeAuto: implement update_omnix_wp() — OMNIX WP control loop

Drives g2.omni_ctrl from wp_nav destination + OMNI_YAW_MODE target.
g2.wp_nav.update() still ticks so mission progression works.
Degrades to HOLD on lost position estimate.

Build now passes again. Wired in next task."
```

---

## Task 4: Wire the dispatch in ModeAuto::update() + initialize state in _enter()

**Files:**
- Modify: `Rover/mode_auto.cpp:5-35` (the `_enter()` body — capture initial yaw)
- Modify: `Rover/mode_auto.cpp:82-96` (the `SubMode::WP` case in `update()`)

- [ ] **Step 1: Capture initial yaw in _enter()**

Add to `ModeAuto::_enter()`, near the bottom before `return true` (around current line 33):

```cpp
    // Snapshot heading for OMNIX yaw strategies LOCK_INITIAL / MANUAL_RC base
    _omni_initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
    _omni_rc_yaw_integ = 0.0f;
    _omni_active = (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX);
```

If frame is OMNIX, reset the controller too:
```cpp
    if (_omni_active) {
        g2.omni_ctrl.reset();
    }
```

- [ ] **Step 2: Dispatch in update() SubMode::WP**

Find this block in `Rover/mode_auto.cpp` (around line 82-96):
```cpp
        case SubMode::WP:
        {
            // boats loiter once the waypoint is reached
            bool keep_navigating = true;
            if (rover.is_boat() && g2.wp_nav.reached_destination() && !g2.wp_nav.is_fast_waypoint()) {
                keep_navigating = !start_loiter();
            }

            // update navigation controller
            if (keep_navigating) {
                navigate_to_waypoint();
            }
            break;
        }
```

Replace with:
```cpp
        case SubMode::WP:
        {
            // OMNIX 4-thruster holonomic: use g2.omni_ctrl branch
            if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
                update_omnix_wp();
                break;
            }

            // boats loiter once the waypoint is reached
            bool keep_navigating = true;
            if (rover.is_boat() && g2.wp_nav.reached_destination() && !g2.wp_nav.is_fast_waypoint()) {
                keep_navigating = !start_loiter();
            }

            // update navigation controller (differential-drive path)
            if (keep_navigating) {
                navigate_to_waypoint();
            }
            break;
        }
```

- [ ] **Step 3: Add _exit() cleanup**

Update `ModeAuto::_exit()` to reset the controller when leaving OMNIX AUTO:

```cpp
void ModeAuto::_exit()
{
    // stop running the mission
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        mission.stop();
    }

    // reset OMNIX controller state when leaving OMNIX AUTO
    if (_omni_active) {
        g2.omni_ctrl.reset();
        _omni_active = false;
    }
}
```

- [ ] **Step 4: Build + flash**

Build:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected `Project build complete.` with no errors. Binary size grows by a small amount (~1-3 KB) because we now actually use `g2.omni_ctrl` from a second caller.

Flash:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_dp.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: `Hash of data verified.` + `Leaving... Hard resetting via RTS pin...`

- [ ] **Step 5: Commit**

```bash
git add Rover/mode_auto.cpp
git commit -m "ModeAuto: wire OMNIX branch into SubMode::WP + _enter/_exit

_enter snapshots initial yaw (for LOCK_INITIAL/MANUAL_RC base) and
resets g2.omni_ctrl when frame is OMNIX.

update() SubMode::WP dispatches to update_omnix_wp() when frame is
OMNIX; differential-drive path unchanged otherwise.

_exit resets g2.omni_ctrl when leaving OMNIX AUTO so other modes
don't inherit stale integrator state."
```

---

## Task 5: Write omni_auto_test.py bench script

**Files:**
- Create: `bench/verify/omni_auto_test.py`

Standalone Python regression test that uploads a small mission, switches to AUTO, verifies the OMNIX branch is actually driving lateral output (not the diff-drive degenerate case).

- [ ] **Step 1: Create the script**

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OMNIX AUTO 模式回归测试 — 验证 update_omnix_wp() 真的在用横移自由度。

测试流程:
  1. 配 OMNIX 帧 (FRAME_CLASS=2/FRAME_TYPE=2)
  2. 设 ARMING_CHECK=0, OMNI_YAW_MODE=1 (TANGENT)
  3. 等 GPS lock + EKF 稳定
  4. 上传一个简单 mission: TAKEOFF + 2 个相对 NED 偏移的 waypoint
  5. force-arm, 切 AUTO
  6. 采样 SERVO_OUTPUT_RAW 20s 期间, 持续发 RC override (override 失效)
  7. 验证: servo1/3 与 servo2/4 出现反向偏移 (lateral 被使用了)
  8. 切 MANUAL, disarm, restore params

通过条件: lateral_evidence = True (4 路 servo 出现 OMNIX 混控特征,不是
单纯前进 4 路同向)

用法: python omni_auto_test.py [--port COM10]
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


def get_current_loc(m, timeout=5):
    msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if msg is None:
        return None
    return (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)


def upload_mission(m, current_lat, current_lon):
    """Upload a 3-item mission: home(idx 0) + 2 waypoints 10m N, 10m E."""
    # 1 deg lat = ~111320 m. 10m offset = 10/111320 deg lat
    dlat = 10.0 / 111320.0
    dlon = 10.0 / (111320.0 * abs(0.5))  # approximate, OK for jakarta

    items = [
        # home
        (0, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current_lat, current_lon, 0.0),
        # 10m north of current
        (1, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current_lat + dlat, current_lon, 0.0),
        # 10m east of current
        (2, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current_lat, current_lon + dlon, 0.0),
    ]

    m.mav.mission_count_send(m.target_system, m.target_component, len(items),
                             mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    for seq, cmd, lat, lon, alt in items:
        # wait MISSION_REQUEST
        req = m.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                           blocking=True, timeout=5)
        if not req or req.seq != seq:
            log(f"  upload bad request: {req}")
            return False
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            cmd, 0, 1, 0, 0, 0, 0,
            int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
    ack = m.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        log("  mission uploaded OK")
        return True
    log(f"  mission_ack bad: {ack}")
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

    # configure
    set_param(m, "FRAME_CLASS", 2)
    set_param(m, "FRAME_TYPE", 2)
    set_param(m, "ARMING_CHECK", 0)
    set_param(m, "OMNI_YAW_MODE", 1)  # TANGENT

    # GPS
    if not wait_gps(m, 8, 120):
        log("FAIL: no GPS; cannot test AUTO")
        sys.exit(2)
    results["gps_ok"] = True

    # current location for relative mission
    loc = get_current_loc(m)
    if loc is None:
        log("FAIL: no GLOBAL_POSITION_INT"); sys.exit(3)
    log(f"  current loc: lat={loc[0]:.7f} lon={loc[1]:.7f}")

    # upload mission
    if not upload_mission(m, loc[0], loc[1]):
        log("FAIL: mission upload"); sys.exit(4)
    results["mission_uploaded"] = True

    # let EKF stabilize while pumping override
    log("--- EKF stabilize 30s ---")
    pump(m, 30)

    # force-arm, then AUTO
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            1, FORCE, 0, 0, 0, 0, 0)
    pump(m, 4)

    set_mode(m, 10)  # AUTO
    pump(m, 3)

    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    armed = hb and bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    in_auto = hb and hb.custom_mode == 10
    log(f"  armed={armed} in_auto={in_auto}")
    results["armed_in_auto"] = bool(armed and in_auto)

    if not (armed and in_auto):
        log("FAIL: could not arm in AUTO")
        # cleanup
        set_mode(m, 0)
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                0, 0, 0, 0, 0, 0, 0, 0)
        for k, v in results.items():
            log(f"  {k:20s} {'PASS' if v else 'FAIL'}")
        sys.exit(5)

    # --- sample SERVO_OUTPUT_RAW during AUTO ---
    log("--- sample SERVO_OUTPUT_RAW 20s ---")
    servos = pump(m, 20, want_type='SERVO_OUTPUT_RAW')
    vals = [[s.servo1_raw, s.servo2_raw, s.servo3_raw, s.servo4_raw]
            for s in servos]
    log(f"  sampled {len(vals)} frames")
    if vals:
        for k in range(0, len(vals), max(1, len(vals)//8)):
            log(f"    {vals[k]}")

    # check lateral evidence:
    # if OMNIX branch is alive, in straight-line motion forward(s1+s3) and
    # forward(s2+s4) should match, but as we turn or crab, s1/s3 and s2/s4
    # diverge (different signs). Look for any frame where (s1-1500) and
    # (s2-1500) have opposite signs.
    lateral_evidence = False
    for s in vals:
        d1 = s[0] - 1500
        d2 = s[1] - 1500
        d3 = s[2] - 1500
        d4 = s[3] - 1500
        if abs(d1) > 30 and abs(d2) > 30 and (d1 * d2 < 0 or d3 * d4 < 0):
            lateral_evidence = True
            break
    log(f"  lateral_evidence: {lateral_evidence}")
    results["lateral_used"] = lateral_evidence

    # cleanup
    set_mode(m, 0)  # MANUAL
    pump(m, 1)
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            0, 0, 0, 0, 0, 0, 0, 0)
    pump(m, 2)

    log("")
    log("=" * 46)
    log("OMNIX AUTO 回归汇总")
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

- [ ] **Step 2: Test the script syntax compiles**

```bash
python -c "import py_compile; py_compile.compile('bench/verify/omni_auto_test.py', doraise=True)"
```

Should exit 0. If syntax error, fix.

- [ ] **Step 3: Commit**

```bash
git add bench/verify/omni_auto_test.py
git commit -m "bench: omni_auto_test — verify OMNIX AUTO uses lateral

Uploads a 2-WP mission (10m N + 10m E from current), switches to
AUTO with OMNI_YAW_MODE=TANGENT, samples SERVO_OUTPUT_RAW for 20s,
verifies lateral evidence (s1/s2 or s3/s4 opposite-sign offsets).
Requires GPS lock (sats >= 8); reports SKIP cleanly otherwise."
```

---

## Task 6: GPS-gated hardware regression

**Files:** None modified. Pure verification.

- [ ] **Step 1: Check GPS status**

```bash
python bench/verify/gps_check.py --port COM10
```

If `sats < 8`: GPS not usable. Report this as a deferred regression and STOP — do not modify any code. Note in commit message that hardware test deferred.

- [ ] **Step 2: Run omni_auto_test.py**

```bash
python bench/verify/omni_auto_test.py --port COM10
```

Expected `总体: PASS` with sub-results:
- `gps_ok = True`
- `mission_uploaded = True`
- `armed_in_auto = True`
- `lateral_used = True` ← **the critical OMNIX evidence**

Failure analysis:
- `armed_in_auto = False` → EK3_SRC config may need updating for AUTO (cf. memory note `ek3_src_baroless_config.md`)
- `lateral_used = False` → OMNIX branch isn't running. Check that `g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX` actually fires (could be a SubMode mismatch). Diagnostic: check `_submode` is `SubMode::WP` after AUTO entry.

- [ ] **Step 3: Run dp_final_test.py + omnix_mix_test.py regression** (P1 regression)

```bash
python bench/verify/dp_final_test.py --port COM10
python bench/verify/omnix_mix_test.py --port COM10
```

Both must still PASS. P2 should not break DP or MANUAL behavior.

- [ ] **Step 4: Capture evidence**

If everything passes, note in memory:
- P2 hardware verified: `omni_auto_test.py` PASS at commit `<HEAD SHA>`, sats=X, HDOP=Y
- P1 regressions confirmed: dp_final_test PASS, omnix_mix_test PASS

If GPS unavailable, mark P2 as "code complete, hardware-verified pending GPS recovery" — same status P1's `dp_final_test.py` is in.

---

## Task 7: Update memory + close P2

**Files:**
- Modify: `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\omnix_dp_mode.md`
- Modify: `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\ar_omnicontrol_library.md`
- Modify: `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\MEMORY.md`

- [ ] **Step 1: Append P2 done section to omnix_dp_mode.md**

```markdown

## P2 完成 (2026-05-XX):ModeAuto OMNIX 分支 + 4 个 OMNI_YAW_MODE
- `ModeAuto::update_omnix_wp()` 替代 `navigate_to_waypoint()` 当 frame=OMNIX
- `compute_omnix_target_yaw()` 实现 4 个策略:LOCK_INITIAL / TANGENT / POINT_NEXT_WP / MANUAL_RC
- `g2.wp_nav.update()` 继续 tick(管理 waypoint 进度)
- 失位 → HOLD 降级,沿用 P1 模式
- 验证:`omni_auto_test.py` PASS(待 GPS),P1 回归(dp_final / omnix_mix)PASS
- 下一步 P3:ModeGuided OMNIX 分支(结构同 P2,目标来源换成 GCS SET_POSITION_TARGET)
```

- [ ] **Step 2: Append to ar_omnicontrol_library.md**

Add a "P2 consumer" section:
```markdown

## P2 (2026-05-XX) — ModeAuto OMNIX consumer

`ModeAuto::update_omnix_wp()` 是第一个非 ModeDP 的 g2.omni_ctrl 消费者。
按 P1 reviewer 提醒:每次 _enter() 都 `g2.omni_ctrl.reset()`,_exit() 也 reset
(避免 stale state)。已实现。

涉及的 OMNI_YAW_MODE 取值:
- 0 LOCK_INITIAL — 进 AUTO 时 snapshot,蟹形走
- 1 TANGENT (默认) — 跟 wp_nav.nav_bearing_cd 切线
- 2 POINT_NEXT_WP — 用 wp_nav.wp_bearing_cd 指向 dest
- 3 MANUAL_RC — RC ch4 控艏向,90 deg/s @ full stick
```

- [ ] **Step 3: Update MEMORY.md index**

Find the existing OMNIX entry and append:
```markdown
- P2 (2026-05-XX): ModeAuto OMNIX 分支 + 4 个 OMNI_YAW_MODE 策略,详见 [omnix_dp_mode.md]
```

- [ ] **Step 4: Done**

P2 complete. Source repo clean.

---

## Self-Review

**Spec coverage (against `2026-05-20-omnix-path-modes-design.md`):**
- ✅ §2 (per-mode target sources) — AUTO row: `wp_nav.get_target_location()` → Task 3 uses `g2.wp_nav.get_destination()` + NED conversion
- ✅ §3 (OMNI_YAW_MODE) — all 4 strategies implemented in Task 2
- ✅ §4 (failsafe to HOLD on lost position) — Task 3 `update_omnix_wp()` + Task 4 `_exit()` cleanup
- ✅ §6 P2 row — "ModeAuto OMNIX 分支 + OMNI_YAW_MODE 实现"
- ⚠️ §3 default `OMNI_YAW_MODE=1` (TANGENT) — already set in P1 T6
- ⚠️ `OMNI_LOS_LOOK` not yet consumed — current TANGENT implementation reads `nav_bearing_cd()` directly from AR_WPNav (which has its own look-ahead via SCurve trajectory). `OMNI_LOS_LOOK` becomes relevant only if we later add a custom tangent calculation. Acceptable to defer.

**Placeholder scan:** no TBDs. Every code block is concrete.

**Type/name consistency:**
- `update_omnix_wp()` and `compute_omnix_target_yaw()` declared in Task 1, defined in Tasks 2/3, called in Task 4 — names consistent.
- `_omni_initial_yaw`, `_omni_rc_yaw_integ`, `_omni_active` — all 3 members declared in Task 1, used in Tasks 2/4.
- `g2._omni_yaw_mode` — was declared in P1 Task 6 as `AP_Int8 _omni_yaw_mode;` — name matches what Task 2 reads.

**Known risks:**
- `AR_WPNav::nav_bearing_cd()` semantics: must verify in-flight that the value tracks the desired heading along the SCurve, not the current heading. If wrong, TANGENT mode would lock yaw to vehicle heading (nonsense). Hardware verification (Task 6) catches this if `lateral_used = False` in straight runs.
- RC ch4 read via `rc().channel(3)` — if no RC input is connected (board on bench), `get_radio_in()` returns 0 or 1500. The deadband should suppress noise; `_omni_rc_yaw_integ` stays at 0 → MANUAL_RC degenerates to LOCK_INITIAL. Safe.
- `set_mode(rover.mode_hold, EKF_FAILSAFE)` from inside `update_omnix_wp()` — same pattern reviewed and approved in P1 T5. Safe.

**Out of scope for P2 (next plans):**
- P3 ModeGuided OMNIX branch
- P4 LOITER / DOCK OMNIX
- P5 RTL / SMART_RTL OMNIX
- P6 出水验证 (water testing)
- Tuning OMNI_LOS_LOOK if a custom TANGENT calc replaces AR_WPNav's
