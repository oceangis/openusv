# OMNIX Path Modes P5 — ModeRTL + ModeSmartRTL OMNIX Branches (Final)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add OMNIX branches to `ModeRTL` and `ModeSmartRTL` so "fly home" missions and SmartRTL path-retrace also use the 3-DOF controller. After P5, all six modes in spec §2 ARE on the OMNIX path: DP / AUTO / GUIDED / LOITER / RTL / SMART_RTL. Dock stays disabled (`MODE_DOCK_ENABLED=0`).

**Architecture:** Same pattern as P2/P3/P4 — each mode's `update()` early-returns to a private `update_omnix_wp()` when frame is OMNIX, feeds `g2.wp_nav.get_destination()` to `g2.omni_ctrl`, uses the shared `omnix_compute_target_yaw()` helper (from P4) for yaw strategy. Both modes share the same OMNIX state struct (`OmniYawState`) introduced in P4.

ModeSmartRTL has multiple sub-states (WaitForPathCleanup / PathFollow / StopAtHome / Failure). Only the `PathFollow` state calls `navigate_to_waypoint()` — so the OMNIX branch only replaces that. StopAtHome/Failure delegate to ModeLoiter (already OMNIX-enabled in P4 — bonus win, RTL "stop at home" gets OMNIX station-keep for free).

**Tech Stack:** No new libraries. Reuses everything from P1-P4.

**Spec:** `docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md`
**Predecessors:** P1+P2+P3+P4 (commits `abd83c0..5676089b`)

---

## File Structure

**Modified files:**
- `Rover/mode.h` — add `OmniYawState _omni_yaw_state` + `bool _omni_active` + `void update_omnix_wp()` to `ModeRTL` AND `ModeSmartRTL` (mirror P2/P3 pattern)
- `Rover/mode_rtl.cpp` — implement `update_omnix_wp()`; dispatch from `update()`; snapshot in `_enter()`
- `Rover/mode_smart_rtl.cpp` — implement `update_omnix_wp()`; dispatch from `update()` PathFollow case; snapshot in `_enter()`

**New bench script:**
- `bench/verify/omni_rtl_test.py` — switch to RTL on OMNIX frame, verify lateral evidence in SERVO_OUTPUT_RAW

**SmartRTL no bench test**: SmartRTL needs the boat to have moved (so it has a path to retrace). Hard to fake on a stationary bench. Functional test deferred to water testing.

---

## Task 1: Add ModeRTL OMNIX state + declaration

**Files:**
- Modify: `Rover/mode.h` (the `class ModeRTL` body)

- [ ] **Step 1: Find class ModeRTL**

```bash
grep -n "class ModeRTL" Rover/mode.h
```

Read its body. Note: `ModeRTL` and `ModeSmartRTL` are likely adjacent.

- [ ] **Step 2: Add OMNIX members to ModeRTL**

In the `private:` (or last `protected:`) section of `class ModeRTL`, before its closing `};`, add:

```cpp
private:
    // --- OMNIX branch state (P5) ---
    OmniYawState _omni_yaw_state;
    bool _omni_active{false};

    // OMNIX-only WP control for RTL. Replaces navigate_to_waypoint() when
    // FRAME_TYPE==OMNIX. Reads target from g2.wp_nav (home/rally), yaw per
    // OMNI_YAW_MODE, dispatches to g2.omni_ctrl. Degrades to HOLD on lost
    // position.
    void update_omnix_wp();
```

If `ModeRTL` already has a `private:` section, append within it; otherwise add the new `private:` section.

- [ ] **Step 3: Build to verify the class header parses**

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: build SUCCEEDS (declarations only, no caller yet — same pattern as P3 T1 / P2 T1).

- [ ] **Step 4: Commit**

```bash
git add Rover/mode.h
git commit -m "ModeRTL: declare update_omnix_wp + OMNIX state members

Adds OmniYawState + _omni_active + update_omnix_wp() declaration,
mirroring P2/P3 pattern. Definition follows in T2."
```

---

## Task 2: Implement ModeRTL::update_omnix_wp() + wire dispatch + _enter snapshot

**Files:**
- Modify: `Rover/mode_rtl.cpp`

This task bundles three small edits because ModeRTL is simple enough — no benefit to splitting like we did for Auto/Guided.

- [ ] **Step 1: Append update_omnix_wp() implementation**

Append to `Rover/mode_rtl.cpp`:

```cpp
// OMNIX-only WP control for RTL. Called from update() when frame is OMNIX.
// Reads target from g2.wp_nav (home or rally point set in _enter), yaw
// per OMNI_YAW_MODE, drives g2.omni_ctrl. Degrades to HOLD on lost position.
void ModeRTL::update_omnix_wp()
{
    g2.wp_nav.update(rover.G_Dt);

    if (g2.wp_nav.reached_destination()) {
        // Send notification once
        if (send_notification) {
            send_notification = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reached destination");
        }

        // Boats loiter (Loiter has its own OMNIX branch — bonus from P4)
        if (rover.is_boat()) {
            if (!_loitering) {
                _loitering = rover.mode_loiter.enter();
            }
            if (_loitering) {
                rover.mode_loiter.update();
            } else {
                g2.motors.set_throttle(0.0f);
                g2.motors.set_lateral(0.0f);
                g2.motors.set_steering(0.0f);
            }
        } else {
            g2.motors.set_throttle(0.0f);
            g2.motors.set_lateral(0.0f);
            g2.motors.set_steering(0.0f);
        }
        _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
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
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
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
    const Vector2f target_pos_ned = origin.get_distance_NE(g2.wp_nav.get_destination());

    // --- Target yaw via shared helper ---
    const float target_yaw = omnix_compute_target_yaw(_omni_yaw_state, rover.G_Dt);

    // --- Drive controller ---
    g2.omni_ctrl.set_target(target_pos_ned, target_yaw);
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
}
```

- [ ] **Step 2: Modify ModeRTL::_enter() to snapshot OMNIX state**

Before the existing `return true;` at the end of `ModeRTL::_enter()`, add:

```cpp
    // OMNIX P5: snapshot heading + reset controller if frame is OMNIX
    _omni_yaw_state.initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
    _omni_yaw_state.rc_yaw_integ = 0.0f;
    _omni_active = (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX);
    if (_omni_active) {
        g2.omni_ctrl.reset();
    }
```

- [ ] **Step 3: Dispatch in update()**

In `ModeRTL::update()`, at the very beginning (before the existing `if (!g2.wp_nav.reached_destination())` check), add:

```cpp
    // OMNIX 4-thruster holonomic branch (P5)
    if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
        update_omnix_wp();
        return;
    }
```

- [ ] **Step 4: Build + flash**

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: `Project build complete.`

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_dp.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: `Hash of data verified.`

- [ ] **Step 5: Commit**

```bash
git add Rover/mode_rtl.cpp
git commit -m "ModeRTL: add OMNIX branch — fly home via g2.omni_ctrl

_enter() snapshots heading + resets controller when frame is OMNIX.
update() dispatches to update_omnix_wp() which drives g2.omni_ctrl
toward wp_nav's home/rally destination using shared
omnix_compute_target_yaw helper.

On reached_destination, boats delegate to ModeLoiter (which is also
OMNIX-enabled per P4) — RTL's 'stop at home' inherits station-keep
for free. Degrades to HOLD on lost position.

Flashed."
```

---

## Task 3: Add ModeSmartRTL OMNIX state + declaration

**Files:**
- Modify: `Rover/mode.h` (the `class ModeSmartRTL` body)

Same as Task 1 but for SmartRTL.

- [ ] **Step 1: Add OMNIX members**

Inside `#if MODE_SMARTRTL_ENABLED` ... `class ModeSmartRTL { ... }`, add to the private section (before closing `};`):

```cpp
private:
    // --- OMNIX branch state (P5) ---
    OmniYawState _omni_yaw_state;
    bool _omni_active{false};

    // OMNIX-only WP control for SmartRTL PathFollow state. Replaces
    // navigate_to_waypoint() when FRAME_TYPE==OMNIX. Same pattern as
    // ModeRTL but acts on each path-pop point.
    void update_omnix_wp();
```

- [ ] **Step 2: Build**

Expected: SUCCEEDS (declaration only).

- [ ] **Step 3: Commit**

```bash
git add Rover/mode.h
git commit -m "ModeSmartRTL: declare update_omnix_wp + OMNIX state

Same pattern as ModeRTL — mirrors P2/P3/P4 OMNIX state convention."
```

---

## Task 4: Implement ModeSmartRTL::update_omnix_wp() + wire dispatch + _enter

**Files:**
- Modify: `Rover/mode_smart_rtl.cpp`

ModeSmartRTL is a state machine. OMNIX dispatch goes inside the `SmartRTLState::PathFollow` case — the other states (WaitForPathCleanup / StopAtHome / Failure) don't navigate or delegate to Loiter (which is already OMNIX-aware).

- [ ] **Step 1: Append update_omnix_wp() implementation**

Append to `Rover/mode_smart_rtl.cpp` (inside the `#if MODE_SMARTRTL_ENABLED` block, before the closing `#endif`):

```cpp
// OMNIX-only WP control for SmartRTL PathFollow state. Called when
// frame is OMNIX. Replaces navigate_to_waypoint() with g2.omni_ctrl
// drive toward the current popped path point.
void ModeSmartRTL::update_omnix_wp()
{
    g2.wp_nav.update(rover.G_Dt);

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
        g2.motors.set_throttle(0.0f);
        g2.motors.set_lateral(0.0f);
        g2.motors.set_steering(0.0f);
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
    const Vector2f target_pos_ned = origin.get_distance_NE(g2.wp_nav.get_destination());

    // --- Target yaw via shared helper ---
    const float target_yaw = omnix_compute_target_yaw(_omni_yaw_state, rover.G_Dt);

    // --- Drive controller ---
    g2.omni_ctrl.set_target(target_pos_ned, target_yaw);
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
}
```

- [ ] **Step 2: Modify ModeSmartRTL::_enter() to snapshot OMNIX state**

Before the existing `return true;` at the end of `ModeSmartRTL::_enter()`, add:

```cpp
    // OMNIX P5: snapshot heading + reset controller if frame is OMNIX
    _omni_yaw_state.initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
    _omni_yaw_state.rc_yaw_integ = 0.0f;
    _omni_active = (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX);
    if (_omni_active) {
        g2.omni_ctrl.reset();
    }
```

- [ ] **Step 3: Dispatch in update() PathFollow case**

Find this block in `ModeSmartRTL::update()`:
```cpp
        case SmartRTLState::PathFollow:
            // load point if required
            if (_load_point) {
                ...
                _load_point = false;
            }
            // update navigation controller
            navigate_to_waypoint();
            ...
```

The OMNIX dispatch goes BETWEEN the load-point block and `navigate_to_waypoint()`. Replace `navigate_to_waypoint();` with:

```cpp
            // OMNIX 4-thruster holonomic branch (P5)
            if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
                update_omnix_wp();
            } else {
                // update navigation controller (differential-drive path)
                navigate_to_waypoint();
            }
```

The `if (g2.wp_nav.reached_destination()) { _load_point = true; }` check below stays as-is — both paths feed `wp_nav` so reached_destination works for either.

- [ ] **Step 4: Build + flash**

Same commands as Task 2 Step 4.

- [ ] **Step 5: Commit**

```bash
git add Rover/mode_smart_rtl.cpp
git commit -m "ModeSmartRTL: add OMNIX branch in PathFollow state

_enter() snapshots heading + resets controller when frame is OMNIX.
update() PathFollow case dispatches to update_omnix_wp() when OMNIX;
diff-drive navigate_to_waypoint() path otherwise.

StopAtHome / Failure states already delegate to ModeLoiter (OMNIX
since P4), so SmartRTL gets full OMNIX coverage for free.

Flashed."
```

---

## Task 5: Bench script omni_rtl_test.py

**Files:**
- Create: `bench/verify/omni_rtl_test.py`

- [ ] **Step 1: Create the script**

Write `bench/verify/omni_rtl_test.py`:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OMNIX RTL 模式回归测试 — 验证 ModeRTL::update_omnix_wp() 使用横移。

测试流程:
  1. 配 OMNIX 帧 + OMNI_YAW_MODE=1 (TANGENT) + ARMING_CHECK=0
  2. 等 GPS lock
  3. force-arm, MANUAL 模式开几秒(让 home 设置在当前位置附近)
  4. 切回 MANUAL, 模拟 "boat 在场外" — 设 home 到东北方向 10m
  5. 切 RTL (mode 11), 让 boat 'return to home'
  6. 采样 SERVO_OUTPUT_RAW 20s
  7. 验证: lateral_evidence (s1/s2 或 s3/s4 反向偏移)

注: home 设置在 ArduPilot 通常是 arm 时自动捕获,移动 home 需要
MAV_CMD_DO_SET_HOME (param1=0 自动用当前位置). 为了 RTL 真的有距离
驱动,先 arm 一次设 home(原点),然后用 SET_HOME 把 home 挪到偏移点。

GPS lock 不可用时 SKIP (exit 2)。

用法: python omni_rtl_test.py [--port COM10]
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
SET_HOME = mavutil.mavlink.MAV_CMD_DO_SET_HOME
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


def set_mode(m, n):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)


def set_home(m, lat, lon, alt):
    """MAV_CMD_DO_SET_HOME with explicit coordinates (param1=0)."""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        SET_HOME, 0,
        0,  # use specified (not current)
        0, 0, 0,
        lat, lon, alt,
    )


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
    set_param(m, "OMNI_YAW_MODE", 1)  # TANGENT

    if not wait_gps(m, 8, 120):
        log("SKIP: no GPS; cannot test RTL")
        m.close()
        sys.exit(2)
    results["gps_ok"] = True

    loc = get_current_loc(m)
    if loc is None:
        log("FAIL: no GLOBAL_POSITION_INT"); sys.exit(3)
    log(f"  current loc: lat={loc[0]:.7f} lon={loc[1]:.7f}")

    log("--- EKF stabilize 30s ---")
    pump(m, 30)

    # force-arm to capture home, then set home 10m NE of current
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            1, FORCE, 0, 0, 0, 0, 0)
    pump(m, 4)

    # set home 10m N + 5m E so RTL has a clear destination to drive to
    dlat = 10.0 / 111320.0
    dlon = 5.0 / (111320.0 * max(0.1, math.cos(math.radians(loc[0]))))
    home_lat = loc[0] + dlat
    home_lon = loc[1] + dlon
    log(f"  setting home to: lat={home_lat:.7f} lon={home_lon:.7f}")
    set_home(m, home_lat, home_lon, 0.0)
    pump(m, 2)

    # switch to RTL (mode 11)
    set_mode(m, 11)  # RTL
    pump(m, 3)

    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    armed = hb and bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    in_rtl = hb and hb.custom_mode == 11
    log(f"  armed={armed} in_rtl={in_rtl}")
    results["armed_in_rtl"] = bool(armed and in_rtl)

    if not (armed and in_rtl):
        log("FAIL: could not arm in RTL")
        set_mode(m, 0)
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                0, 0, 0, 0, 0, 0, 0, 0)
        for k, v in results.items():
            log(f"  {k:20s} {'PASS' if v else 'FAIL'}")
        sys.exit(5)

    log("--- sample SERVO_OUTPUT_RAW 20s ---")
    servos = pump(m, 20, want_type='SERVO_OUTPUT_RAW')
    vals = [[s.servo1_raw, s.servo2_raw, s.servo3_raw, s.servo4_raw]
            for s in servos]
    log(f"  sampled {len(vals)} frames")
    if vals:
        for k in range(0, len(vals), max(1, len(vals)//8)):
            log(f"    {vals[k]}")

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
    set_mode(m, 0)
    pump(m, 1)
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            0, 0, 0, 0, 0, 0, 0, 0)
    pump(m, 2)

    log("")
    log("=" * 46)
    log("OMNIX RTL 回归汇总")
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
python -c "import py_compile; py_compile.compile('bench/verify/omni_rtl_test.py', doraise=True)"
```

- [ ] **Step 3: Commit**

```bash
git add bench/verify/omni_rtl_test.py
git commit -m "bench: omni_rtl_test — verify OMNIX RTL uses lateral

Force-arms to capture home, moves home 10m N + 5m E via SET_HOME,
switches to RTL, samples SERVO_OUTPUT_RAW 20s, verifies lateral
evidence. SKIPs cleanly without GPS."
```

SmartRTL bench is NOT created — the mode needs a saved path to retrace, which requires the boat to have moved. Validation deferred to water testing.

---

## Task 6: Hardware regression

**Files:** none modified.

- [ ] **Step 1: GPS check**

```bash
python bench/verify/gps_check.py --port COM10
```

- [ ] **Step 2: Run all existing bench scripts + the new RTL one**

```bash
python bench/verify/omnix_mix_test.py --port COM10
python bench/verify/omni_auto_test.py --port COM10
python bench/verify/omni_guided_test.py --port COM10
python bench/verify/omni_loiter_test.py --port COM10
python bench/verify/omni_rtl_test.py --port COM10
```

Acceptance:
- `omnix_mix_test.py` 总体: PASS (REQUIRED — mixer untouched)
- omni_*_test.py: each must exit 0 (PASS with GPS) or 2 (SKIP without GPS)
- No exception traces, no FAIL exit codes

If `omnix_mix_test.py` fails → BLOCKED.

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
while time.time() - t0 < 10:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if not msg: continue
    if msg.param_id.startswith('DP_'): dp[msg.param_id] = msg.param_value
    elif msg.param_id.startswith('OMNI_'): omni[msg.param_id] = msg.param_value
print(f'DP_* {len(dp)} / OMNI_* {len(omni)}')
"
```

Expected: 11 DP_* + 2 OMNI_*. Any drift = BLOCKED.

---

## Task 7: Memory close (final)

**Files:**
- Append to `omnix_dp_mode.md`
- Append to `ar_omnicontrol_library.md`
- Update `MEMORY.md` (P1-P5 final entry)

Document the completion of all OMNIX path modes (P1-P5). Note that ModeDock stays disabled and water testing (P6) is the remaining piece.

---

## Self-Review

**Spec coverage:**
- ✅ Spec §6 P5 row: "ModeRTL + ModeSmartRTL OMNIX 分支" — both done
- ✅ Spec §2 RTL row: "home / rally (沿用现有 wp_nav)" — Task 2 uses `g2.wp_nav.get_destination()` which is set to home/rally in `_enter`
- ✅ Spec §2 SMART_RTL row: "smart_rtl.thorough_cleanup_path() 当前点" — Task 4 uses wp_nav.get_destination which is set by the existing PathFollow state's load-point logic
- ✅ Failsafe to HOLD (spec §4) — both update_omnix_wp implementations call `set_mode(mode_hold, EKF_FAILSAFE)` on lost position

**Placeholder scan:** No TBDs.

**Type/name consistency:**
- `OmniYawState`, `omnix_compute_target_yaw`, `_omni_yaw_state`, `_omni_active`, `update_omnix_wp` — names match P4 conventions
- `get_distance_NE` (not `_float`) — same as P2/P3/P4
- `ModeReason::EKF_FAILSAFE` — same as P1-P4

**Known risks:**
- ModeRTL's `_loitering` flag interaction with OMNIX path: the OMNIX `reached_destination` block above calls `rover.mode_loiter.enter()` then `rover.mode_loiter.update()` — same as diff-drive. ModeLoiter is OMNIX-aware (P4), so this nests cleanly. The only risk is if ModeLoiter::_enter() resets `g2.omni_ctrl` while RTL is mid-mission and then re-enters PathFollow — but RTL doesn't go back to PathFollow once reached, so safe.
- ModeSmartRTL's StopAtHome state delegates to ModeLoiter too — same OMNIX nesting, same low risk.

**Out of scope after P5:**
- P6: water testing (requires actual boat in water)
- ModeDock: permanently disabled
- HeadingAndSpeed / TurnRateAndSpeed Guided submodes — deferred indefinitely
- GPS-gated bench tests — pending board near window/outside (4 scripts: dp_final, omni_auto, omni_guided, omni_loiter, omni_rtl all need GPS)

**P5 completes spec §6 phases 1-5.** After P5, the only remaining work is the user-driven water testing phase.
