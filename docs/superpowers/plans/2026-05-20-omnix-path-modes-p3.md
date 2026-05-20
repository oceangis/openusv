# OMNIX Path Modes P3 — ModeGuided OMNIX Branch (WP submode)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `ModeGuided` use OMNIX 3-DOF control when `FRAME_TYPE == OMNIX` and the current submode is `SubMode::WP`. GCS-driven "fly to GPS coordinate" missions (MAV_CMD_DO_REPOSITION, SET_POSITION_TARGET_GLOBAL_INT) will use lateral/heading independently instead of degrading to differential drive. Other Guided submodes (HeadingAndSpeed/TurnRateAndSpeed/SteeringAndThrottle/Loiter) remain on diff-drive — they either lack a position target (heading+speed only) or delegate to other modes covered in P4.

**Architecture:** Mirror P2's pattern. `ModeGuided::update()` SubMode::WP dispatches to a new `update_omnix_wp()` private method when frame is OMNIX. The new path: extract target from `g2.wp_nav.get_destination()` (same as Auto), compute target yaw per `OMNI_YAW_MODE`, drive `g2.omni_ctrl`, degrade to HOLD on lost position. ModeGuided gets its own `_omni_initial_yaw` / `_omni_active` state (parallel to ModeAuto) — duplicate but explicit. Factoring shared helper into AR_OmniControl is deferred to P4 cleanup.

**Tech Stack:** ESP-IDF 5.5.1, existing `AR_OmniControl` + `AR_WPNav`, no new libraries.

**Spec:** `docs/superpowers/specs/2026-05-20-omnix-path-modes-design.md`
**Predecessors:** P1 (`abd83c0..ed44edb3`) + P2 (`791ad05..8753dcae`)

---

## File Structure

**Modified files:**
- `Rover/mode.h` — add `update_omnix_wp()` private method to `ModeGuided`; add `_omni_initial_yaw` + `_omni_active` state members
- `Rover/mode_guided.cpp` — implement `update_omnix_wp()`; dispatch from `update()` SubMode::WP; snapshot yaw in `_enter()`; reset controller in exit path

**Reused unchanged:**
- `libraries/AR_OmniControl/AR_OmniControl.{h,cpp}` — same `g2.omni_ctrl` instance, same API
- `ModeAuto::compute_omnix_target_yaw()` — NOT directly callable from ModeGuided (it's private to ModeAuto and uses ModeAuto's `_omni_initial_yaw`). P3 implements an equivalent helper in ModeGuided. (Deferred refactor: move to a free function or AR_OmniControl method in P4.)

**New bench script:**
- `bench/verify/omni_guided_test.py` — uploads a single GCS-driven destination via `MAV_CMD_DO_REPOSITION`, verifies lateral evidence

---

## Background — How GCS sets a Guided target

`ModeGuided` exposes `set_desired_location(const Location &destination, ...)` which is the entry point for:
- `MAV_CMD_DO_REPOSITION` (mission item or runtime command)
- `SET_POSITION_TARGET_GLOBAL_INT` (continuous target stream)

Both end up storing the destination via `g2.wp_nav.set_desired_location()` and setting `_guided_mode = SubMode::WP`. From that point on, `ModeGuided::update()` SubMode::WP branch runs `navigate_to_waypoint()` — exactly the same flow as `ModeAuto::SubMode::WP`.

The OMNIX branch can read the target via `g2.wp_nav.get_destination()` — identical to how P2's `ModeAuto::update_omnix_wp()` reads it. No new ModeGuided-specific accessor needed.

**No GCS-yaw priority rule applies in SubMode::WP** — that priority rule from the spec (§3) applies to `set_desired_heading_and_speed()` and other yaw-carrying setters. SubMode::WP destinations are bare Locations (no yaw field). P3 always uses `OMNI_YAW_MODE` for SubMode::WP.

---

## Task 1: Add OMNIX state members + method declaration to ModeGuided

**Files:**
- Modify: `Rover/mode.h` (the `class ModeGuided` body, around lines 527-630)

- [ ] **Step 1: Read existing ModeGuided class declaration**

```bash
grep -n "class ModeGuided" Rover/mode.h
```

Read from that line through the closing `};`. Identify the existing `private:` section.

- [ ] **Step 2: Add private members + helper declaration**

In `Rover/mode.h`, in the `private:` section of `class ModeGuided`, at the end (just before the closing `};`), add:

```cpp
    // --- OMNIX branch state (P3) ---
    // initial yaw captured at mode entry, used by OMNI_YAW_MODE=0 (LOCK_INITIAL)
    float _omni_initial_yaw{0.0f};
    // integrator for OMNI_YAW_MODE=3 (MANUAL_RC)
    float _omni_rc_yaw_integ{0.0f};
    // true if this Guided entry is on OMNIX frame; gates exit cleanup
    bool  _omni_active{false};

    // OMNIX-only WP control. Replaces navigate_to_waypoint() in SubMode::WP
    // when FRAME_TYPE==OMNIX. Reads target from g2.wp_nav (GCS-set destination),
    // yaw per OMNI_YAW_MODE, dispatches to g2.omni_ctrl. Degrades to HOLD on
    // lost position.
    void update_omnix_wp();

    // Computes target heading (rad) per OMNI_YAW_MODE.
    float compute_omnix_target_yaw(float current_yaw, float dt);
```

These declarations mirror what ModeAuto got in P2 T1. The duplication is intentional — see plan header note about factoring deferred to P4.

- [ ] **Step 3: Build**

```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Expected: build SUCCEEDS (declarations only, no callers, same as P2 T1 behavior).

- [ ] **Step 4: Commit**

```bash
git add Rover/mode.h
git commit -m "ModeGuided: declare update_omnix_wp + compute_omnix_target_yaw

Adds private OMNIX yaw state (LOCK_INITIAL snapshot, MANUAL_RC
integrator, active flag) and two helper method declarations,
mirroring P2 ModeAuto. Definitions follow in T2/T3."
```

---

## Task 2: Implement compute_omnix_target_yaw() — all 4 strategies (parallel to P2 T2)

**Files:**
- Modify: `Rover/mode_guided.cpp` (append at end of file)

The helper is identical in algorithmic content to `ModeAuto::compute_omnix_target_yaw` (P2 T2). The duplication is acceptable for P3.

- [ ] **Step 1: Append implementation**

Append to `Rover/mode_guided.cpp`:

```cpp
// Compute target heading for the OMNIX Guided WP branch.
// Strategy chosen by g2._omni_yaw_mode parameter.
// Returns radians.
float ModeGuided::compute_omnix_target_yaw(float current_yaw, float dt)
{
    const uint8_t mode = (uint8_t)g2._omni_yaw_mode.get();

    switch (mode) {
    case 0: // LOCK_INITIAL — snapshot taken on entry
        return _omni_initial_yaw;

    case 1: { // TANGENT — follow path tangent from AR_WPNav
        const float bearing_deg = g2.wp_nav.nav_bearing_cd() * 0.01f;
        return radians(bearing_deg);
    }

    case 2: { // POINT_NEXT_WP — bearing to destination from current position
        const float bearing_deg = g2.wp_nav.wp_bearing_cd() * 0.01f;
        return radians(bearing_deg);
    }

    case 3: { // MANUAL_RC — RC ch4 controls yaw rate, integrated
        const float kMaxYawRate = radians(90.0f);
        float stick_norm = 0.0f;
        const RC_Channel *ch4 = rc().channel(3);   // 0-indexed -> channel 4
        if (ch4 != nullptr) {
            const uint16_t pwm = ch4->get_radio_in();
            stick_norm = constrain_float((pwm - 1500.0f) / 500.0f, -1.0f, 1.0f);
            if (fabsf(stick_norm) < 0.05f) stick_norm = 0.0f;
        }
        if (is_positive(dt)) {
            _omni_rc_yaw_integ += stick_norm * kMaxYawRate * dt;
            _omni_rc_yaw_integ = wrap_PI(_omni_rc_yaw_integ);
        }
        return wrap_PI(_omni_initial_yaw + _omni_rc_yaw_integ);
    }

    default:
        return _omni_initial_yaw;
    }
    (void)current_yaw;
}
```

- [ ] **Step 2: Build**

Same command as Task 1 Step 3. Expected: build succeeds (no caller yet).

- [ ] **Step 3: Commit**

```bash
git add Rover/mode_guided.cpp
git commit -m "ModeGuided: implement compute_omnix_target_yaw (all 4 OMNI_YAW_MODE)

Identical algorithm to ModeAuto::compute_omnix_target_yaw (P2 T2)
operating on ModeGuided's own _omni_initial_yaw / _omni_rc_yaw_integ.
Duplication acceptable for P3; factor into shared helper in P4."
```

---

## Task 3: Implement update_omnix_wp()

**Files:**
- Modify: `Rover/mode_guided.cpp` (append after compute_omnix_target_yaw)

The OMNIX WP control loop. Same shape as `ModeAuto::update_omnix_wp` (P2 T3) but with Guided-specific completion handling. Guided's diff-drive path uses `_distance_to_destination` update and a `send_notification` flag on completion — the OMNIX branch preserves those.

- [ ] **Step 1: Append implementation**

Append to `Rover/mode_guided.cpp`:

```cpp
// OMNIX-only Guided WP control. Called from ModeGuided::update() SubMode::WP
// when g2.motors.get_frame_type() == FRAME_TYPE_OMNIX.
//
// Responsibilities:
//   - tick g2.wp_nav for waypoint progression
//   - on reached_destination: send notification + start loiter/stop
//   - extract destination Location -> NED target offset
//   - compute target yaw per OMNI_YAW_MODE
//   - drive g2.omni_ctrl + push outputs to motors
//   - degrade to HOLD on lost position or no origin
void ModeGuided::update_omnix_wp()
{
    g2.wp_nav.update(rover.G_Dt);

    if (g2.wp_nav.reached_destination()) {
        if (send_notification) {
            send_notification = false;
            rover.gcs().send_mission_item_reached_message(0);
        }
        if (rover.is_boat()) {
            if (!start_loiter()) {
                stop_vehicle();
            }
        } else {
            stop_vehicle();
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
    const Location& dest = g2.wp_nav.get_destination();
    const Vector2f target_pos_ned = origin.get_distance_NE(dest);

    // --- Target yaw ---
    const float current_yaw = radians(ahrs.yaw_sensor * 0.01f);
    const float target_yaw  = compute_omnix_target_yaw(current_yaw, rover.G_Dt);

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

Use `get_distance_NE` (not `get_distance_NE_float`) — verified API name in P2 T3.

- [ ] **Step 2: Build**

Expected: build succeeds. Both helpers now defined; no caller yet.

- [ ] **Step 3: Commit**

```bash
git add Rover/mode_guided.cpp
git commit -m "ModeGuided: implement update_omnix_wp() — OMNIX Guided WP loop

Drives g2.omni_ctrl from wp_nav GCS-set destination. Preserves
diff-drive completion behavior (send_notification +
start_loiter/stop_vehicle on reached). Degrades to HOLD on lost
position estimate."
```

---

## Task 4: Wire dispatch in ModeGuided::update() SubMode::WP + _enter()

**Files:**
- Modify: `Rover/mode_guided.cpp:3-20` (the `_enter()` body)
- Modify: `Rover/mode_guided.cpp:24-50` (the SubMode::WP case in `update()`)

- [ ] **Step 1: Snapshot yaw + reset omni_ctrl in _enter()**

Find `ModeGuided::_enter()` (around line 3). Before `return true;` add:

```cpp
    // OMNIX P3: snapshot heading for yaw strategies + reset controller if OMNIX
    _omni_initial_yaw = radians(ahrs.yaw_sensor * 0.01f);
    _omni_rc_yaw_integ = 0.0f;
    _omni_active = (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX);
    if (_omni_active) {
        g2.omni_ctrl.reset();
    }
```

- [ ] **Step 2: Dispatch in SubMode::WP case**

Find the existing block (around lines 24-50):
```cpp
        case SubMode::WP:
        {
            // check if we've reached the destination
            if (!g2.wp_nav.reached_destination()) {
                // update navigation controller
                navigate_to_waypoint();
            } else {
                // send notification
                if (send_notification) {
                    send_notification = false;
                    rover.gcs().send_mission_item_reached_message(0);
                }

                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
                // update distance to destination
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            }
            break;
        }
```

Replace with:
```cpp
        case SubMode::WP:
        {
            // OMNIX 4-thruster holonomic branch (P3)
            if (g2.motors.get_frame_type() == AP_MotorsUGV::FRAME_TYPE_OMNIX) {
                update_omnix_wp();
                break;
            }

            // check if we've reached the destination
            if (!g2.wp_nav.reached_destination()) {
                // update navigation controller (differential-drive path)
                navigate_to_waypoint();
            } else {
                // send notification
                if (send_notification) {
                    send_notification = false;
                    rover.gcs().send_mission_item_reached_message(0);
                }

                // we have reached the destination so stay here
                if (rover.is_boat()) {
                    if (!start_loiter()) {
                        stop_vehicle();
                    }
                } else {
                    stop_vehicle();
                }
                // update distance to destination
                _distance_to_destination = rover.current_loc.get_distance(g2.wp_nav.get_destination());
            }
            break;
        }
```

The OMNIX branch early-returns BEFORE the diff-drive logic.

- [ ] **Step 3: ModeGuided has no virtual _exit()**

Verify by reading `Rover/mode.h` for the `ModeGuided` class — does it override `_exit()`? If yes, add OMNIX reset at the end (mirror P2 T4):
```cpp
if (_omni_active) {
    g2.omni_ctrl.reset();
    _omni_active = false;
}
```

If `ModeGuided` does NOT override `_exit()`, leave it alone. The integrator will be reset at the next `_enter()` of any OMNIX-aware mode (because `_enter` always calls `reset()` when active). The risk is small — across one mode switch, integrators might briefly be stale if the next consumer doesn't `reset()`. But all current consumers (ModeDP P1, ModeAuto P2, ModeGuided P3) do `reset()` on `_enter`, so this is safe.

Document the decision in the commit message either way.

- [ ] **Step 4: Build + flash**

Build:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_now.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_build_log.txt 2>&1' -Wait -NoNewWindow"
```

Flash:
```bash
powershell.exe -NoProfile -Command "Start-Process -FilePath 'cmd.exe' -ArgumentList '/c F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_dp.bat > F:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\_flash_log.txt 2>&1' -Wait -NoNewWindow"
```

Both must succeed.

- [ ] **Step 5: Commit**

```bash
git add Rover/mode_guided.cpp
git commit -m "ModeGuided: wire OMNIX branch into SubMode::WP + _enter

_enter() snapshots heading + resets g2.omni_ctrl when frame is OMNIX.
update() SubMode::WP dispatches to update_omnix_wp() when OMNIX;
diff-drive path unchanged otherwise.

Note: ModeGuided does not override _exit() in this tree, so omni_ctrl
state persists past mode exit. Next OMNIX consumer's _enter() resets
it. All current consumers (DP/Auto/Guided) do this -- safe in practice.

Flashed -- OMNIX Guided WP now active on hardware."
```

---

## Task 5: Write omni_guided_test.py bench script

**Files:**
- Create: `bench/verify/omni_guided_test.py`

Similar to `omni_auto_test.py` (P2 T5) but exercises Guided instead of Auto: configure OMNIX, switch directly to Guided, send a single `MAV_CMD_DO_REPOSITION` to a target 10m N + 5m E, sample SERVO_OUTPUT_RAW for 20s, verify lateral evidence.

- [ ] **Step 1: Create the script**

Write `bench/verify/omni_guided_test.py`:

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OMNIX Guided 模式回归测试 — 验证 ModeGuided::update_omnix_wp() 使用横移。

测试流程:
  1. 配 OMNIX 帧 + OMNI_YAW_MODE=1 (TANGENT) + ARMING_CHECK=0
  2. 等 GPS lock
  3. force-arm, 切 GUIDED (mode 15)
  4. 发 MAV_CMD_DO_REPOSITION 到 10m N + 5m E
  5. 采样 SERVO_OUTPUT_RAW 20s
  6. 验证: lateral_evidence = True (s1/s2 或 s3/s4 反向偏移)
  7. cleanup: MANUAL + disarm

GPS lock 不可用时 SKIP (exit 2)。

用法: python omni_guided_test.py [--port COM10]
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
DO_REPOSITION = mavutil.mavlink.MAV_CMD_DO_REPOSITION
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


def send_reposition(m, lat_deg, lon_deg, alt_m):
    """Send MAV_CMD_DO_REPOSITION with destination lat/lon/alt."""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        DO_REPOSITION, 0,
        -1,  # speed: default
        0, 0, float('nan'),  # bitmask, radius, yaw
        lat_deg, lon_deg, alt_m,
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

    # configure
    set_param(m, "FRAME_CLASS", 2)
    set_param(m, "FRAME_TYPE", 2)
    set_param(m, "ARMING_CHECK", 0)
    set_param(m, "OMNI_YAW_MODE", 1)  # TANGENT

    if not wait_gps(m, 8, 120):
        log("SKIP: no GPS; cannot test Guided")
        m.close()
        sys.exit(2)
    results["gps_ok"] = True

    loc = get_current_loc(m)
    if loc is None:
        log("FAIL: no GLOBAL_POSITION_INT"); sys.exit(3)
    log(f"  current loc: lat={loc[0]:.7f} lon={loc[1]:.7f}")

    # let EKF stabilize
    log("--- EKF stabilize 30s ---")
    pump(m, 30)

    # force-arm + Guided (mode 15)
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            1, FORCE, 0, 0, 0, 0, 0)
    pump(m, 4)
    set_mode(m, 15)  # GUIDED
    pump(m, 3)

    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    armed = hb and bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    in_guided = hb and hb.custom_mode == 15
    log(f"  armed={armed} in_guided={in_guided}")
    results["armed_in_guided"] = bool(armed and in_guided)

    if not (armed and in_guided):
        log("FAIL: could not arm in Guided")
        set_mode(m, 0)
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                0, 0, 0, 0, 0, 0, 0, 0)
        for k, v in results.items():
            log(f"  {k:20s} {'PASS' if v else 'FAIL'}")
        sys.exit(5)

    # Reposition target: 10m N + 5m E from current
    dlat = 10.0 / 111320.0
    # 1 deg lon at lat L = ~111320 * cos(L) m
    dlon = 5.0 / (111320.0 * max(0.1, math.cos(math.radians(loc[0]))))
    target_lat = loc[0] + dlat
    target_lon = loc[1] + dlon
    log(f"  reposition target: lat={target_lat:.7f} lon={target_lon:.7f}")
    send_reposition(m, target_lat, target_lon, 0.0)
    pump(m, 2)

    # --- sample SERVO_OUTPUT_RAW for 20s ---
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
    set_mode(m, 0)  # MANUAL
    pump(m, 1)
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            0, 0, 0, 0, 0, 0, 0, 0)
    pump(m, 2)

    log("")
    log("=" * 46)
    log("OMNIX Guided 回归汇总")
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
python -c "import py_compile; py_compile.compile('bench/verify/omni_guided_test.py', doraise=True)"
```

- [ ] **Step 3: Commit**

```bash
git add bench/verify/omni_guided_test.py
git commit -m "bench: omni_guided_test — verify OMNIX Guided WP uses lateral

Switches to Guided, sends MAV_CMD_DO_REPOSITION to 10m N + 5m E,
samples SERVO_OUTPUT_RAW 20s, verifies lateral evidence
(s1/s2 or s3/s4 opposite-sign offsets). SKIPs cleanly without GPS."
```

---

## Task 6: GPS-gated hardware regression

**Files:** none modified.

- [ ] **Step 1: GPS check**

```bash
python bench/verify/gps_check.py --port COM10
```

Decision tree same as P2 T6: GPS healthy → run omni_guided_test; otherwise SKIP.

- [ ] **Step 2: P1 + P2 regressions still pass**

```bash
python bench/verify/omnix_mix_test.py --port COM10
python bench/verify/omni_auto_test.py --port COM10
```

`omnix_mix_test.py` must PASS — mixer must remain untouched. `omni_auto_test.py` is expected to SKIP/PASS depending on GPS.

If either fails non-GPS-relatedly → BLOCKED.

- [ ] **Step 3: omni_guided_test.py**

```bash
python bench/verify/omni_guided_test.py --port COM10
```

Acceptable outcomes:
- exit 0 (PASS) with `lateral_used = True`
- exit 2 (SKIP — no GPS)

Any other failure → DONE_WITH_CONCERNS with details.

- [ ] **Step 4: Param sanity (verify no EEPROM corruption)**

```bash
python -c "
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('COM10', baud=115200)
m.wait_heartbeat()
m.mav.param_request_list_send(m.target_system, m.target_component)
seen = {}
t0 = time.time()
while time.time() - t0 < 8:
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
    if msg and (msg.param_id.startswith('DP_') or msg.param_id.startswith('OMNI_')):
        seen[msg.param_id] = msg.param_value
print('DP_*/OMNI_* params seen:', len(seen))
"
```

Expected: 11 DP_* + 2 OMNI_* = 13 total. Same as P2 T6.

If any drift: BLOCKED.

---

## Task 7: Update memory + close P3

**Files:**
- Append to `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\omnix_dp_mode.md`
- Append to `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\ar_omnicontrol_library.md`
- Update `C:\Users\qdxia\.claude\projects\F--opensource-usv-esp32-esp32s3rover-ardupilot-rover-esp32s3-idf\memory\MEMORY.md`

Document the P3 commits, hardware regression results, and flag the duplication concern for P4 refactor.

Example content for `omnix_dp_mode.md` append:

```markdown

## P3 完成 (2026-05-XX):ModeGuided OMNIX 分支 (WP submode)

**目的**:让 GCS 发的 `MAV_CMD_DO_REPOSITION` / `SET_POSITION_TARGET_GLOBAL_INT` 在 OMNIX 帧下也走 g2.omni_ctrl,不降级。

**改动**:
- ModeGuided 加 `_omni_initial_yaw / _omni_rc_yaw_integ / _omni_active` + `update_omnix_wp()` + `compute_omnix_target_yaw()`(与 ModeAuto P2 几乎完全相同)
- `update()` SubMode::WP 加 OMNIX 分支 early-return
- `_enter()` snapshot yaw + reset omni_ctrl
- ModeGuided 没有 override `_exit()` — 下个 OMNIX 模式 `_enter()` 时自动 reset(已确认所有现有 OMNIX 消费者都会 reset)

**未覆盖的 Guided submodes**(留 P4+):
- `HeadingAndSpeed`、`TurnRateAndSpeed`:无位置目标,只有 heading+speed → 严格说不需要横移
- `SteeringAndThrottle`:GCS MANUAL_CONTROL 透传,没有 lateral 输入 → 不动
- `Loiter`:delegate 到 ModeLoiter.update() → P4 给 ModeLoiter 加 OMNIX 分支

**重要 — P3 后已知 duplication**:`compute_omnix_target_yaw` 在 ModeAuto + ModeGuided 各一份,完全相同。P4 应该做的事:抽出来当 free function 或挪进 `AR_OmniControl` API。在那之前每次加新 OMNIX 模式都得复制这个 helper。
```

- [ ] **Step 1-4: write the memory updates**

(Specific content lines provided above; adjust dates as needed)

- [ ] **Step 5: Done — no git commit required**

P3 complete. Source repo clean.

---

## Self-Review

**Spec coverage:**
- ✅ `Guided` row in spec §2 (target from GCS SET_POSITION_TARGET) — covered for SubMode::WP via wp_nav.get_destination()
- ✅ `OMNI_YAW_MODE` (spec §3) — all 4 strategies in compute_omnix_target_yaw
- ✅ Failsafe to HOLD (spec §4) — update_omnix_wp + EKF_FAILSAFE
- ⚠️ Spec §3 "GUIDED yaw priority" rule (GCS yaw overrides OMNI_YAW_MODE) — only applies to HeadingAndSpeed submode (not WP). P3 SubMode::WP destinations have no GCS-supplied yaw. Spec rule moves to P5+ when we tackle HeadingAndSpeed.
- ⚠️ Spec §6 P3 row: "ModeGuided OMNIX 分支" — covered

**Placeholder scan:** No TBDs. Every code block is concrete.

**Type/name consistency:**
- `ModeGuided::update_omnix_wp()` + `compute_omnix_target_yaw()` declared T1, defined T2/T3, called T4 — names match
- `_omni_initial_yaw`, `_omni_rc_yaw_integ`, `_omni_active` — declared T1, used T2 (yaw integ), T4 (_enter snapshot)
- `g2._omni_yaw_mode`, `g2.wp_nav`, `g2.omni_ctrl` — names match existing P1/P2 wiring
- `get_distance_NE` (not `_float`) — same name used in P2 T3 (verified API)

**Known risks:**
- `ModeGuided` has no `_exit()` override. If user switches OMNIX Guided → diff-drive mode (eg MANUAL) → another OMNIX mode, the integrator carries stale state until the next reset. Mitigation: every OMNIX consumer's `_enter()` calls `g2.omni_ctrl.reset()`. Risk: low.
- Duplicate `compute_omnix_target_yaw` between Auto+Guided — code maintenance hazard. Mitigation: factor into shared helper in P4 plan (explicitly called out).
- HeadingAndSpeed / TurnRateAndSpeed submodes stay diff-drive — OMNIX users who send those messages won't see lateral. Documented; deferred to a future enhancement.

**Out of scope (next plans):**
- P4: ModeLoiter + ModeDock OMNIX. Also factor shared compute_omnix_target_yaw helper.
- P5: ModeRTL + ModeSmartRTL OMNIX
- P6: Water testing
- OMNIX support for Guided HeadingAndSpeed (with GCS yaw priority) — deferred
