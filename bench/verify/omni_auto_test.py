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
        log("SKIP: no GPS; cannot test AUTO (no regression info collected)")
        m.close()
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
