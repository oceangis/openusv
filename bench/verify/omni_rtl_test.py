#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OMNIX RTL 模式回归测试 — 验证 ModeRTL::update_omnix_wp() 使用横移。

测试流程:
  1. 配 OMNIX 帧 + OMNI_YAW_MODE=1 (TANGENT) + ARMING_CHECK=0
  2. 等 GPS lock
  3. force-arm 设 home(EKF 自动捕获当前位置为 home)
  4. 用 MAV_CMD_DO_SET_HOME 把 home 挪到 10m N + 5m E
  5. 切 RTL (mode 11), boat 'return' 到偏移 home
  6. 采样 SERVO_OUTPUT_RAW 20s
  7. 验证: lateral_evidence (s1/s2 或 s3/s4 反向偏移)

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

    # Force-arm to capture initial home (EKF sets home to current pos automatically)
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            1, FORCE, 0, 0, 0, 0, 0)
    pump(m, 4)

    # Move home 10m N + 5m E so RTL has a clear destination
    dlat = 10.0 / 111320.0
    dlon = 5.0 / (111320.0 * max(0.1, math.cos(math.radians(loc[0]))))
    home_lat = loc[0] + dlat
    home_lon = loc[1] + dlon
    log(f"  setting home to: lat={home_lat:.7f} lon={home_lon:.7f}")
    set_home(m, home_lat, home_lon, 0.0)
    pump(m, 2)

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
