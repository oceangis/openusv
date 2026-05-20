#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
等 GPS 恢复后自动跑 DP 带载测试。

轮询 GPS,一旦拿到 3D fix(>=6 星)就执行:进 DP → force-arm → 观察电机。
GPS 一直不恢复则超时报告。

用法：python dp_wait_and_test.py [--port COM10] [--wait-min 9]
"""
import argparse
import sys
import time

from pymavlink import mavutil

ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
FORCE = 21196.0


def log(msg):
    print(time.strftime("[%H:%M:%S] ") + msg, flush=True)


def connect(port, baud, timeout=60):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            m = mavutil.mavlink_connection(port, baud=baud)
            if m.wait_heartbeat(timeout=8):
                return m
            m.close()
        except Exception:
            pass
        time.sleep(2)
    return None


def gps_status(m):
    """返回 (fix_type, sats) 或 None。"""
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                                   3, 1)
    t0 = time.time()
    while time.time() - t0 < 6:
        msg = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if msg:
            return msg.fix_type, msg.satellites_visible
    return None


def override(m):
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    1500, 0, 1500, 1500, 0, 0, 0, 0)


def pump(m, seconds, want=None):
    got = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        override(m)
        msg = m.recv_match(blocking=True, timeout=0.05)
        if msg and want and msg.get_type() == want:
            got.append(msg)
        time.sleep(0.02)
    return got


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.4)


def set_mode(m, n):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)


def cur_mode(m):
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    return hb.custom_mode if hb else None


def run_dp_test(m):
    """GPS 就绪后跑 DP 带载测试。"""
    results = {}
    set_param(m, "ARMING_CHECK", 0)
    set_param(m, "DP_POS_DB", 0)
    pump(m, 5)

    log("进 DP,重试 6 次 ...")
    entered = False
    for i in range(6):
        set_mode(m, 17)
        pump(m, 3)
        mode = cur_mode(m)
        log(f"  第{i+1}次 custom_mode={mode}")
        if mode == 17:
            entered = True
            break
        pump(m, 3)
    results["dp_entered"] = entered

    if entered:
        for _ in range(10):
            m.recv_match(blocking=False)
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                1, FORCE, 0, 0, 0, 0, 0)
        acks = pump(m, 4, want='COMMAND_ACK')
        ack = next((a.result for a in acks if a.command == ARM), None)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) \
            if hb else False
        log(f"  force-arm ACK={ack} armed={armed}")
        results["dp_armed"] = armed
        if armed:
            log("观察 DP 驱动 SERVO_OUTPUT_RAW (20s) ...")
            so = pump(m, 20, want='SERVO_OUTPUT_RAW')
            vals = [[s.servo1_raw, s.servo2_raw, s.servo3_raw, s.servo4_raw]
                    for s in so]
            for k in range(0, len(vals), max(1, len(vals) // 8)):
                log(f"    {vals[k]}")
            moved = any(abs(v - 1500) > 5 for s in vals for v in s)
            log(f"  DP 驱动电机有输出: {moved} (样本 {len(vals)})")
            results["dp_drives_motors"] = moved
            results["dp_stable"] = (cur_mode(m) == 17)
            m.mav.command_long_send(m.target_system, m.target_component,
                                    ARM, 0, 0, 0, 0, 0, 0, 0, 0)
            pump(m, 2)

    set_param(m, "DP_POS_DB", 1.5)
    set_mode(m, 0)
    for _ in range(10):
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.02)

    log("=" * 44)
    log("DP 带载测试结果")
    allpass = True
    for k, v in results.items():
        mark = "PASS" if v else ("SKIP" if v is None else "FAIL")
        if v is False:
            allpass = False
        log(f"  {k:20s} {mark}")
    log("总体: " + ("PASS" if allpass else "FAIL"))
    return allpass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wait-min", type=float, default=9.0)
    args = ap.parse_args()

    deadline = time.time() + args.wait_min * 60
    log(f"等 GPS 恢复(最多 {args.wait_min:.0f} 分钟)...")
    while time.time() < deadline:
        m = connect(args.port, args.baud, timeout=30)
        if m is None:
            log("  连不上,20s 后重试")
            time.sleep(20)
            continue
        gs = gps_status(m)
        if gs is None:
            log("  没收到 GPS_RAW_INT")
        else:
            ft, sats = gs
            log(f"  GPS fix_type={ft} sats={sats}")
            if ft >= 3 and sats >= 6:
                log("GPS 已恢复 3D fix —— 开跑 DP 带载测试")
                ok = run_dp_test(m)
                m.close()
                sys.exit(0 if ok else 1)
        m.close()
        time.sleep(20)

    log(f"超时:{args.wait_min:.0f} 分钟内 GPS 未恢复 3D fix")
    log("DP 带载测试待 GPS 恢复后再跑(把 GPS 天线移到能看见天空处)")
    sys.exit(2)


if __name__ == "__main__":
    main()
