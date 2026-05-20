#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DP 最终带载验证 — override 全程不断 + DP 进入重试,确保真进 DP 再 force-arm,
观察 DP 控制律实际驱动 4 电机。

用法：python dp_final_test.py [--port COM10]
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
    """跑 `seconds` 秒,期间持续发 override,返回收到的指定类型消息列表。"""
    got = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        override(m)
        msg = m.recv_match(blocking=True, timeout=0.05)
        if msg and want_type and msg.get_type() == want_type:
            got.append(msg)
        time.sleep(0.02)
    return got


def cur_mode(m):
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    return hb.custom_mode if hb else None


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

    set_param(m, "ARMING_CHECK", 0)
    set_param(m, "DP_POS_DB", 0)      # 关死区,让 DP 对 GPS 微漂有反应

    # 等 EKF 充分稳定(边等边发 override 压 RC failsafe)
    log("--- 等 EKF 稳定 (40s) ---")
    pump(m, 40)

    # --- DP 进入重试 ---
    log("--- 进 DP (mode 17),最多重试 6 次 ---")
    entered = False
    for i in range(6):
        set_mode(m, 17)
        pump(m, 3)
        mode = cur_mode(m)
        log(f"  第{i+1}次: custom_mode={mode}")
        if mode == 17:
            entered = True
            break
        pump(m, 3)   # 等 EKF 再稳一会儿
    results["dp_entered"] = entered

    if entered:
        # --- force-arm(override 持续)---
        log("--- DP 下 force-arm ---")
        for _ in range(10):
            m.recv_match(blocking=False)
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                1, FORCE, 0, 0, 0, 0, 0)
        acks = pump(m, 4, want_type='COMMAND_ACK')
        arm_ack = next((a.result for a in acks if a.command == ARM), None)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) \
            if hb else False
        log(f"  arm ACK={arm_ack}  armed={armed}  mode={hb.custom_mode if hb else '?'}")
        results["dp_armed"] = armed

        if armed:
            # --- 观察 DP 驱动电机 ---
            log("--- DP 驱动 SERVO_OUTPUT_RAW (20s, DP_POS_DB=0) ---")
            servos = pump(m, 20, want_type='SERVO_OUTPUT_RAW')
            vals = [[s.servo1_raw, s.servo2_raw, s.servo3_raw, s.servo4_raw]
                    for s in servos]
            for k in range(0, len(vals), max(1, len(vals)//8)):
                log(f"    {vals[k]}")
            moved = any(abs(v - 1500) > 5 for s in vals for v in s)
            log(f"  DP 驱动电机有输出变化: {moved}  (样本 {len(vals)})")
            results["dp_drives_motors"] = moved
            # 还在 DP 吗
            still = cur_mode(m)
            log(f"  观察后仍在模式: {still} (DP=17)")
            results["dp_stable_armed"] = (still == 17)
            # disarm
            m.mav.command_long_send(m.target_system, m.target_component,
                                    ARM, 0, 0, 0, 0, 0, 0, 0, 0)
            pump(m, 2)

    # 收尾
    set_param(m, "DP_POS_DB", 1.5)
    set_mode(m, 0)
    for _ in range(10):
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.02)

    log("")
    log("=" * 46)
    log("DP 最终带载验证汇总")
    log("=" * 46)
    allpass = True
    for k, v in results.items():
        mark = "PASS" if v else ("SKIP" if v is None else "FAIL")
        if v is False:
            allpass = False
        log(f"  {k:22s} {mark}")
    log("=" * 46)
    log("总体: " + ("PASS" if allpass else "FAIL"))
    m.close()
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
