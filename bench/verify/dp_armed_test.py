#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DP 带载验证 — force-arm 后测 OMNIX 混控 + DP 实际驱动电机。

桌面无 SBUS 接收机,常规解锁被 RC failsafe 挡 → 用 force-arm (param2=21196)。

测试项：
  1. MANUAL force-arm → RC override 测前进/横移/转艏混控
  2. DP 模式 force-arm + DP_POS_DB=0 → 观察 DP 控制律实际驱动 4 电机
  3. 收尾:disarm + 恢复 DP_POS_DB

OMNIX 混控:m0=thr-str-lat  m1=thr-str+lat  m2=thr+str-lat  m3=thr+str+lat
用法：python dp_armed_test.py [--port COM10]
"""
import argparse
import sys
import time

from pymavlink import mavutil

ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
FORCE = 21196.0   # force-arm magic (bypass checks + failsafe)


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


def get_param(m, name, timeout=6):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id.strip('\x00') == name:
            return msg.param_value
    return None


def read_servos(m, timeout=1):
    so = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=timeout)
    if so:
        return [so.servo1_raw, so.servo2_raw, so.servo3_raw, so.servo4_raw]
    return None


def set_mode(m, n):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)


def get_mode(m, timeout=4):
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
    return hb.custom_mode if hb else None


def is_armed(m, timeout=4):
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
    if not hb:
        return None
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def force_arm(m, arm=True):
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            1 if arm else 0, FORCE if arm else 0,
                            0, 0, 0, 0, 0)
    time.sleep(2)
    return is_armed(m)


def send_override(m, ch1, ch3, ch4):
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    ch1, 0, ch3, ch4, 0, 0, 0, 0)


def hold(m, ch1, ch3, ch4, seconds):
    samples = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        send_override(m, ch1, ch3, ch4)
        s = read_servos(m, 0.03)
        if s:
            samples.append(s)
        time.sleep(0.01)
    if not samples:
        return None
    n = len(samples)
    return [round(sum(x[i] for x in samples) / n) for i in range(4)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    results = {}

    m = connect(args.port, args.baud)
    if m is None:
        log("FAIL: 连不上"); sys.exit(1)

    # 确认 OMNIX 配置还在(上轮测试已设并持久化)
    ft = get_param(m, "FRAME_TYPE")
    s1 = get_param(m, "SERVO1_FUNCTION")
    log(f"FRAME_TYPE={ft}  SERVO1_FUNCTION={s1}")
    set_param(m, "ARMING_CHECK", 0)

    # ===== 测试 1:MANUAL force-arm + 混控 =====
    log("--- 测试1: MANUAL force-arm + OMNIX 混控 ---")
    set_mode(m, 0)               # MANUAL
    time.sleep(1)
    for _ in range(40):          # 预热 override
        send_override(m, 1500, 1500, 1500)
        time.sleep(0.02)
    armed = force_arm(m, True)
    log(f"  MANUAL force-arm: {armed}")
    results["manual_force_arm"] = bool(armed)

    if armed:
        neutral = hold(m, 1500, 1500, 1500, 2)
        fwd = hold(m, 1500, 1750, 1500, 3)
        lat = hold(m, 1500, 1500, 1750, 3)
        yawt = hold(m, 1750, 1500, 1500, 3)
        hold(m, 1500, 1500, 1500, 1)
        log(f"  中立        servo1-4 = {neutral}")
        log(f"  前进(thr+)  servo1-4 = {fwd}")
        log(f"  横移(lat+)  servo1-4 = {lat}")
        log(f"  转艏(str+)  servo1-4 = {yawt}")
        fwd_ok = lat_ok = yaw_ok = False
        if neutral and fwd and lat and yawt:
            d = [fwd[i] - neutral[i] for i in range(4)]
            fwd_ok = all(x > 20 for x in d) or all(x < -20 for x in d)
            dl = [lat[i] - neutral[i] for i in range(4)]
            lat_ok = (dl[0] * dl[1] < 0) and (dl[2] * dl[3] < 0)
            dy = [yawt[i] - neutral[i] for i in range(4)]
            yaw_ok = (dy[0] * dy[2] < 0) and (dy[1] * dy[3] < 0)
        log(f"  前进混控={fwd_ok}  横移混控={lat_ok}  转艏混控={yaw_ok}")
        results["mix_forward"] = fwd_ok
        results["mix_lateral"] = lat_ok
        results["mix_yaw"] = yaw_ok
        force_arm(m, False)
        log("  已 disarm")

    # 释放 override
    for _ in range(10):
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.02)
    time.sleep(1)

    # ===== 测试 2:DP 带载 — DP 控制律实际驱动电机 =====
    log("--- 测试2: DP 模式 force-arm + DP_POS_DB=0 ---")
    db_orig = get_param(m, "DP_POS_DB")
    set_param(m, "DP_POS_DB", 0)      # 关死区,让 DP 对 GPS 微漂有反应
    set_mode(m, 17)                    # DP
    time.sleep(2)
    mode_now = get_mode(m)
    log(f"  custom_mode={mode_now}")
    results["dp_entered"] = (mode_now == 17)

    if mode_now == 17:
        armed = force_arm(m, True)
        log(f"  DP force-arm: {armed}")
        results["dp_force_arm"] = bool(armed)
        if armed:
            log("  采 DP 驱动下的 SERVO_OUTPUT_RAW (15s) ...")
            seen = []
            t0 = time.time()
            while time.time() - t0 < 15:
                s = read_servos(m, 1)
                if s:
                    seen.append(s)
                    if len(seen) % 4 == 1:
                        log(f"    servo1-4 = {s}")
            # DP 是否真的在动电机(任一通道偏离过 1500)
            moved = any(abs(v - 1500) > 5 for s in seen for v in s)
            log(f"  DP 驱动电机有输出变化: {moved}  (样本数 {len(seen)})")
            results["dp_drives_motors"] = moved
            force_arm(m, False)
            log("  已 disarm")

    # 收尾
    if db_orig is not None:
        set_param(m, "DP_POS_DB", db_orig)
        log(f"  恢复 DP_POS_DB={db_orig}")
    set_mode(m, 0)

    log("")
    log("=" * 46)
    log("DP 带载验证汇总")
    log("=" * 46)
    allpass = True
    for k, v in results.items():
        mark = "PASS" if v else ("SKIP" if v is None else "FAIL")
        if v is False:
            allpass = False
        log(f"  {k:24s} {mark}")
    log("=" * 46)
    log("总体: " + ("PASS" if allpass else "FAIL"))
    m.close()
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
