#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OMNIX 4 推进器混控验证 — 配 4 电机输出后测 output_omni() 混控。

测试项：
  1. 配 SERVO1-4_FUNCTION=33/34/35/36 (OMNIX 4 电机) + FRAME=2/2 → 重启
  2. 重启后无 "unable to setup motor" 报错
  3. 逐个电机测试 (MAV_CMD_DO_MOTOR_TEST) — 4 路通道各自能动
  4. MANUAL + RC override 混控测试:
       前进 → 4 路同向
       横移 → servo1/3 与 servo2/4 反向
       转艏 → servo1/2 与 servo3/4 反向
  5. 收尾:中立 + disarm

OMNIX 混控 (AP_MotorsUGV setup_omni):
  m0=thr-str-lat  m1=thr-str+lat  m2=thr+str-lat  m3=thr+str+lat

用法：python omnix_mix_test.py [--port COM10]
"""
import argparse
import sys
import time

from pymavlink import mavutil

DO_MOTOR_TEST = mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST


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


def drain_text(m, seconds, tag):
    texts = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            texts.append(msg.text.strip())
            log(f"  [{tag}] {msg.text.strip()}")
    return texts


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.4)


def reboot(m):
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                            0, 1, 0, 0, 0, 0, 0, 0)


def read_servos(m, timeout=2):
    so = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=timeout)
    if so:
        return [so.servo1_raw, so.servo2_raw, so.servo3_raw, so.servo4_raw]
    return None


def motor_test(m, motor_num, throttle_pct=20, secs=3):
    """MAV_CMD_DO_MOTOR_TEST 单个电机,返回该窗口内 servo 最大偏移。"""
    m.mav.command_long_send(m.target_system, m.target_component,
                            DO_MOTOR_TEST, 0,
                            motor_num, 0, throttle_pct, secs, 0, 0, 0)
    peak = [0, 0, 0, 0]
    t0 = time.time()
    while time.time() - t0 < secs + 1:
        s = read_servos(m, 1)
        if s:
            for i in range(4):
                peak[i] = max(peak[i], abs(s[i] - 1500))
    return peak


def send_override(m, ch1, ch3, ch4):
    """RC override: ch1=steer ch3=throttle ch4=lateral,其余释放。"""
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    ch1, 0, ch3, ch4, 0, 0, 0, 0)


def hold(m, ch1, ch3, ch4, seconds):
    """持续 ~30Hz 发 override,返回采样到的 servo 平均值。"""
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


def is_armed(m, timeout=3):
    hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
    if not hb:
        return None
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    results = {}

    m = connect(args.port, args.baud)
    if m is None:
        log("FAIL: 连不上"); sys.exit(1)

    # --- 1. 配 OMNIX 4 电机输出 ---
    log("--- 配 SERVO1-4_FUNCTION=33/34/35/36 + FRAME 2/2 ---")
    set_param(m, "FRAME_CLASS", 2)
    set_param(m, "FRAME_TYPE", 2)
    set_param(m, "SERVO1_FUNCTION", 33)   # Motor1
    set_param(m, "SERVO2_FUNCTION", 34)   # Motor2
    set_param(m, "SERVO3_FUNCTION", 35)   # Motor3
    set_param(m, "SERVO4_FUNCTION", 36)   # Motor4
    log("  重启 ...")
    reboot(m)
    m.close()
    time.sleep(8)

    # --- 2. 重启后检查电机 setup ---
    m = connect(args.port, args.baud)
    if m is None:
        log("FAIL: 重启后连不上"); sys.exit(1)
    boot = drain_text(m, 18, "boot")
    motor_err = any("unable to setup motor" in t for t in boot)
    results["no_motor_setup_error"] = not motor_err
    log(f"  电机 setup 报错: {motor_err}")

    # --- 3. 逐个电机测试 ---
    log("--- 逐个电机测试 (DO_MOTOR_TEST 20%) ---")
    motor_ok = True
    for mn in range(1, 5):
        peak = motor_test(m, mn, 20, 3)
        moved = peak[mn - 1] > 30          # 该电机对应 servo 明显偏移
        log(f"  电机{mn}: servo偏移峰值={peak}  {'OK' if moved else '无反应'}")
        if not moved:
            motor_ok = False
        time.sleep(1)
    results["motor_test_all_respond"] = motor_ok

    # --- 4. MANUAL + RC override 混控 ---
    log("--- MANUAL 模式 + RC override 混控测试 ---")
    # 先持续发中立 override 以清除 RC failsafe
    for _ in range(60):
        send_override(m, 1500, 1500, 1500)
        time.sleep(0.02)
    m.set_mode(0)   # MANUAL
    time.sleep(1)
    # 先设 ARMING_CHECK=0 + 用 force param2=21196 解锁，边 arm 边持续发 override
    set_param(m, "ARMING_CHECK", 0)
    for _ in range(30):
        send_override(m, 1500, 1500, 1500)
        time.sleep(0.02)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 21196.0, 0, 0, 0, 0, 0)
    # 持续发 override 等待 arm 生效（不能 sleep，否则 RC failsafe 触发）
    t_arm = time.time()
    armed = False
    while time.time() - t_arm < 4:
        send_override(m, 1500, 1500, 1500)
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=0.1)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                break
        time.sleep(0.02)
    log(f"  解锁状态: {armed}")
    results["manual_armed"] = bool(armed)

    if armed:
        neutral = hold(m, 1500, 1500, 1500, 2)
        log(f"  中立:        servo1-4 = {neutral}")
        fwd = hold(m, 1500, 1750, 1500, 3)
        log(f"  前进(thr+):  servo1-4 = {fwd}")
        lat = hold(m, 1500, 1500, 1750, 3)
        log(f"  横移(lat+):  servo1-4 = {lat}")
        yawt = hold(m, 1750, 1500, 1500, 3)
        log(f"  转艏(str+):  servo1-4 = {yawt}")
        hold(m, 1500, 1500, 1500, 1)

        # 混控模式判定
        fwd_ok = lat_ok = yaw_ok = False
        if neutral and fwd and lat and yawt:
            # 前进:4 路同向(都偏离中立同号)
            d = [fwd[i] - neutral[i] for i in range(4)]
            fwd_ok = all(x > 20 for x in d) or all(x < -20 for x in d)
            # 横移:servo1,3 与 servo2,4 反向
            dl = [lat[i] - neutral[i] for i in range(4)]
            lat_ok = (dl[0] * dl[1] < 0) and (dl[2] * dl[3] < 0)
            # 转艏:servo1,2 与 servo3,4 反向
            dy = [yawt[i] - neutral[i] for i in range(4)]
            yaw_ok = (dy[0] * dy[2] < 0) and (dy[1] * dy[3] < 0)
        log(f"  前进混控={fwd_ok}  横移混控={lat_ok}  转艏混控={yaw_ok}")
        results["mix_forward"] = fwd_ok
        results["mix_lateral"] = lat_ok
        results["mix_yaw"] = yaw_ok

        # disarm
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(1)

    # 释放 override
    for _ in range(10):
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.02)

    # --- 汇总 ---
    log("")
    log("=" * 46)
    log("OMNIX 混控验证汇总")
    log("=" * 46)
    allpass = True
    for k, v in results.items():
        mark = "PASS" if v else ("SKIP" if v is None else "FAIL")
        if v is False:
            allpass = False
        log(f"  {k:26s} {mark}")
    log("=" * 46)
    log("总体: " + ("PASS" if allpass else "FAIL"))
    m.close()
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
