#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DP 解锁诊断 — 查 DP 模式下 force-arm 为何失败,并在解锁后观察 DP 驱动电机。

策略:进 DP 前后持续发 RC override(压 RC failsafe),force-arm 后读
COMMAND_ACK 拿确切结果码。

用法：python dp_arm_diag.py [--port COM10]
"""
import argparse
import sys
import time

from pymavlink import mavutil

ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
FORCE = 21196.0

ACK_RESULT = {
    0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
    3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS",
}


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


def set_mode(m, n):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n)


def hb(m, timeout=4):
    return m.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)


def override(m, ch1=1500, ch3=1500, ch4=1500):
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    ch1, 0, ch3, ch4, 0, 0, 0, 0)


def stream_override(m, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        override(m)
        time.sleep(0.025)


def arm_with_ack(m, force=True):
    """发 arm 命令,收集 STATUSTEXT + COMMAND_ACK,返回 (armed, ack, texts)."""
    # 清空缓冲
    for _ in range(20):
        m.recv_match(blocking=False)
    m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                            1, FORCE if force else 0, 0, 0, 0, 0, 0)
    ack = None
    texts = []
    t0 = time.time()
    while time.time() - t0 < 4:
        override(m)
        msg = m.recv_match(type=['COMMAND_ACK', 'STATUSTEXT'],
                           blocking=True, timeout=0.3)
        if msg is None:
            continue
        if msg.get_type() == 'COMMAND_ACK' and msg.command == ARM:
            ack = msg.result
        elif msg.get_type() == 'STATUSTEXT':
            texts.append(msg.text.strip())
    h = hb(m)
    armed = bool(h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) \
        if h else None
    return armed, ack, texts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    m = connect(args.port, args.baud)
    if m is None:
        log("FAIL: 连不上"); sys.exit(1)

    set_param(m, "ARMING_CHECK", 0)
    set_param(m, "DP_POS_DB", 0)

    # 进 DP 前先压住 RC failsafe
    log("--- 进 DP 前预热 RC override 3s ---")
    stream_override(m, 3)

    log("--- 切 DP (mode 17) ---")
    set_mode(m, 17)
    stream_override(m, 2)
    h = hb(m)
    log(f"  custom_mode = {h.custom_mode if h else '?'}")

    log("--- DP 下 force-arm (带 override + ACK) ---")
    armed, ack, texts = arm_with_ack(m, force=True)
    for t in texts:
        log(f"  [arm] {t}")
    ack_s = ACK_RESULT.get(ack, str(ack))
    log(f"  COMMAND_ACK = {ack_s}")
    log(f"  armed = {armed}")

    if not armed:
        # 再试:普通 arm(非 force)看 ACK 区别
        log("--- 对照:普通 arm(非 force)---")
        armed2, ack2, texts2 = arm_with_ack(m, force=False)
        for t in texts2:
            log(f"  [arm2] {t}")
        log(f"  COMMAND_ACK = {ACK_RESULT.get(ack2, str(ack2))}  armed={armed2}")

    if armed:
        log("--- DP 已解锁,观察 DP 驱动 SERVO_OUTPUT_RAW (15s) ---")
        seen = []
        t0 = time.time()
        while time.time() - t0 < 15:
            override(m)
            so = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True,
                              timeout=0.3)
            if so:
                s = [so.servo1_raw, so.servo2_raw, so.servo3_raw,
                     so.servo4_raw]
                seen.append(s)
                if len(seen) % 8 == 1:
                    log(f"    servo1-4 = {s}")
        moved = any(abs(v - 1500) > 5 for s in seen for v in s)
        log(f"  DP 驱动电机有输出: {moved}  (样本 {len(seen)})")
        # disarm
        m.mav.command_long_send(m.target_system, m.target_component, ARM, 0,
                                0, 0, 0, 0, 0, 0, 0)
        time.sleep(1)

    set_param(m, "DP_POS_DB", 1.5)
    set_mode(m, 0)
    for _ in range(10):
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.02)
    log("诊断结束")
    m.close()


if __name__ == "__main__":
    main()
