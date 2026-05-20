#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DP 模式固件验证 — 烧录后用 MAVLink 测 ModeDP。

测试项：
  1. 连接 + 抓启动 STATUSTEXT(banner / 有没有 boot loop)
  2. DP_* 参数是否齐全(11 个)
  3. 门禁:FRAME_TYPE != OMNIX 时切 mode 17 → 应被拒
  4. 设 FRAME_CLASS=2 / FRAME_TYPE=2 → 重启
  5. 重启后切 mode 17 → 应成功(custom_mode==17)
  6. DP 模式下采 SERVO_OUTPUT_RAW(看 4 路输出)
  7. 切回 MANUAL

用法：python dp_mode_test.py [--port COM12] [--baud 115200]
"""
import argparse
import sys
import time

from pymavlink import mavutil

DP_MODE = 17
MANUAL_MODE = 0
DP_PARAMS = ["DP_POS_P", "DP_POS_I", "DP_POS_D", "DP_YAW_P", "DP_YAW_I",
             "DP_YAW_D", "DP_POS_DB", "DP_YAW_DB", "DP_SPEED", "DP_IMAX",
             "DP_OPTIONS"]


def log(msg):
    print(time.strftime("[%H:%M:%S] ") + msg, flush=True)


def connect(port, baud, timeout=90):
    """连接并等心跳，板子在烧录后重启期间会重试。"""
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
            log(f"  重试中: {e}")
        time.sleep(2)
    return None


def drain_statustext(m, seconds, tag):
    """收集 STATUSTEXT，返回文本列表。"""
    texts = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        msg = m.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            t = msg.text.strip()
            texts.append(t)
            log(f"  [{tag}] {t}")
    return texts


def get_mode(m, timeout=5):
    msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
    return msg.custom_mode if msg else None


def set_mode(m, mode_num):
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        mode_num)


def get_param(m, name, timeout=8):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg and msg.param_id.strip('\x00') == name:
            return msg.param_value
    return None


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                         float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM12")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    results = {}

    # --- 1. 连接 + 启动信息 ---
    m = connect(args.port, args.baud)
    if m is None:
        log("FAIL: 连不上板子")
        sys.exit(1)
    log("--- 抓启动 STATUSTEXT (20s) ---")
    boot = drain_statustext(m, 20, "boot")
    results["boot_connect"] = True
    results["boot_ready"] = any("ArduPilot Ready" in t for t in boot)

    # --- 2. DP_* 参数齐全? ---
    log("--- 检查 DP_* 参数 ---")
    missing = []
    for p in DP_PARAMS:
        v = get_param(m, p)
        if v is None:
            missing.append(p)
            log(f"  {p}: 缺失!")
        else:
            log(f"  {p} = {v}")
    results["dp_params_complete"] = (len(missing) == 0)

    # --- 3. 门禁: 非 OMNIX 切 DP 应被拒 ---
    log("--- 门禁测试: 当前 FRAME_TYPE 下切 mode 17 ---")
    ft = get_param(m, "FRAME_TYPE")
    log(f"  当前 FRAME_TYPE = {ft}")
    if ft is not None and int(ft) != 2:
        for _ in range(5):
            m.recv_match(type='STATUSTEXT', blocking=False)
        set_mode(m, DP_MODE)
        time.sleep(2)
        txt = drain_statustext(m, 4, "gate")
        time.sleep(1)
        mode_now = get_mode(m)
        rejected = (mode_now != DP_MODE)
        gate_msg = any("DP" in t and ("OMNIX" in t or "requires" in t)
                       for t in txt)
        log(f"  切模式后 custom_mode={mode_now} (期望!=17), 拒绝={rejected}")
        results["gate_rejects_non_omnix"] = rejected
    else:
        log("  FRAME_TYPE 已是 2,跳过门禁拒绝测试")
        results["gate_rejects_non_omnix"] = None

    # --- 4. 设 OMNIX + 重启 ---
    log("--- 设 FRAME_CLASS=2 / FRAME_TYPE=2 (OMNIX) ---")
    set_param(m, "FRAME_CLASS", 2)
    set_param(m, "FRAME_TYPE", 2)
    time.sleep(1)
    log("  重启板子 ...")
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                            0, 1, 0, 0, 0, 0, 0, 0)
    m.close()
    time.sleep(8)

    # --- 5. 重连 + 切 DP 应成功 ---
    m = connect(args.port, args.baud)
    if m is None:
        log("FAIL: 重启后连不上")
        sys.exit(1)
    drain_statustext(m, 15, "reboot")
    ft = get_param(m, "FRAME_TYPE")
    log(f"  重启后 FRAME_TYPE = {ft}")
    results["frame_type_persisted"] = (ft is not None and int(ft) == 2)

    log("--- 切 mode 17 (OMNIX 下应成功) ---")
    for _ in range(5):
        m.recv_match(type='STATUSTEXT', blocking=False)
    set_mode(m, DP_MODE)
    time.sleep(2)
    txt = drain_statustext(m, 4, "dp")
    mode_now = get_mode(m)
    entered = (mode_now == DP_MODE)
    enter_msg = any("DP" in t and ("hold" in t.lower() or "position" in t)
                    for t in txt)
    log(f"  custom_mode={mode_now} (期望 17), 进入={entered}")
    results["dp_mode_enters"] = entered

    # --- 6. DP 下采 SERVO_OUTPUT_RAW ---
    if entered:
        log("--- DP 模式下 SERVO_OUTPUT_RAW (8s) ---")
        t0 = time.time()
        n = 0
        while time.time() - t0 < 8:
            so = m.recv_match(type='SERVO_OUTPUT_RAW', blocking=True,
                              timeout=2)
            if so:
                n += 1
                if n % 5 == 1:
                    log(f"  servo1-4: {so.servo1_raw} {so.servo2_raw} "
                        f"{so.servo3_raw} {so.servo4_raw}")
        results["servo_output_seen"] = (n > 0)

    # --- 7. 切回 MANUAL ---
    log("--- 切回 MANUAL ---")
    set_mode(m, MANUAL_MODE)
    time.sleep(2)
    log(f"  custom_mode={get_mode(m)}")

    # --- 汇总 ---
    log("")
    log("=" * 46)
    log("DP 模式固件验证汇总")
    log("=" * 46)
    allpass = True
    for k, v in results.items():
        mark = "PASS" if v else ("SKIP" if v is None else "FAIL")
        if v is False:
            allpass = False
        log(f"  {k:28s} {mark}")
    log("=" * 46)
    log("总体: " + ("PASS" if allpass else "FAIL"))
    m.close()
    sys.exit(0 if allpass else 1)


if __name__ == "__main__":
    main()
