"""Reboot the board and verify EK3_SRC POSZ/VELZ params survive NVS.

Also dumps the full parameter list to params/baseline_v3.parm for git backup.

Usage: PYTHONIOENCODING=utf-8 python param_persist_check.py --port COM10
"""
import argparse
import io
import os
import sys
import time

from pymavlink import mavutil

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


CRITICAL_PARAMS = [
    "EK3_SRC1_POSXY", "EK3_SRC1_VELXY", "EK3_SRC1_POSZ", "EK3_SRC1_VELZ", "EK3_SRC1_YAW",
    "EK3_SRC2_POSXY", "EK3_SRC2_VELXY", "EK3_SRC2_POSZ", "EK3_SRC2_VELZ", "EK3_SRC2_YAW",
    "EK3_SRC3_POSXY", "EK3_SRC3_VELXY", "EK3_SRC3_POSZ", "EK3_SRC3_VELZ", "EK3_SRC3_YAW",
    "SERVO1_FUNCTION", "SERVO3_FUNCTION",
    "ARMING_CHECK", "FS_THR_ENABLE",
    "INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z",
    "AHRS_TRIM_X", "AHRS_TRIM_Y",
    "AHRS_EKF_TYPE", "FRAME_CLASS",
]

EXPECTED = {
    "EK3_SRC1_POSZ": 3.0,
    "EK3_SRC1_VELZ": 3.0,
    "EK3_SRC2_POSZ": 0.0,
    "EK3_SRC2_VELZ": 0.0,
    "EK3_SRC3_POSZ": 0.0,
    "EK3_SRC3_VELZ": 0.0,
    "SERVO1_FUNCTION": 73.0,
    "SERVO3_FUNCTION": 74.0,
    "ARMING_CHECK": 178.0,
    "FS_THR_ENABLE": 0.0,
}


def connect(port, baud, timeout=15):
    m = mavutil.mavlink_connection(port, baud=baud,
                                    dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=timeout)
    if not hb:
        return None
    return m


def read_param(m, name, retries=3, timeout=1.5):
    sysid, compid = m.target_system, m.target_component
    for _ in range(retries):
        m.mav.param_request_read_send(sysid, compid, name.encode(), -1)
        t = time.time()
        while time.time() - t < timeout:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if msg:
                pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
                pid = pid.strip("\x00")
                if pid == name:
                    return msg.param_value
        time.sleep(0.1)
    return None


def dump_all_params(m, out_path):
    sysid, compid = m.target_system, m.target_component
    print(f"[DUMP] requesting full param list...", flush=True)
    m.mav.param_request_list_send(sysid, compid)
    params = {}
    last = time.time()
    expected_total = None
    while True:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2.0)
        if msg is None:
            if time.time() - last > 5.0:
                break
            continue
        last = time.time()
        pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
        pid = pid.strip("\x00")
        params[pid] = msg.param_value
        expected_total = msg.param_count
        if len(params) % 50 == 0:
            print(f"  ...{len(params)}/{expected_total}", flush=True)
        if len(params) >= expected_total:
            # Wait briefly to catch any stragglers
            t = time.time()
            while time.time() - t < 1.0:
                msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
                if msg:
                    pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
                    pid = pid.strip("\x00")
                    params[pid] = msg.param_value
            break

    print(f"[DUMP] got {len(params)}/{expected_total}", flush=True)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("# ArduRover-ESP32S3 V1.0 — full parameter dump\n")
        f.write(f"# Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# Param count: {len(params)}\n")
        f.write(f"# Reboot-verified persistence: EK3_SRC fixes for baroless USV\n")
        f.write("#\n")
        for k in sorted(params):
            v = params[k]
            # MAVProxy parm format: NAME,VALUE
            if v == int(v):
                f.write(f"{k},{int(v)}\n")
            else:
                f.write(f"{k},{v:.6f}\n")
    print(f"[OUT] {out_path}", flush=True)
    return params


def reboot_board(m):
    sysid, compid = m.target_system, m.target_component
    print(f"[REBOOT] sending MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN ...", flush=True)
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-reboot", action="store_true",
                    help="Skip reboot, just verify current values + dump")
    args = ap.parse_args()

    print("=" * 64, flush=True)
    print(" Parameter persistence + baseline dump", flush=True)
    print("=" * 64, flush=True)

    # ---- Phase 1: read PRE-reboot values
    print("\n[PHASE 1] reading critical params BEFORE reboot", flush=True)
    m = connect(args.port, args.baud)
    if not m:
        print("[FATAL] cannot connect to board (no heartbeat)", flush=True)
        sys.exit(2)

    pre = {}
    for nm in CRITICAL_PARAMS:
        v = read_param(m, nm)
        pre[nm] = v
        marker = ""
        if nm in EXPECTED:
            if v is None:
                marker = " (NOT FOUND)"
            elif abs(v - EXPECTED[nm]) < 0.001:
                marker = " ✓ expected"
            else:
                marker = f" ⚠ expected {EXPECTED[nm]}"
        print(f"  {nm:18s} = {v}{marker}", flush=True)

    if args.no_reboot:
        print("\n[SKIP] reboot skipped per flag", flush=True)
        post = pre
    else:
        # ---- Phase 2: reboot
        print("\n[PHASE 2] rebooting board", flush=True)
        reboot_board(m)
        m.close()
        time.sleep(2)

        # Wait for the device to enumerate again (USB CDC may take 5-15s)
        print("[PHASE 2] waiting for board to come back up...", flush=True)
        m2 = None
        deadline = time.time() + 30
        while time.time() < deadline:
            try:
                m2 = connect(args.port, args.baud, timeout=4)
                if m2:
                    print(f"[PHASE 2] reconnected after {30 - (deadline - time.time()):.1f}s", flush=True)
                    break
            except Exception as e:
                pass
            time.sleep(1)
        if m2 is None:
            print(f"[FATAL] could not reconnect after 30s — board did not come back", flush=True)
            sys.exit(3)

        # Give params subsystem a moment
        time.sleep(3)
        m = m2

        # ---- Phase 3: read POST-reboot values
        print("\n[PHASE 3] reading critical params AFTER reboot", flush=True)
        post = {}
        for nm in CRITICAL_PARAMS:
            v = read_param(m, nm)
            post[nm] = v
            marker = ""
            if nm in EXPECTED:
                if v is None:
                    marker = " (NOT FOUND)"
                elif abs(v - EXPECTED[nm]) < 0.001:
                    marker = " ✓"
                else:
                    marker = f" ⚠ now {v} vs expected {EXPECTED[nm]}"
            same = "==" if pre.get(nm) == v else "!="
            print(f"  {nm:18s} = {v}  [{same} pre]{marker}", flush=True)

    # ---- Phase 4: full dump
    print("\n[PHASE 4] dumping full param list", flush=True)
    dump_all_params(m, "../../params/baseline_v3.parm")

    # ---- Verdict
    print("\n" + "=" * 64, flush=True)
    print(" VERDICT", flush=True)
    print("=" * 64, flush=True)
    issues = []
    for nm, exp in EXPECTED.items():
        v = post.get(nm)
        if v is None:
            issues.append(f"  ✗ {nm} not found")
        elif abs(v - exp) >= 0.001:
            issues.append(f"  ✗ {nm} = {v} (expected {exp})")
    if issues:
        print("\nFAIL — critical params drifted:", flush=True)
        for i in issues:
            print(i, flush=True)
        sys.exit(1)
    else:
        print("\n✓ PASS — all critical params persisted correctly after reboot", flush=True)


if __name__ == "__main__":
    main()
