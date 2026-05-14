"""Factory-reset NVS and verify code-level default override actually kicked in.

This is the only way to prove that the `AP_Param::set_default_by_name()` calls
we added to `Rover/Parameters.cpp` work, because while NVS holds user-saved
values those values always win over compile-time defaults.

Procedure:
  1. SNAPSHOT current critical params (for "expected after restore" check)
  2. Verify baseline_v3.parm exists (we'll need it to recover user-saved
     calibration after the reset wipes NVS)
  3. Send MAV_CMD_PREFLIGHT_STORAGE param1=2 (reset all params to default)
  4. Reboot the board
  5. Reconnect and read EK3_SRC{1,2,3}_POSZ — these come from CODE DEFAULTS
     now (Parameters.cpp set_default_by_name); they MUST be 3,0,0 not the
     stock 1,1,1
  6. Restore user values from baseline_v3.parm via PARAM_SET
  7. Verify INS_ACCOFFS_X is back to its calibrated value

Run with:  PYTHONIOENCODING=utf-8 python factory_reset_verify.py --port COM10
Add --no-restore if you only want the reset+verify and will load the baseline
manually via MAVProxy.
"""
import argparse
import os
import sys
import time

from pymavlink import mavutil

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

BASELINE_PATH = "../../params/baseline_v3.parm"

# What we expect AFTER factory reset, BEFORE restoring the baseline.
# These values come from the code-level set_default_by_name() override.
EXPECTED_AFTER_RESET = {
    "EK3_SRC1_POSZ": 3.0,  # set by our override (was 1=BARO in upstream)
    "EK3_SRC2_POSZ": 0.0,  # set by our override (was 1=BARO in upstream)
    "EK3_SRC3_POSZ": 0.0,  # set by our override (was 1=BARO in upstream)
}

# What MUST be back after restoring baseline_v3.parm
EXPECTED_AFTER_RESTORE = {
    "EK3_SRC1_POSZ": 3.0,
    "EK3_SRC2_POSZ": 0.0,
    "EK3_SRC3_POSZ": 0.0,
    "SERVO1_FUNCTION": 73.0,
    "SERVO3_FUNCTION": 74.0,
    "ARMING_CHECK": 178.0,
    "FS_THR_ENABLE": 0.0,
}

INS_PARAMS_TO_RESTORE = [
    "INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z",
    "INS_ACCSCAL_X", "INS_ACCSCAL_Y", "INS_ACCSCAL_Z",
    "INS_GYROFFS_X", "INS_GYROFFS_Y", "INS_GYROFFS_Z",
    "AHRS_TRIM_X", "AHRS_TRIM_Y",
]


def banner(t):
    print("\n" + "=" * 64, flush=True)
    print(f"  {t}", flush=True)
    print("=" * 64, flush=True)


def connect(port, baud, timeout=15):
    m = mavutil.mavlink_connection(port, baud=baud,
                                    dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=timeout)
    if not hb:
        return None
    return m


def reconnect(port, baud, max_wait=45):
    print(f"[RECONNECT] waiting up to {max_wait}s for board to come back...", flush=True)
    deadline = time.time() + max_wait
    while time.time() < deadline:
        try:
            m = connect(port, baud, timeout=4)
            if m:
                print(f"[RECONNECT] OK after {max_wait - (deadline - time.time()):.1f}s", flush=True)
                return m
        except Exception:
            pass
        time.sleep(1)
    return None


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


def write_param(m, name, value):
    sysid, compid = m.target_system, m.target_component
    m.mav.param_set_send(sysid, compid, name.encode(), float(value),
                          mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t = time.time()
    while time.time() - t < 2.0:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if msg:
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            pid = pid.strip("\x00")
            if pid == name:
                return msg.param_value
    return None


def reset_all_params(m):
    sysid, compid = m.target_system, m.target_component
    # MAV_CMD_PREFLIGHT_STORAGE: param1=2 = reset parameters to factory defaults
    # ArduPilot reads param1 directly; values 1=write, 2=reset, 0=read
    print("[RESET] sending MAV_CMD_PREFLIGHT_STORAGE param1=2 (reset params)", flush=True)
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0,
        2,    # param1: reset to defaults
        0, 0, 0, 0, 0, 0)
    t = time.time()
    while time.time() - t < 5:
        msg = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.3)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
            r = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result, "?").name
            print(f"[RESET] ack: {r}", flush=True)
            return r
    return "no_ack"


def reboot(m):
    sysid, compid = m.target_system, m.target_component
    print("[REBOOT] sending PREFLIGHT_REBOOT_SHUTDOWN", flush=True)
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(0.3)
    m.close()


def parse_parm_file(path):
    """Parse MAVProxy .parm file into {name: value}."""
    params = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "," in line:
                k, v = line.split(",", 1)
            else:
                parts = line.split()
                if len(parts) != 2:
                    continue
                k, v = parts
            try:
                params[k.strip()] = float(v.strip())
            except ValueError:
                pass
    return params


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-restore", action="store_true",
                    help="Skip the post-reset baseline restore (manual recovery needed)")
    ap.add_argument("--baseline", default=BASELINE_PATH)
    args = ap.parse_args()

    banner("Pre-flight checks")
    if not os.path.exists(args.baseline):
        print(f"[FATAL] baseline file not found: {args.baseline}", flush=True)
        print("        Cannot safely run factory reset without recovery file.", flush=True)
        sys.exit(2)
    baseline_params = parse_parm_file(args.baseline)
    print(f"[OK] baseline has {len(baseline_params)} params loaded from {args.baseline}", flush=True)

    banner("Connect")
    m = connect(args.port, args.baud)
    if m is None:
        print("[FATAL] cannot connect", flush=True); sys.exit(2)

    # PRE snapshot
    banner("Phase 1 — current values (pre-reset)")
    pre = {}
    for nm in list(EXPECTED_AFTER_RESET.keys()) + INS_PARAMS_TO_RESTORE[:3]:
        v = read_param(m, nm)
        pre[nm] = v
        print(f"  {nm:18s} = {v}", flush=True)

    banner("Phase 2 — factory reset NVS")
    ack = reset_all_params(m)
    if ack != "MAV_RESULT_ACCEPTED":
        print(f"[FATAL] reset not accepted (ack={ack})", flush=True)
        sys.exit(3)

    # Some firmware requires a reboot after reset to apply
    time.sleep(0.5)
    reboot(m)
    time.sleep(2)
    m = reconnect(args.port, args.baud, max_wait=45)
    if m is None:
        print("[FATAL] board did not come back after reset+reboot", flush=True)
        sys.exit(3)
    time.sleep(3)  # let param subsystem settle

    banner("Phase 3 — read post-reset values (should be code defaults)")
    post = {}
    issues = []
    for nm, exp in EXPECTED_AFTER_RESET.items():
        v = read_param(m, nm)
        post[nm] = v
        mark = ""
        if v is None:
            mark = " ✗ NOT FOUND"
            issues.append((nm, v, exp))
        elif abs(v - exp) < 0.001:
            mark = f" ✓ matches code-default override ({exp})"
        else:
            mark = f" ✗ got {v}, expected {exp} (override didn't kick in!)"
            issues.append((nm, v, exp))
        print(f"  {nm:18s} = {v}{mark}", flush=True)

    # Also check that INS calibration WAS wiped — sanity check that reset worked
    v_acc = read_param(m, "INS_ACCOFFS_X")
    print(f"  INS_ACCOFFS_X      = {v_acc}  ({'✓ wiped' if v_acc is not None and abs(v_acc) < 0.001 else '⚠ still has value — reset may not have run'})", flush=True)
    post["INS_ACCOFFS_X"] = v_acc

    if issues:
        print("\n" + "=" * 64, flush=True)
        print(" ✗ FAIL — code-default override did NOT take effect", flush=True)
        print("=" * 64, flush=True)
        for nm, got, exp in issues:
            print(f"  {nm}: got {got}, expected {exp}", flush=True)
        # Still try to restore so we don't leave the board in a bad state
    else:
        print("\n" + "=" * 64, flush=True)
        print(" ✓ PHASE 3 PASS — code-default override mechanism WORKS", flush=True)
        print("=" * 64, flush=True)

    if args.no_restore:
        print("\n[SKIP] not restoring baseline (use --no-restore flag was set).", flush=True)
        print(f"       Manually: mavproxy.py --master={args.port} --baudrate={args.baud}", flush=True)
        print(f"       Then:    > param load {args.baseline}", flush=True)
        return

    banner("Phase 4 — restore baseline_v3.parm via PARAM_SET")
    sent = 0
    failed = []
    t0 = time.time()
    for nm, v in baseline_params.items():
        # skip params with "READ ONLY" or special prefixes we shouldn't touch
        if nm.startswith("STAT_"):  # statistics, autopilot tracks them itself
            continue
        result = write_param(m, nm, v)
        if result is None or abs(result - v) > 0.01:
            failed.append((nm, v, result))
        sent += 1
        if sent % 50 == 0:
            print(f"  ...sent {sent}/{len(baseline_params)} (elapsed {time.time()-t0:.0f}s)",
                  flush=True)
    print(f"[RESTORE] sent {sent} params in {time.time()-t0:.1f}s. Failed: {len(failed)}",
          flush=True)
    if failed[:10]:
        for nm, v, result in failed[:10]:
            print(f"  ⚠ {nm}: tried {v}, got {result}", flush=True)

    # Verify restore
    banner("Phase 5 — verify restore")
    final_issues = []
    for nm, exp in EXPECTED_AFTER_RESTORE.items():
        v = read_param(m, nm)
        ok = v is not None and abs(v - exp) < 0.001
        mark = "✓" if ok else "✗"
        print(f"  {mark} {nm:18s} = {v} (expected {exp})", flush=True)
        if not ok:
            final_issues.append((nm, v, exp))

    v_acc = read_param(m, "INS_ACCOFFS_X")
    expected_acc = baseline_params.get("INS_ACCOFFS_X")
    if expected_acc is not None and v_acc is not None and abs(v_acc - expected_acc) < 0.001:
        print(f"  ✓ INS_ACCOFFS_X     = {v_acc} (restored from baseline {expected_acc})", flush=True)
    else:
        print(f"  ✗ INS_ACCOFFS_X     = {v_acc} (baseline says {expected_acc})", flush=True)
        final_issues.append(("INS_ACCOFFS_X", v_acc, expected_acc))

    print("\n" + "=" * 64, flush=True)
    if not issues and not final_issues:
        print(" ✓ ALL CHECKS PASSED", flush=True)
        print("   - Code-level default override mechanism works (Phase 3 ✓)", flush=True)
        print("   - User calibration successfully restored from baseline_v3.parm", flush=True)
    else:
        print(" ⚠ SOME ISSUES — review above", flush=True)
    print("=" * 64, flush=True)


if __name__ == "__main__":
    main()
