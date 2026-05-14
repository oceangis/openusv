"""Restore params from baseline_v3.parm via PARAM_SET. Standalone use after a
factory reset (where factory_reset_verify.py's restore phase didn't run)."""
import argparse
import sys
import time
from pymavlink import mavutil

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def parse_parm(path):
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


def write_param(m, name, value, timeout=2.0):
    sysid, compid = m.target_system, m.target_component
    m.mav.param_set_send(sysid, compid, name.encode(), float(value),
                          mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t = time.time()
    while time.time() - t < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if msg:
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            pid = pid.strip("\x00")
            if pid == name:
                return msg.param_value
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--baseline", default="../../params/baseline_v3.parm")
    args = ap.parse_args()

    params = parse_parm(args.baseline)
    print(f"[LOAD] {len(params)} params from {args.baseline}", flush=True)

    m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                    dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=15)
    if not hb:
        print("[FATAL] no heartbeat", flush=True); sys.exit(2)
    print(f"[OK] connected", flush=True)

    sent = 0
    failed = []
    skipped = []
    t0 = time.time()
    for nm, v in params.items():
        if nm.startswith("STAT_"):
            skipped.append(nm)
            continue
        result = write_param(m, nm, v)
        if result is None:
            failed.append((nm, v, None))
        elif abs(result - v) > 0.01:
            failed.append((nm, v, result))
        sent += 1
        if sent % 50 == 0:
            print(f"  ...{sent}/{len(params)} (elapsed {time.time()-t0:.0f}s, failed={len(failed)})",
                  flush=True)

    print(f"\n[DONE] restored {sent} params in {time.time()-t0:.1f}s. Failed: {len(failed)}", flush=True)
    if failed[:15]:
        print("Failures (first 15):")
        for nm, v, r in failed[:15]:
            print(f"  {nm}: tried {v}, got {r}", flush=True)
    if len(failed) > 15:
        print(f"  ... and {len(failed)-15} more", flush=True)


if __name__ == "__main__":
    main()
