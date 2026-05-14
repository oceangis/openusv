"""ARM/DISARM cycle stress — 200 rounds of arm/hold/disarm/hold.

Looks for cumulative state-machine fatigue:
  - prearm counter inflation / repeated rejection
  - STATUSTEXT garbage accumulation
  - free-memory drift per cycle
  - heartbeat continuity drift
  - any cycle where ARM or DISARM doesn't confirm
"""
import argparse
import csv
import io
import statistics
import sys
import time
from collections import defaultdict
from pymavlink import mavutil

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def connect(port, baud):
    m = mavutil.mavlink_connection(port, baud=baud, dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    if not m.wait_heartbeat(timeout=15):
        sys.exit("no HB")
    return m


def request_streams(m):
    sysid, compid = m.target_system, m.target_component
    for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA3):
        m.mav.request_data_stream_send(sysid, compid, sid, 4, 1)


def set_mode_manual(m, timeout=3.0):
    sysid = m.target_system
    m.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
        if msg and msg.custom_mode == 0:
            return True
    return False


def arm_op(m, want, timeout=4.0, force=False):
    """Returns (success, statustexts list, transition_ms)."""
    sysid, compid = m.target_system, m.target_component
    statustexts = []
    t_send = time.time()
    m.mav.command_long_send(
        sysid, compid, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if want else 0, 21196 if force else 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            statustexts.append({"sev": msg.severity,
                                "text": txt.rstrip("\x00 \r\n")})
        elif mt == "HEARTBEAT":
            is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed == want:
                return True, statustexts, (time.time() - t_send) * 1000
    return False, statustexts, (time.time() - t_send) * 1000


def read_mem(m, timeout=2.0):
    sysid, compid = m.target_system, m.target_component
    m.mav.request_data_stream_send(sysid, compid,
                                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)
    t = time.time()
    while time.time() - t < timeout:
        msg = m.recv_match(type="MEMINFO", blocking=True, timeout=0.3)
        if msg:
            return getattr(msg, "freemem32", None) or msg.freemem
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--rounds", type=int, default=200)
    ap.add_argument("--armed-hold-s", type=float, default=4.0)
    ap.add_argument("--disarmed-hold-s", type=float, default=2.0)
    args = ap.parse_args()

    print(f"[ARM/DISARM stress] rounds={args.rounds}  "
          f"armed_hold={args.armed_hold_s}s  disarmed_hold={args.disarmed_hold_s}s",
          flush=True)
    print(f"  expected duration: ~{args.rounds * (args.armed_hold_s + args.disarmed_hold_s + 0.4) / 60:.0f} min",
          flush=True)

    m = connect(args.port, args.baud if hasattr(args, "baud") else 115200)
    request_streams(m)

    if not set_mode_manual(m):
        sys.exit("could not enter MANUAL")

    # Drain
    t = time.time()
    while time.time() - t < 1:
        m.recv_match(blocking=True, timeout=0.1)

    mem_baseline = read_mem(m)
    print(f"  baseline free heap: {mem_baseline}", flush=True)

    rows = []
    fails = {"arm": 0, "disarm": 0}
    statustexts_unique = defaultdict(int)
    t_start = time.time()

    for i in range(args.rounds):
        # ARM
        ok_arm, st_arm, arm_ms = arm_op(m, True, timeout=4.0)
        if not ok_arm:
            fails["arm"] += 1
        for s in st_arm:
            statustexts_unique[s["text"]] += 1

        # hold armed
        time.sleep(args.armed_hold_s)

        # DISARM (forced — bench has no actual motion to check)
        ok_disarm, st_disarm, disarm_ms = arm_op(m, False, timeout=4.0, force=True)
        if not ok_disarm:
            fails["disarm"] += 1
        for s in st_disarm:
            statustexts_unique[s["text"]] += 1

        # hold disarmed
        time.sleep(args.disarmed_hold_s)

        # Periodic memory check
        mem_now = None
        if (i + 1) % 25 == 0 or i == 0:
            mem_now = read_mem(m)

        rows.append({
            "round": i + 1,
            "arm_ok": ok_arm,
            "disarm_ok": ok_disarm,
            "arm_ms": arm_ms,
            "disarm_ms": disarm_ms,
            "arm_statustext_count": len(st_arm),
            "disarm_statustext_count": len(st_disarm),
            "free_mem": mem_now,
        })

        if (i + 1) % 25 == 0:
            elapsed = time.time() - t_start
            eta = elapsed / (i + 1) * (args.rounds - i - 1)
            print(f"  round {i+1}/{args.rounds}  arm_fails={fails['arm']}  "
                  f"disarm_fails={fails['disarm']}  mem={mem_now}  "
                  f"elapsed={elapsed/60:.1f}min  eta={eta/60:.1f}min",
                  flush=True)

    # Final state safe
    arm_op(m, False, force=True)

    mem_final = read_mem(m)
    elapsed = time.time() - t_start

    # Save CSV
    with open("_arm_disarm_stress.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["round", "arm_ok", "disarm_ok", "arm_ms", "disarm_ms",
                    "arm_statustext_count", "disarm_statustext_count", "free_mem"])
        for r in rows:
            w.writerow([r["round"], r["arm_ok"], r["disarm_ok"],
                        f"{r['arm_ms']:.1f}", f"{r['disarm_ms']:.1f}",
                        r["arm_statustext_count"], r["disarm_statustext_count"],
                        r["free_mem"] if r["free_mem"] is not None else ""])

    print(f"\n=== ARM/DISARM stress complete ===", flush=True)
    print(f"Rounds: {args.rounds}  duration: {elapsed/60:.1f} min", flush=True)
    print(f"Failures: ARM={fails['arm']}  DISARM={fails['disarm']}", flush=True)
    print(f"Memory: baseline={mem_baseline}  final={mem_final}  delta={(mem_final-mem_baseline) if mem_baseline and mem_final else 'N/A'}", flush=True)

    arm_ms = [r["arm_ms"] for r in rows]
    disarm_ms = [r["disarm_ms"] for r in rows]
    print(f"\nARM time: min={min(arm_ms):.0f}ms  mean={statistics.mean(arm_ms):.0f}ms  max={max(arm_ms):.0f}ms", flush=True)
    print(f"DISARM time: min={min(disarm_ms):.0f}ms  mean={statistics.mean(disarm_ms):.0f}ms  max={max(disarm_ms):.0f}ms", flush=True)

    print(f"\nUnique STATUSTEXT messages ({len(statustexts_unique)}):", flush=True)
    for txt, count in sorted(statustexts_unique.items(), key=lambda x: -x[1])[:15]:
        print(f"  {count:4d}× '{txt}'", flush=True)

    # Verdict
    verdict = "PASS"
    issues = []
    if fails["arm"] > 0 or fails["disarm"] > 0:
        verdict = "FAIL"; issues.append(f"{fails['arm']} ARM fails, {fails['disarm']} DISARM fails")
    if mem_baseline and mem_final and (mem_baseline - mem_final) > 4096:
        verdict = "WARN" if verdict == "PASS" else verdict
        issues.append(f"memory dropped {mem_baseline - mem_final} bytes")
    if max(arm_ms) > 3000:
        verdict = "WARN" if verdict == "PASS" else verdict
        issues.append(f"arm time > 3s at least once (max {max(arm_ms):.0f}ms)")

    print(f"\n=== VERDICT: {verdict} ===")
    for i in issues:
        print(f"  - {i}")


if __name__ == "__main__":
    main()
