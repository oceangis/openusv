"""EKF variance response under mode-switching and ARM/DISARM events.

Quantifies how much EKF velocity_variance, pos_horiz_variance, and
compass_variance perturb when the autopilot transitions between modes
or arms/disarms. Healthy expectation: variances stay <0.1 with brief
transients.

Two phases (each ~5 min):
  Phase A: 50 rounds MANUAL <-> HOLD <-> ACRO mode switching
  Phase B: 50 rounds ARM (in MANUAL) -> hold -> DISARM
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


MODES = {"MANUAL": 0, "ACRO": 1, "HOLD": 4}


def connect(port, baud):
    print(f"[CONNECT] {port} @ {baud}", flush=True)
    m = mavutil.mavlink_connection(port, baud=baud, dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=15)
    if not hb:
        print("[FATAL] no HB", flush=True); sys.exit(2)
    print(f"[OK] sys={m.target_system}", flush=True)
    return m


def request_streams(m):
    sysid, compid = m.target_system, m.target_component
    for sid in (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS):
        m.mav.request_data_stream_send(sysid, compid, sid, 10, 1)


def set_mode(m, name, timeout=3.0):
    sysid = m.target_system
    target = MODES[name]
    m.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
        if msg and msg.custom_mode == target:
            return True
    return False


def arm(m, want, timeout=4.0, force=False):
    sysid, compid = m.target_system, m.target_component
    m.mav.command_long_send(
        sysid, compid, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if want else 0, 21196 if force else 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
        if msg is None:
            continue
        is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if is_armed == want:
            return True
    return False


def sample_window(m, duration_s):
    """Collect EKF + MEMINFO samples in the window. Returns dict of lists."""
    out = {"vel_var": [], "pos_h_var": [], "compass_var": [],
           "ekf_flags": set(), "free_mem": [],
           "n_msgs": 0}
    t0 = time.time()
    while time.time() - t0 < duration_s:
        msg = m.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        out["n_msgs"] += 1
        mt = msg.get_type()
        if mt == "EKF_STATUS_REPORT":
            out["vel_var"].append(msg.velocity_variance)
            out["pos_h_var"].append(msg.pos_horiz_variance)
            out["compass_var"].append(msg.compass_variance)
            out["ekf_flags"].add(msg.flags)
        elif mt == "MEMINFO":
            free = getattr(msg, "freemem32", None) or msg.freemem
            out["free_mem"].append(free)
    return out


def summarize(samples):
    """Convert list-of-floats to (max, mean, n)."""
    def stat(vs):
        if not vs:
            return (None, None, 0)
        return (max(vs), statistics.mean(vs), len(vs))
    return {
        "vel_var":     stat(samples["vel_var"]),
        "pos_h_var":   stat(samples["pos_h_var"]),
        "compass_var": stat(samples["compass_var"]),
        "free_mem":    (max(samples["free_mem"]) if samples["free_mem"] else None,
                        min(samples["free_mem"]) if samples["free_mem"] else None,
                        len(samples["free_mem"])),
        "ekf_flags": ",".join(f"0x{x:04X}" for x in sorted(samples["ekf_flags"])),
        "n_msgs": samples["n_msgs"],
    }


def phase_a_mode_switching(m, n_rounds=50):
    print(f"\n=== Phase A: {n_rounds} rounds mode switching ===", flush=True)
    sequence = ["MANUAL", "HOLD", "ACRO", "HOLD", "MANUAL"]  # 5 modes per round
    results = []
    for i in range(n_rounds):
        for mode_name in sequence:
            t_switch = time.time()
            ok = set_mode(m, mode_name, timeout=2.0)
            transition_ms = (time.time() - t_switch) * 1000
            # Sample 1.2s window for EKF response to the event
            samples = sample_window(m, 1.2)
            row = summarize(samples)
            row["round"] = i + 1
            row["target_mode"] = mode_name
            row["transition_ok"] = ok
            row["transition_ms"] = transition_ms
            results.append(row)
        if (i + 1) % 10 == 0:
            print(f"  round {i+1}/{n_rounds} ...", flush=True)
    print(f"  [done] {len(results)} mode transitions captured", flush=True)
    return results


def phase_b_arm_disarm(m, n_rounds=50):
    print(f"\n=== Phase B: {n_rounds} rounds ARM/DISARM ===", flush=True)
    # Make sure we're in MANUAL
    set_mode(m, "MANUAL")
    arm(m, False, timeout=3.0, force=True)
    results = []
    arm_fails = 0
    for i in range(n_rounds):
        # ARM
        t0 = time.time()
        ok_arm = arm(m, True, timeout=3.0)
        arm_ms = (time.time() - t0) * 1000
        samples_armed = sample_window(m, 1.0)
        row_armed = summarize(samples_armed)
        row_armed["round"] = i + 1
        row_armed["event"] = "armed"
        row_armed["transition_ok"] = ok_arm
        row_armed["transition_ms"] = arm_ms
        if not ok_arm:
            arm_fails += 1

        # DISARM
        t0 = time.time()
        ok_disarm = arm(m, False, timeout=3.0, force=True)
        disarm_ms = (time.time() - t0) * 1000
        samples_disarmed = sample_window(m, 0.8)
        row_disarmed = summarize(samples_disarmed)
        row_disarmed["round"] = i + 1
        row_disarmed["event"] = "disarmed"
        row_disarmed["transition_ok"] = ok_disarm
        row_disarmed["transition_ms"] = disarm_ms

        results.append(row_armed)
        results.append(row_disarmed)
        if (i + 1) % 10 == 0:
            print(f"  round {i+1}/{n_rounds}  arm_fails={arm_fails}", flush=True)
    print(f"  [done] arm/disarm fails: {arm_fails}/{n_rounds}", flush=True)
    return results, arm_fails


def write_csv(rows, path, cols):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in rows:
            w.writerow([r.get(c) for c in cols])
    print(f"[OUT] {path}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode-rounds", type=int, default=50)
    ap.add_argument("--arm-rounds", type=int, default=50)
    args = ap.parse_args()

    m = connect(args.port, args.baud)
    request_streams(m)

    # Initial baseline
    print("\n[BASELINE] 5s pre-test EKF baseline...", flush=True)
    set_mode(m, "MANUAL")
    baseline = summarize(sample_window(m, 5.0))
    vv_max, vv_mean, n = baseline["vel_var"]
    ph_max, ph_mean, _ = baseline["pos_h_var"]
    cv_max, cv_mean, _ = baseline["compass_var"]
    print(f"  baseline vel_var={vv_mean:.3f}/{vv_max:.3f}  pos_h_var={ph_mean:.3f}/{ph_max:.3f}  compass_var={cv_mean:.3f}/{cv_max:.3f}",
          flush=True)
    print(f"  baseline ekf_flags={baseline['ekf_flags']}  free_mem max={baseline['free_mem'][0]}", flush=True)

    phase_a = phase_a_mode_switching(m, args.mode_rounds)
    phase_b, arm_fails = phase_b_arm_disarm(m, args.arm_rounds)

    # Final state safe
    set_mode(m, "MANUAL")
    arm(m, False, force=True)

    # Stats
    print("\n=== SUMMARY ===")
    vv_all = []
    ph_all = []
    cv_all = []
    mem_min = baseline["free_mem"][1]
    flags_seen = set()
    if baseline["ekf_flags"]:
        flags_seen.update(baseline["ekf_flags"].split(","))
    for r in phase_a + phase_b:
        if r["vel_var"][1] is not None: vv_all.append(r["vel_var"][0])
        if r["pos_h_var"][1] is not None: ph_all.append(r["pos_h_var"][0])
        if r["compass_var"][1] is not None: cv_all.append(r["compass_var"][0])
        if r["free_mem"][1] is not None:
            mem_min = min(mem_min, r["free_mem"][1])
        if r["ekf_flags"]: flags_seen.update(r["ekf_flags"].split(","))

    def report(name, vals):
        if not vals:
            print(f"  {name}: no data")
            return
        print(f"  {name}: max={max(vals):.4f}  mean_of_max={statistics.mean(vals):.4f}  worst10={sorted(vals,reverse=True)[:10]}")

    print(f"Baseline:")
    print(f"  vel_var = {baseline['vel_var'][1]:.4f}")
    print(f"  pos_h_var = {baseline['pos_h_var'][1]:.4f}")
    print(f"  compass_var = {baseline['compass_var'][1]:.4f}")
    print(f"Under events:")
    report("vel_var across events", vv_all)
    report("pos_h_var across events", ph_all)
    report("compass_var across events", cv_all)
    print(f"\nMode switch transitions: {sum(1 for r in phase_a if r['transition_ok'])}/{len(phase_a)} OK")
    print(f"ARM attempts:    {sum(1 for r in phase_b if r['event']=='armed' and r['transition_ok'])}/{args.arm_rounds} OK")
    print(f"DISARM attempts: {sum(1 for r in phase_b if r['event']=='disarmed' and r['transition_ok'])}/{args.arm_rounds} OK")
    print(f"Free memory: baseline_max={baseline['free_mem'][0]} min_seen={mem_min}  delta={mem_min - baseline['free_mem'][0]:+d}")
    print(f"EKF flags seen across all: {sorted(flags_seen)}")

    cols = ["round", "target_mode", "event", "transition_ok", "transition_ms",
            "vel_var", "pos_h_var", "compass_var", "ekf_flags",
            "free_mem", "n_msgs"]
    write_csv(phase_a, "_ekf_phase_a_modes.csv", cols)
    write_csv(phase_b, "_ekf_phase_b_arm.csv", cols)

    verdict = "PASS"
    issues = []
    if vv_all and max(vv_all) > 1.0:
        verdict = "WARN"; issues.append(f"vel_var max {max(vv_all):.3f} > 1.0")
    if ph_all and max(ph_all) > 1.0:
        verdict = "WARN"; issues.append(f"pos_h_var max {max(ph_all):.3f} > 1.0")
    if cv_all and max(cv_all) > 1.0:
        verdict = "WARN"; issues.append(f"compass_var max {max(cv_all):.3f} > 1.0")
    if mem_min < baseline['free_mem'][0] - 4096:
        verdict = "WARN"; issues.append(f"memory dropped {baseline['free_mem'][0] - mem_min} bytes")
    if arm_fails > 0:
        verdict = "FAIL"; issues.append(f"{arm_fails} ARM attempts failed")

    print(f"\n=== VERDICT: {verdict} ===")
    for i in issues:
        print(f"  - {i}")


if __name__ == "__main__":
    main()
