"""30-minute silent soak — pure listener, no commands sent.

Goal: catch memory leaks, RC_CHANNELS / SBUS RMT dropout, IMU rate drift,
EKF flag flapping, and STATUSTEXT anomalies that only show up over time.

Board assumed: static on bench, USB-powered, no ESC, no SBUS receiver.

Output:
  _soak_buckets.csv   — per-minute aggregates (plot-friendly)
  _soak_events.jsonl  — every STATUSTEXT + every EKF flag change
  _soak_report.md     — final pass/fail verdict + trend summary
"""
import argparse
import csv
import json
import math
import statistics
import sys
import time
from collections import defaultdict

from pymavlink import mavutil


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM10")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--minutes", type=float, default=30.0)
    p.add_argument("--bucket-s", type=float, default=60.0,
                   help="seconds per aggregation bucket")
    return p.parse_args()


def connect(port, baud):
    print(f"[CONNECT] {port} @ {baud}", flush=True)
    m = mavutil.mavlink_connection(
        port, baud=baud, dialect="ardupilotmega",
        source_system=255, source_component=0,
    )
    hb = m.wait_heartbeat(timeout=20)
    if not hb:
        print("[FATAL] no HEARTBEAT in 20s — board off or wrong port", flush=True)
        sys.exit(2)
    state = mavutil.mavlink.enums["MAV_STATE"].get(hb.system_status, "?").name
    print(f"[OK] sys={m.target_system} comp={m.target_component} state={state}",
          flush=True)
    return m


def request_streams(m):
    sysid, compid = m.target_system, m.target_component
    # Generous request — we want a full picture
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    for stream_id in (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                      mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA3):
        m.mav.request_data_stream_send(sysid, compid, stream_id, 4, 1)


class Bucket:
    """Per-minute aggregate."""
    __slots__ = (
        "t_start", "counts", "free_first", "free_last", "free_min",
        "imu_temps", "vibe_x", "vibe_y", "vibe_z", "clipping_last",
        "ekf_flags_seen", "ekf_vel_max", "ekf_compass_max",
        "statustext_count", "statustext_warn_count",
        "hb_state_last", "rc_channels_count", "raw_imu_count",
    )

    def __init__(self, t_start):
        self.t_start = t_start
        self.counts = defaultdict(int)
        self.free_first = None
        self.free_last = None
        self.free_min = None
        self.imu_temps = []
        self.vibe_x = []
        self.vibe_y = []
        self.vibe_z = []
        self.clipping_last = 0
        self.ekf_flags_seen = set()
        self.ekf_vel_max = 0.0
        self.ekf_compass_max = 0.0
        self.statustext_count = 0
        self.statustext_warn_count = 0
        self.hb_state_last = None
        self.rc_channels_count = 0
        self.raw_imu_count = 0


def main():
    args = parse_args()
    duration_s = args.minutes * 60.0
    bucket_s = args.bucket_s

    m = connect(args.port, args.baud)
    request_streams(m)

    print(f"[SOAK] duration={args.minutes:.1f} min  bucket={bucket_s:.0f}s", flush=True)
    print(f"[NOTE] silent monitor — no commands will be sent to the board", flush=True)

    t0 = time.time()
    buckets = []
    current = Bucket(t0)
    buckets.append(current)

    events = []  # list of (t_rel, kind, payload) for jsonl
    prev_ekf_flags = None
    last_meminfo_print = 0.0

    # Per-stream timestamps to compute rate later
    all_msg_ts = defaultdict(list)

    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed >= duration_s:
                break

            # Roll bucket if needed
            if now - current.t_start >= bucket_s:
                # finalize previous bucket print
                summarize_bucket(len(buckets), current, now)
                current = Bucket(now)
                buckets.append(current)
                # Re-request streams every minute — if ArduPilot drops the
                # subscription we want to see it as a rate dip, not silence.
                try:
                    request_streams(m)
                except Exception:
                    pass

            msg = m.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == "BAD_DATA":
                continue
            current.counts[mt] += 1
            all_msg_ts[mt].append(now)

            if mt == "HEARTBEAT":
                current.hb_state_last = msg.system_status
            elif mt == "MEMINFO":
                free = getattr(msg, "freemem32", None) or msg.freemem
                if current.free_first is None:
                    current.free_first = free
                current.free_last = free
                if current.free_min is None or free < current.free_min:
                    current.free_min = free
            elif mt == "RAW_IMU":
                current.raw_imu_count += 1
                current.imu_temps.append(msg.temperature)
            elif mt == "VIBRATION":
                current.vibe_x.append(msg.vibration_x)
                current.vibe_y.append(msg.vibration_y)
                current.vibe_z.append(msg.vibration_z)
                current.clipping_last = msg.clipping_0 + msg.clipping_1 + msg.clipping_2
            elif mt == "RC_CHANNELS":
                current.rc_channels_count += 1
            elif mt == "EKF_STATUS_REPORT":
                current.ekf_flags_seen.add(msg.flags)
                current.ekf_vel_max = max(current.ekf_vel_max, msg.velocity_variance)
                current.ekf_compass_max = max(current.ekf_compass_max, msg.compass_variance)
                if prev_ekf_flags is not None and msg.flags != prev_ekf_flags:
                    events.append({
                        "t": elapsed, "kind": "ekf_flag_change",
                        "from": f"0x{prev_ekf_flags:04X}",
                        "to": f"0x{msg.flags:04X}",
                    })
                    print(f"  [EKF] @{elapsed:.0f}s flags 0x{prev_ekf_flags:04X} -> 0x{msg.flags:04X}",
                          flush=True)
                prev_ekf_flags = msg.flags
            elif mt == "STATUSTEXT":
                sev = msg.severity
                txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
                txt = txt.rstrip("\x00 \r\n")
                current.statustext_count += 1
                sev_name = mavutil.mavlink.enums["MAV_SEVERITY"].get(sev, "?").name
                if sev <= 4:
                    current.statustext_warn_count += 1
                events.append({
                    "t": elapsed, "kind": "statustext",
                    "sev": sev, "sev_name": sev_name, "text": txt,
                })
                if sev <= 4:
                    print(f"  [{sev_name}] @{elapsed:.0f}s  {txt}", flush=True)
    except KeyboardInterrupt:
        print("\n[INTERRUPT] stopping soak early", flush=True)

    actual = time.time() - t0
    # Finalize the last bucket
    summarize_bucket(len(buckets), current, time.time())

    print(f"\n[DONE] soaked {actual:.0f}s ({actual/60:.1f} min)", flush=True)

    write_csv(buckets, bucket_s)
    write_events(events)
    write_report(buckets, events, actual, bucket_s)


def summarize_bucket(n, b, now):
    secs = max(1, int(now - b.t_start))
    total = sum(b.counts.values())
    free_delta = (b.free_last - b.free_first) if (b.free_first is not None and b.free_last is not None) else 0
    # MAVLink RAW_IMU.temperature is in centidegrees Celsius (0.01 °C units)
    temp = statistics.mean(b.imu_temps) / 100.0 / 100.0 if b.imu_temps else 0.0
    print(f"\n[BUCKET #{n}] +{secs:3d}s   msgs={total:4d}  "
          f"HB={b.counts.get('HEARTBEAT', 0):2d}  "
          f"RC={b.rc_channels_count:3d}  IMU={b.raw_imu_count:3d}  "
          f"mem_delta={free_delta:+5d}  T={temp:.1f}C  "
          f"WARN={b.statustext_warn_count}", flush=True)


def write_csv(buckets, bucket_s):
    with open("_soak_buckets.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "bucket", "t_start_rel_s",
            "msgs_total", "heartbeats", "rc_channels", "raw_imu",
            "mem_first", "mem_last", "mem_min", "mem_delta",
            "imu_temp_mean",
            "vibe_x_max", "vibe_y_max", "vibe_z_max", "clipping_total",
            "ekf_flags_seen", "ekf_vel_max", "ekf_compass_max",
            "statustext_total", "statustext_warn",
            "hb_state_last",
        ])
        t0 = buckets[0].t_start if buckets else 0
        for i, b in enumerate(buckets, 1):
            w.writerow([
                i, f"{b.t_start - t0:.1f}",
                sum(b.counts.values()),
                b.counts.get("HEARTBEAT", 0),
                b.rc_channels_count,
                b.raw_imu_count,
                b.free_first if b.free_first is not None else "",
                b.free_last if b.free_last is not None else "",
                b.free_min if b.free_min is not None else "",
                (b.free_last - b.free_first) if (b.free_first is not None and b.free_last is not None) else "",
                f"{statistics.mean(b.imu_temps) / 100.0:.2f}" if b.imu_temps else "",
                f"{max(b.vibe_x):.3f}" if b.vibe_x else "",
                f"{max(b.vibe_y):.3f}" if b.vibe_y else "",
                f"{max(b.vibe_z):.3f}" if b.vibe_z else "",
                b.clipping_last,
                ",".join(f"0x{x:04X}" for x in sorted(b.ekf_flags_seen)) or "",
                f"{b.ekf_vel_max:.3f}",
                f"{b.ekf_compass_max:.3f}",
                b.statustext_count,
                b.statustext_warn_count,
                b.hb_state_last if b.hb_state_last is not None else "",
            ])
    print("[OUT] _soak_buckets.csv", flush=True)


def write_events(events):
    with open("_soak_events.jsonl", "w", encoding="utf-8") as f:
        for e in events:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
    print(f"[OUT] _soak_events.jsonl ({len(events)} events)", flush=True)


def write_report(buckets, events, actual_s, bucket_s):
    lines = []
    lines.append("# USV Long Soak Report")
    lines.append(f"\nDate: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"\nDuration: {actual_s:.0f}s ({actual_s/60:.1f} min)")
    lines.append(f"Buckets: {len(buckets)} × {bucket_s:.0f}s")
    lines.append("\n## Verdict\n")

    verdict = []  # list of (mark, line)

    # 1. HEARTBEAT continuity — should be ~1Hz. Scale expected threshold by the
    # actual bucket duration so the final (truncated) bucket isn't a false alarm.
    bucket_durations = []
    for i, b in enumerate(buckets):
        if i + 1 < len(buckets):
            bucket_durations.append(buckets[i+1].t_start - b.t_start)
        else:
            bucket_durations.append(actual_s - (b.t_start - buckets[0].t_start))
    hb_per_bucket = [b.counts.get("HEARTBEAT", 0) for b in buckets]
    bad_hb = []
    # Tolerate 0.9 Hz average (under heavy traffic HB can slip to ~1.05 s)
    for i, (n, dur) in enumerate(zip(hb_per_bucket, bucket_durations), 1):
        expected = max(1, int(dur * 0.9))
        if n < expected:
            bad_hb.append((i, n, expected))
    if bad_hb:
        verdict.append(("FAIL",
            f"HEARTBEAT dropout: " + ", ".join(f"#{i} got {n}, expected >={e}" for i, n, e in bad_hb)))
    else:
        verdict.append(("PASS", f"HEARTBEAT stable: {min(hb_per_bucket)}..{max(hb_per_bucket)} per minute"))

    # 2. RC_CHANNELS stream — SBUS RMT driver health proxy.
    # If RC_CHANNELS appears in the first bucket but not later, RMT may have died.
    rc_per_bucket = [b.rc_channels_count for b in buckets]
    if rc_per_bucket[0] > 0:
        bad_rc = [i for i, n in enumerate(rc_per_bucket, 1) if n == 0]
        if bad_rc:
            verdict.append(("FAIL", f"RC_CHANNELS stream died in bucket(s) #{bad_rc} — possible SBUS/RMT crash"))
        elif min(rc_per_bucket) < rc_per_bucket[0] * 0.5:
            verdict.append(("WARN", f"RC_CHANNELS rate dropped significantly: bucket1={rc_per_bucket[0]}, min={min(rc_per_bucket)}"))
        else:
            verdict.append(("PASS", f"RC_CHANNELS stable: {min(rc_per_bucket)}..{max(rc_per_bucket)} per minute"))
    else:
        verdict.append(("INFO", "RC_CHANNELS not emitted (no SBUS RX connected — expected on bench)"))

    # 3. Memory leak — compare first vs last bucket free memory
    mems = [b.free_last for b in buckets if b.free_last is not None]
    if len(mems) >= 2:
        first_free = mems[0]
        last_free = mems[-1]
        min_free = min(mems)
        leak = first_free - last_free
        leak_per_min = leak / (actual_s / 60.0) if actual_s > 0 else 0
        line = f"free first={first_free}  last={last_free}  delta={-leak:+d}  min={min_free}  rate={-leak_per_min:+.1f} B/min"
        if leak > 4096:  # >4KB over the run
            verdict.append(("FAIL", f"Memory leak suspected: {line}"))
        elif leak > 512:
            verdict.append(("WARN", f"Possible slow leak: {line}"))
        else:
            verdict.append(("PASS", f"Memory stable: {line}"))
    else:
        verdict.append(("INFO", "no MEMINFO samples"))

    # 4. IMU sample rate stability
    imu_per_bucket = [b.raw_imu_count for b in buckets]
    if imu_per_bucket[0] > 0:
        bad_imu = [i for i, n in enumerate(imu_per_bucket, 1) if n < imu_per_bucket[0] * 0.7]
        if bad_imu:
            verdict.append(("WARN", f"RAW_IMU rate dipped in bucket(s) #{bad_imu}"))
        else:
            verdict.append(("PASS", f"RAW_IMU stable: {min(imu_per_bucket)}..{max(imu_per_bucket)} per minute"))

    # 5. IMU temperature drift
    temp_means = [statistics.mean(b.imu_temps) / 100.0 for b in buckets if b.imu_temps]
    if len(temp_means) >= 2:
        t_drift = temp_means[-1] - temp_means[0]
        verdict.append(("INFO", f"IMU temp: start={temp_means[0]:.1f}C end={temp_means[-1]:.1f}C drift={t_drift:+.1f}C"))

    # 6. EKF flag flapping
    flag_changes = [e for e in events if e["kind"] == "ekf_flag_change"]
    if len(flag_changes) > 5:
        verdict.append(("WARN", f"EKF flags changed {len(flag_changes)} times — possible instability"))
    elif flag_changes:
        verdict.append(("INFO", f"EKF flags changed {len(flag_changes)} times (transient)"))
    else:
        verdict.append(("PASS", "EKF flags stable (no transitions)"))

    # 7. STATUSTEXT — any WARNING (sev<=4) is suspicious in silent soak
    warns = [e for e in events if e["kind"] == "statustext" and e["sev"] <= 4]
    if warns:
        verdict.append(("WARN", f"{len(warns)} WARNING+ STATUSTEXT messages — see _soak_events.jsonl"))
    else:
        verdict.append(("PASS", "No WARNING/CRITICAL STATUSTEXT"))

    # 8. HEARTBEAT state — must not flap to CRITICAL/EMERGENCY mid-soak
    states = [b.hb_state_last for b in buckets if b.hb_state_last is not None]
    if states:
        if any(s in (5, 6) for s in states):  # CRITICAL=5, EMERGENCY=6
            verdict.append(("FAIL", f"Board reported CRITICAL/EMERGENCY state during soak"))
        else:
            uniq = sorted(set(states))
            names = [mavutil.mavlink.enums["MAV_STATE"].get(s, "?").name for s in uniq]
            verdict.append(("PASS", f"System state stable: {names}"))

    fails = [v for v in verdict if v[0] == "FAIL"]
    warns = [v for v in verdict if v[0] == "WARN"]
    overall = "PASS" if not fails and not warns else ("FAIL" if fails else "WARN")
    lines.append(f"**Overall: {overall}**\n")
    for mark, line in verdict:
        emoji = {"PASS": "✓", "FAIL": "✗", "WARN": "⚠", "INFO": "·"}[mark]
        lines.append(f"- {emoji} **{mark}** — {line}")

    # Per-bucket table
    lines.append("\n## Per-Bucket Detail\n")
    lines.append("| # | t(s) | msgs | HB | RC | IMU | mem | T(C) | EKF flags | WARN |")
    lines.append("|---|------|------|----|----|-----|-----|------|-----------|------|")
    t0 = buckets[0].t_start if buckets else 0
    for i, b in enumerate(buckets, 1):
        flags_str = ",".join(f"0x{x:04X}" for x in sorted(b.ekf_flags_seen)) or "-"
        mem_str = f"{b.free_last}" if b.free_last is not None else "-"
        temp_str = f"{statistics.mean(b.imu_temps) / 100.0:.1f}" if b.imu_temps else "-"
        lines.append(f"| {i} | {b.t_start - t0:.0f} | {sum(b.counts.values())} | "
                     f"{b.counts.get('HEARTBEAT', 0)} | {b.rc_channels_count} | "
                     f"{b.raw_imu_count} | {mem_str} | {temp_str} | "
                     f"{flags_str} | {b.statustext_warn_count} |")

    # All events (capped)
    lines.append("\n## Events Log (first 50)\n")
    for e in events[:50]:
        if e["kind"] == "statustext":
            lines.append(f"- `+{e['t']:.0f}s` [{e['sev_name']}] {e['text']}")
        elif e["kind"] == "ekf_flag_change":
            lines.append(f"- `+{e['t']:.0f}s` EKF flags {e['from']} → {e['to']}")
    if len(events) > 50:
        lines.append(f"\n*…and {len(events)-50} more in `_soak_events.jsonl`*")

    txt = "\n".join(lines)
    with open("_soak_report.md", "w", encoding="utf-8") as f:
        f.write(txt)
    print("[OUT] _soak_report.md", flush=True)
    print("\n" + "="*60)
    print(f"OVERALL: {overall}")
    print("="*60)


if __name__ == "__main__":
    main()
