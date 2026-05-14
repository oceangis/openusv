"""30-minute work-load soak — board ARMED in MANUAL with 50 Hz RC override.

Differs from `long_soak.py` (silent baseline) by stressing the RC input + servo
output paths: a worker thread streams RC_CHANNELS_OVERRIDE at 50 Hz (neutral
1500 µs on all 8 channels) for the entire run, simulating an active operator
holding the sticks centred. This is the closest bench analogue of "vessel under
LoRa MAVLink control, holding station".

Board assumed: static on bench, USB-powered, no ESC connected, no SBUS receiver.
Because no ESC is wired, the SERVO_OUTPUT_RAW pulses go nowhere — safe.

Output:
  _workload_buckets.csv   — per-minute aggregates
  _workload_events.jsonl  — STATUSTEXT, EKF flag changes, unplanned DISARM
  _workload_report.md     — verdict + per-bucket table
"""
import argparse
import csv
import io
import json
import statistics
import sys
import threading
import time
from collections import defaultdict

from pymavlink import mavutil

# Windows console defaults to GBK; force UTF-8 so non-ASCII (µ, ✓, etc.) prints
# don't crash the long-running soak.
if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM10")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--minutes", type=float, default=30.0)
    p.add_argument("--bucket-s", type=float, default=60.0)
    p.add_argument("--rc-hz", type=float, default=50.0,
                   help="RC override stream rate")
    return p.parse_args()


def connect(port, baud):
    print(f"[CONNECT] {port} @ {baud}", flush=True)
    m = mavutil.mavlink_connection(
        port, baud=baud, dialect="ardupilotmega",
        source_system=255, source_component=0,
    )
    hb = m.wait_heartbeat(timeout=20)
    if not hb:
        print("[FATAL] no HEARTBEAT", flush=True)
        sys.exit(2)
    state = mavutil.mavlink.enums["MAV_STATE"].get(hb.system_status, "?").name
    print(f"[OK] sys={m.target_system} comp={m.target_component} state={state}", flush=True)
    return m


def request_streams(m):
    sysid, compid = m.target_system, m.target_component
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    for stream_id in (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                      mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA3):
        m.mav.request_data_stream_send(sysid, compid, stream_id, 4, 1)


def set_mode(m, mode_id, timeout=4.0):
    sysid = m.target_system
    m.mav.set_mode_send(
        sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
        if msg and msg.custom_mode == mode_id:
            return True
    return False


def arm(m, want=True, timeout=5.0):
    sysid, compid = m.target_system, m.target_component
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if want else 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
        if msg is None:
            continue
        is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if is_armed == want:
            return True
    return False


def is_armed(hb):
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


class RCStreamer(threading.Thread):
    """Push RC_CHANNELS_OVERRIDE at fixed rate. Default centre 1500 on all 8."""

    def __init__(self, m, hz=50.0, channels=None):
        super().__init__(daemon=True)
        self.m = m
        self.period = 1.0 / hz
        self.lock = threading.Lock()
        self.chans = channels if channels else (1500,) * 8
        self.stop_evt = threading.Event()
        self.sent = 0
        self.errors = 0

    def set_channels(self, *chans):
        with self.lock:
            self.chans = tuple(list(chans) + [1500] * (8 - len(chans)))

    def run(self):
        sysid, compid = self.m.target_system, self.m.target_component
        next_t = time.time()
        while not self.stop_evt.is_set():
            with self.lock:
                c = self.chans
            try:
                self.m.mav.rc_channels_override_send(sysid, compid, *c)
                self.sent += 1
            except Exception:
                self.errors += 1
            next_t += self.period
            slack = next_t - time.time()
            if slack > 0:
                time.sleep(slack)
            else:
                # Behind — reset reference to avoid spiralling catch-up
                next_t = time.time()

    def stop(self):
        self.stop_evt.set()


class Bucket:
    __slots__ = (
        "t_start", "counts",
        "free_first", "free_last", "free_min",
        "imu_temps", "vibe_x", "vibe_y", "vibe_z", "clipping_last",
        "ekf_flags_seen", "ekf_vel_max", "ekf_compass_max",
        "statustext_count", "statustext_warn_count",
        "hb_state_last", "rc_channels_count", "raw_imu_count",
        "servo_count", "servo1_vals", "servo3_vals",
        "armed_last",
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
        self.servo_count = 0
        self.servo1_vals = []
        self.servo3_vals = []
        self.armed_last = None


def main():
    args = parse_args()
    duration_s = args.minutes * 60.0
    bucket_s = args.bucket_s
    rc_hz = args.rc_hz

    m = connect(args.port, args.baud)
    request_streams(m)

    # ------ Pre-flight: MANUAL + ARM
    print("\n[PREP] switching to MANUAL …", flush=True)
    if not set_mode(m, 0):  # MANUAL = 0
        print("[FATAL] could not enter MANUAL", flush=True)
        sys.exit(3)
    print("[PREP] arming …", flush=True)
    if not arm(m, True):
        print("[FATAL] arm failed — check ARMING_CHECK / FS_THR_ENABLE", flush=True)
        sys.exit(3)
    print("[PREP] ARMED OK", flush=True)

    # ------ Start RC streamer at 50 Hz, centre stick
    streamer = RCStreamer(m, hz=rc_hz)
    streamer.start()
    print(f"[PREP] RC override @ {rc_hz} Hz on all channels = 1500 us", flush=True)
    time.sleep(0.5)  # warm-up

    print(f"\n[SOAK] duration={args.minutes:.1f} min  bucket={bucket_s:.0f}s  rc_hz={rc_hz}", flush=True)

    t0 = time.time()
    buckets = [Bucket(t0)]
    current = buckets[-1]
    events = []
    prev_ekf_flags = None
    prev_armed = True
    unplanned_disarm = False

    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed >= duration_s:
                break

            if now - current.t_start >= bucket_s:
                summarize_bucket(len(buckets), current, now, streamer)
                current = Bucket(now)
                buckets.append(current)
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

            if mt == "HEARTBEAT":
                current.hb_state_last = msg.system_status
                current.armed_last = is_armed(msg)
                if prev_armed and not current.armed_last:
                    unplanned_disarm = True
                    events.append({
                        "t": elapsed, "kind": "unplanned_disarm",
                        "state": msg.system_status,
                    })
                    print(f"  [CRIT] @{elapsed:.0f}s UNPLANNED DISARM — board dropped armed flag", flush=True)
                prev_armed = current.armed_last
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
            elif mt == "SERVO_OUTPUT_RAW":
                current.servo_count += 1
                current.servo1_vals.append(msg.servo1_raw)
                current.servo3_vals.append(msg.servo3_raw)
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
        print("\n[INTERRUPT] stopping work-load soak early", flush=True)

    actual = time.time() - t0
    summarize_bucket(len(buckets), current, time.time(), streamer)

    # ------ Cleanup: stop streamer + DISARM
    print(f"\n[CLEANUP] stopping RC override after {streamer.sent} sends "
          f"({streamer.errors} errors)…", flush=True)
    streamer.stop()
    streamer.join(2.0)
    print("[CLEANUP] disarming …", flush=True)
    if not arm(m, False):
        print("[WARN] disarm not confirmed — sending again", flush=True)
        arm(m, False, timeout=3.0)
    print("[CLEANUP] done", flush=True)

    print(f"\n[DONE] soaked {actual:.0f}s ({actual/60:.1f} min)", flush=True)

    write_csv(buckets)
    write_events(events)
    write_report(buckets, events, actual, bucket_s, streamer, rc_hz, unplanned_disarm)


def summarize_bucket(n, b, now, streamer):
    secs = max(1, int(now - b.t_start))
    total = sum(b.counts.values())
    free_delta = (b.free_last - b.free_first) if (b.free_first is not None and b.free_last is not None) else 0
    temp = statistics.mean(b.imu_temps) / 100.0 if b.imu_temps else 0.0
    armed_str = "ARM" if b.armed_last else ("DISARM" if b.armed_last is False else "??")
    s1 = statistics.mean(b.servo1_vals) if b.servo1_vals else 0
    s3 = statistics.mean(b.servo3_vals) if b.servo3_vals else 0
    print(f"\n[BUCKET #{n}] +{secs:3d}s  msgs={total:4d}  HB={b.counts.get('HEARTBEAT',0):2d}  "
          f"RC={b.rc_channels_count:3d}  IMU={b.raw_imu_count:3d}  "
          f"SRV={b.servo_count:3d}(s1={s1:.0f}/s3={s3:.0f})  "
          f"mem={free_delta:+5d}  T={temp:.1f}C  [{armed_str}]  "
          f"OVR_sent={streamer.sent}", flush=True)


def write_csv(buckets):
    with open("_workload_buckets.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "bucket", "t_start_rel_s",
            "msgs_total", "heartbeats", "rc_channels", "raw_imu", "servo_out",
            "servo1_mean", "servo1_min", "servo1_max",
            "servo3_mean", "servo3_min", "servo3_max",
            "mem_first", "mem_last", "mem_min", "mem_delta",
            "imu_temp_mean",
            "vibe_x_max", "vibe_y_max", "vibe_z_max", "clipping_total",
            "ekf_flags_seen", "ekf_vel_max", "ekf_compass_max",
            "statustext_total", "statustext_warn",
            "hb_state_last", "armed_last",
        ])
        t0 = buckets[0].t_start if buckets else 0
        for i, b in enumerate(buckets, 1):
            w.writerow([
                i, f"{b.t_start - t0:.1f}",
                sum(b.counts.values()),
                b.counts.get("HEARTBEAT", 0),
                b.rc_channels_count, b.raw_imu_count, b.servo_count,
                f"{statistics.mean(b.servo1_vals):.1f}" if b.servo1_vals else "",
                min(b.servo1_vals) if b.servo1_vals else "",
                max(b.servo1_vals) if b.servo1_vals else "",
                f"{statistics.mean(b.servo3_vals):.1f}" if b.servo3_vals else "",
                min(b.servo3_vals) if b.servo3_vals else "",
                max(b.servo3_vals) if b.servo3_vals else "",
                b.free_first if b.free_first is not None else "",
                b.free_last if b.free_last is not None else "",
                b.free_min if b.free_min is not None else "",
                (b.free_last - b.free_first) if (b.free_first is not None and b.free_last is not None) else "",
                f"{statistics.mean(b.imu_temps)/100.0:.2f}" if b.imu_temps else "",
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
                "1" if b.armed_last else ("0" if b.armed_last is False else ""),
            ])
    print("[OUT] _workload_buckets.csv", flush=True)


def write_events(events):
    with open("_workload_events.jsonl", "w", encoding="utf-8") as f:
        for e in events:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
    print(f"[OUT] _workload_events.jsonl ({len(events)} events)", flush=True)


def write_report(buckets, events, actual_s, bucket_s, streamer, rc_hz, unplanned_disarm):
    lines = []
    lines.append("# USV Work-Load Soak Report")
    lines.append(f"\nDate: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"\nDuration: {actual_s:.0f}s ({actual_s/60:.1f} min)")
    lines.append(f"Bucket size: {bucket_s:.0f}s × {len(buckets)}")
    lines.append(f"RC override: {rc_hz} Hz neutral, sent={streamer.sent}, errors={streamer.errors}")
    expected_overrides = int(actual_s * rc_hz)
    lines.append(f"  expected ~{expected_overrides} sends, actual {streamer.sent} ({100.0*streamer.sent/expected_overrides:.1f}%)")
    lines.append("\n## Verdict\n")

    verdict = []

    # 1. Armed throughout — the headline metric
    if unplanned_disarm:
        verdict.append(("FAIL", "Board unexpectedly DISARMED mid-soak — see events"))
    else:
        verdict.append(("PASS", "Board stayed ARMED the entire run"))

    # 2. HEARTBEAT
    bucket_durations = []
    for i, b in enumerate(buckets):
        if i + 1 < len(buckets):
            bucket_durations.append(buckets[i+1].t_start - b.t_start)
        else:
            bucket_durations.append(actual_s - (b.t_start - buckets[0].t_start))
    hb_per_bucket = [b.counts.get("HEARTBEAT", 0) for b in buckets]
    bad_hb = []
    # HEARTBEAT is nominally 1 Hz but under heavy MAVLink traffic (50 Hz RC
    # override + 5 Hz GPS + ...) it can slip to 1.05 s. Tolerate 0.9 Hz
    # average (= 90% of bucket duration) — only flag real silence.
    for i, (n, dur) in enumerate(zip(hb_per_bucket, bucket_durations), 1):
        expected = max(1, int(dur * 0.9))
        if n < expected:
            bad_hb.append((i, n, expected))
    if bad_hb:
        verdict.append(("FAIL", "HEARTBEAT dropout: " +
                        ", ".join(f"#{i} got {n}, expected >={e}" for i, n, e in bad_hb)))
    else:
        verdict.append(("PASS", f"HEARTBEAT stable: {min(hb_per_bucket)}..{max(hb_per_bucket)}/min"))

    # 3. Memory leak — the second most important metric under work-load
    mems = [b.free_last for b in buckets if b.free_last is not None]
    if len(mems) >= 2:
        first_free, last_free = mems[0], mems[-1]
        min_free = min(mems)
        leak = first_free - last_free
        leak_per_min = leak / (actual_s / 60.0) if actual_s > 0 else 0
        line = f"free first={first_free} last={last_free} delta={-leak:+d} min={min_free} rate={-leak_per_min:+.1f} B/min"
        if leak > 4096:
            verdict.append(("FAIL", f"Memory leak under load: {line}"))
        elif leak > 512:
            verdict.append(("WARN", f"Slow leak under load: {line}"))
        else:
            verdict.append(("PASS", f"Memory stable under load: {line}"))

    # 4. SERVO_OUTPUT_RAW — pulses should be present and stable at neutral
    all_s1 = [v for b in buckets for v in b.servo1_vals]
    all_s3 = [v for b in buckets for v in b.servo3_vals]
    if all_s1 and all_s3:
        s1_mean = statistics.mean(all_s1)
        s3_mean = statistics.mean(all_s3)
        s1_range = max(all_s1) - min(all_s1)
        s3_range = max(all_s3) - min(all_s3)
        line = (f"servo1 mean={s1_mean:.0f} range={s1_range} | "
                f"servo3 mean={s3_mean:.0f} range={s3_range}")
        if abs(s1_mean - 1500) > 30 or abs(s3_mean - 1500) > 30:
            verdict.append(("WARN", f"Servo midpoint drift: {line}"))
        elif s1_range > 100 or s3_range > 100:
            verdict.append(("WARN", f"Servo jitter: {line}"))
        else:
            verdict.append(("PASS", f"Servo output stable at neutral: {line}"))
    else:
        verdict.append(("WARN", "No SERVO_OUTPUT_RAW captured — stream may not have been requested"))

    # 5. RC_CHANNELS still ticking under load
    rc_per_bucket = [b.rc_channels_count for b in buckets]
    if rc_per_bucket[0] > 0:
        bad_rc = [i for i, n in enumerate(rc_per_bucket, 1) if n == 0]
        if bad_rc:
            verdict.append(("FAIL", f"RC_CHANNELS stream died in #{bad_rc} — SBUS/RCInput crashed under load"))
        elif min(rc_per_bucket) < rc_per_bucket[0] * 0.5:
            verdict.append(("WARN", f"RC_CHANNELS rate dropped: first={rc_per_bucket[0]} min={min(rc_per_bucket)}"))
        else:
            verdict.append(("PASS", f"RC_CHANNELS rate stable: {min(rc_per_bucket)}..{max(rc_per_bucket)}/min"))

    # 6. IMU rate stability
    imu_per_bucket = [b.raw_imu_count for b in buckets]
    if imu_per_bucket and imu_per_bucket[0] > 0:
        bad_imu = [i for i, n in enumerate(imu_per_bucket, 1) if n < imu_per_bucket[0] * 0.7]
        if bad_imu:
            verdict.append(("WARN", f"RAW_IMU rate dipped in #{bad_imu}"))
        else:
            verdict.append(("PASS", f"RAW_IMU stable: {min(imu_per_bucket)}..{max(imu_per_bucket)}/min"))

    # 7. EKF flags / transitions
    flag_changes = [e for e in events if e["kind"] == "ekf_flag_change"]
    if len(flag_changes) > 5:
        verdict.append(("WARN", f"EKF flags changed {len(flag_changes)} times under load"))
    elif flag_changes:
        verdict.append(("INFO", f"EKF flags changed {len(flag_changes)} times (transient)"))
    else:
        verdict.append(("PASS", "EKF flags stable"))

    # 8. STATUSTEXT warnings
    warns = [e for e in events if e["kind"] == "statustext" and e["sev"] <= 4]
    if warns:
        verdict.append(("WARN", f"{len(warns)} STATUSTEXT WARN+ messages"))
    else:
        verdict.append(("PASS", "No WARNING/CRITICAL STATUSTEXT"))

    # 9. RC override stream stayed at target rate
    override_pct = 100.0 * streamer.sent / expected_overrides if expected_overrides else 0
    if override_pct < 90:
        verdict.append(("WARN", f"RC override stream underran target: {override_pct:.1f}% of expected"))
    else:
        verdict.append(("PASS", f"RC override delivery {override_pct:.1f}% of {expected_overrides}"))

    # IMU temperature info
    temp_means = [statistics.mean(b.imu_temps) / 100.0 for b in buckets if b.imu_temps]
    if len(temp_means) >= 2:
        t_drift = temp_means[-1] - temp_means[0]
        verdict.append(("INFO", f"IMU temp: start={temp_means[0]:.1f}C end={temp_means[-1]:.1f}C drift={t_drift:+.1f}C"))

    fails = [v for v in verdict if v[0] == "FAIL"]
    warn_lines = [v for v in verdict if v[0] == "WARN"]
    overall = "PASS" if not fails and not warn_lines else ("FAIL" if fails else "WARN")
    lines.append(f"**Overall: {overall}**\n")
    for mark, line in verdict:
        emoji = {"PASS": "✓", "FAIL": "✗", "WARN": "⚠", "INFO": "·"}[mark]
        lines.append(f"- {emoji} **{mark}** — {line}")

    # Per-bucket table
    lines.append("\n## Per-Bucket Detail\n")
    lines.append("| # | t(s) | msgs | HB | RC | IMU | SRV | s1µ | s3µ | mem | T(C) | EKF | ARM | WARN |")
    lines.append("|---|------|------|----|----|-----|-----|-----|-----|-----|------|-----|-----|------|")
    t0 = buckets[0].t_start if buckets else 0
    for i, b in enumerate(buckets, 1):
        flags_str = ",".join(f"0x{x:04X}" for x in sorted(b.ekf_flags_seen)) or "-"
        mem_str = f"{b.free_last}" if b.free_last is not None else "-"
        temp_str = f"{statistics.mean(b.imu_temps)/100.0:.1f}" if b.imu_temps else "-"
        s1m = f"{statistics.mean(b.servo1_vals):.0f}" if b.servo1_vals else "-"
        s3m = f"{statistics.mean(b.servo3_vals):.0f}" if b.servo3_vals else "-"
        arm_str = "1" if b.armed_last else ("0" if b.armed_last is False else "?")
        lines.append(f"| {i} | {b.t_start-t0:.0f} | {sum(b.counts.values())} | "
                     f"{b.counts.get('HEARTBEAT',0)} | {b.rc_channels_count} | "
                     f"{b.raw_imu_count} | {b.servo_count} | {s1m} | {s3m} | "
                     f"{mem_str} | {temp_str} | {flags_str} | {arm_str} | "
                     f"{b.statustext_warn_count} |")

    # Events
    lines.append("\n## Events Log (first 50)\n")
    for e in events[:50]:
        if e["kind"] == "statustext":
            lines.append(f"- `+{e['t']:.0f}s` [{e['sev_name']}] {e['text']}")
        elif e["kind"] == "ekf_flag_change":
            lines.append(f"- `+{e['t']:.0f}s` EKF flags {e['from']} → {e['to']}")
        elif e["kind"] == "unplanned_disarm":
            lines.append(f"- `+{e['t']:.0f}s` **UNPLANNED DISARM** (state={e['state']})")
    if len(events) > 50:
        lines.append(f"\n*…and {len(events)-50} more in `_workload_events.jsonl`*")

    txt = "\n".join(lines)
    with open("_workload_report.md", "w", encoding="utf-8") as f:
        f.write(txt)
    print("[OUT] _workload_report.md", flush=True)
    print("\n" + "=" * 60)
    print(f"OVERALL: {overall}")
    print("=" * 60)


if __name__ == "__main__":
    main()
