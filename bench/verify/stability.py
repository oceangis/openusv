"""USV stability test suite — Phase 1-3 via MAVLink on COM58.

Board assumed: static, flat, no GPS fix, no RC receiver, USB-powered.

Phase 1: 60s health baseline
Phase 2: Mode switching matrix (GPS-free modes only)
Phase 3: Servo output verification via DO_MOTOR_TEST low-throttle pulses

Outputs:
  _stability_phase1.json
  _stability_phase2.json
  _stability_phase3.json
  _stability_report.md
"""
import json
import math
import statistics
import sys
import time
from collections import defaultdict

from pymavlink import mavutil

PORT = "COM58"
BAUD = 115200

# Result accumulators
results = {"phase1": {}, "phase2": {}, "phase3": {}}


def banner(t):
    print(f"\n{'='*60}\n  {t}\n{'='*60}", flush=True)


def connect():
    print(f"[CONNECT] {PORT} @ {BAUD}", flush=True)
    m = mavutil.mavlink_connection(
        PORT, baud=BAUD, dialect="ardupilotmega",
        source_system=255, source_component=0,
    )
    hb = m.wait_heartbeat(timeout=15)
    if not hb:
        print("[FATAL] no HEARTBEAT")
        sys.exit(2)
    print(f"[OK] sys={m.target_system} comp={m.target_component} "
          f"type={mavutil.mavlink.enums['MAV_TYPE'].get(hb.type, '?').name} "
          f"state={mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status, '?').name}")
    return m, hb


# ---------------------------------------------------------------- Phase 1
def phase1(m):
    banner("Phase 1: 60s Health Baseline")
    sysid, compid = m.target_system, m.target_component
    # Request all streams at 4Hz
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    # Also explicitly request RAW_SENSORS and EXTRA1/2/3
    for stream_id in (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA3):
        m.mav.request_data_stream_send(sysid, compid, stream_id, 4, 1)

    DURATION = 60.0
    counts = defaultdict(int)
    timestamps = defaultdict(list)
    last = {}
    statustext = []
    ekf_samples = []
    meminfo_samples = []
    sys_status_first = None
    sys_status_last = None
    heartbeats = []
    raw_imu_samples = []
    vibe_samples = []

    print(f"[SAMPLING] {DURATION}s ...", flush=True)
    t0 = time.time()
    while time.time() - t0 < DURATION:
        msg = m.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "BAD_DATA":
            continue
        now = time.time()
        counts[mt] += 1
        timestamps[mt].append(now)
        last[mt] = msg
        if mt == "STATUSTEXT":
            sev = msg.severity
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            txt = txt.rstrip("\x00 \r\n")
            statustext.append({"t": now - t0, "sev": sev,
                               "sev_name": mavutil.mavlink.enums["MAV_SEVERITY"].get(sev, "?").name,
                               "text": txt})
        elif mt == "EKF_STATUS_REPORT":
            ekf_samples.append({
                "flags": msg.flags,
                "vel": msg.velocity_variance,
                "pos_h": msg.pos_horiz_variance,
                "pos_v": msg.pos_vert_variance,
                "compass": msg.compass_variance,
                "terrain": msg.terrain_alt_variance,
            })
        elif mt == "MEMINFO":
            meminfo_samples.append({
                "brkval": msg.brkval, "freemem": msg.freemem,
                "freemem32": getattr(msg, "freemem32", None),
            })
        elif mt == "SYS_STATUS":
            if sys_status_first is None:
                sys_status_first = msg
            sys_status_last = msg
        elif mt == "HEARTBEAT":
            heartbeats.append({
                "base_mode": msg.base_mode,
                "custom_mode": msg.custom_mode,
                "state": msg.system_status,
                "state_name": mavutil.mavlink.enums["MAV_STATE"].get(msg.system_status, "?").name,
            })
        elif mt == "RAW_IMU":
            raw_imu_samples.append((msg.xacc, msg.yacc, msg.zacc,
                                    msg.xgyro, msg.ygyro, msg.zgyro,
                                    msg.xmag, msg.ymag, msg.zmag,
                                    msg.temperature))
        elif mt == "VIBRATION":
            vibe_samples.append((msg.vibration_x, msg.vibration_y, msg.vibration_z,
                                 msg.clipping_0, msg.clipping_1, msg.clipping_2))

    actual = time.time() - t0
    print(f"[DONE] sampled {actual:.1f}s, {sum(counts.values())} msgs total", flush=True)

    # Compute per-stream rate (msgs/sec)
    rates = {}
    for mt_, ts in timestamps.items():
        if len(ts) >= 2:
            span = ts[-1] - ts[0]
            rates[mt_] = (len(ts) - 1) / span if span > 0 else 0.0
        else:
            rates[mt_] = 0.0

    # Decode sensor bitmaps
    sensor_decode = {}
    if sys_status_last:
        present = sys_status_last.onboard_control_sensors_present
        enabled = sys_status_last.onboard_control_sensors_enabled
        health  = sys_status_last.onboard_control_sensors_health
        sensor_decode = decode_sensor_bits(present, enabled, health)

    # IMU/compass stats
    imu_stats = {}
    if raw_imu_samples:
        cols = list(zip(*raw_imu_samples))
        names = ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro",
                 "xmag", "ymag", "zmag", "temp"]
        for name, col in zip(names, cols):
            imu_stats[name] = {
                "mean": statistics.mean(col),
                "stdev": statistics.pstdev(col) if len(col) > 1 else 0.0,
                "min": min(col), "max": max(col),
            }

    vibe_stats = {}
    if vibe_samples:
        vx = [v[0] for v in vibe_samples]
        vy = [v[1] for v in vibe_samples]
        vz = [v[2] for v in vibe_samples]
        clip = [v[3] + v[4] + v[5] for v in vibe_samples]
        vibe_stats = {
            "vx_mean": statistics.mean(vx), "vx_max": max(vx),
            "vy_mean": statistics.mean(vy), "vy_max": max(vy),
            "vz_mean": statistics.mean(vz), "vz_max": max(vz),
            "clipping_total": clip[-1] if clip else 0,
        }

    ekf_summary = {}
    if ekf_samples:
        for k in ("vel", "pos_h", "pos_v", "compass", "terrain"):
            vals = [s[k] for s in ekf_samples]
            ekf_summary[k] = {"max": max(vals), "mean": statistics.mean(vals)}
        ekf_summary["flags_last"] = ekf_samples[-1]["flags"]

    mem_summary = {}
    if meminfo_samples:
        free32 = [m_.get("freemem32") or m_["freemem"] for m_ in meminfo_samples]
        mem_summary = {
            "free_first": free32[0],
            "free_last": free32[-1],
            "free_delta": free32[-1] - free32[0],
            "free_min": min(free32),
            "samples": len(meminfo_samples),
        }

    state_changes = []
    if heartbeats:
        last_state = heartbeats[0]["state"]
        for hb in heartbeats[1:]:
            if hb["state"] != last_state:
                state_changes.append((last_state, hb["state"]))
                last_state = hb["state"]

    out = {
        "duration_s": actual,
        "msg_counts": dict(counts),
        "msg_rates": rates,
        "sensor_bitmap": sensor_decode,
        "imu_stats": imu_stats,
        "vibration": vibe_stats,
        "ekf": ekf_summary,
        "memory": mem_summary,
        "statustext": statustext,
        "heartbeat_state_changes": state_changes,
        "first_state": heartbeats[0] if heartbeats else None,
        "last_state": heartbeats[-1] if heartbeats else None,
    }

    # Print summary
    print("\n--- Phase 1 summary ---")
    print(f"Telemetry msg types: {len(counts)}")
    print("Top 12 streams (msg/s):")
    for k, r in sorted(rates.items(), key=lambda x: -x[1])[:12]:
        print(f"  {k:28s} {r:5.2f} Hz  ({counts[k]} msgs)")
    print(f"\nSTATUSTEXT: {len(statustext)} msgs")
    for s in statustext[:20]:
        print(f"  [{s['sev_name']}] @{s['t']:5.1f}s  {s['text']}")
    if sensor_decode:
        print("\nSensor health (present/enabled/health):")
        for s, st in sensor_decode.items():
            mark = "OK" if st["healthy"] else ("OFF" if not st["enabled"] else "BAD")
            print(f"  [{mark}] {s}  p={int(st['present'])} e={int(st['enabled'])} h={int(st['healthy'])}")
    if imu_stats:
        print("\nIMU stats:")
        for k in ("xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", "xmag", "ymag", "zmag", "temp"):
            v = imu_stats[k]
            print(f"  {k:6s}  mean={v['mean']:9.2f}  std={v['stdev']:7.2f}  range=({v['min']:7.1f},{v['max']:7.1f})")
        # Sanity: static board, accel magnitude should be ≈ 1000 mg
        ax = imu_stats["xacc"]["mean"]
        ay = imu_stats["yacc"]["mean"]
        az = imu_stats["zacc"]["mean"]
        amag = math.sqrt(ax*ax + ay*ay + az*az)
        print(f"  |accel| static = {amag:.1f} mg  (expect ~1000 mg = 1g)")
        out["accel_magnitude_static_mg"] = amag
    if vibe_stats:
        print(f"\nVibration: x={vibe_stats['vx_max']:.2f} y={vibe_stats['vy_max']:.2f} z={vibe_stats['vz_max']:.2f}  clipping_total={vibe_stats['clipping_total']}")
    if ekf_summary:
        print("\nEKF variances (max over 60s):")
        for k in ("vel", "pos_h", "pos_v", "compass", "terrain"):
            print(f"  {k:10s} max={ekf_summary[k]['max']:.3f}  mean={ekf_summary[k]['mean']:.3f}")
        print(f"  flags=0x{ekf_summary['flags_last']:04X}")
    if mem_summary:
        d = mem_summary["free_delta"]
        print(f"\nMemory: free first={mem_summary['free_first']}, last={mem_summary['free_last']}, delta={d:+d} bytes, min={mem_summary['free_min']}")

    results["phase1"] = out
    return out


# Sensor bitmap names from MAV_SYS_STATUS_SENSOR
SENSOR_BITS = [
    (0x01, "gyro_3d"), (0x02, "accel_3d"), (0x04, "mag_3d"),
    (0x08, "abs_pressure"), (0x10, "diff_pressure"),
    (0x20, "gps"), (0x40, "optical_flow"), (0x80, "vision_position"),
    (0x100, "laser_position"), (0x200, "external_gnd_truth"),
    (0x400, "angular_rate_ctrl"), (0x800, "attitude_stabilization"),
    (0x1000, "yaw_position"), (0x2000, "z_altitude_control"),
    (0x4000, "xy_position_control"),
    (0x8000, "motor_outputs"), (0x10000, "rc_receiver"),
    (0x20000, "gyro2"), (0x40000, "accel2"), (0x80000, "mag2"),
    (0x100000, "geofence"), (0x200000, "ahrs"), (0x400000, "terrain"),
    (0x800000, "reverse_motor"), (0x1000000, "logging"),
    (0x2000000, "battery"), (0x4000000, "proximity"),
    (0x8000000, "satcom"), (0x10000000, "prearm_check"),
    (0x20000000, "obstacle_avoidance"), (0x40000000, "propulsion"),
    (0x80000000, "extension_used"),
]


def decode_sensor_bits(present, enabled, health):
    out = {}
    for bit, name in SENSOR_BITS:
        p = bool(present & bit)
        e = bool(enabled & bit)
        h = bool(health & bit)
        if p or e or h:
            out[name] = {"present": p, "enabled": e, "healthy": h}
    return out


# ---------------------------------------------------------------- Phase 2
# Rover custom_mode values (ArduRover)
ROVER_MODES = {
    "MANUAL": 0,
    "ACRO": 1,
    "STEERING": 3,
    "HOLD": 4,
    "LOITER": 5,        # needs GPS
    "FOLLOW": 6,
    "SIMPLE": 7,
    "DOCK": 8,
    "AUTO": 10,         # needs GPS
    "RTL": 11,          # needs GPS
    "SMART_RTL": 12,    # needs GPS
    "GUIDED": 15,       # needs GPS
    "INITIALIZING": 16,
}


def phase2(m):
    banner("Phase 2: Mode Switching Matrix (GPS-free modes)")
    sysid, compid = m.target_system, m.target_component
    test_modes = ["MANUAL", "ACRO", "STEERING", "HOLD", "MANUAL"]
    transitions = []
    statustext = []
    drained = drain_msgs(m, 0.5)
    for target_name in test_modes:
        target = ROVER_MODES[target_name]
        print(f"\n[SWITCH] -> {target_name} (custom_mode={target})", flush=True)
        # Send DO_SET_MODE via SET_MODE
        m.mav.set_mode_send(
            sysid,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            target,
        )
        t0 = time.time()
        ack = None
        confirmed = False
        actual_mode = None
        # Watch for heartbeat with new custom_mode (preferred) AND any STATUSTEXT
        while time.time() - t0 < 4.0:
            msg = m.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == "STATUSTEXT":
                txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
                txt = txt.rstrip("\x00 \r\n")
                statustext.append({"target": target_name, "t": time.time() - t0,
                                   "sev": msg.severity,
                                   "sev_name": mavutil.mavlink.enums["MAV_SEVERITY"].get(msg.severity, "?").name,
                                   "text": txt})
            elif mt == "HEARTBEAT":
                actual_mode = msg.custom_mode
                if msg.custom_mode == target:
                    confirmed = True
                    break
            elif mt == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                ack = msg
        dt = time.time() - t0
        result_name = (mavutil.mavlink.enums["MAV_RESULT"].get(ack.result, "?").name
                       if ack else "no_ack")
        transitions.append({
            "target": target_name, "target_id": target,
            "confirmed": confirmed,
            "actual_mode_id": actual_mode,
            "ack_result": result_name,
            "time_s": dt,
        })
        print(f"  result: confirmed={confirmed}  ack={result_name}  dt={dt:.2f}s  actual_mode_id={actual_mode}")
        # short rest
        time.sleep(0.5)

    out = {
        "transitions": transitions,
        "statustext": statustext,
        "pass": all(t["confirmed"] for t in transitions),
    }
    print(f"\n--- Phase 2 summary ---  pass={out['pass']}")
    for tr in transitions:
        mark = "OK " if tr["confirmed"] else "FAIL"
        print(f"  [{mark}] -> {tr['target']:10s} ack={tr['ack_result']:25s} dt={tr['time_s']:.2f}s")
    if statustext:
        print(f"  STATUSTEXT during mode switches: {len(statustext)}")
        for s in statustext[:10]:
            print(f"    [{s['sev_name']}] {s['target']} @{s['t']:.1f}s: {s['text']}")
    results["phase2"] = out
    return out


def drain_msgs(m, dur):
    t0 = time.time()
    n = 0
    while time.time() - t0 < dur:
        msg = m.recv_match(blocking=True, timeout=0.1)
        if msg:
            n += 1
    return n


# ---------------------------------------------------------------- Phase 3
def phase3(m):
    banner("Phase 3: Servo Output via DO_MOTOR_TEST (low power)")
    sysid, compid = m.target_system, m.target_component
    # Make sure we are in MANUAL or HOLD so motor test is safe
    m.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        ROVER_MODES["HOLD"])
    time.sleep(0.5)
    drain_msgs(m, 0.5)

    # Bump SERVO_OUTPUT_RAW rate
    m.mav.request_data_stream_send(
        sysid, compid, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1)

    tests = []
    # MAV_CMD_DO_MOTOR_TEST: param1=motor(1=throttle),
    # param2=throttle type (0=PWM, 1=%full, 2=%pilot),
    # param3=value, param4=timeout_s, param5=motor_count
    # Rover: motor 1 = throttle, motor 2 = steering for skid/boat
    cases = [
        ("throttle_+10%", 1, 1, 10, 1.0),
        ("throttle_-10%", 1, 1, -10, 1.0),
        ("steering_+20%", 2, 1, 20, 1.0),
        ("steering_-20%", 2, 1, -20, 1.0),
    ]
    for name, motor, thr_type, value, timeout in cases:
        print(f"\n[TEST] {name}: motor={motor} type={thr_type} value={value}% dur={timeout}s")
        # Sample baseline 0.5s
        base = capture_servos(m, 0.5)
        # Send test
        m.mav.command_long_send(
            sysid, compid,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,
            motor, thr_type, value, timeout, 0, 0, 0,
        )
        # Wait + sample during pulse
        during = capture_servos(m, timeout + 0.3)
        ack = m.recv_match(type="COMMAND_ACK", blocking=False, timeout=0.5)
        ack_result = "no_ack"
        if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
            ack_result = mavutil.mavlink.enums["MAV_RESULT"].get(ack.result, "?").name
        # Wait for motor to stop, sample 0.5s after
        time.sleep(0.5)
        rest = capture_servos(m, 0.5)
        rec = {
            "name": name, "motor": motor, "value": value, "timeout": timeout,
            "ack": ack_result,
            "base":   {f"servo{k+1}": base[k]  for k in range(8) if base[k]  is not None},
            "during": {f"servo{k+1}": during[k] for k in range(8) if during[k] is not None},
            "rest":   {f"servo{k+1}": rest[k]   for k in range(8) if rest[k]   is not None},
        }
        tests.append(rec)
        print(f"  ack={ack_result}")
        print(f"  baseline:  servo1={fmt(base[0])}  servo3={fmt(base[2])}")
        print(f"  during  :  servo1={fmt(during[0])}  servo3={fmt(during[2])}")
        print(f"  rest    :  servo1={fmt(rest[0])}  servo3={fmt(rest[2])}")
        time.sleep(0.5)

    out = {"tests": tests}
    results["phase3"] = out
    return out


def fmt(v):
    if v is None:
        return "  -  "
    mn, mx, mean = v
    return f"({mn}..{mx} m={mean:.0f})"


def capture_servos(m, dur):
    t0 = time.time()
    chans = [[] for _ in range(8)]
    while time.time() - t0 < dur:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.2)
        if msg is None:
            continue
        for i in range(8):
            v = getattr(msg, f"servo{i+1}_raw", 0)
            if v != 0:
                chans[i].append(v)
    out = []
    for c in chans:
        if c:
            out.append((min(c), max(c), sum(c) / len(c)))
        else:
            out.append(None)
    return out


# ---------------------------------------------------------------- main
def main():
    m, hb0 = connect()
    p1 = phase1(m)
    p2 = phase2(m)
    p3 = phase3(m)

    for k, v in results.items():
        with open(f"_stability_{k}.json", "w", encoding="utf-8") as f:
            json.dump(v, f, indent=2, default=str)
    print("\nJSON saved.")
    write_report()


def write_report():
    p1 = results["phase1"]
    p2 = results["phase2"]
    p3 = results["phase3"]

    lines = []
    lines.append("# USV Stability Test Report")
    lines.append(f"\nDate: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"\nPort: {PORT} @ {BAUD}")
    lines.append("\n## Phase 1 — 60s Health Baseline\n")
    lines.append(f"- Duration: {p1.get('duration_s', 0):.1f}s")
    lines.append(f"- Telemetry msg types: {len(p1.get('msg_counts', {}))}")
    if p1.get("accel_magnitude_static_mg"):
        amag = p1["accel_magnitude_static_mg"]
        lines.append(f"- Static |accel| = {amag:.1f} mg (expect ~1000)")
        if abs(amag - 1000) > 100:
            lines.append(f"  - ⚠ Off by {amag - 1000:+.0f} mg from 1g")
        else:
            lines.append(f"  - ✓ Within 100 mg tolerance")
    # Gyro noise on static board
    if p1.get("imu_stats"):
        gx = p1["imu_stats"]["xgyro"]["stdev"]
        gy = p1["imu_stats"]["ygyro"]["stdev"]
        gz = p1["imu_stats"]["zgyro"]["stdev"]
        lines.append(f"- Static gyro stdev: x={gx:.2f} y={gy:.2f} z={gz:.2f} mrad/s")
    # Sensor health
    sb = p1.get("sensor_bitmap", {})
    if sb:
        bad = [n for n, st in sb.items() if st["enabled"] and not st["healthy"]]
        if bad:
            lines.append(f"- ⚠ Unhealthy enabled sensors: {', '.join(bad)}")
        else:
            lines.append("- ✓ All enabled sensors healthy")
        off = [n for n, st in sb.items() if st["present"] and not st["enabled"]]
        if off:
            lines.append(f"- (Present but disabled: {', '.join(off)})")
    # EKF
    ekf = p1.get("ekf", {})
    if ekf:
        var_vel = ekf.get("vel", {}).get("max", 0)
        var_compass = ekf.get("compass", {}).get("max", 0)
        lines.append(f"- EKF max variance: vel={var_vel:.3f} compass={var_compass:.3f}")
        if var_vel > 1.0 or var_compass > 1.0:
            lines.append("  - ⚠ High EKF variance (>1.0)")
    # Memory
    mem = p1.get("memory", {})
    if mem:
        d = mem["free_delta"]
        lines.append(f"- Memory: free_first={mem['free_first']}, free_last={mem['free_last']}, delta={d:+d}, min={mem['free_min']}")
        if d < -1024:
            lines.append(f"  - ⚠ Free memory dropped by {-d} bytes in 60s (possible leak)")
    # STATUSTEXT
    st = p1.get("statustext", [])
    crits = [s for s in st if s["sev"] <= 2]
    if crits:
        lines.append(f"- ⚠ {len(crits)} CRITICAL/ALERT/EMERGENCY messages")
        for s in crits[:10]:
            lines.append(f"  - [{s['sev_name']}] {s['text']}")
    warns = [s for s in st if s["sev"] == 4]
    if warns:
        lines.append(f"- {len(warns)} WARNING messages")
        for s in warns[:10]:
            lines.append(f"  - {s['text']}")
    notices = [s for s in st if s["sev"] >= 5]
    if notices:
        lines.append(f"- {len(notices)} NOTICE/INFO messages")
        for s in notices[:5]:
            lines.append(f"  - {s['text']}")

    lines.append("\n## Phase 2 — Mode Switching Matrix\n")
    pass_count = sum(1 for t in p2["transitions"] if t["confirmed"])
    lines.append(f"- {pass_count}/{len(p2['transitions'])} transitions confirmed")
    for t in p2["transitions"]:
        mark = "✓" if t["confirmed"] else "✗"
        lines.append(f"  - {mark} → {t['target']:10s}  ack={t['ack_result']}  dt={t['time_s']:.2f}s")
    if p2["statustext"]:
        lines.append(f"- STATUSTEXT during mode switches: {len(p2['statustext'])}")
        for s in p2["statustext"][:8]:
            lines.append(f"  - [{s['sev_name']}] {s['target']}: {s['text']}")

    lines.append("\n## Phase 3 — Servo Output Verification\n")
    for t in p3["tests"]:
        lines.append(f"### {t['name']} (motor={t['motor']}, value={t['value']}%)")
        lines.append(f"  - ACK: `{t['ack']}`")
        # check that during-pulse servo deviates from baseline
        servo_id = "servo1" if t["motor"] == 1 else "servo3"
        b = t["base"].get(servo_id)
        d = t["during"].get(servo_id)
        r = t["rest"].get(servo_id)
        if d:
            lines.append(f"  - baseline {servo_id}: mean={b[2]:.0f}" if b else "  - baseline: -")
            lines.append(f"  - during   {servo_id}: range {d[0]}..{d[1]} mean={d[2]:.0f}")
            lines.append(f"  - rest     {servo_id}: mean={r[2]:.0f}" if r else "  - rest: -")
            if b and abs(d[2] - b[2]) > 20:
                lines.append(f"  - ✓ servo deviated by {d[2]-b[2]:+.0f} μs during pulse")
            else:
                lines.append(f"  - ⚠ no significant servo response detected")
            if r and abs(r[2] - 1500) > 30:
                lines.append(f"  - ⚠ servo did not return to neutral (1500), now {r[2]:.0f}")
            else:
                lines.append("  - ✓ servo returned near neutral after pulse")

    lines.append("\n## Stability Recommendations\n")
    recs = build_recommendations(p1, p2, p3)
    for r in recs:
        lines.append(f"- {r}")

    txt = "\n".join(lines)
    with open("_stability_report.md", "w", encoding="utf-8") as f:
        f.write(txt)
    print("\n" + txt)
    print(f"\n[REPORT] _stability_report.md ({len(txt)} chars)")


def build_recommendations(p1, p2, p3):
    recs = []
    # Static accel sanity
    amag = p1.get("accel_magnitude_static_mg", 0)
    if amag and abs(amag - 1000) > 100:
        recs.append(f"Run INS_ACCSCAL — static |accel|={amag:.0f} mg is off 1g by {amag-1000:+.0f} mg.")
    # Gyro noise
    imu = p1.get("imu_stats", {})
    if imu:
        gnoise = max(imu["xgyro"]["stdev"], imu["ygyro"]["stdev"], imu["zgyro"]["stdev"])
        if gnoise > 5.0:
            recs.append(f"High static gyro noise ({gnoise:.1f}). Check IMU mounting/vibration damping.")
    # Sensor bitmap
    bad = []
    sb = p1.get("sensor_bitmap", {})
    for n, st in sb.items():
        if st["enabled"] and not st["healthy"]:
            bad.append(n)
    if bad:
        recs.append(f"Unhealthy sensors block prearm: {', '.join(bad)}. Disable via params if not used (e.g., BATT_MONITOR=0, ARMING_CHECK bitmask).")
    if "rc_receiver" in sb and sb["rc_receiver"]["enabled"] and not sb["rc_receiver"]["healthy"]:
        recs.append("RC failsafe blocking prearm — set FS_THR_ENABLE=0 if you control via MAVLink only.")
    if "gps" in sb and sb["gps"]["enabled"] and not sb["gps"]["healthy"]:
        recs.append("GPS unhealthy indoor — expected. Outdoor with sky view will fix; meanwhile use modes that don't need GPS (MANUAL/HOLD/ACRO/STEERING).")
    if "battery" in sb and sb["battery"]["enabled"] and not sb["battery"]["healthy"]:
        recs.append("Battery monitor enabled but unhealthy. Set BATT_MONITOR=0 (or wire BMS) to clear.")
    # EKF
    ekf = p1.get("ekf", {})
    if ekf:
        if ekf.get("compass", {}).get("max", 0) > 1.0:
            recs.append("EKF compass variance high — run COMPASS_CAL outdoors away from metal.")
        if ekf.get("vel", {}).get("max", 0) > 1.0:
            recs.append("EKF velocity variance high — likely GPS no-fix indoor. Re-check outdoors.")
    # Memory leak
    mem = p1.get("memory", {})
    if mem.get("free_delta", 0) < -512:
        recs.append(f"Memory dropped {-mem['free_delta']} bytes in 60s — investigate longer soak (Phase 7).")
    # State
    last_st = p1.get("last_state", {}).get("state_name", "")
    if last_st == "CRITICAL":
        recs.append("System remained MAV_STATE_CRITICAL — caused by failed prearm; address sensor bitmap above.")
    if not p2.get("pass"):
        recs.append("Some mode transitions failed — review Phase 2 ACKs above.")
    if not recs:
        recs.append("No stability issues detected in Phase 1-3. Recommend Phase 7 long soak before sea trial.")
    return recs


if __name__ == "__main__":
    main()
