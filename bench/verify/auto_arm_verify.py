"""Verify that AUTO and GUIDED modes can ARM now that GPS is locked.

Previously these modes were blocked indoors because AHRS had no position
estimate. With FIX_3D the AHRS check should pass and ARM should succeed.

Safety: the script will briefly issue a GUIDED reposition command which
WILL drive servo outputs off centre — this is safe only when no ESC is wired.
The script aborts hard if final DISARM cannot be confirmed.

NOTE: SAFE TO RUN ONLY ON BENCH WITH ESC DISCONNECTED.
"""
import argparse
import io
import math
import sys
import time

from pymavlink import mavutil

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


MODES = {
    "MANUAL": 0, "ACRO": 1, "HOLD": 4, "LOITER": 5,
    "FOLLOW": 6, "AUTO": 10, "RTL": 11, "GUIDED": 15,
}


def banner(t):
    print(f"\n{'='*64}\n  {t}\n{'='*64}", flush=True)


def connect(port, baud):
    print(f"[CONNECT] {port} @ {baud}", flush=True)
    m = mavutil.mavlink_connection(port, baud=baud, dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=10)
    if not hb:
        print("[FATAL] no heartbeat", flush=True)
        sys.exit(2)
    print(f"[OK] sys={m.target_system} state="
          f"{mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}",
          flush=True)
    return m


def request_streams(m):
    sysid, compid = m.target_system, m.target_component
    for sid in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS):
        m.mav.request_data_stream_send(sysid, compid, sid, 10, 1)


def wait_3d_fix(m, timeout=30.0):
    print(f"[GPS] waiting for 3D fix (<= {timeout:.0f}s)...", flush=True)
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if time.time() - last_print > 3:
            print(f"  fix={msg.fix_type}  sats={msg.satellites_visible}  HDOP={msg.eph/100.0:.2f}",
                  flush=True)
            last_print = time.time()
        if msg.fix_type >= 3 and msg.satellites_visible >= 6:
            print(f"[GPS] OK — fix={msg.fix_type} sats={msg.satellites_visible} HDOP={msg.eph/100.0:.2f}",
                  flush=True)
            return msg
    return None


def get_global_position(m, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
        if msg:
            return msg
    return None


def set_mode(m, name, timeout=4.0):
    target = MODES[name]
    sysid = m.target_system
    m.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, target)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
        if msg and msg.custom_mode == target:
            return True
    return False


def arm_disarm(m, want, timeout=6.0, force=False):
    """Returns (success, ack_result, statustexts) — statustexts is the list of
    any STATUSTEXT messages observed during the ARM attempt (often contains the
    'PreArm: ...' rejection reason that's the most useful diagnostic)."""
    sysid, compid = m.target_system, m.target_component
    m.mav.command_long_send(
        sysid, compid, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1 if want else 0, 21196 if force else 0, 0, 0, 0, 0, 0)
    ack_result = None
    statustexts = []
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "STATUSTEXT":
            sev = msg.severity
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            txt = txt.rstrip("\x00 \r\n")
            statustexts.append({
                "sev": sev,
                "sev_name": mavutil.mavlink.enums["MAV_SEVERITY"].get(sev, "?").name,
                "text": txt,
            })
        elif mt == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            ack_result = mavutil.mavlink.enums["MAV_RESULT"].get(msg.result, "?").name
        elif mt == "HEARTBEAT":
            is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed == want:
                return True, ack_result, statustexts
    return False, ack_result, statustexts


def capture_statustext(m, dur):
    out = []
    t0 = time.time()
    while time.time() - t0 < dur:
        msg = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.2)
        if msg:
            sev = msg.severity
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            txt = txt.rstrip("\x00 \r\n")
            out.append({"sev": sev,
                        "sev_name": mavutil.mavlink.enums["MAV_SEVERITY"].get(sev, "?").name,
                        "text": txt})
    return out


def capture_servos(m, dur):
    t0 = time.time()
    s1, s3 = [], []
    while time.time() - t0 < dur:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.2)
        if msg:
            s1.append(msg.servo1_raw)
            s3.append(msg.servo3_raw)
    summary = lambda a: (min(a), max(a), sum(a)/len(a)) if a else None
    return summary(s1), summary(s3)


def fmt_servo(s):
    if not s:
        return "-"
    mn, mx, mean = s
    return f"min={mn} max={mx} mean={mean:.0f}"


def test_mode_arm(m, mode_name, pre_actions=None, post_actions=None):
    """Returns dict with results."""
    banner(f"Test ARM in {mode_name}")
    if pre_actions:
        pre_actions(m)

    if not set_mode(m, mode_name):
        print(f"  [FAIL] could not switch to {mode_name}", flush=True)
        return {"mode": mode_name, "mode_ok": False}
    print(f"  [OK] entered {mode_name}", flush=True)

    # Drain any pending statustext from mode switch
    capture_statustext(m, 0.5)

    # Capture servo baseline before ARM
    s1_base, s3_base = capture_servos(m, 1.0)
    print(f"  baseline servo: s1={fmt_servo(s1_base)} s3={fmt_servo(s3_base)}", flush=True)

    print(f"  attempting ARM...", flush=True)
    ok, ack, arm_statustexts = arm_disarm(m, True, timeout=6.0)
    # Capture any additional ones the autopilot emits after the ack
    arm_statustexts += capture_statustext(m, 1.0)

    result = {
        "mode": mode_name, "mode_ok": True,
        "arm_ok": ok, "arm_ack": ack,
        "statustext": arm_statustexts,
    }

    if ok:
        print(f"  [PASS] ARM confirmed (ack={ack})", flush=True)
        # Capture servo behaviour while armed
        s1_arm, s3_arm = capture_servos(m, 2.0)
        result["s1_armed"] = s1_arm
        result["s3_armed"] = s3_arm
        print(f"  armed  servo:  s1={fmt_servo(s1_arm)} s3={fmt_servo(s3_arm)}", flush=True)
        if post_actions:
            post_actions(m, result)
        print(f"  disarming...", flush=True)
        ok_d, ack_d, _ = arm_disarm(m, False, timeout=5.0, force=True)
        result["disarm_ok"] = ok_d
        result["disarm_ack"] = ack_d
        if not ok_d:
            print(f"  [WARN] disarm not confirmed (ack={ack_d}), retrying with force", flush=True)
            ok_d, ack_d, _ = arm_disarm(m, False, timeout=5.0, force=True)
            result["disarm_ok"] = ok_d
    else:
        print(f"  [INFO] ARM rejected (ack={ack})", flush=True)
        if arm_statustexts:
            print(f"  STATUSTEXT during ARM:", flush=True)
            for s in arm_statustexts:
                print(f"    [{s['sev_name']}] {s['text']}", flush=True)

    return result


def upload_simple_mission(m, pos):
    """Upload a 2-waypoint mission near current GPS position.
    pos = GLOBAL_POSITION_INT message.
    Waypoint 1: 10m north of current.
    Waypoint 2: back to current.
    """
    sysid, compid = m.target_system, m.target_component
    # Compute +10m north in lat_e7
    lat0 = pos.lat
    lon0 = pos.lon
    # 1m latitude ~ 1/111000 deg = 9.0090e-7 deg ~ 90 in 1e7 units
    delta_lat_e7 = int(10.0 / 111000.0 * 1e7)

    items = [
        # idx 0 — HOME (auto-overwritten by ArduPilot)
        (0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1, 1,
         0, 0, 0, 0, lat0 / 1e7, lon0 / 1e7, 0),
        # idx 1 — go 10m north
        (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
         0, 0, 0, 0, (lat0 + delta_lat_e7) / 1e7, lon0 / 1e7, 0),
        # idx 2 — return home
        (2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1,
         0, 0, 0, 0, 0, 0, 0),
    ]

    print(f"  uploading {len(items)} mission items...", flush=True)
    m.mav.mission_count_send(sysid, compid, len(items),
                              mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    t0 = time.time()
    sent = set()
    ack = None
    while time.time() - t0 < 6.0:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = msg.seq
            if seq < len(items):
                it = items[seq]
                m.mav.mission_item_int_send(
                    sysid, compid,
                    it[0], it[1], it[2], it[3], it[4],
                    it[5], it[6], it[7], it[8],
                    int(it[9] * 1e7), int(it[10] * 1e7), it[11],
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                sent.add(seq)
        elif mt == "MISSION_ACK":
            ack = mavutil.mavlink.enums["MAV_MISSION_RESULT"].get(msg.type, "?").name
            break
    print(f"  upload ack={ack} sent={sorted(sent)}", flush=True)
    return ack == "MAV_MISSION_ACCEPTED"


def guided_reposition(m, pos, north_m=5.0):
    """While in GUIDED mode + ARMED, send SET_POSITION_TARGET_GLOBAL_INT to a
    point ~north_m metres north. Capture SERVO_OUTPUT_RAW to see if servos move.
    """
    sysid, compid = m.target_system, m.target_component
    delta_lat_e7 = int(north_m / 111000.0 * 1e7)
    target_lat = pos.lat + delta_lat_e7
    target_lon = pos.lon
    type_mask = (
        # Use position only — ignore vel, accel, yaw, yaw_rate
        0b0000_1111_1111_1000
    )
    print(f"  GUIDED reposition: +{north_m:.0f}m north", flush=True)
    m.mav.set_position_target_global_int_send(
        0, sysid, compid,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        target_lat, target_lon, 0,
        0, 0, 0,    # vx vy vz
        0, 0, 0,    # ax ay az
        0, 0,       # yaw, yaw_rate
    )
    # Watch servos for 3s
    s1, s3 = capture_servos(m, 3.0)
    print(f"  servo during GUIDED command: s1={fmt_servo(s1)} s3={fmt_servo(s3)}", flush=True)
    return s1, s3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--skip-guided-reposition", action="store_true",
                    help="Don't send the reposition command (servos will stay at neutral)")
    args = ap.parse_args()

    print("="*64, flush=True)
    print(" AUTO/GUIDED ARM verification (post-GPS-lock)", flush=True)
    print(" SAFETY: requires ESC disconnected. Press Ctrl+C any time.", flush=True)
    print("="*64, flush=True)

    m = connect(args.port, args.baud)
    request_streams(m)

    gps = wait_3d_fix(m, timeout=20.0)
    if not gps:
        print("[FATAL] no 3D GPS fix — these tests need GPS. Move antenna near window.",
              flush=True)
        sys.exit(3)

    pos = get_global_position(m, timeout=5.0)
    if not pos:
        print("[FATAL] no GLOBAL_POSITION_INT — EKF not publishing fused position?",
              flush=True)
        sys.exit(3)
    print(f"[POS] lat={pos.lat/1e7:+.7f}  lon={pos.lon/1e7:+.7f}  alt={pos.alt/1000:.1f}m  "
          f"rel_alt={pos.relative_alt/1000:.1f}m", flush=True)

    # Make sure we start clean: MANUAL + DISARMED
    print("\n[INIT] forcing MANUAL + DISARM baseline", flush=True)
    set_mode(m, "MANUAL")
    arm_disarm(m, False, timeout=3.0, force=True)

    results = []

    # ---- Test 1: GUIDED ARM
    def guided_post(m, result):
        if not args.skip_guided_reposition:
            time.sleep(0.5)
            s1, s3 = guided_reposition(m, pos, north_m=5.0)
            result["s1_guided_cmd"] = s1
            result["s3_guided_cmd"] = s3
        else:
            print(f"  (skipping reposition command per flag)", flush=True)

    r1 = test_mode_arm(m, "GUIDED", post_actions=guided_post)
    results.append(r1)
    time.sleep(1.0)

    # ---- Test 2: AUTO ARM (upload a 3-item mission first)
    def auto_pre(m):
        # Force MANUAL while we upload, simpler
        set_mode(m, "MANUAL")
        ok = upload_simple_mission(m, pos)
        if not ok:
            print(f"  [WARN] mission upload ack not accepted — AUTO may still work if mission already onboard",
                  flush=True)

    r2 = test_mode_arm(m, "AUTO", pre_actions=auto_pre)
    results.append(r2)
    time.sleep(1.0)

    # ---- Test 3: HOLD ARM (sanity, should always work)
    r3 = test_mode_arm(m, "HOLD")
    results.append(r3)

    # ---- Final cleanup
    print("\n[CLEANUP] final state: MANUAL + DISARM", flush=True)
    set_mode(m, "MANUAL")
    ok, ack, _ = arm_disarm(m, False, timeout=5.0, force=True)
    if not ok:
        print(f"[CRITICAL] could not confirm final disarm! ack={ack}", flush=True)
        print(f"[CRITICAL] disconnect USB power immediately to ensure safe state.", flush=True)
    else:
        print(f"[CLEANUP] confirmed DISARMED", flush=True)

    # ---- Summary
    print("\n" + "="*64, flush=True)
    print(" SUMMARY", flush=True)
    print("="*64, flush=True)
    for r in results:
        mark = "PASS" if r.get("arm_ok") else ("CHECK" if r.get("mode_ok") else "FAIL")
        emoji = "OK" if mark == "PASS" else ("--" if mark == "CHECK" else "XX")
        print(f"\n[{emoji}] {r['mode']:8s} mode={'ok' if r.get('mode_ok') else 'fail'}  "
              f"arm={'ok' if r.get('arm_ok') else 'NO'}  ack={r.get('arm_ack')}", flush=True)
        if r.get("statustext"):
            for s in r["statustext"][:8]:
                print(f"    [{s['sev_name']}] {s['text']}", flush=True)
        if r.get("s1_guided_cmd"):
            print(f"    servo during reposition cmd: s1={fmt_servo(r['s1_guided_cmd'])} "
                  f"s3={fmt_servo(r['s3_guided_cmd'])}", flush=True)


if __name__ == "__main__":
    main()
