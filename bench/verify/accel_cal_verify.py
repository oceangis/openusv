"""Verify accel calibration result on COM58.

1. Read INS_ACCOFFS_*, INS_ACCSCAL_*, AHRS_TRIM_*
2. Sample 10s of RAW_IMU at level position
3. Check SYS_STATUS — accel + ahrs should be healthy now
4. Capture any prearm errors remaining
"""
import math
import statistics
import time
from collections import defaultdict
from pymavlink import mavutil

PORT = "COM58"
BAUD = 115200


def read_param(m, name, timeout=3):
    sysid, compid = m.target_system, m.target_component
    m.mav.param_request_read_send(sysid, compid, name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if msg is None:
            continue
        pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
        pid = pid.strip("\x00")
        if pid == name:
            return msg.param_value
    return None


def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                   source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=10)
    print(f"connected, state="
          f"{mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}")

    print("\n=== INS calibration values ===")
    for p in ("INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z",
              "INS_ACCSCAL_X", "INS_ACCSCAL_Y", "INS_ACCSCAL_Z",
              "INS_GYROFFS_X", "INS_GYROFFS_Y", "INS_GYROFFS_Z",
              "AHRS_TRIM_X", "AHRS_TRIM_Y", "AHRS_TRIM_Z"):
        v = read_param(m, p)
        print(f"  {p:15s} = {v}")

    print("\n=== Sampling 10s of RAW_IMU (board level) ===")
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10, 1)
    samples = []
    statustext = []
    t0 = time.time()
    while time.time() - t0 < 10:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "RAW_IMU":
            samples.append((msg.xacc, msg.yacc, msg.zacc,
                            msg.xgyro, msg.ygyro, msg.zgyro))
        elif msg.get_type() == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            statustext.append((msg.severity, txt.rstrip("\x00 \r\n")))

    if samples:
        cols = list(zip(*samples))
        names = ["xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro"]
        print(f"  Samples: {len(samples)}")
        print(f"  {'ch':6s} {'mean':>9s} {'stdev':>9s} {'min':>7s} {'max':>7s}")
        for n, c in zip(names, cols):
            print(f"  {n:6s} {statistics.mean(c):9.2f} {statistics.pstdev(c):9.2f}"
                  f" {min(c):7d} {max(c):7d}")
        ax = statistics.mean(cols[0])
        ay = statistics.mean(cols[1])
        az = statistics.mean(cols[2])
        amag = math.sqrt(ax*ax + ay*ay + az*az)
        print(f"\n  |accel| = {amag:.1f} mg  (expect ≈1000)")
        if abs(amag - 1000) < 30:
            print("  ✓ Within 30 mg of 1g")
        else:
            print(f"  ⚠ Off 1g by {amag - 1000:+.1f} mg")
        tilt_deg = math.degrees(math.atan2(math.sqrt(ax*ax + ay*ay), abs(az)))
        print(f"  Tilt angle = {tilt_deg:.3f}°")

    print(f"\n=== STATUSTEXT during 10s ===")
    crit = [s for s in statustext if s[0] <= 2]
    print(f"  {len(crit)} CRITICAL/ALERT/EMERGENCY")
    for sev, txt in statustext:
        sevname = mavutil.mavlink.enums["MAV_SEVERITY"].get(sev, "?").name
        print(f"  [{sevname}] {txt}")

    # SYS_STATUS
    print(f"\n=== SYS_STATUS sensor health ===")
    ss = m.recv_match(type="SYS_STATUS", blocking=True, timeout=3)
    if ss:
        for bit, name in [
            (0x01,"gyro"),(0x02,"accel"),(0x04,"mag"),
            (0x20,"gps"),(0x200000,"ahrs"),(0x1000000,"logging"),
            (0x10000,"rc"),(0x2000000,"battery"),(0x10000000,"prearm"),
        ]:
            p = bool(ss.onboard_control_sensors_present & bit)
            e = bool(ss.onboard_control_sensors_enabled & bit)
            h = bool(ss.onboard_control_sensors_health  & bit)
            mark = "  -  " if not e else ("OK " if h else "BAD")
            print(f"  [{mark}] {name:8s} p={int(p)} e={int(e)} h={int(h)}")

    # Final heartbeat
    hb2 = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    if hb2:
        print(f"\nHEARTBEAT state: "
              f"{mavutil.mavlink.enums['MAV_STATE'].get(hb2.system_status,'?').name}")


if __name__ == "__main__":
    main()
