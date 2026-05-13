"""Switch from Ackermann (steer+throttle) to skid-steering (dual throttle).

  SERVO1_FUNCTION: 26 (GroundSteering) -> 73 (ThrottleLeft)
  SERVO3_FUNCTION: 70 (Throttle)       -> 74 (ThrottleRight)

After change + reboot:
  - have_skid_steering() should return true
  - SERVO_OUTPUT_RAW servo1 should respond to throttle*(1-steering_factor)
  - SERVO_OUTPUT_RAW servo3 should respond to throttle*(1+steering_factor)

Verification:
  1. Read pre values
  2. PARAM_SET both
  3. Verify echo + RAM
  4. Soft reboot
  5. Verify FLASH persistence
  6. Test: in MANUAL (or HOLD), inject a steering command via RC_CHANNELS_OVERRIDE
     and check servo1/servo3 diverge as expected
"""
import sys
import time
from pymavlink import mavutil

PORT = "COM58"
BAUD = 115200


def robust_read(m, name, retries=3):
    sysid, compid = m.target_system, m.target_component
    for _ in range(retries):
        m.mav.param_request_read_send(sysid, compid, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 1.2:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is None:
                continue
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            pid = pid.strip("\x00")
            if pid == name:
                return msg.param_value, msg.param_type
        time.sleep(0.1)
    return None, None


def robust_write(m, name, value, ptype, retries=3, timeout_each=2.5):
    sysid, compid = m.target_system, m.target_component
    for attempt in range(retries):
        m.mav.param_set_send(sysid, compid, name.encode(), float(value), ptype)
        t0 = time.time()
        while time.time() - t0 < timeout_each:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
            if msg is None:
                continue
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            pid = pid.strip("\x00")
            if pid == name and abs(msg.param_value - value) < 0.01:
                return msg.param_value, attempt + 1
        time.sleep(0.2)
    # echo lost? readback
    v, _ = robust_read(m, name)
    if v is not None and abs(v - value) < 0.01:
        return v, retries
    return None, retries


def reboot(m):
    sysid, compid = m.target_system, m.target_component
    print("\n[REBOOT]")
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    m.close()
    time.sleep(8)
    m2 = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    m2.wait_heartbeat(timeout=15)
    time.sleep(8)
    return m2


CHANGES = [
    ("SERVO1_FUNCTION", 73, "ThrottleLeft  (-> GPIO 12 / H10 pin5)"),
    ("SERVO3_FUNCTION", 74, "ThrottleRight (-> GPIO 45 / H10 pin1)"),
]


def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                   source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=10)
    print(f"connected  state="
          f"{mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}")

    print("\n=== Pre-change ===")
    for name, target, _ in CHANGES:
        v, t = robust_read(m, name)
        print(f"  {name:18s} = {v}  type={t}")

    print("\n=== Apply ===")
    for name, target, label in CHANGES:
        v, t = robust_read(m, name)
        if v is None:
            print(f"  SKIP {name}: not readable")
            continue
        echoed, attempts = robust_write(m, name, target, t)
        ok = echoed is not None and abs(echoed - target) < 0.01
        mark = "✓" if ok else "✗"
        print(f"  [{mark}] {name:18s}: {v} -> {target}  echo={echoed}  attempts={attempts}  ({label})")

    print("\n=== RAM verify ===")
    time.sleep(0.5)
    for name, target, _ in CHANGES:
        v, _ = robust_read(m, name)
        ok = v is not None and abs(v - target) < 0.01
        mark = "✓" if ok else "✗"
        print(f"  [{mark}] {name:18s} = {v}")

    m = reboot(m)

    print("\n=== Post-reboot FLASH verify ===")
    all_ok = True
    for name, target, _ in CHANGES:
        v, _ = robust_read(m, name)
        ok = v is not None and abs(v - target) < 0.01
        mark = "✓" if ok else "✗"
        if not ok:
            all_ok = False
        print(f"  [{mark}] {name:18s} = {v}  (expected {target})")

    # Look for any boot-time STATUSTEXT about skid steering
    print("\n=== Boot STATUSTEXT (10s) ===")
    sysid, compid = m.target_system, m.target_component
    m.mav.request_data_stream_send(sysid, compid,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    t0 = time.time()
    while time.time() - t0 < 10:
        msg = m.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            sev = mavutil.mavlink.enums["MAV_SEVERITY"].get(msg.severity, "?").name
            txt = txt.rstrip("\x00 \r\n")
            print(f"  @{time.time()-t0:.1f}s [{sev}] {txt}")

    # Quick servo response test: switch to MANUAL, inject RC override, observe SERVO1 vs SERVO3
    print("\n=== Quick servo response test (MANUAL + RC override) ===")
    # Switch to MANUAL
    m.mav.set_mode_send(sysid,
                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                         0)
    time.sleep(0.6)

    # Baseline (no input)
    t0 = time.time()
    base = {1: [], 3: []}
    while time.time() - t0 < 1.5:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.3)
        if msg:
            base[1].append(msg.servo1_raw)
            base[3].append(msg.servo3_raw)
    print(f"  baseline servo1={sorted(set(base[1]))}  servo3={sorted(set(base[3]))}")

    # Try positive steering with mid throttle
    # RC_CHANNELS_OVERRIDE: 1500=center, 1000=min, 2000=max
    # In rover: chan1=steer(roll), chan3=throttle, chan4=yaw
    print("  inject RC override: steer=1700 (right), throttle=1600 (fwd)")
    m.mav.rc_channels_override_send(sysid, compid,
                                     1700,  # 1: steering
                                     1500,  # 2
                                     1600,  # 3: throttle
                                     1500,  # 4
                                     0, 0, 0, 0)
    time.sleep(0.5)
    pulse = {1: [], 3: []}
    t0 = time.time()
    while time.time() - t0 < 1.5:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.3)
        if msg:
            pulse[1].append(msg.servo1_raw)
            pulse[3].append(msg.servo3_raw)
    print(f"  during   servo1={sorted(set(pulse[1]))}  servo3={sorted(set(pulse[3]))}")

    # Release override
    m.mav.rc_channels_override_send(sysid, compid,
                                     0, 0, 0, 0, 0, 0, 0, 0)
    print(f"\n  expected with skid steering: servo1 (left) and servo3 (right) DIFFER (one increases, one decreases) when steer is non-center")

    print(f"\n{'='*40}\nResult: {'SUCCESS ✓' if all_ok else 'PARTIAL ⚠'}")


if __name__ == "__main__":
    main()
