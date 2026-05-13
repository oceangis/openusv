"""ARM in MANUAL mode and verify skid steering differential output.

Sequence:
  1. switch to MANUAL mode
  2. ARM (MAV_CMD_COMPONENT_ARM_DISARM, param1=1)
  3. Capture baseline SERVO1/SERVO3 for 1s (expect 1500/1500 or both same)
  4. Inject RC override: steer=1700 (right), throttle=1600 (forward) → SERVO1 should DECREASE, SERVO3 should INCREASE (right turn = left motor slow, right motor fast)
  5. Capture 2s
  6. Inject RC override: steer=1300 (left), throttle=1600 (forward) → reversed
  7. Capture 2s
  8. Inject RC override: steer=1500 (straight), throttle=1700 (more forward) → SERVO1 ≈ SERVO3 both elevated
  9. Capture 2s
  10. Release override + DISARM

Safety: no motors attached. User explicitly confirmed.
"""
import statistics
import sys
import time
from pymavlink import mavutil

PORT = "COM58"
BAUD = 115200


def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                   source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=10)
    sysid, compid = m.target_system, m.target_component
    print(f"connected  state="
          f"{mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}  "
          f"custom_mode={hb.custom_mode}")

    # Bump SERVO_OUTPUT_RAW rate
    m.mav.request_data_stream_send(sysid, compid,
                                   mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1)
    time.sleep(0.5)

    # 1. MANUAL mode
    print("\n[STEP 1] switch to MANUAL (custom_mode=0)")
    m.mav.set_mode_send(sysid,
                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
    time.sleep(0.7)
    # confirm
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if hb:
        print(f"  current custom_mode={hb.custom_mode}")

    # 2. ARM
    print("\n[STEP 2] sending ARM (MAV_CMD_COMPONENT_ARM_DISARM param1=1)")
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    arm_ack = None
    arm_statustext = []
    t0 = time.time()
    while time.time() - t0 < 4:
        msg = m.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "COMMAND_ACK" and msg.command == 400:
            arm_ack = msg
            break
        elif mt == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            arm_statustext.append((msg.severity, txt.rstrip("\x00 \r\n")))
    if arm_ack:
        res = mavutil.mavlink.enums["MAV_RESULT"].get(arm_ack.result, "?").name
        print(f"  ARM ACK: {res}")
    else:
        print("  (no ACK in 4s)")
    for sev, txt in arm_statustext:
        sevname = mavutil.mavlink.enums["MAV_SEVERITY"].get(sev, "?").name
        print(f"    [{sevname}] {txt}")
    # Check if armed via heartbeat
    time.sleep(0.5)
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
    print(f"  HEARTBEAT base_mode=0x{hb.base_mode:02X}  armed={armed}")

    if not armed:
        print("\n  [ABORT] arming failed — cannot verify differential output")
        return

    # 3. Baseline
    def sample_servos(label, duration):
        print(f"  [SAMPLE {label}]  duration={duration}s")
        s1, s3 = [], []
        t0 = time.time()
        while time.time() - t0 < duration:
            msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.2)
            if msg:
                s1.append(int(msg.servo1_raw))
                s3.append(int(msg.servo3_raw))
        if s1 and s3:
            print(f"    SERVO1: mean={statistics.mean(s1):.0f}  range=[{min(s1)},{max(s1)}]  N={len(s1)}")
            print(f"    SERVO3: mean={statistics.mean(s3):.0f}  range=[{min(s3)},{max(s3)}]  N={len(s3)}")
            print(f"    DIFF (SERVO1 - SERVO3): {statistics.mean(s1) - statistics.mean(s3):+.1f}")
        return s1, s3

    print("\n[STEP 3] baseline (no input, armed, MANUAL)")
    base_s1, base_s3 = sample_servos("baseline", 1.5)

    # 4. Right turn + forward
    print("\n[STEP 4] inject: steer=1700 (right), throttle=1600 (forward)")
    m.mav.rc_channels_override_send(sysid, compid,
                                     1700, 1500, 1600, 1500, 0, 0, 0, 0)
    time.sleep(0.4)
    r1, r3 = sample_servos("right_turn_fwd", 1.5)

    # 5. Left turn + forward
    print("\n[STEP 5] inject: steer=1300 (left), throttle=1600 (forward)")
    m.mav.rc_channels_override_send(sysid, compid,
                                     1300, 1500, 1600, 1500, 0, 0, 0, 0)
    time.sleep(0.4)
    l1, l3 = sample_servos("left_turn_fwd", 1.5)

    # 6. Straight + more throttle
    print("\n[STEP 6] inject: steer=1500 (straight), throttle=1700 (faster)")
    m.mav.rc_channels_override_send(sysid, compid,
                                     1500, 1500, 1700, 1500, 0, 0, 0, 0)
    time.sleep(0.4)
    f1, f3 = sample_servos("straight_fast", 1.5)

    # 7. Release override + disarm
    print("\n[STEP 7] release override, DISARM")
    m.mav.rc_channels_override_send(sysid, compid,
                                     0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.3)
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0,
    )
    t0 = time.time()
    while time.time() - t0 < 2:
        msg = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == 400:
            print(f"  DISARM ACK: {mavutil.mavlink.enums['MAV_RESULT'].get(msg.result, '?').name}")
            break

    # Final analysis
    print("\n" + "="*55)
    print("DIFFERENTIAL VERIFICATION")
    print("="*55)
    if r1 and r3 and l1 and l3:
        r_diff = statistics.mean(r1) - statistics.mean(r3)
        l_diff = statistics.mean(l1) - statistics.mean(l3)
        f_diff = statistics.mean(f1) - statistics.mean(f3) if (f1 and f3) else 0
        print(f"  Baseline   diff (S1-S3) = {statistics.mean(base_s1)-statistics.mean(base_s3):+.0f}")
        print(f"  Right turn diff (S1-S3) = {r_diff:+.0f}  (expect negative: LEFT motor slows for right turn)")
        print(f"  Left turn  diff (S1-S3) = {l_diff:+.0f}  (expect positive: LEFT motor speeds up for left turn)")
        print(f"  Straight   diff (S1-S3) = {f_diff:+.0f}  (expect ≈0: both motors equal)")
        # Verdict
        if abs(r_diff) > 30 and abs(l_diff) > 30 and (r_diff * l_diff) < 0:
            print(f"\n  ✓ SKID STEERING DIFFERENTIAL CONFIRMED — sign flips correctly with steer direction")
        else:
            print(f"\n  ⚠ Differential not clearly observed — see numbers above")


if __name__ == "__main__":
    main()
