"""Continuous RC override test — verify skid steering differential output.

Sends RC_CHANNELS_OVERRIDE at 50 Hz from a background thread while main thread
samples SERVO_OUTPUT_RAW. This keeps the override "fresh" so ArduPilot honors
the injected values throughout the entire sample window.

Test matrix (steer, throttle):
  (1500, 1500) — TRUE NEUTRAL: expect SERVO1≈SERVO3≈1500
  (1700, 1600) — slight right + forward
  (1300, 1600) — slight left + forward
  (1900, 1500) — FULL RIGHT, no throttle
  (1100, 1500) — FULL LEFT, no throttle
  (1500, 1700) — straight + half forward
  (1500, 1300) — straight + half reverse
  (1500, 1500) — back to neutral
"""
import sys
import statistics
import threading
import time
from pymavlink import mavutil

PORT = "COM58"
BAUD = 115200


class RCOverrideStreamer:
    """Background thread sending RC_CHANNELS_OVERRIDE at fixed rate."""
    def __init__(self, m, rate_hz=50):
        self.m = m
        self.period = 1.0 / rate_hz
        self.lock = threading.Lock()
        self.chans = [0, 0, 0, 0, 0, 0, 0, 0]
        self.stop_flag = False
        self.thr = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thr.start()

    def stop(self):
        self.stop_flag = True
        self.thr.join(timeout=1.0)

    def set(self, ch1, ch2, ch3, ch4, ch5=0, ch6=0, ch7=0, ch8=0):
        with self.lock:
            self.chans = [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]

    def release(self):
        with self.lock:
            self.chans = [0, 0, 0, 0, 0, 0, 0, 0]

    def _run(self):
        sysid = self.m.target_system
        compid = self.m.target_component
        while not self.stop_flag:
            with self.lock:
                c = list(self.chans)
            try:
                self.m.mav.rc_channels_override_send(
                    sysid, compid, *c)
            except Exception:
                pass
            time.sleep(self.period)


def sample_servos(m, duration):
    s1, s3 = [], []
    t0 = time.time()
    while time.time() - t0 < duration:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.15)
        if msg:
            s1.append(int(msg.servo1_raw))
            s3.append(int(msg.servo3_raw))
    return s1, s3


def stats(name, s1, s3):
    if not s1 or not s3:
        print(f"  {name}: (no samples)")
        return
    m1 = statistics.mean(s1)
    m3 = statistics.mean(s3)
    print(f"  {name:25s}  S1 mean={m1:.0f} std={statistics.pstdev(s1):.1f} range=[{min(s1)},{max(s1)}]"
          f"   S3 mean={m3:.0f} std={statistics.pstdev(s3):.1f} range=[{min(s3)},{max(s3)}]"
          f"   DIFF={m1-m3:+.0f}")


def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                   source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=10)
    sysid, compid = m.target_system, m.target_component
    print(f"connected  state="
          f"{mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}")

    m.mav.request_data_stream_send(sysid, compid,
                                   mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1)
    time.sleep(0.5)

    # MANUAL mode
    print("\n[setup] MANUAL mode")
    m.mav.set_mode_send(sysid,
                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
    time.sleep(0.7)

    # ARM
    print("[setup] ARM")
    m.mav.command_long_send(sysid, compid,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0, 1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    armed = False
    while time.time() - t0 < 4:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
    if not armed:
        print("  ARM FAILED")
        return
    print("  armed ✓")

    # Start streamer
    streamer = RCOverrideStreamer(m, rate_hz=50)
    streamer.start()

    test_matrix = [
        ("centered (1500,1500)",   1500, 1500),
        ("right slight (1700,1600)", 1700, 1600),
        ("left slight (1300,1600)",  1300, 1600),
        ("right FULL (1900,1500)",   1900, 1500),
        ("left FULL (1100,1500)",    1100, 1500),
        ("straight fwd (1500,1700)", 1500, 1700),
        ("straight rev (1500,1300)", 1500, 1300),
        ("back to center (1500,1500)", 1500, 1500),
    ]

    print(f"\n{'='*55}")
    print("TEST MATRIX (50 Hz continuous RC override)")
    print(f"{'='*55}")
    results = []
    for label, steer, thr in test_matrix:
        print(f"\n>>> {label}  steer={steer} thr={thr}")
        # Set override
        streamer.set(steer, 1500, thr, 1500)
        # Allow ArduPilot a bit to react
        time.sleep(0.4)
        s1, s3 = sample_servos(m, 1.2)
        stats("output", s1, s3)
        results.append((label, steer, thr, s1, s3))

    # Cleanup
    streamer.release()
    time.sleep(0.3)
    streamer.stop()
    # Explicitly clear override
    m.mav.rc_channels_override_send(sysid, compid, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.3)

    # DISARM
    print("\n[teardown] DISARM")
    m.mav.command_long_send(sysid, compid,
                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             0, 0, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 2:
        msg = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
        if msg and msg.command == 400:
            print(f"  DISARM: {mavutil.mavlink.enums['MAV_RESULT'].get(msg.result, '?').name}")
            break

    # Summary
    print(f"\n{'='*55}")
    print("SUMMARY (diff = SERVO1 - SERVO3, mid=mean)")
    print(f"{'='*55}")
    print(f"  {'condition':30s} {'S1 mean':>8} {'S3 mean':>8} {'DIFF':>7} {'mid':>6}")
    for label, st, th, s1, s3 in results:
        if s1 and s3:
            m1 = statistics.mean(s1)
            m3 = statistics.mean(s3)
            mid = (m1 + m3) / 2
            print(f"  {label:30s} {m1:8.0f} {m3:8.0f} {m1-m3:+7.0f} {mid:6.0f}")


if __name__ == "__main__":
    main()
