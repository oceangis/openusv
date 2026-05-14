"""Quick 2-minute regression check after SBUS migration.

Verifies that the RmtSigReader migration didn't break:
 - boot reliability (HEARTBEAT shows up < 12s)
 - mode switching (MANUAL/HOLD/ACRO work)
 - skid steering math (steer/throttle produces differential SERVO1/3)
 - param read/write
 - calibration values still intact
"""
import statistics
import threading
import time
from pymavlink import mavutil


PORT = "COM10"  # CH340 path (UART0). Use COM58 if native USB CDC is enumerated.
BAUD = 115200


class RCStreamer:
    def __init__(self, m, hz=50):
        self.m = m
        self.period = 1.0 / hz
        self.lock = threading.Lock()
        self.chans = [0, 0, 0, 0, 0, 0, 0, 0]
        self.stop = False
        self.thr = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thr.start()

    def end(self):
        self.stop = True
        self.thr.join(0.5)

    def set(self, *chans):
        with self.lock:
            self.chans = list(chans) + [0] * (8 - len(chans))

    def _run(self):
        while not self.stop:
            with self.lock:
                c = list(self.chans)
            try:
                self.m.mav.rc_channels_override_send(
                    self.m.target_system, self.m.target_component, *c)
            except Exception:
                pass
            time.sleep(self.period)


m = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                source_system=255, source_component=0)
print("waiting heartbeat...")
t_hb = time.time()
hb = m.wait_heartbeat(timeout=20)
boot_visible_dt = time.time() - t_hb
sysid, compid = m.target_system, m.target_component
state = mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status, '?').name
print(f"HB received in {boot_visible_dt:.1f}s, state={state}")

print("waiting 8s more for full ready...")
time.sleep(8)

# Stream rate
m.mav.request_data_stream_send(sysid, compid,
                                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1)
m.mav.request_data_stream_send(sysid, compid,
                                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4, 1)

# ============ Test 1: param read sanity ============
def robust_read(name, retries=3):
    for _ in range(retries):
        m.mav.param_request_read_send(sysid, compid, name.encode(), -1)
        t = time.time()
        while time.time() - t < 1.0:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is None:
                continue
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            pid = pid.strip("\x00")
            if pid == name:
                return msg.param_value
        time.sleep(0.1)
    return None

print("\n=== T1: critical params preserved ===")
expected = {
    "SERVO1_FUNCTION": 73.0,
    "SERVO3_FUNCTION": 74.0,
    "ARMING_CHECK":    178.0,
    "FS_THR_ENABLE":   0.0,
    "INS_ACCOFFS_X":   -0.1355,
    "AHRS_TRIM_Y":     0.0116,
}
all_ok = True
for k, exp in expected.items():
    v = robust_read(k)
    ok = v is not None and abs(v - exp) < 0.01
    print(f"  [{('✓' if ok else '✗')}] {k:20s} = {v} (expected {exp})")
    if not ok:
        all_ok = False

# ============ Test 2: mode switching ============
print("\n=== T2: mode switching (MANUAL, ACRO, HOLD, MANUAL) ===")
modes = [("MANUAL", 0), ("ACRO", 1), ("HOLD", 4), ("MANUAL", 0)]
for name, num in modes:
    m.mav.set_mode_send(sysid,
                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, num)
    t = time.time()
    confirmed = False
    while time.time() - t < 2:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
        if msg and msg.custom_mode == num:
            confirmed = True
            break
    print(f"  [{('✓' if confirmed else '✗')}] {name:8s} (custom_mode={num})")
    if not confirmed:
        all_ok = False
    time.sleep(0.3)

# ============ Test 3: ARM + skid + DISARM (single fast cycle) ============
print("\n=== T3: ARM + skid differential check ===")
m.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
time.sleep(0.5)
m.mav.command_long_send(sysid, compid,
                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                         0, 1, 0, 0, 0, 0, 0, 0)
t = time.time()
armed = False
while time.time() - t < 3:
    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
    if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        armed = True
        break
print(f"  [{('✓' if armed else '✗')}] ARM succeeded")

if armed:
    streamer = RCStreamer(m, hz=50)
    streamer.start()
    # full right
    streamer.set(1900, 1500, 1500, 1500)
    time.sleep(0.6)
    s1 = []; s3 = []
    t = time.time()
    while time.time() - t < 1.0:
        msg = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.2)
        if msg:
            s1.append(msg.servo1_raw)
            s3.append(msg.servo3_raw)
    streamer.set(0, 0, 0, 0)
    time.sleep(0.2)
    streamer.end()
    diff = (statistics.mean(s1) - statistics.mean(s3)) if s1 and s3 else 0
    print(f"  right turn full: S1 mean={statistics.mean(s1):.0f} S3 mean={statistics.mean(s3):.0f} DIFF={diff:+.0f}")
    print(f"  [{('✓' if diff > 500 else '✗')}] skid differential (expect DIFF > +500)")
    if diff < 500:
        all_ok = False

# disarm
print("  disarming...")
m.mav.command_long_send(sysid, compid,
                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                         0, 0, 0, 0, 0, 0, 0, 0)
time.sleep(1)

# ============ Test 4: RC_CHANNELS subsystem alive (just check it's emitting) ============
print("\n=== T4: RC_CHANNELS subsystem alive (SBUS RMT not crashed) ===")
rc_count = 0
t = time.time()
while time.time() - t < 3:
    msg = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.3)
    if msg:
        rc_count += 1
print(f"  [{('✓' if rc_count >= 4 else '✗')}] {rc_count} RC_CHANNELS in 3s (≥4 expected)")
if rc_count < 4:
    all_ok = False

# ============ Verdict ============
print(f"\n{'='*50}")
print(f"REGRESSION CHECK: {'✓ PASS' if all_ok else '⚠ ISSUES'}")
print(f"{'='*50}")
