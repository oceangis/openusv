"""Monitor RC_CHANNELS over MAVLink (COM58) to verify SBUS reception.

Run alongside sbus_simulator.py. The simulator sends SBUS frames to
GPIO 8 via USB-TTL on another COM port; this script connects to the
autopilot via the main USB CDC (COM58) and reports RC_CHANNELS values.

Pass criteria:
- RC_CHANNELS messages appear (means AP_RCProtocol locked on SBUS_NI)
- chan1/chan3 values follow the simulator's pattern (1500 → 1700 → 1300 → sweep → 1500)
- chancount >= 16
- rssi > 0 (means signal is being decoded)
"""
import statistics
import time
from collections import defaultdict
from pymavlink import mavutil

PORT = "COM58"
BAUD = 115200

m = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                source_system=255, source_component=0)
m.wait_heartbeat(timeout=10)
sysid, compid = m.target_system, m.target_component
print(f"connected sys={sysid}")

# Request RC_CHANNELS @ 10 Hz
m.mav.request_data_stream_send(sysid, compid,
                                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1)

print(f"\n{'t':>5s}  {'count':>5s}  {'rssi':>4s}  {'ch1':>5s}  {'ch2':>5s}  {'ch3':>5s}  {'ch4':>5s}  {'ch5':>5s}  {'ch6':>5s}")
print("-" * 75)

t_start = time.time()
all_chans = defaultdict(list)
last_print = 0
sample_count = 0

try:
    while time.time() - t_start < 35:
        msg = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=0.3)
        if msg is None:
            continue
        sample_count += 1
        for i in range(1, 17):
            v = getattr(msg, f"chan{i}_raw", 0)
            if v != 0:
                all_chans[i].append(v)

        # print at most 4Hz
        now = time.time()
        if now - last_print > 0.25:
            last_print = now
            t = now - t_start
            print(f"{t:5.1f}  {msg.chancount:5d}  {msg.rssi:4d}  "
                  f"{msg.chan1_raw:5d}  {msg.chan2_raw:5d}  "
                  f"{msg.chan3_raw:5d}  {msg.chan4_raw:5d}  "
                  f"{msg.chan5_raw:5d}  {msg.chan6_raw:5d}")
except KeyboardInterrupt:
    pass

print("\n=== Summary ===")
print(f"  RC_CHANNELS messages received: {sample_count}")
if sample_count == 0:
    print("  ✗ NO RC_CHANNELS RECEIVED — SBUS not detected")
else:
    for ch in sorted(all_chans):
        vals = all_chans[ch]
        if not vals:
            continue
        mn = min(vals)
        mx = max(vals)
        print(f"  ch{ch}: range [{mn}, {mx}]  span={mx-mn}us  N={len(vals)}")
    spans = [max(v) - min(v) for v in all_chans.values() if len(v) > 1]
    if spans and max(spans) > 200:
        print(f"\n  ✓ At least one channel showed >200us movement → SBUS RECEPTION WORKING")
    else:
        print(f"\n  ⚠ No channel showed significant movement")
        print(f"    Possible causes: simulator not running, wrong GPIO, missing inversion, baud mismatch")
