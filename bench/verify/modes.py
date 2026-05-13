"""Probe every ArduRover mode number to see which exist in this firmware build."""
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection("COM58", baud=115200, dialect="ardupilotmega",
                                source_system=255, source_component=0)
m.wait_heartbeat(timeout=10)
sysid, compid = m.target_system, m.target_component

# All ArduRover mode numbers from mode.h
MODES = [
    (0,  "MANUAL"),
    (1,  "ACRO"),
    (3,  "STEERING"),
    (4,  "HOLD"),
    (5,  "LOITER"),
    (6,  "FOLLOW"),
    (7,  "SIMPLE"),
    (8,  "DOCK"),
    (9,  "CIRCLE"),
    (10, "AUTO"),
    (11, "RTL"),
    (12, "SMART_RTL"),
    (13, "POSHOLD"),
    (15, "GUIDED"),
    (16, "INITIALISING"),
]

# First switch to MANUAL as a known-good baseline
m.mav.set_mode_send(sysid, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
time.sleep(1)

print(f"{'#':>3} {'name':12s}  {'switch':10s}  {'STATUSTEXT':40s}")
print("-" * 78)

for num, name in MODES:
    # try switching
    m.mav.set_mode_send(sysid,
                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                         num)
    # Watch for next HEARTBEAT or STATUSTEXT
    statustext = []
    actual_mode = None
    t0 = time.time()
    while time.time() - t0 < 2.0:
        msg = m.recv_match(blocking=True, timeout=0.3)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "HEARTBEAT":
            actual_mode = msg.custom_mode
            if actual_mode == num:
                break
        elif mt == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            txt = txt.rstrip("\x00 \r\n")
            statustext.append(txt)

    if actual_mode == num:
        result = "✓ entered"
    elif actual_mode is not None:
        result = f"✗ stayed in {actual_mode}"
    else:
        result = "? no HB"

    st_summary = " | ".join(statustext)[:38] if statustext else ""
    print(f"{num:3d} {name:12s}  {result:10s}  {st_summary}")
    # Always return to MANUAL between attempts to clean state
    m.mav.set_mode_send(sysid,
                         mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
    time.sleep(0.5)
