#!/usr/bin/env python3
"""
USV SITL Smoke Test

Launches each USV frame in ArduPilot SITL and verifies:
  1. SITL boots and sends heartbeats
  2. GPS lock is obtained
  3. Vehicle can arm in MANUAL mode
  4. Vehicle can disarm

Requirements:
  - ArduPilot built for SITL:  waf configure --board sitl && waf rover
  - pymavlink:  pip install pymavlink
  - setup_sitl.sh already run to register USV models

Usage:
  python test_usv_smoke.py                    # test all frames
  python test_usv_smoke.py usv-diff usv-rudder  # test specific frames
  AP_ROOT=/path/to/ardupilot python test_usv_smoke.py
"""

import os
import sys
import time
import signal
import subprocess
import argparse
from pathlib import Path

# ---------------------------------------------------------------------------
# pymavlink import
# ---------------------------------------------------------------------------
try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
ALL_FRAMES = [
    "usv-diff",
    "usv-diff-bow",
    "usv-xframe",
    "usv-wingsail-flap",
    "usv-wingsail",
    "usv-rudder",
]

HOME_LOC = "22.5024,113.9150,0,270"  # Shenzhen Bay

# Timeouts (seconds)
BOOT_TIMEOUT = 60
GPS_TIMEOUT = 30
ARM_TIMEOUT = 15
DISARM_TIMEOUT = 10

# MAVLink constants
MAV_MODE_MANUAL = 0
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_RESULT_ACCEPTED = 0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def find_ap_root():
    """Find ArduPilot root directory."""
    if os.environ.get("AP_ROOT"):
        return os.environ["AP_ROOT"]

    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent.parent

    # Check common relative locations
    candidates = [
        project_root / ".." / ".." / "ardupilot-master",
        project_root / ".." / "ardupilot",
        Path("f:/opensource/usv_esp32/ardupilot-master"),
    ]
    for p in candidates:
        if p.exists() and (p / "Tools" / "autotest" / "sim_vehicle.py").exists():
            return str(p.resolve())

    return None


def check_setup(ap_root):
    """Verify SITL setup is complete."""
    vehicleinfo = Path(ap_root) / "Tools" / "autotest" / "pysim" / "vehicleinfo.py"
    if not vehicleinfo.exists():
        return False, f"vehicleinfo.py not found at {vehicleinfo}"

    content = vehicleinfo.read_text()
    if '"usv-diff"' not in content:
        return False, (
            "USV models not registered. Run setup_sitl.sh first.\n"
            "  bash sitl/scripts/setup_sitl.sh"
        )

    sim_vehicle = Path(ap_root) / "Tools" / "autotest" / "sim_vehicle.py"
    if not sim_vehicle.exists():
        return False, f"sim_vehicle.py not found at {sim_vehicle}"

    return True, ""


class SITLProcess:
    """Manages a sim_vehicle.py subprocess."""

    def __init__(self, ap_root, frame, port=5760):
        self.ap_root = ap_root
        self.frame = frame
        self.port = port
        self.proc = None

    def start(self):
        sim_vehicle = os.path.join(
            self.ap_root, "Tools", "autotest", "sim_vehicle.py"
        )
        cmd = [
            sys.executable, sim_vehicle,
            "-v", "Rover",
            "-f", self.frame,
            "-l", HOME_LOC,
            "--no-mavproxy",
            "-S", "5",  # speedup 5x
            "--instance", "0",
        ]
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid if hasattr(os, "setsid") else None,
        )

    def stop(self):
        if self.proc is None:
            return
        try:
            if hasattr(os, "killpg"):
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            else:
                self.proc.terminate()
            self.proc.wait(timeout=10)
        except Exception:
            try:
                self.proc.kill()
                self.proc.wait(timeout=5)
            except Exception:
                pass
        self.proc = None

    def is_alive(self):
        if self.proc is None:
            return False
        return self.proc.poll() is None


class SmokeTestResult:
    """Result of a single frame smoke test."""

    def __init__(self, frame):
        self.frame = frame
        self.boot_ok = False
        self.gps_ok = False
        self.arm_ok = False
        self.disarm_ok = False
        self.error = ""
        self.duration = 0.0

    @property
    def passed(self):
        return self.boot_ok and self.gps_ok and self.arm_ok and self.disarm_ok

    def summary_line(self):
        checks = [
            ("boot", self.boot_ok),
            ("gps", self.gps_ok),
            ("arm", self.arm_ok),
            ("disarm", self.disarm_ok),
        ]
        parts = []
        for name, ok in checks:
            parts.append(f"{name}:{'OK' if ok else 'FAIL'}")
        status = "PASS" if self.passed else "FAIL"
        line = f"  [{status}] {self.frame:20s}  {' | '.join(parts)}  ({self.duration:.1f}s)"
        if self.error:
            line += f"\n         error: {self.error}"
        return line


# ---------------------------------------------------------------------------
# Core test logic
# ---------------------------------------------------------------------------
def wait_heartbeat(conn, timeout):
    """Wait for first heartbeat from SITL."""
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and msg.get_srcSystem() != 255:
            return True
    return False


def wait_gps_fix(conn, timeout):
    """Wait for 3D GPS fix (fix_type >= 3)."""
    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if msg and msg.fix_type >= 3:
            return True
    return False


def try_arm(conn, timeout):
    """Attempt to arm in MANUAL mode and verify."""
    # Set MANUAL mode (mode number 0 for Rover)
    conn.set_mode(0)
    time.sleep(1)

    # Send arm command
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # arm
        21196,  # force arm magic number
        0, 0, 0, 0, 0,
    )

    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and (msg.base_mode & 128):  # MAV_MODE_FLAG_SAFETY_ARMED
            return True
    return False


def try_disarm(conn, timeout):
    """Attempt to disarm and verify."""
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        0,  # disarm
        21196,  # force
        0, 0, 0, 0, 0,
    )

    start = time.time()
    while time.time() - start < timeout:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg and not (msg.base_mode & 128):
            return True
    return False


def run_smoke_test(ap_root, frame, port=5760):
    """Run smoke test for a single frame. Returns SmokeTestResult."""
    result = SmokeTestResult(frame)
    t0 = time.time()
    sitl = SITLProcess(ap_root, frame, port)
    conn = None

    try:
        # Start SITL
        print(f"  Starting SITL ({frame}) ...")
        sitl.start()
        time.sleep(3)  # let SITL initialize

        if not sitl.is_alive():
            result.error = "SITL process exited immediately"
            return result

        # Connect via MAVLink
        conn_str = f"tcp:127.0.0.1:{port}"
        conn = mavutil.mavlink_connection(conn_str, source_system=255)

        # 1) Wait for heartbeat
        print(f"  Waiting for heartbeat ...")
        result.boot_ok = wait_heartbeat(conn, BOOT_TIMEOUT)
        if not result.boot_ok:
            result.error = "No heartbeat received"
            return result

        conn.wait_heartbeat()
        print(f"  Heartbeat OK (sys={conn.target_system})")

        # 2) Wait for GPS
        print(f"  Waiting for GPS fix ...")
        result.gps_ok = wait_gps_fix(conn, GPS_TIMEOUT)
        if not result.gps_ok:
            result.error = "No GPS 3D fix"
            return result
        print(f"  GPS fix OK")

        # 3) Arm
        print(f"  Arming ...")
        result.arm_ok = try_arm(conn, ARM_TIMEOUT)
        if not result.arm_ok:
            result.error = "Failed to arm"
            return result
        print(f"  Armed OK")

        # 4) Disarm
        print(f"  Disarming ...")
        result.disarm_ok = try_disarm(conn, DISARM_TIMEOUT)
        if not result.disarm_ok:
            result.error = "Failed to disarm"
            return result
        print(f"  Disarmed OK")

    except Exception as e:
        result.error = str(e)
    finally:
        result.duration = time.time() - t0
        if conn:
            conn.close()
        sitl.stop()

    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="USV SITL Smoke Test")
    parser.add_argument(
        "frames", nargs="*", default=ALL_FRAMES,
        help=f"Frames to test (default: all). Available: {', '.join(ALL_FRAMES)}",
    )
    parser.add_argument(
        "--port", type=int, default=5760,
        help="MAVLink TCP port (default: 5760)",
    )
    args = parser.parse_args()

    # Validate frames
    for f in args.frames:
        if f not in ALL_FRAMES:
            print(f"ERROR: Unknown frame '{f}'. Available: {', '.join(ALL_FRAMES)}")
            sys.exit(1)

    # Find ArduPilot
    ap_root = find_ap_root()
    if not ap_root:
        print("ERROR: ArduPilot root not found. Set AP_ROOT env var.")
        sys.exit(1)

    ok, msg = check_setup(ap_root)
    if not ok:
        print(f"ERROR: {msg}")
        sys.exit(1)

    print("=" * 60)
    print("  USV SITL Smoke Test")
    print("=" * 60)
    print(f"  ArduPilot : {ap_root}")
    print(f"  Frames    : {', '.join(args.frames)}")
    print(f"  Home      : {HOME_LOC}")
    print("=" * 60)
    print()

    results = []
    for frame in args.frames:
        print(f"[{len(results)+1}/{len(args.frames)}] Testing: {frame}")
        r = run_smoke_test(ap_root, frame, args.port)
        results.append(r)
        print()

    # Summary
    passed = sum(1 for r in results if r.passed)
    failed = len(results) - passed

    print("=" * 60)
    print("  SMOKE TEST RESULTS")
    print("=" * 60)
    for r in results:
        print(r.summary_line())
    print("-" * 60)
    print(f"  Total: {len(results)}  Passed: {passed}  Failed: {failed}")
    if failed == 0:
        print("  ALL FRAMES PASSED")
    else:
        print("  SOME FRAMES FAILED")
    print("=" * 60)

    sys.exit(0 if failed == 0 else 1)


if __name__ == "__main__":
    main()
