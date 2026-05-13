"""Comprehensive parameter R/W validation.

Covers:
 1. Read single param by name (verify type info)
 2. Read non-existent param (no crash, no false reply)
 3. Write same value (idempotent)
 4. Write different value, verify echo within timeout
 5. Re-read after write, verify persistence in RAM
 6. Multi-type test (INT8, INT16, FLOAT) for type-aware encoding
 7. Bulk write 5 params in rapid succession, measure timing
 8. Soft reboot, verify ALL changes preserved across power-cycle
 9. Restore all originals + verify

Safe params used (no operational impact when changed within reason):
  - SYSID_MYGCS    (INT16) GCS sysid to listen to  default 255
  - WP_RADIUS      (FLOAT) waypoint pass radius     default 2.0 m
  - WP_SPEED       (FLOAT) waypoint nav speed       default 3.5 m/s
  - SR0_RAW_SENS   (INT16) stream rate raw sensors  default varies
  - MIS_RESTART    (INT8)  mission restart behavior default 0
"""
import sys
import time
from pymavlink import mavutil

PORT = "COM58"
BAUD = 115200

# Parameters to use for testing (will be modified then restored)
# (name, test_value, type_hint_for_logging)
TEST_PARAMS = [
    ("SYSID_MYGCS",     250.0, "INT16"),
    ("WP_RADIUS",       3.5,   "FLOAT"),
    ("WP_SPEED",        4.2,   "FLOAT"),
    ("SR0_RAW_SENS",    5.0,   "INT16"),
    ("MIS_RESTART",     1.0,   "INT8"),
]

# Track results
results = {"read": [], "write": [], "persistence": [], "errors": []}


def read_param(m, name, timeout=3.0):
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
            return msg.param_value, msg.param_type, time.time() - t0
    return None, None, time.time() - t0


def write_param(m, name, value, ptype, timeout=4.0):
    sysid, compid = m.target_system, m.target_component
    t0 = time.time()
    m.mav.param_set_send(sysid, compid, name.encode(), float(value), ptype)
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if msg is None:
            continue
        pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
        pid = pid.strip("\x00")
        if pid == name:
            return msg.param_value, time.time() - t0
    return None, time.time() - t0


def reboot_and_reconnect(m):
    sysid, compid = m.target_system, m.target_component
    print("\n  [REBOOT] sending MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN ...")
    m.mav.command_long_send(
        sysid, compid,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0,
    )
    m.close()
    time.sleep(8)
    m2 = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    hb = m2.wait_heartbeat(timeout=15)
    print(f"  [RECONNECT] state="
          f"{mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}")
    # Wait for board to settle
    time.sleep(8)
    return m2


def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD, dialect="ardupilotmega",
                                   source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=10)
    print(f"connected. state="
          f"{mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}")

    print("\n" + "="*60)
    print("PHASE 1: Read originals + verify types")
    print("="*60)
    originals = {}
    for name, _, type_hint in TEST_PARAMS:
        v, t, dt = read_param(m, name)
        if v is None:
            print(f"  ✗ {name:18s} NO REPLY in {dt:.2f}s")
            results["errors"].append(f"read fail: {name}")
        else:
            originals[name] = (v, t)
            print(f"  ✓ {name:18s} = {v:>10}  type={t} ({type_hint})  dt={dt*1000:.0f}ms")
            results["read"].append({"name": name, "value": v, "type": t, "dt_ms": dt*1000})

    print("\n" + "="*60)
    print("PHASE 2: Non-existent param")
    print("="*60)
    fake = "DOES_NOT_EXIST"
    v, t, dt = read_param(m, fake, timeout=2.5)
    if v is None:
        print(f"  ✓ {fake}: no reply (expected) in {dt:.2f}s — no crash")
    else:
        print(f"  ⚠ unexpected reply for {fake}: v={v}")

    print("\n" + "="*60)
    print("PHASE 3: Write same value (idempotent)")
    print("="*60)
    name = "WP_RADIUS"
    cur_v, cur_t = originals.get(name, (None, None))
    if cur_v is None:
        print(f"  skip — couldn't read {name} earlier")
    else:
        echoed, dt = write_param(m, name, cur_v, cur_t)
        if echoed is not None and abs(echoed - cur_v) < 0.01:
            print(f"  ✓ Write same value {cur_v} → echo {echoed} in {dt*1000:.0f}ms")
        else:
            print(f"  ⚠ unexpected: echo={echoed}")

    print("\n" + "="*60)
    print("PHASE 4: Multi-type write + verify echo")
    print("="*60)
    write_changes = {}
    for name, test_val, type_hint in TEST_PARAMS:
        cur_v, cur_t = originals.get(name, (None, None))
        if cur_v is None:
            continue
        echoed, dt = write_param(m, name, test_val, cur_t)
        ok = (echoed is not None and abs(echoed - test_val) < 0.01)
        mark = "✓" if ok else "✗"
        print(f"  [{mark}] {name:15s} ({type_hint:5s}): {cur_v} -> {test_val}"
              f"  echo={echoed}  dt={dt*1000:.0f}ms")
        results["write"].append({"name": name, "from": cur_v,
                                 "target": test_val, "echo": echoed,
                                 "ok": ok, "dt_ms": dt*1000})
        if ok:
            write_changes[name] = test_val
        time.sleep(0.1)

    print("\n" + "="*60)
    print("PHASE 5: Re-read after writes (RAM persistence)")
    print("="*60)
    time.sleep(1)
    for name, target in write_changes.items():
        v, _, dt = read_param(m, name)
        ok = (v is not None and abs(v - target) < 0.01)
        mark = "✓" if ok else "✗"
        print(f"  [{mark}] {name:15s} re-read = {v}  (expected {target})  dt={dt*1000:.0f}ms")

    print("\n" + "="*60)
    print("PHASE 6: Bulk write timing (5 params rapid sequence)")
    print("="*60)
    bulk_test_value = {
        "SYSID_MYGCS":   251.0,
        "WP_RADIUS":     4.0,
        "WP_SPEED":      5.0,
        "SR0_RAW_SENS":  6.0,
        "MIS_RESTART":   0.0,
    }
    t0 = time.time()
    successes = 0
    for name, target in bulk_test_value.items():
        if name not in originals:
            continue
        _, cur_t = originals[name]
        echoed, dt = write_param(m, name, target, cur_t, timeout=3.0)
        if echoed is not None and abs(echoed - target) < 0.01:
            successes += 1
    bulk_dt = time.time() - t0
    print(f"  bulk write: {successes}/{len(bulk_test_value)} OK in {bulk_dt:.2f}s "
          f"({bulk_dt/len(bulk_test_value)*1000:.0f}ms per param avg)")

    print("\n" + "="*60)
    print("PHASE 7: Soft reboot + verify FLASH persistence")
    print("="*60)
    m = reboot_and_reconnect(m)
    # Verify all 5 bulk-changed values are preserved after reboot
    preserved_count = 0
    for name, expected in bulk_test_value.items():
        v, _, dt = read_param(m, name, timeout=4)
        ok = (v is not None and abs(v - expected) < 0.01)
        mark = "✓" if ok else "✗"
        print(f"  [{mark}] {name:15s} post-reboot = {v}  (expected {expected})")
        if ok:
            preserved_count += 1
        results["persistence"].append({"name": name, "expected": expected,
                                       "actual": v, "ok": ok})
    print(f"\n  FLASH PERSISTENCE: {preserved_count}/{len(bulk_test_value)} survived reboot")

    print("\n" + "="*60)
    print("PHASE 8: Restore originals")
    print("="*60)
    restored = 0
    for name in bulk_test_value:
        if name not in originals:
            continue
        orig_v, orig_t = originals[name]
        echoed, dt = write_param(m, name, orig_v, orig_t)
        ok = (echoed is not None and abs(echoed - orig_v) < 0.01)
        if ok:
            restored += 1
            print(f"  ✓ {name:15s} restored to {orig_v}")
        else:
            print(f"  ✗ {name:15s} FAILED restore (got {echoed})")
    print(f"\n  RESTORED: {restored}/{len(bulk_test_value)}")

    print("\n" + "="*60)
    print("FINAL SUMMARY")
    print("="*60)
    print(f"  reads OK:           {len(results['read'])}/{len(TEST_PARAMS)}")
    print(f"  writes OK:          {sum(1 for w in results['write'] if w['ok'])}/{len(results['write'])}")
    print(f"  persistence OK:     {sum(1 for p in results['persistence'] if p['ok'])}/{len(results['persistence'])}")
    print(f"  restore OK:         {restored}/{len(bulk_test_value)}")
    print(f"  errors:             {len(results['errors'])}")
    if results["read"]:
        avg_read = sum(r["dt_ms"] for r in results["read"]) / len(results["read"])
        print(f"  avg read latency:   {avg_read:.0f}ms")
    if results["write"]:
        avg_write = sum(w["dt_ms"] for w in results["write"]) / len(results["write"])
        print(f"  avg write latency:  {avg_write:.0f}ms")


if __name__ == "__main__":
    main()
