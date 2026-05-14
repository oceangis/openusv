"""Mission lifecycle stress — 100 rounds of upload/readback/clear.

Validates mission storage (separate from parameter NVS) under repeated
write/erase cycles. Each round:
  1. Upload 5-item mission (3 NAV_WAYPOINT + 1 LOITER + 1 RTL)
  2. Verify mission count
  3. Read back items, byte-compare against what was sent
  4. Clear all
  5. Verify count = 0

Looks for:
  - Mission storage corruption (readback != sent)
  - mission_count drift (e.g. clear not actually clearing)
  - free-heap drift across cycles
  - Increased ack latency over time
"""
import argparse
import csv
import io
import statistics
import sys
import time
from pymavlink import mavutil

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass


def connect(port, baud):
    m = mavutil.mavlink_connection(port, baud=baud, dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    if not m.wait_heartbeat(timeout=15):
        sys.exit("no HB")
    return m


def make_items(base_lat_e7, base_lon_e7, round_idx):
    """5 mission items. Round index encoded in slight lat offsets so we can
    distinguish round N's data from corrupted previous data."""
    delta = 100 * round_idx  # ~1cm per round, distinguishable
    return [
        # idx 0 — HOME (auto-overwritten by ArduPilot, but still sent)
        (0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1, 1,
         0, 0, 0, 0, base_lat_e7, base_lon_e7, 0),
        # idx 1
        (1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
         0, 0, 0, 0, base_lat_e7 + delta, base_lon_e7, 0),
        # idx 2
        (2, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
         0, 0, 0, 0, base_lat_e7 + delta, base_lon_e7 + delta, 0),
        # idx 3
        (3, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
         0, 0, 0, 0, base_lat_e7, base_lon_e7 + delta, 0),
        # idx 4 — RTL
        (4, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 1,
         0, 0, 0, 0, 0, 0, 0),
    ]


def upload(m, items, timeout=8.0):
    sysid, compid = m.target_system, m.target_component
    t0 = time.time()
    m.mav.mission_count_send(sysid, compid, len(items),
                              mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    ack = None
    while time.time() - t0 < timeout:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = msg.seq
            if 0 <= seq < len(items):
                it = items[seq]
                m.mav.mission_item_int_send(
                    sysid, compid,
                    it[0], it[1], it[2], it[3], it[4],
                    it[5], it[6], it[7], it[8],
                    int(it[9]), int(it[10]), it[11],
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
                sent.add(seq)
        elif mt == "MISSION_ACK":
            ack = mavutil.mavlink.enums["MAV_MISSION_RESULT"].get(msg.type, "?").name
            break
    return ack, sent, (time.time() - t0) * 1000


def readback(m, expected_count, timeout=8.0):
    sysid, compid = m.target_system, m.target_component
    t0 = time.time()
    m.mav.mission_request_list_send(sysid, compid,
                                     mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    # wait for MISSION_COUNT
    count = None
    items_received = {}
    while time.time() - t0 < timeout:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "MISSION_COUNT":
            count = msg.count
            # request all items
            for i in range(count):
                m.mav.mission_request_int_send(
                    sysid, compid, i,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        elif mt == "MISSION_ITEM_INT":
            items_received[msg.seq] = (
                msg.seq, msg.frame, msg.command, msg.current, msg.autocontinue,
                msg.param1, msg.param2, msg.param3, msg.param4,
                msg.x, msg.y, msg.z)
            if len(items_received) >= (count or expected_count):
                break
    return count, items_received, (time.time() - t0) * 1000


def clear_all(m, timeout=4.0):
    sysid, compid = m.target_system, m.target_component
    t0 = time.time()
    m.mav.mission_clear_all_send(sysid, compid,
                                  mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    while time.time() - t0 < timeout:
        msg = m.recv_match(type="MISSION_ACK", blocking=True, timeout=0.5)
        if msg:
            return mavutil.mavlink.enums["MAV_MISSION_RESULT"].get(msg.type, "?").name, (time.time() - t0) * 1000
    return "no_ack", (time.time() - t0) * 1000


def compare_items(sent, received):
    """Returns list of (seq, field, sent_val, received_val) mismatches.

    Skips ArduPilot's known mission-readback normalisations:
      - seq 0 (HOME) is overwritten with current GPS fix; without a fix it
        reads back as zeros. Either way, the operator-set values don't survive.
      - RETURN_TO_LAUNCH (cmd 20) frame is normalised to MAV_FRAME_GLOBAL since
        the command doesn't use position; the frame the operator sends is lost.
    """
    mismatches = []
    for seq, sent_it in [(s[0], s) for s in sent]:
        if seq == 0:
            continue
        if seq not in received:
            mismatches.append((seq, "missing", sent_it, None))
            continue
        rcv = received[seq]
        is_rtl = (sent_it[2] == mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)
        for fname, fidx in [("frame", 1), ("command", 2), ("x", 9), ("y", 10)]:
            if is_rtl and fname == "frame":
                continue  # ArduPilot normalises RTL frame; not a real mismatch
            if abs(sent_it[fidx] - rcv[fidx]) > 1:
                mismatches.append((seq, fname, sent_it[fidx], rcv[fidx]))
    return mismatches


def read_mem(m, timeout=2.0):
    sysid, compid = m.target_system, m.target_component
    m.mav.request_data_stream_send(sysid, compid,
                                    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)
    t = time.time()
    while time.time() - t < timeout:
        msg = m.recv_match(type="MEMINFO", blocking=True, timeout=0.3)
        if msg:
            return getattr(msg, "freemem32", None) or msg.freemem
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rounds", type=int, default=100)
    args = ap.parse_args()

    print(f"[Mission stress] {args.rounds} rounds = upload(5) -> readback -> clear",
          flush=True)
    m = connect(args.port, args.baud)

    # Base location — Jakarta-ish from earlier GPS data, but exact doesn't matter
    BASE_LAT = -61258299
    BASE_LON = 1068311730

    mem_baseline = read_mem(m)
    print(f"  baseline free heap: {mem_baseline}", flush=True)

    rows = []
    fails = {"upload": 0, "readback_count": 0, "byte_mismatch": 0, "clear": 0}
    t_start = time.time()

    for i in range(args.rounds):
        items = make_items(BASE_LAT, BASE_LON, i)

        # UPLOAD
        ack_up, sent_set, up_ms = upload(m, items)
        if ack_up != "MAV_MISSION_ACCEPTED":
            fails["upload"] += 1

        # READBACK
        count, recv_items, rb_ms = readback(m, len(items))
        count_ok = (count == len(items))
        if not count_ok:
            fails["readback_count"] += 1

        # BYTE COMPARE
        mismatches = compare_items(items, recv_items)
        if mismatches:
            fails["byte_mismatch"] += len(mismatches)

        # CLEAR
        ack_clear, clr_ms = clear_all(m)
        if ack_clear != "MAV_MISSION_ACCEPTED":
            fails["clear"] += 1

        # Verify cleared
        count_after, _, _ = readback(m, 0)
        clear_verified = (count_after == 0)
        if not clear_verified:
            fails["clear"] += 1  # double-count if clear didn't actually clear

        # Periodic mem
        mem = None
        if (i + 1) % 20 == 0 or i == 0:
            mem = read_mem(m)

        rows.append({
            "round": i + 1,
            "upload_ack": ack_up,
            "upload_ms": up_ms,
            "readback_count": count,
            "readback_ms": rb_ms,
            "byte_mismatches": len(mismatches),
            "clear_ack": ack_clear,
            "clear_ms": clr_ms,
            "clear_verified": clear_verified,
            "free_mem": mem,
        })

        if (i + 1) % 20 == 0:
            elapsed = time.time() - t_start
            eta = elapsed / (i + 1) * (args.rounds - i - 1)
            print(f"  round {i+1}/{args.rounds}  "
                  f"upload_fails={fails['upload']}  count_fails={fails['readback_count']}  "
                  f"byte_mismatches={fails['byte_mismatch']}  clear_fails={fails['clear']}  "
                  f"mem={mem}  elapsed={elapsed/60:.1f}min  eta={eta/60:.1f}min",
                  flush=True)

    mem_final = read_mem(m)
    elapsed = time.time() - t_start

    # Save CSV
    with open("_mission_stress.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["round", "upload_ack", "upload_ms", "readback_count",
                    "readback_ms", "byte_mismatches", "clear_ack", "clear_ms",
                    "clear_verified", "free_mem"])
        for r in rows:
            w.writerow([r["round"], r["upload_ack"], f"{r['upload_ms']:.1f}",
                        r["readback_count"], f"{r['readback_ms']:.1f}",
                        r["byte_mismatches"], r["clear_ack"],
                        f"{r['clear_ms']:.1f}", r["clear_verified"],
                        r["free_mem"] if r["free_mem"] is not None else ""])

    print(f"\n=== Mission stress complete ===")
    print(f"Rounds: {args.rounds}  duration: {elapsed/60:.1f} min")
    print(f"Failures: upload={fails['upload']}  count_mismatch={fails['readback_count']}  "
          f"byte_mismatch={fails['byte_mismatch']}  clear={fails['clear']}")
    print(f"Memory: baseline={mem_baseline}  final={mem_final}  "
          f"delta={(mem_final-mem_baseline) if mem_baseline and mem_final else 'N/A'}")

    up_ms = [r["upload_ms"] for r in rows]
    rb_ms = [r["readback_ms"] for r in rows]
    clr_ms = [r["clear_ms"] for r in rows]
    print(f"\nUpload time : min={min(up_ms):.0f}ms  mean={statistics.mean(up_ms):.0f}ms  max={max(up_ms):.0f}ms")
    print(f"Readback time: min={min(rb_ms):.0f}ms  mean={statistics.mean(rb_ms):.0f}ms  max={max(rb_ms):.0f}ms")
    print(f"Clear time   : min={min(clr_ms):.0f}ms  mean={statistics.mean(clr_ms):.0f}ms  max={max(clr_ms):.0f}ms")

    verdict = "PASS"
    issues = []
    if any(fails.values()):
        verdict = "FAIL"
        for k, v in fails.items():
            if v > 0:
                issues.append(f"{k}: {v}")
    if mem_baseline and mem_final and (mem_baseline - mem_final) > 4096:
        verdict = "WARN" if verdict == "PASS" else verdict
        issues.append(f"memory dropped {mem_baseline - mem_final} bytes")

    print(f"\n=== VERDICT: {verdict} ===")
    for i in issues:
        print(f"  - {i}")


if __name__ == "__main__":
    main()
