"""Offline analysis of MAVProxy session.tlog.

Reads the binary tlog (recorded by MAVProxy via UDP bridge) and extracts:
 - message type counts and rates
 - all STATUSTEXTs by severity
 - mode change timeline
 - EKF flag transitions
 - SYS_STATUS errors_count1-4 timeline
 - HEARTBEAT state changes
 - parameter set events
 - mission protocol events

Demonstrates the value of MAVProxy's tlog as a permanent record.
"""
import sys
import time
from collections import defaultdict, Counter
from pymavlink import mavutil

TLOG = "session.tlog"


def main():
    print(f"[OPEN] {TLOG}", flush=True)
    m = mavutil.mavlink_connection(TLOG, dialect="ardupilotmega")

    counts = Counter()
    by_sys = Counter()
    statustext = []
    mode_changes = []
    state_changes = []
    ekf_flag_transitions = []
    error_counts_first = None
    error_counts_last = None
    params_set = []
    mission_events = []
    cmd_events = []
    first_t = None
    last_t = None
    last_mode = None
    last_state = None
    last_ekf = None
    msg_count = 0

    while True:
        msg = m.recv_match()
        if msg is None:
            break
        mt = msg.get_type()
        if mt == "BAD_DATA":
            continue
        msg_count += 1
        counts[mt] += 1
        by_sys[(msg._header.srcSystem, msg._header.srcComponent)] += 1

        # tlog has time prefix in mavlink_connection.timestamp
        ts = m._timestamp if hasattr(m, "_timestamp") else None
        if first_t is None and ts:
            first_t = ts
        if ts:
            last_t = ts

        if mt == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            statustext.append({"t": ts, "sev": msg.severity,
                               "text": txt.rstrip("\x00 \r\n")})
        elif mt == "HEARTBEAT":
            if msg.custom_mode != last_mode and last_mode is not None:
                mode_changes.append({"t": ts, "from": last_mode, "to": msg.custom_mode})
            last_mode = msg.custom_mode
            if msg.system_status != last_state and last_state is not None:
                state_changes.append({
                    "t": ts,
                    "from": mavutil.mavlink.enums["MAV_STATE"].get(last_state, "?").name,
                    "to":   mavutil.mavlink.enums["MAV_STATE"].get(msg.system_status, "?").name,
                })
            last_state = msg.system_status
        elif mt == "EKF_STATUS_REPORT":
            if msg.flags != last_ekf and last_ekf is not None:
                ekf_flag_transitions.append({"t": ts,
                                             "from": hex(last_ekf),
                                             "to": hex(msg.flags)})
            last_ekf = msg.flags
        elif mt == "SYS_STATUS":
            ec = (msg.errors_count1, msg.errors_count2,
                  msg.errors_count3, msg.errors_count4)
            if error_counts_first is None:
                error_counts_first = ec
            error_counts_last = ec
        elif mt == "PARAM_SET":
            pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            params_set.append({"t": ts, "name": pid.strip("\x00"),
                               "value": msg.param_value})
        elif mt in ("MISSION_COUNT", "MISSION_REQUEST_LIST",
                    "MISSION_CLEAR_ALL", "MISSION_ACK", "MISSION_REQUEST",
                    "MISSION_REQUEST_INT", "MISSION_ITEM_INT", "MISSION_ITEM"):
            mission_events.append({"t": ts, "type": mt})
        elif mt == "COMMAND_LONG":
            cmd_events.append({"t": ts, "cmd": msg.command,
                               "param1": msg.param1, "param5": msg.param5})
        elif mt == "COMMAND_ACK":
            cmd_events.append({"t": ts, "type": "ACK",
                               "cmd": msg.command,
                               "result": mavutil.mavlink.enums["MAV_RESULT"].get(msg.result, "?").name})

    dur = (last_t - first_t) if (first_t and last_t) else 0
    print(f"\n[TOTAL] {msg_count} msgs over {dur:.1f}s "
          f"= {msg_count/dur:.1f} msg/s")

    print(f"\n=== Message types ({len(counts)}) — top 15 ===")
    for mt, c in counts.most_common(15):
        rate = c / dur if dur > 0 else 0
        print(f"  {mt:28s} {c:5d}  ({rate:5.2f} Hz)")

    print(f"\n=== Source sysid/compid ===")
    for (s, c), n in by_sys.most_common(10):
        role = "vehicle" if s == 1 else ("MAVProxy(?)" if s == 254 else f"sys{s}")
        print(f"  ({s:3d},{c:3d}) {role:14s}  {n} msgs")

    print(f"\n=== STATUSTEXT ({len(statustext)} total) ===")
    sev_counts = Counter(s["sev"] for s in statustext)
    for sev, n in sorted(sev_counts.items()):
        sevname = mavutil.mavlink.enums["MAV_SEVERITY"].get(sev, "?").name
        print(f"  {sevname:18s} = {n}")
    if statustext:
        # show unique texts
        unique_texts = Counter(s["text"] for s in statustext)
        print("  Unique messages:")
        for txt, n in unique_texts.most_common(15):
            print(f"    [{n}x] {txt}")

    print(f"\n=== Mode changes ({len(mode_changes)}) ===")
    for c in mode_changes[:10]:
        print(f"  {c['from']} -> {c['to']}")

    print(f"\n=== HEARTBEAT state changes ({len(state_changes)}) ===")
    for c in state_changes[:10]:
        print(f"  {c['from']} -> {c['to']}")

    print(f"\n=== EKF flags transitions ({len(ekf_flag_transitions)}) ===")
    for c in ekf_flag_transitions[:10]:
        print(f"  {c['from']} -> {c['to']}")

    print(f"\n=== SYS_STATUS errors_count[1-4] ===")
    if error_counts_first:
        print(f"  first: {error_counts_first}")
        print(f"  last:  {error_counts_last}")
        if error_counts_first == error_counts_last:
            print("  ✓ NO ERROR COUNT INCREMENTS")
        else:
            delta = tuple(b-a for a, b in zip(error_counts_first, error_counts_last))
            print(f"  ⚠ DELTA: {delta}")

    print(f"\n=== PARAM_SET events from GCS ({len(params_set)}) ===")
    for p in params_set[:10]:
        print(f"  {p['name']} = {p['value']}")

    print(f"\n=== Mission protocol events ({len(mission_events)}) ===")
    mev_counter = Counter(e["type"] for e in mission_events)
    for t, n in mev_counter.most_common():
        print(f"  {t:28s} {n}")

    print(f"\n=== COMMAND events ({len(cmd_events)}) ===")
    cmd_id_counter = Counter()
    for c in cmd_events:
        if "type" in c and c["type"] == "ACK":
            cmd_id_counter[f"ACK({c['cmd']})={c['result']}"] += 1
        else:
            cmd_id_counter[f"CMD({c['cmd']})"] += 1
    for k, n in cmd_id_counter.most_common(15):
        print(f"  {k:50s} {n}")


if __name__ == "__main__":
    main()
