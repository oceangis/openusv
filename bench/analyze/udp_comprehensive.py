"""Comprehensive 60s telemetry via UDP bridge (MAVProxy holds COM58).

Tests not covered before:
 1. Heartbeat timing jitter statistics (precise inter-arrival intervals)
 2. CPU load over time (SYS_STATUS.load)
 3. Drop rate / errors_comm
 4. Mode change behavior under sustained streams
 5. EKF flags evolution
 6. All sensor channels rate consistency
"""
import statistics
import time
from collections import defaultdict
from pymavlink import mavutil

DEST = "udpin:127.0.0.1:14550"
DURATION = 60.0


def main():
    m = mavutil.mavlink_connection(DEST, dialect="ardupilotmega",
                                   source_system=251, source_component=0)
    hb = m.wait_heartbeat(timeout=15)
    sysid, compid = m.target_system, m.target_component
    print(f"[OK] sys={sysid} via UDP bridge")

    # Request all streams at 4Hz
    for stream_id in (mavutil.mavlink.MAV_DATA_STREAM_ALL,):
        m.mav.request_data_stream_send(sysid, compid, stream_id, 4, 1)

    # Sample
    print(f"[SAMPLING {DURATION}s]", flush=True)
    msg_times = defaultdict(list)
    msg_counts = defaultdict(int)
    sys_status_history = []
    heartbeat_history = []
    ekf_history = []
    statustext = []
    mode_changes = []
    last_mode = None

    t0 = time.time()
    while time.time() - t0 < DURATION:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "BAD_DATA":
            continue
        now = time.time()
        msg_times[mt].append(now)
        msg_counts[mt] += 1
        if mt == "HEARTBEAT":
            heartbeat_history.append({
                "t": now - t0,
                "base_mode": msg.base_mode,
                "custom_mode": msg.custom_mode,
                "system_status": msg.system_status,
            })
            if msg.custom_mode != last_mode:
                if last_mode is not None:
                    mode_changes.append((now - t0, last_mode, msg.custom_mode))
                last_mode = msg.custom_mode
        elif mt == "SYS_STATUS":
            sys_status_history.append({
                "t": now - t0,
                "load": msg.load,
                "drop_rate_comm": msg.drop_rate_comm,
                "errors_comm": msg.errors_comm,
                "errors_count1": msg.errors_count1,
                "errors_count2": msg.errors_count2,
                "errors_count3": msg.errors_count3,
                "errors_count4": msg.errors_count4,
            })
        elif mt == "EKF_STATUS_REPORT":
            ekf_history.append({
                "t": now - t0,
                "flags": msg.flags,
                "vel": msg.velocity_variance,
                "pos_h": msg.pos_horiz_variance,
                "compass": msg.compass_variance,
            })
        elif mt == "STATUSTEXT":
            txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            statustext.append({
                "t": now - t0,
                "sev": msg.severity,
                "text": txt.rstrip("\x00 \r\n"),
            })

    print(f"[DONE] {time.time()-t0:.1f}s, {sum(msg_counts.values())} msgs")

    # ---- Analysis ----
    print(f"\n=== HEARTBEAT timing jitter ({len(heartbeat_history)} HBs) ===")
    if len(heartbeat_history) >= 2:
        hbs = [h["t"] for h in heartbeat_history]
        deltas = [hbs[i+1] - hbs[i] for i in range(len(hbs)-1)]
        mean = statistics.mean(deltas) * 1000
        std = statistics.pstdev(deltas) * 1000
        mn = min(deltas) * 1000
        mx = max(deltas) * 1000
        print(f"  Inter-arrival: mean={mean:.1f}ms  stdev={std:.2f}ms  range=({mn:.1f}, {mx:.1f})ms")
        print(f"  Expected: 1000ms (1 Hz)")
        if std < 50:
            print(f"  ✓ Heartbeat jitter excellent (<50ms stdev)")
        elif std < 200:
            print(f"  ✓ Heartbeat jitter acceptable")
        else:
            print(f"  ⚠ Heartbeat jitter HIGH")

    print(f"\n=== CPU load + comm errors ({len(sys_status_history)} samples) ===")
    if sys_status_history:
        loads = [s["load"] for s in sys_status_history]
        drops = [s["drop_rate_comm"] for s in sys_status_history]
        errs  = [s["errors_comm"] for s in sys_status_history]
        ec1   = [s["errors_count1"] for s in sys_status_history]
        ec2   = [s["errors_count2"] for s in sys_status_history]
        print(f"  CPU load (1/1000):  mean={statistics.mean(loads):.0f} "
              f"min={min(loads)}  max={max(loads)}  "
              f"=> {statistics.mean(loads)/10:.1f}% avg")
        print(f"  drop_rate_comm:     mean={statistics.mean(drops):.1f} max={max(drops)}")
        print(f"  errors_comm:        first={errs[0]}  last={errs[-1]}  delta={errs[-1]-errs[0]}")
        print(f"  errors_count1:      first={ec1[0]}  last={ec1[-1]}  delta={ec1[-1]-ec1[0]}")
        print(f"  errors_count2:      first={ec2[0]}  last={ec2[-1]}  delta={ec2[-1]-ec2[0]}")
        if max(loads) > 800:
            print(f"  ⚠ Peak CPU load >80%")
        else:
            print(f"  ✓ CPU load below 80%")

    print(f"\n=== EKF flags / variances ({len(ekf_history)} samples) ===")
    if ekf_history:
        flag_set = set(e["flags"] for e in ekf_history)
        print(f"  Distinct EKF flags seen: {sorted(hex(f) for f in flag_set)}")
        v_max = max(e["vel"] for e in ekf_history)
        ph_max = max(e["pos_h"] for e in ekf_history)
        cm_max = max(e["compass"] for e in ekf_history)
        print(f"  vel_variance max:     {v_max:.4f}")
        print(f"  pos_h_variance max:   {ph_max:.4f}")
        print(f"  compass_variance max: {cm_max:.4f}")

    print(f"\n=== Mode changes during run ===")
    if mode_changes:
        for t, from_m, to_m in mode_changes:
            print(f"  @{t:5.1f}s  mode {from_m} -> {to_m}")
    else:
        print("  (none — stayed in single mode)")

    print(f"\n=== STATUSTEXT ({len(statustext)} msgs) ===")
    crit = [s for s in statustext if s["sev"] <= 2]
    print(f"  CRITICAL/ALERT/EMERGENCY: {len(crit)}")
    for s in statustext[:8]:
        sevname = mavutil.mavlink.enums["MAV_SEVERITY"].get(s["sev"], "?").name
        print(f"  @{s['t']:5.1f}s [{sevname}] {s['text']}")

    print(f"\n=== Stream rates achieved (msg/s) — top 12 ===")
    rates = []
    for mt, ts in msg_times.items():
        if len(ts) >= 2:
            span = ts[-1] - ts[0]
            rate = (len(ts) - 1) / span if span > 0 else 0
            rates.append((mt, rate, len(ts)))
    rates.sort(key=lambda x: -x[1])
    for mt, rate, n in rates[:12]:
        print(f"  {mt:28s}  {rate:5.2f} Hz  ({n} msgs)")
    print(f"\n  total unique types: {len(msg_times)}")
    print(f"  total messages: {sum(msg_counts.values())}")


if __name__ == "__main__":
    main()
