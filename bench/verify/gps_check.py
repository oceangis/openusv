"""60-second GPS status snapshot — fix_type, sats, HDOP, position drift.

Run this AFTER any soak completes so it has the serial port. Will not contend
for COM10 with another mavlink consumer.
"""
import argparse
import statistics
import sys
import time

from pymavlink import mavutil


FIX_TYPES = {
    0: "NO_GPS", 1: "NO_FIX", 2: "FIX_2D", 3: "FIX_3D",
    4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED",
    7: "STATIC", 8: "PPP",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    print(f"[CONNECT] {args.port} @ {args.baud}", flush=True)
    m = mavutil.mavlink_connection(args.port, baud=args.baud,
                                    dialect="ardupilotmega",
                                    source_system=255, source_component=0)
    hb = m.wait_heartbeat(timeout=10)
    if not hb:
        print("[FATAL] no HEARTBEAT", flush=True)
        sys.exit(2)
    sysid, compid = m.target_system, m.target_component
    print(f"[OK] sys={sysid} state={mavutil.mavlink.enums['MAV_STATE'].get(hb.system_status,'?').name}",
          flush=True)

    # Push GPS-related streams hard
    for sid in (mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA3):
        m.mav.request_data_stream_send(sysid, compid, sid, 5, 1)

    gps_samples = []
    pos_samples = []
    ekf_flags_seen = set()
    last_ekf = None
    last_print = 0.0

    t0 = time.time()
    print(f"\n[SAMPLE] {args.seconds:.0f}s ...", flush=True)
    while time.time() - t0 < args.seconds:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "GPS_RAW_INT":
            gps_samples.append({
                "t": time.time() - t0,
                "fix_type": msg.fix_type,
                "sats": msg.satellites_visible,
                "hdop": msg.eph / 100.0,    # cm to m
                "vdop": msg.epv / 100.0,
                "lat_e7": msg.lat,
                "lon_e7": msg.lon,
                "alt_mm": msg.alt,
                "vel_cms": msg.vel,
            })
        elif mt == "GLOBAL_POSITION_INT":
            pos_samples.append({
                "lat_e7": msg.lat, "lon_e7": msg.lon,
                "alt_mm": msg.alt, "rel_alt_mm": msg.relative_alt,
            })
        elif mt == "EKF_STATUS_REPORT":
            ekf_flags_seen.add(msg.flags)
            last_ekf = msg

        # Print live status every 5 s
        if time.time() - last_print > 5 and gps_samples:
            g = gps_samples[-1]
            print(f"  +{g['t']:.0f}s  fix={FIX_TYPES.get(g['fix_type'], g['fix_type'])}  "
                  f"sats={g['sats']:2d}  HDOP={g['hdop']:.2f}  "
                  f"lat={g['lat_e7']/1e7:+.7f}  lon={g['lon_e7']/1e7:+.7f}  "
                  f"alt={g['alt_mm']/1000.0:.1f}m", flush=True)
            last_print = time.time()

    # ---- Summary
    print("\n" + "=" * 64, flush=True)
    print("GPS Snapshot Summary", flush=True)
    print("=" * 64, flush=True)
    print(f"GPS_RAW_INT msgs: {len(gps_samples)}  (rate={len(gps_samples)/args.seconds:.1f} Hz)", flush=True)
    print(f"GLOBAL_POSITION_INT msgs: {len(pos_samples)}  (rate={len(pos_samples)/args.seconds:.1f} Hz)", flush=True)

    if not gps_samples:
        print("\n[WARN] no GPS_RAW_INT messages — GPS subsystem may not be enabled "
              "or driver is silent. Check GPS_TYPE param.", flush=True)
        sys.exit(0)

    fix_types_seen = sorted({g["fix_type"] for g in gps_samples})
    fix_type_last = gps_samples[-1]["fix_type"]
    print(f"\nFix types seen: {[FIX_TYPES.get(t, t) for t in fix_types_seen]}", flush=True)
    print(f"Final fix: {FIX_TYPES.get(fix_type_last, fix_type_last)}", flush=True)

    sats = [g["sats"] for g in gps_samples]
    print(f"Sats visible: min={min(sats)} max={max(sats)} mean={statistics.mean(sats):.1f}  final={sats[-1]}",
          flush=True)

    hdops = [g["hdop"] for g in gps_samples if g["hdop"] > 0]
    if hdops:
        print(f"HDOP: min={min(hdops):.2f}  max={max(hdops):.2f}  final={hdops[-1]:.2f}", flush=True)

    # Position spread — if locked, this tells us static error
    if fix_type_last >= 3:
        with_fix = [g for g in gps_samples if g["fix_type"] >= 3]
        lats = [g["lat_e7"] / 1e7 for g in with_fix]
        lons = [g["lon_e7"] / 1e7 for g in with_fix]
        alts = [g["alt_mm"] / 1000.0 for g in with_fix]
        # ~111 km per degree latitude
        lat_spread_m = (max(lats) - min(lats)) * 111000.0
        # longitude varies with latitude (cos)
        import math
        cos_lat = math.cos(math.radians(statistics.mean(lats))) if lats else 1.0
        lon_spread_m = (max(lons) - min(lons)) * 111000.0 * cos_lat
        print(f"\nWith 3D-fix samples: {len(with_fix)}", flush=True)
        print(f"  lat:    {statistics.mean(lats):+.7f}  spread={lat_spread_m:.2f} m", flush=True)
        print(f"  lon:    {statistics.mean(lons):+.7f}  spread={lon_spread_m:.2f} m", flush=True)
        print(f"  alt:    {statistics.mean(alts):.1f} m  range=[{min(alts):.1f}..{max(alts):.1f}]", flush=True)
        print(f"  pos std drift (good indoor: <5m): "
              f"~{max(lat_spread_m, lon_spread_m):.1f} m", flush=True)
    else:
        print(f"\n[INFO] no 3D fix during the sample window — indoor expected, "
              f"move antenna near a window or outside", flush=True)

    print(f"\nEKF flags seen: {[f'0x{x:04X}' for x in sorted(ekf_flags_seen)]}", flush=True)
    if last_ekf:
        print(f"Last EKF variances: vel={last_ekf.velocity_variance:.3f}  "
              f"pos_h={last_ekf.pos_horiz_variance:.3f}  "
              f"compass={last_ekf.compass_variance:.3f}", flush=True)


if __name__ == "__main__":
    main()
