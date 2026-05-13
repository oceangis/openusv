"""SBUS Frame Simulator (non-inverted, for direct GPIO 8 injection).

Hardware setup:
  USB-TTL adapter TX  →  ESP32-S3 GPIO 8  (bypass board's Q53 inverter)
  USB-TTL adapter GND →  board GND

If you instead connect to the board's SBUS input header (before Q53),
the signal needs HARDWARE inversion — this script won't work, you'd need
a discrete NPN+resistor inverter circuit.

Protocol:
  - 25 bytes per frame
  - 100000 baud, 8E2 (8 data bits, even parity, 2 stop bits) — INVERTED
    on a real SBUS line, but for "SBUS_NI" we send it non-inverted.
  - Frame rate: 14ms (~71 Hz) typical, or 7ms in high-speed mode

Frame layout:
  byte 0       : 0x0F (start marker)
  bytes 1-22   : 16 channels × 11 bits each, little-endian packed
  byte 23      : flags (failsafe, frame_lost, ch17, ch18)
  byte 24      : 0x00 (end marker)

ArduPilot's AP_RCProtocol auto-detects SBUS / SBUS_NI / DSM by analyzing
pulse patterns from the RMT capture. After ~3 valid frames it locks on.

Usage:
  Open one terminal:    bench/verify/sbus_simulator.py --port COM10
  Open another:         monitor RC_CHANNELS on MAVLink via COM58
                        (e.g. bench/verify/sbus_verify.py)

The simulator cycles through patterns:
  Pattern 1 (0-5s):   all channels = 1500 (center)
  Pattern 2 (5-10s):  ch1=1700 (right roll), ch3=1600 (up throttle)
  Pattern 3 (10-15s): ch1=1300 (left), ch3=1400 (down)
  Pattern 4 (15-20s): sweep ch1 1000→2000 then back
  Pattern 5 (20s+):   all back to 1500
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("install: pip install pyserial")
    sys.exit(1)


def encode_sbus_frame(channels, failsafe=False, frame_lost=False):
    """Pack 16 channels (each 11-bit value 0-2047, where 992 ≈ 1500us PWM)
    into 25-byte SBUS frame. Returns bytes object.
    """
    if len(channels) != 16:
        raise ValueError("need exactly 16 channels")
    frame = bytearray(25)
    frame[0] = 0x0F  # SBUS start byte

    # Pack 16 channels × 11 bits = 176 bits = 22 bytes
    bits = 0
    bit_pos = 0
    for ch in channels:
        v = max(0, min(2047, int(ch)))
        bits |= (v << bit_pos)
        bit_pos += 11
    # Now bits is 176 bits wide. Extract bytes little-endian.
    for i in range(22):
        frame[1 + i] = (bits >> (i * 8)) & 0xFF

    # byte 23: flags
    flags = 0
    if failsafe:
        flags |= 0x08
    if frame_lost:
        flags |= 0x04
    frame[23] = flags

    frame[24] = 0x00  # SBUS end byte
    return bytes(frame)


def pwm_to_sbus(pwm_us):
    """Convert PWM µs (1000-2000 typical) to SBUS 11-bit value (0-2047).
    SBUS 992 corresponds to 1500us (center).
    Scale: 172 ≈ 1000us, 1811 ≈ 2000us, (range 1639 over 1000us = 1.639 per us)
    """
    return int(round((pwm_us - 1500) * 1.6 + 992))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="USB-TTL COM port (e.g. COM10)")
    ap.add_argument("--baud", type=int, default=100000, help="SBUS baud (default 100000)")
    ap.add_argument("--frame-ms", type=float, default=14.0, help="Frame interval (default 14ms)")
    ap.add_argument("--duration", type=float, default=30.0, help="Total run time (default 30s)")
    args = ap.parse_args()

    print(f"[OPEN] {args.port} @ {args.baud} baud, 8E2")
    try:
        ser = serial.Serial(
            args.port, args.baud,
            bytesize=8, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO,
            timeout=0,
        )
    except Exception as e:
        print(f"[ERROR] open {args.port}: {e}")
        sys.exit(2)

    print(f"[INFO] sending non-inverted SBUS frames; connect TX directly to GPIO 8")
    print(f"[INFO] cycle through test patterns for {args.duration}s\n")

    period = args.frame_ms / 1000.0
    t_start = time.time()
    frame_count = 0
    last_print = 0
    last_pattern = None

    while time.time() - t_start < args.duration:
        t = time.time() - t_start

        # pattern selection
        if t < 5:
            channels = [pwm_to_sbus(1500)] * 16
            pattern = "center (all 1500)"
        elif t < 10:
            channels = [pwm_to_sbus(1500)] * 16
            channels[0] = pwm_to_sbus(1700)  # ch1
            channels[2] = pwm_to_sbus(1600)  # ch3
            pattern = "right+fwd (ch1=1700, ch3=1600)"
        elif t < 15:
            channels = [pwm_to_sbus(1500)] * 16
            channels[0] = pwm_to_sbus(1300)
            channels[2] = pwm_to_sbus(1400)
            pattern = "left+rev (ch1=1300, ch3=1400)"
        elif t < 20:
            channels = [pwm_to_sbus(1500)] * 16
            # sweep ch1
            phase = (t - 15) / 5.0  # 0..1
            sweep_us = 1000 + int(1000 * (1 - abs(2 * phase - 1)))
            channels[0] = pwm_to_sbus(sweep_us)
            pattern = f"sweep ch1={sweep_us}"
        else:
            channels = [pwm_to_sbus(1500)] * 16
            pattern = "back to center"

        if pattern != last_pattern:
            print(f"  @{t:5.1f}s  pattern: {pattern}")
            last_pattern = pattern

        frame = encode_sbus_frame(channels)
        ser.write(frame)
        ser.flush()
        frame_count += 1

        if time.time() - last_print > 1.0:
            actual_rate = frame_count / max(0.001, t)
            print(f"    [{frame_count} frames, {actual_rate:.1f} Hz]")
            last_print = time.time()

        # sleep to next frame
        next_frame_time = t_start + frame_count * period
        sleep_for = next_frame_time - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)

    ser.close()
    print(f"\n[DONE] sent {frame_count} frames in {time.time()-t_start:.1f}s")


if __name__ == "__main__":
    main()
