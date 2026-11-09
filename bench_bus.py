#!/usr/bin/env python3
"""Measure how fast the servo bus can be read - READ ONLY, the arm never moves.

    python3 bench_bus.py          # real arm (port/baud from the official arm_settings.py)
    python3 bench_bus.py --mock   # simulated arm

Prints the time one servo reply takes, whether SYNC READ (all 5 joints in one request)
works on this board, and the fluid-recording sample rate each method can reach.
"""
from __future__ import annotations

import argparse
import statistics
import time

from motion import load_config, open_bus
from poses import JOINT_IDS
from servo_bus import ADDR_PRESENT_POSITION, INST_READ, ServoError, build_packet, mock_bus, parse_status

N = 40


def ms(xs):
    return f"{statistics.median(xs) * 1000:5.1f} ms (min {min(xs) * 1000:.1f}, max {max(xs) * 1000:.1f})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mock", action="store_true")
    a = ap.parse_args()
    cfg = load_config()
    bus = mock_bus(time_scale=1)[0] if a.mock else open_bus(cfg)
    print(f"port {cfg['port']} @ {cfg['baud']} baud\n")

    # 1) one reply, split into 'waiting for the first byte' and 'whole packet'
    first, whole = [], []
    pkt = build_packet(1, INST_READ, [ADDR_PRESENT_POSITION, 2])
    for _ in range(N):
        with bus._lock:
            bus._flush()
            t0 = time.perf_counter()
            bus.ser.write(pkt)
            got, t_first, done = bytearray(), None, False
            while not done and time.perf_counter() - t0 < 0.25:
                c = bus.ser.read(bus.ser.in_waiting or 1)
                if not c:
                    continue
                if t_first is None:
                    t_first = time.perf_counter()
                got += c
                while True:
                    st, got = parse_status(got)
                    if st is None:
                        break
                    if st["raw"] != pkt and st["id"] == 1:      # skip our own echo
                        done = True
            if done:
                first.append(t_first - t0)
                whole.append(time.perf_counter() - t0)
    if not first:
        print("one servo reply : NO REPLY from servo 1 - check power / cable / port")
        return
    print(f"one servo reply : first byte after {ms(first)}")
    print(f"                  whole reply   {ms(whole)}")

    # 2) the 5 single reads (what the recorder falls back to)
    t = []
    for _ in range(N):
        t0 = time.perf_counter()
        try:
            for j in JOINT_IDS:
                bus.read_position(j)
            t.append(time.perf_counter() - t0)
        except ServoError:
            pass
    if t:
        print(f"5 single reads  : {ms(t)}  ->  max ~{1 / statistics.median(t):.0f} Hz recording")
    else:
        print("5 single reads  : failed")

    # 3) SYNC READ (all 5 in one request)
    t, fails = [], 0
    for _ in range(N):
        t0 = time.perf_counter()
        try:
            bus.sync_read_positions(JOINT_IDS)
            t.append(time.perf_counter() - t0)
        except ServoError:
            fails += 1
    if t:
        print(f"SYNC READ       : {ms(t)}  ->  max ~{1 / statistics.median(t):.0f} Hz recording"
              + (f"   ({fails}/{N} failed)" if fails else ""))
    else:
        print(f"SYNC READ       : not supported by this board ({fails}/{N} failed)")

    per = statistics.median(first) * 1000
    print()
    if per > 5:
        print(f"VERDICT: every request waits ~{per:.0f} ms before the servo's answer even starts.\n"
              "At 115200 baud a reply only takes ~0.7 ms on the wire, so the delay is in the\n"
              "USB / ESP32 bridge, not the servos. Send this whole output back.")
    else:
        print("VERDICT: replies are fast. Send this whole output back if recording is still slow.")


if __name__ == "__main__":
    main()
