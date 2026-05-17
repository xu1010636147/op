#!/usr/bin/env python3
"""
Sniff BYD ACC_MPC_STATE (addr=790) on BOTH bus 0 and bus 2.

Usage:
  cd /data/openpilot && python3 sniff_790.py

- Prints the first frame per bus (INIT).
- Prints each subsequent frame ONLY if the 8 bytes changed, with a
  per-byte bit-diff annotation.
- Prints a 5-second running count of 790 frames per bus so you can
  verify which bus actually carries this ID on your car.

In the car: toggle LKA / LDW via the infotainment menu, wait ~2 s
between each toggle. The bit(s) that flip on a toggle are the real
LKAS_Config signal.
"""
import sys
import time
import cereal.messaging as messaging

BUSES = (0, 2)
ADDR = 790


def bit_diff(old: bytes, new: bytes):
    changes = []
    for i, (a, b) in enumerate(zip(old, new)):
        x = a ^ b
        if x:
            bits = [str(j) for j in range(8) if x & (1 << j)]
            changes.append(f"b{i}.bit{{{','.join(bits)}}}")
    return changes


def main():
    sock = messaging.sub_sock('can', addr='127.0.0.1')
    print(f"[sniff_790] listening for buses={BUSES} addr={ADDR} (0x{ADDR:X})  Ctrl+C to stop", flush=True)
    last = {b: None for b in BUSES}
    count = {b: 0 for b in BUSES}
    t0 = time.monotonic()
    last_stat = t0
    while True:
        can_recv = messaging.drain_sock(sock, wait_for_one=True)
        for x in can_recv:
            for y in x.can:
                if y.src in BUSES and y.address == ADDR:
                    bus = y.src
                    count[bus] += 1
                    dat = bytes(y.dat)
                    if last[bus] is None:
                        hexs = ' '.join(f"{b:02X}" for b in dat)
                        print(f"[t={time.monotonic()-t0:7.3f}] bus{bus} INIT {hexs}", flush=True)
                        last[bus] = dat
                    elif dat != last[bus]:
                        hexs = ' '.join(f"{b:02X}" for b in dat)
                        changes = ','.join(bit_diff(last[bus], dat))
                        print(f"[t={time.monotonic()-t0:7.3f}] bus{bus} {hexs}  | changed: {changes}", flush=True)
                        last[bus] = dat

        if time.monotonic() - last_stat > 5:
            last_stat = time.monotonic()
            stat = ' '.join(f"bus{b}={count[b]}" for b in BUSES)
            print(f"[t={time.monotonic()-t0:7.3f}] [stats] 790 count: {stat}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[sniff_790] stopped", flush=True)
        sys.exit(0)
