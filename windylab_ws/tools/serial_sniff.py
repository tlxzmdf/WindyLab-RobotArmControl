#!/usr/bin/env python3
"""Listen on arm serial port and report incoming bytes / protocol frames."""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial not installed: pip3 install pyserial")
    sys.exit(1)

# ProtocolV1 recv header: 0x55 0xAA (see protocol_v1 LinkFrame)
RX_HDR = bytes([0x55, 0xAA])
# ProtocolV1 send header: 0xFE 0xAA
TX_HDR = bytes([0xFE, 0xAA])


def hex_preview(data: bytes, limit: int = 64) -> str:
    chunk = data[:limit]
    s = " ".join(f"{b:02X}" for b in chunk)
    if len(data) > limit:
        s += f" ... (+{len(data) - limit} bytes)"
    return s


def count_frames(buf: bytes, header: bytes) -> int:
    n = 0
    i = 0
    while True:
        j = buf.find(header, i)
        if j < 0:
            break
        n += 1
        i = j + 1
    return n


def main():
    p = argparse.ArgumentParser(description="Sniff arm serial port for incoming data")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--duration", type=float, default=8.0, help="Listen seconds")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[FAIL] Cannot open {args.port}: {e}")
        print("Check: USB plugged in? run `ls -la /dev/ttyUSB*`")
        return 1

    print(f"[OK] Opened {args.port} @ {args.baud} baud, listening {args.duration}s ...")
    print("Tip: ensure mechanical arm is powered on and no other process uses this port.")

    buf = bytearray()
    t0 = time.time()
    idle_chunks = 0

    try:
        while time.time() - t0 < args.duration:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                buf.extend(chunk)
                idle_chunks = 0
                print(f"  +{len(chunk):4d} bytes  total={len(buf):6d}  preview: {hex_preview(chunk, 32)}")
            else:
                idle_chunks += 1
                time.sleep(0.05)
    finally:
        ser.close()

    elapsed = time.time() - t0
    print()
    print("=== Summary ===")
    print(f"Duration   : {elapsed:.1f}s")
    print(f"Total bytes: {len(buf)}")
    print(f"RX frames  : {count_frames(bytes(buf), RX_HDR)}  (header 55 AA)")
    print(f"TX frames  : {count_frames(bytes(buf), TX_HDR)}  (header FE AA)")

    if len(buf) == 0:
        print("[FAIL] No data received.")
        print("Possible causes: arm not powered, wrong port, wrong baud, loose wiring, WSL USB disconnected.")
        return 2

    print(f"[OK] Data received. First 128 bytes:\n  {hex_preview(bytes(buf), 128)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
