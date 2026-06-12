#!/usr/bin/env python3
"""
Reads telemetry from Teensy at 192.168.2.177 via UDP port 5001.
Format: [roll, pitch, yaw, depth, imuOk, magOk]

The Teensy sends telemetry to whoever last sent a control packet on port 5000.
This script sends a neutral ping first so the Teensy registers our IP.
"""

import socket
import json
import time
import sys
import os

TEENSY_IP   = '192.168.2.177'
TEENSY_CTRL_PORT  = 5000
TELEM_LISTEN_PORT = 5001

NEUTRAL_PWM = [1500] * 16
NEUTRAL_PKT = json.dumps({"pwms": NEUTRAL_PWM}).encode()

def send_ping(sock_ctrl):
    """Send a neutral control packet so Teensy learns our IP."""
    sock_ctrl.sendto(NEUTRAL_PKT, (TEENSY_IP, TEENSY_CTRL_PORT))

def clear_line():
    sys.stdout.write('\r\033[K')

def fmt_bool(v):
    return '\033[32mOK\033[0m' if v else '\033[31mFAIL\033[0m'

def main():
    sock_telem = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_telem.bind(('0.0.0.0', TELEM_LISTEN_PORT))
    sock_telem.settimeout(2.0)

    sock_ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Registering with Teensy at {TEENSY_IP}:{TEENSY_CTRL_PORT}...")
    send_ping(sock_ctrl)
    print(f"Listening for telemetry on UDP port {TELEM_LISTEN_PORT}")
    print("Press Ctrl+C to stop.\n")

    last_ping = time.time()
    packets = 0
    last_packet_time = None

    try:
        while True:
            # Re-ping every 5 s so Teensy keeps sending to us
            if time.time() - last_ping > 5.0:
                send_ping(sock_ctrl)
                last_ping = time.time()

            try:
                data, addr = sock_telem.recvfrom(256)
            except socket.timeout:
                age = time.time() - last_packet_time if last_packet_time else None
                status = f"(last packet {age:.1f}s ago)" if age else "(no packets yet)"
                clear_line()
                print(f"  Waiting for telemetry from {TEENSY_IP}... {status}", end='', flush=True)
                continue

            raw = data.decode('utf-8', errors='ignore').strip()
            try:
                vals = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if not isinstance(vals, list) or len(vals) < 6:
                continue

            roll, pitch, yaw, depth, imu_ok, mag_ok = vals[:6]
            packets += 1
            last_packet_time = time.time()

            clear_line()
            print(
                f"  Roll: {roll:+7.2f}°  "
                f"Pitch: {pitch:+7.2f}°  "
                f"Yaw: {yaw:+7.2f}°  "
                f"Depth: {depth:6.3f}m  "
                f"IMU:{fmt_bool(imu_ok)}  "
                f"Mag:{fmt_bool(mag_ok)}  "
                f"[#{packets}]",
                end='', flush=True
            )

    except KeyboardInterrupt:
        print(f"\n\nStopped. Received {packets} telemetry packets.")
    finally:
        sock_telem.close()
        sock_ctrl.close()

if __name__ == '__main__':
    main()
