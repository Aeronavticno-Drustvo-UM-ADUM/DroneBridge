#!/usr/bin/env python3
"""
Bench test for the stock DroneBridge ESP-NOW link (plan step 1).

Sends MAVLink GPS_RTCM_DATA (#233) messages with known payloads into the GND unit (e.g. ESP32-C3 USB/JTAG build) and,
if an AIR-side serial port is given, checks that every message comes out of the AIR unit's UART unchanged.

Setup:
    PC USB  ->  GND ESP32 (ESP-NOW GND)  ))) ESP-NOW )))  AIR ESP32 (ESP-NOW AIR)  UART  ->  USB-UART adapter  ->  PC

Requirements (in any Python venv, not the ESP-IDF one):
    pip install pymavlink pyserial

Example:
    python3 test/rtcm_link_test.py --gnd /dev/cu.usbmodem1134101 --air /dev/cu.usbserial-0001 --air-baud 115200
"""

import argparse
import os
import random
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")  # send MAVLink v2 like a real ground control station

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is missing. Install it with: pip install pymavlink pyserial")

RTCM_FIELD_LEN = 180


def build_payload(index: int, rng: random.Random) -> bytes:
    """
    Builds a test payload for one GPS_RTCM_DATA message.

    :param index: Message counter, stored in the first 4 bytes so received messages can be matched to sent ones
    :param rng: Random generator used for the payload length and content
    :return: Payload of 8 to 180 bytes
    """
    length = rng.randint(8, RTCM_FIELD_LEN)
    return index.to_bytes(4, "little") + bytes(rng.getrandbits(8) for _ in range(length - 4))


def receive_loop(air, expected: dict, results: dict, stop: threading.Event) -> None:
    """
    Reads GPS_RTCM_DATA messages from the AIR-side serial port and compares them to the sent payloads.

    :param air: pymavlink connection of the AIR unit's UART
    :param expected: Map of message counter -> (payload, send timestamp), filled by the sender
    :param results: Dictionary that receives the counters "ok", "corrupt", "unknown" and the list "latency_ms"
    :param stop: Event that ends the loop
    """
    while not stop.is_set():
        msg = air.recv_match(type="GPS_RTCM_DATA", blocking=True, timeout=0.2)
        if msg is None:
            continue
        data = bytes(msg.data[:msg.len])
        if len(data) < 4:
            results["unknown"] += 1
            continue
        index = int.from_bytes(data[:4], "little")
        sent = expected.get(index)
        if sent is None:
            results["unknown"] += 1
        elif sent[0] == data and msg.flags == 0:
            results["ok"] += 1
            results["latency_ms"].append((time.monotonic() - sent[1]) * 1000.0)
        else:
            results["corrupt"] += 1


def main() -> int:
    """
    Parses the command line, sends the test messages and prints a summary.

    :return: Process exit code - 0 if all messages arrived unchanged (or no AIR port was given), 1 otherwise
    """
    parser = argparse.ArgumentParser(description="Send GPS_RTCM_DATA through a DroneBridge ESP-NOW link and verify it.")
    parser.add_argument("--gnd", required=True, help="serial port of the GND unit (e.g. /dev/cu.usbmodem1134101)")
    parser.add_argument("--gnd-baud", type=int, default=115200, help="baud of the GND port (ignored for USB/JTAG)")
    parser.add_argument("--air", help="serial port of the USB-UART adapter wired to the AIR unit's UART")
    parser.add_argument("--air-baud", type=int, default=115200, help="baud of the AIR unit's UART")
    parser.add_argument("--count", type=int, default=100, help="number of GPS_RTCM_DATA messages to send")
    parser.add_argument("--rate", type=float, default=10.0, help="messages per second")
    parser.add_argument("--seed", type=int, default=1, help="random seed for the payloads")
    args = parser.parse_args()

    if args.count < 1 or args.rate <= 0:
        parser.error("--count must be >= 1 and --rate > 0")

    rng = random.Random(args.seed)
    gnd = mavutil.mavlink_connection(args.gnd, baud=args.gnd_baud, source_system=255, source_component=190)
    air = mavutil.mavlink_connection(args.air, baud=args.air_baud) if args.air else None

    expected = {}
    results = {"ok": 0, "corrupt": 0, "unknown": 0, "latency_ms": []}
    stop = threading.Event()
    receiver = None
    if air is not None:
        receiver = threading.Thread(target=receive_loop, args=(air, expected, results, stop), daemon=True)
        receiver.start()

    print(f"Sending {args.count} GPS_RTCM_DATA messages at {args.rate:g}/s to {args.gnd}")
    last_heartbeat = 0.0
    for index in range(args.count):
        now = time.monotonic()
        if now - last_heartbeat >= 1.0:
            gnd.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_heartbeat = now
        payload = build_payload(index, rng)
        expected[index] = (payload, time.monotonic())
        gnd.mav.gps_rtcm_data_send(0, len(payload), list(payload.ljust(RTCM_FIELD_LEN, b"\0")))
        time.sleep(1.0 / args.rate)

    if receiver is None:
        print("Done. No --air port given, nothing verified.")
        return 0

    time.sleep(2.0)  # let the last messages arrive
    stop.set()
    receiver.join()

    lost = args.count - results["ok"] - results["corrupt"]
    print(f"Received unchanged: {results['ok']}/{args.count}   corrupt: {results['corrupt']}   "
          f"lost: {lost}   unknown: {results['unknown']}")
    if results["latency_ms"]:
        lat = sorted(results["latency_ms"])
        print(f"Latency ms: min {lat[0]:.0f}  median {lat[len(lat) // 2]:.0f}  max {lat[-1]:.0f}")
    return 0 if results["ok"] == args.count else 1


if __name__ == "__main__":
    sys.exit(main())
