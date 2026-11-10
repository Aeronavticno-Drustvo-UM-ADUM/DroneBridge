#!/usr/bin/env python3
"""
RTK correction emulator for a DroneBridge ESP-NOW link (bench test).

GND side: takes RTCM3 frames from a source, wraps them into MAVLink GPS_RTCM_DATA (#233) like a ground control
station does (fragmentation rules of the MAVLink spec / ArduPilot) and sends them to the GND unit's serial port.

AIR side (optional): reads #233 from the AIR unit's UART (through a USB-UART adapter), re-assembles the RTCM stream,
checks every RTCM3 frame (CRC24Q) and compares it with what was sent.

    PC (this script) --USB--> GND ESP32 ))) ESP-NOW ))) AIR ESP32 --UART (MAVLink)--> USB-UART adapter --> PC

RTCM sources:
    (default)      synthetic: RTCM 1005 (base position) + one filler frame (proprietary type 4094) per second
    --file X       replay a recorded raw RTCM3 file (e.g. saved with u-center or str2str)
    --base PORT    live RTCM3 from a real base receiver on a serial port (e.g. F9P base)

Requirements (in any Python venv, not the ESP-IDF one):
    pip install pymavlink pyserial

Example:
    python3 test/rtk_emulator.py --gnd /dev/cu.usbmodem1134101 --air /dev/cu.usbserial-0001 --duration 30
"""

import argparse
import collections
import os
import struct
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")  # send MAVLink v2 like a real ground control station

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is missing. Install it with: pip install pymavlink pyserial")

RTCM_FIELD_LEN = 180            # GPS_RTCM_DATA.data length
RTCM_MAX_FRAGMENTS = 4          # 2-bit fragment id
RTCM3_PREAMBLE = 0xD3
FILLER_MESSAGE_TYPE = 4094      # proprietary RTCM range, ignored by GNSS receivers
EXAMPLE_BASE_ECEF_M = (4295040.0, 1110885.0, 4567656.0)  # example base position, override with --ecef


def crc24q(data: bytes) -> int:
    """
    Calculates the CRC-24Q checksum used by RTCM3.

    :param data: Bytes to calculate the checksum over (preamble, length and payload)
    :return: 24-bit checksum
    """
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def build_rtcm3_frame(payload: bytes) -> bytes:
    """
    Wraps an RTCM3 message payload into a complete frame (preamble, length, payload, CRC24Q).

    :param payload: Message payload, at most 1023 bytes
    :return: Complete RTCM3 frame
    """
    if len(payload) > 1023:
        raise ValueError("RTCM3 payload too long")
    body = bytes([RTCM3_PREAMBLE, (len(payload) >> 8) & 0x03, len(payload) & 0xFF]) + payload
    return body + crc24q(body).to_bytes(3, "big")


def rtcm3_message_type(frame: bytes) -> int:
    """
    Returns the 12-bit message number of an RTCM3 frame.

    :param frame: Complete RTCM3 frame
    :return: Message number, 0 if the payload is too short to contain one
    """
    return (frame[3] << 4) | (frame[4] >> 4) if len(frame) >= 8 else 0


class BitWriter:
    """Collects big-endian bit fields as used by RTCM3 messages."""

    def __init__(self):
        """Creates an empty bit buffer."""
        self.bits = []

    def put(self, value: int, width: int) -> None:
        """
        Appends a field. Negative values are stored as two's complement.

        :param value: Field value
        :param width: Field width in bits
        """
        if value < 0:
            value += 1 << width
        self.bits.extend((value >> shift) & 1 for shift in range(width - 1, -1, -1))

    def to_bytes(self) -> bytes:
        """
        Returns the collected bits, zero padded to whole bytes.

        :return: Packed bytes
        """
        bits = self.bits + [0] * (-len(self.bits) % 8)
        return bytes(int("".join(map(str, bits[i:i + 8])), 2) for i in range(0, len(bits), 8))


def build_rtcm_1005(station_id: int, ecef_m) -> bytes:
    """
    Builds an RTCM 1005 frame (stationary RTK reference station antenna position).

    :param station_id: Reference station id (0-4095)
    :param ecef_m: Antenna position as ECEF (x, y, z) in metres
    :return: Complete RTCM3 frame
    """
    writer = BitWriter()
    writer.put(1005, 12)                    # DF002 message number
    writer.put(station_id, 12)              # DF003 reference station id
    writer.put(0, 6)                        # DF021 ITRF realization year
    writer.put(1, 1)                        # DF022 GPS
    writer.put(1, 1)                        # DF023 GLONASS
    writer.put(1, 1)                        # DF024 Galileo
    writer.put(0, 1)                        # DF141 reference station indicator
    writer.put(round(ecef_m[0] * 10000), 38)  # DF025 ECEF-X [0.1 mm]
    writer.put(0, 1)                        # DF142 single receiver oscillator
    writer.put(0, 1)                        # DF001 reserved
    writer.put(round(ecef_m[1] * 10000), 38)  # DF026 ECEF-Y [0.1 mm]
    writer.put(0, 2)                        # DF364 quarter cycle indicator
    writer.put(round(ecef_m[2] * 10000), 38)  # DF027 ECEF-Z [0.1 mm]
    return build_rtcm3_frame(writer.to_bytes())


def build_filler_frame(size: int, counter: int) -> bytes:
    """
    Builds a proprietary RTCM3 frame that simulates the load of observation messages (MSM).

    :param size: Payload size in bytes (at least 6)
    :param counter: Running counter written into the payload so every frame is unique
    :return: Complete RTCM3 frame
    """
    writer = BitWriter()
    writer.put(FILLER_MESSAGE_TYPE, 12)
    writer.put(0, 4)
    head = writer.to_bytes() + counter.to_bytes(4, "big")
    body = bytes((counter + i) & 0xFF for i in range(max(size, len(head)) - len(head)))
    return build_rtcm3_frame(head + body)


class Rtcm3Parser:
    """Splits a byte stream into RTCM3 frames with valid CRC."""

    def __init__(self):
        """Creates a parser with an empty buffer."""
        self.buffer = bytearray()
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list:
        """
        Adds bytes to the parser and returns all complete frames found so far.

        :param data: New bytes from the stream
        :return: List of complete RTCM3 frames (bytes) with valid CRC
        """
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(RTCM3_PREAMBLE)
            if start < 0:
                self.skipped_bytes += len(self.buffer)
                self.buffer.clear()
                return frames
            if start:
                self.skipped_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 3:
                return frames
            if self.buffer[1] & 0xFC:           # reserved bits must be zero
                self.skipped_bytes += 1
                del self.buffer[:1]
                continue
            frame_len = 3 + (((self.buffer[1] & 0x03) << 8) | self.buffer[2]) + 3
            if len(self.buffer) < frame_len:
                return frames
            frame = bytes(self.buffer[:frame_len])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], "big"):
                frames.append(frame)
                del self.buffer[:frame_len]
            else:
                self.skipped_bytes += 1
                del self.buffer[:1]


class RtcmReassembler:
    """Re-assembles GPS_RTCM_DATA messages into RTCM data blocks (same rules as ArduPilot)."""

    def __init__(self):
        """Creates an empty re-assembler."""
        self.buffer = bytearray(RTCM_FIELD_LEN * RTCM_MAX_FRAGMENTS)
        self.sequence = 0
        self.received = 0
        self.count = 0
        self.length = 0
        self.discarded = 0

    def add(self, flags: int, data: bytes) -> bytes:
        """
        Adds one GPS_RTCM_DATA message.

        :param flags: GPS_RTCM_DATA.flags
        :param data: GPS_RTCM_DATA.data[:len]
        :return: A complete data block, or b"" if the block is not complete yet
        """
        if not flags & 1:
            return bytes(data)
        fragment = (flags >> 1) & 0x03
        sequence = (flags >> 3) & 0x1F
        if self.received and (sequence != self.sequence or self.received & (1 << fragment)):
            self.discarded += bin(self.received).count("1")
            self.received = 0
            self.count = 0
        self.sequence = sequence
        self.received |= 1 << fragment
        offset = RTCM_FIELD_LEN * fragment
        self.buffer[offset:offset + len(data)] = data
        if len(data) < RTCM_FIELD_LEN:
            self.count = fragment + 1
            self.length = offset + len(data)
        elif self.received == 0x0F:
            self.count = RTCM_MAX_FRAGMENTS
            self.length = RTCM_FIELD_LEN * RTCM_MAX_FRAGMENTS
        if self.count and self.received == (1 << self.count) - 1:
            block = bytes(self.buffer[:self.length])
            self.received = 0
            self.count = 0
            return block
        return b""


class RtcmSender:
    """Wraps RTCM3 frames into GPS_RTCM_DATA messages and sends them to the GND unit."""

    def __init__(self, connection, pace_s: float = 0.0):
        """
        :param connection: pymavlink connection of the GND unit
        :param pace_s: Pause after every GPS_RTCM_DATA message in seconds (0 = send bursts like a GCS does)
        """
        self.connection = connection
        self.pace_s = pace_s
        self.sequence = 0
        self.messages_sent = 0

    def _send(self, flags: int, chunk: bytes) -> None:
        """
        Sends one GPS_RTCM_DATA message.

        :param flags: GPS_RTCM_DATA.flags
        :param chunk: Up to 180 bytes of RTCM data
        """
        self.connection.mav.gps_rtcm_data_send(flags, len(chunk), list(chunk.ljust(RTCM_FIELD_LEN, b"\0")))
        self.messages_sent += 1
        if self.pace_s > 0:
            time.sleep(self.pace_s)

    def send_frame(self, frame: bytes) -> None:
        """
        Sends one RTCM3 frame: unfragmented if it fits into one message, fragmented (up to 4 x 180 bytes) otherwise.
        Frames longer than 720 bytes are sent as consecutive unfragmented messages.

        :param frame: Complete RTCM3 frame
        """
        chunks = [frame[i:i + RTCM_FIELD_LEN] for i in range(0, len(frame), RTCM_FIELD_LEN)]
        if len(chunks) == 1:
            self._send(0, frame)
        elif len(chunks) <= RTCM_MAX_FRAGMENTS:
            for fragment, chunk in enumerate(chunks):
                self._send(1 | (fragment << 1) | ((self.sequence & 0x1F) << 3), chunk)
            if len(frame) % RTCM_FIELD_LEN == 0 and len(chunks) < RTCM_MAX_FRAGMENTS:
                # a block that is an exact multiple of 180 bytes needs an empty last fragment
                self._send(1 | (len(chunks) << 1) | ((self.sequence & 0x1F) << 3), b"")
            self.sequence = (self.sequence + 1) & 0x1F
        else:
            for chunk in chunks:
                self._send(0, chunk)


def receive_loop(air, sent: collections.Counter, stats: dict, stop: threading.Event) -> None:
    """
    Reads GPS_RTCM_DATA from the AIR-side port, re-assembles RTCM3 frames and compares them with the sent frames.

    :param air: pymavlink connection of the AIR unit's UART
    :param sent: Counter of sent frames, shared with the sender (protected by stats["lock"])
    :param stats: Result counters ("ok", "unexpected", "by_type", "lock")
    :param stop: Event that ends the loop
    """
    reassembler = RtcmReassembler()
    parser = Rtcm3Parser()
    while not stop.is_set():
        msg = air.recv_match(type="GPS_RTCM_DATA", blocking=True, timeout=0.2)
        if msg is None:
            continue
        block = reassembler.add(msg.flags, bytes(msg.data[:msg.len]))
        for frame in parser.feed(block):
            with stats["lock"]:
                if sent[frame] > 0:
                    sent[frame] -= 1
                    stats["ok"] += 1
                    stats["by_type"][rtcm3_message_type(frame)] += 1
                else:
                    stats["unexpected"] += 1
    stats["skipped_bytes"] = parser.skipped_bytes
    stats["discarded_fragments"] = reassembler.discarded


def frame_source(args):
    """
    Yields RTCM3 frames from the selected source until the source ends or the duration is over.

    :param args: Parsed command line arguments
    :return: Generator of complete RTCM3 frames
    """
    end = time.monotonic() + args.duration
    if args.file:
        with open(args.file, "rb") as file:
            frames = Rtcm3Parser().feed(file.read())
        if not frames:
            sys.exit(f"No RTCM3 frames found in {args.file}")
        while time.monotonic() < end:
            for frame in frames:
                yield frame
                time.sleep(args.interval)
                if time.monotonic() >= end:
                    return
            if not args.loop:
                return
    elif args.base:
        import serial
        parser = Rtcm3Parser()
        with serial.Serial(args.base, args.base_baud, timeout=0.1) as port:
            while time.monotonic() < end:
                yield from parser.feed(port.read(1024))
    else:
        counter = 0
        while time.monotonic() < end:
            yield build_rtcm_1005(args.station_id, args.ecef)
            yield build_filler_frame(args.filler_bytes, counter)
            counter += 1
            time.sleep(1.0)


def set_gnd_protocol(connection, protocol: int) -> None:
    """
    Sets the serial protocol (SERIAL_TEL_PROTO) of the DroneBridge GND unit on 'connection' via MAVLink PARAM_SET.
    Every MAVLink message the GND reads from serial is also broadcast to the AIR units, so the request must not be a
    broadcast: a GCS heartbeat first makes the GND adopt system id 255 (AIR units keep the id of their flight
    controller), then PARAM_SET is addressed to 255 / component 68 (DroneBridge, MAV_COMP_ID_TELEMETRY_RADIO).
    DroneBridge reads integer parameters byte-wise from the float field (MAVLink "bytewise" encoding).
    In RTCM mode (6) the GND no longer writes to its serial port, so switching to 6 is not acknowledged.

    :param connection: pymavlink connection of the GND unit, source system 255
    :param protocol: 1 MSP/LTM, 4 MAVLink, 5 transparent, 6 RTCM in -> MAVLink
    """
    for _ in range(3):
        connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.3)
    value = struct.unpack("<f", struct.pack("<I", protocol))[0]
    connection.mav.param_set_send(255, mavutil.mavlink.MAV_COMP_ID_TELEMETRY_RADIO, b"SERIAL_TEL_PROTO", value,
                                  mavutil.mavlink.MAV_PARAM_TYPE_UINT8)
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        msg = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        if msg is not None and msg.param_id == "SERIAL_TEL_PROTO":
            got = struct.unpack("<I", struct.pack("<f", msg.param_value))[0] & 0xFF
            print(f"GND confirmed SERIAL_TEL_PROTO = {got}")
            return
    if protocol == 6:
        print("PARAM_SET sent. No reply is expected: in RTCM mode the GND does not write to its serial port.")
    else:
        print("PARAM_SET sent, but the GND did not confirm it.")


def main() -> int:
    """
    Parses the command line, streams RTCM through the link and prints a summary.

    :return: Process exit code - 0 if every frame arrived intact (or no AIR port was given), 1 otherwise
    """
    parser = argparse.ArgumentParser(description="Emulate RTK corrections through a DroneBridge ESP-NOW link.")
    parser.add_argument("--gnd", required=True, help="serial port of the GND unit (e.g. /dev/cu.usbmodem1134101)")
    parser.add_argument("--gnd-baud", type=int, default=115200, help="baud of the GND port (ignored for USB/JTAG)")
    parser.add_argument("--air", help="serial port of the USB-UART adapter wired to the AIR unit's UART")
    parser.add_argument("--air-baud", type=int, default=115200, help="baud of the AIR unit's UART")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to run")
    parser.add_argument("--file", help="replay this raw RTCM3 file instead of synthetic data")
    parser.add_argument("--loop", action="store_true", help="repeat --file until --duration is over")
    parser.add_argument("--interval", type=float, default=0.1, help="seconds between frames when replaying --file")
    parser.add_argument("--base", help="serial port of a real RTK base receiver that outputs RTCM3")
    parser.add_argument("--base-baud", type=int, default=115200, help="baud of the --base receiver")
    parser.add_argument("--filler-bytes", type=int, default=600, help="synthetic: filler payload bytes per second")
    parser.add_argument("--pace-ms", type=float, default=0.0,
                        help="pause after every GPS_RTCM_DATA message in ms (0 = burst like a GCS)")
    parser.add_argument("--raw", action="store_true",
                        help="send raw RTCM3 to the GND like a GNSS base (GND must be in protocol 6, RTCM -> MAVLink)")
    parser.add_argument("--set-gnd-proto", type=int, choices=[1, 4, 5, 6],
                        help="only set the GND serial protocol via MAVLink and exit (switching away from 6 needs the "
                             "web interface, because the GND no longer parses MAVLink in RTCM mode)")
    parser.add_argument("--station-id", type=int, default=0, help="synthetic: RTCM reference station id")
    parser.add_argument("--ecef", type=lambda s: tuple(float(v) for v in s.split(",")), default=EXAMPLE_BASE_ECEF_M,
                        help="synthetic: base position as ECEF x,y,z in metres")
    args = parser.parse_args()
    if args.file and args.base:
        parser.error("use either --file or --base, not both")
    if not 6 <= args.filler_bytes <= 1023:
        parser.error("--filler-bytes must be 6..1023")

    gnd = mavutil.mavlink_connection(args.gnd, baud=args.gnd_baud, source_system=255, source_component=190)
    if args.set_gnd_proto is not None:
        set_gnd_protocol(gnd, args.set_gnd_proto)
        return 0
    sender = RtcmSender(gnd, args.pace_ms / 1000.0)
    sent = collections.Counter()
    stats = {"ok": 0, "unexpected": 0, "by_type": collections.Counter(), "lock": threading.Lock()}
    stop = threading.Event()
    receiver = None
    if args.air:
        air = mavutil.mavlink_connection(args.air, baud=args.air_baud)
        receiver = threading.Thread(target=receive_loop, args=(air, sent, stats, stop), daemon=True)
        receiver.start()

    source = args.file or args.base or "synthetic"
    form = "raw RTCM3, the GND wraps it as #233" if args.raw else "GPS_RTCM_DATA #233"
    print(f"Streaming RTCM ({source}, {form}) to {args.gnd} for up to {args.duration:g} s - Ctrl+C to stop early")
    frames_sent = 0
    bytes_sent = 0
    sent_by_type = collections.Counter()
    last_heartbeat = 0.0
    try:
        for frame in frame_source(args):
            now = time.monotonic()
            if not args.raw and now - last_heartbeat >= 1.0:
                gnd.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_heartbeat = now
            with stats["lock"]:
                sent[frame] += 1
            if args.raw:
                gnd.write(frame)
            else:
                sender.send_frame(frame)
            frames_sent += 1
            bytes_sent += len(frame)
            sent_by_type[rtcm3_message_type(frame)] += 1
    except KeyboardInterrupt:
        pass

    if args.raw:
        print(f"Sent {frames_sent} raw RTCM3 frames ({bytes_sent} bytes)")
    else:
        print(f"Sent {frames_sent} RTCM3 frames ({bytes_sent} bytes) in {sender.messages_sent} GPS_RTCM_DATA messages")
    print("  by type: " + ", ".join(f"{t}: {n}" for t, n in sorted(sent_by_type.items())))
    if receiver is None:
        print("No --air port given, nothing verified.")
        return 0

    time.sleep(2.0)  # let the last messages arrive
    stop.set()
    receiver.join()
    lost = frames_sent - stats["ok"]
    print(f"Received intact: {stats['ok']}/{frames_sent}   lost: {lost}   unexpected: {stats['unexpected']}")
    print("  by type: " + ", ".join(f"{t}: {n}" for t, n in sorted(stats["by_type"].items())))
    print(f"  re-assembly: {stats.get('discarded_fragments', 0)} fragments discarded, "
          f"{stats.get('skipped_bytes', 0)} stray bytes skipped")
    return 0 if lost == 0 and stats["unexpected"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
