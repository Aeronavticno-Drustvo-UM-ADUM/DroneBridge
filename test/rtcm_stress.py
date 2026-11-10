#!/usr/bin/env python3
"""
Hardware stress test for the DroneBridge "RTCM in -> MAVLink #233" ground mode.

    PC --raw RTCM--> GND (serial protocol 6) ))) ESP-NOW ))) AIR --#233 on UART--> USB-UART adapter --> PC

Every RTCM frame sent is unique, so each frame that comes out at the AIR side is matched to the exact frame that went
in. Anything that comes out but was never sent intact (a corrupted frame, a frame formed by noise) is a failure.
While the scenarios run, the AIR unit's USB log is watched for reboots and warnings, and the GND serial port for
disconnects (= GND reboot).

Scenarios: baseline, trickle, garbage, flood, crafted flood, rate ramp, bursts, GND reset, AIR reset, soak.

Requirements (in any Python venv, not the ESP-IDF one):
    pip install pymavlink pyserial
The GND must already be in serial protocol 6 (see rtk_emulator.py --set-gnd-proto 6).

Example:
    python3 test/rtcm_stress.py --gnd /dev/cu.usbmodem1134101 --air /dev/cu.usbserial-11320 \
        --air-log /dev/cu.usbmodem11201 --soak 300
"""

import argparse
import importlib.util
import json
import os
import random
import re
import statistics
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")

try:
    import serial
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink/pyserial missing. Install with: pip install pymavlink pyserial")

_here = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("rtk_emulator", os.path.join(_here, "rtk_emulator.py"))
rtk = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rtk)

EPOCH = [(1005, 19), (1074, 150), (1084, 130), (1094, 150), (1124, 170), (1230, 6)]  # typical F9P base epoch


class FrameFactory:
    """Creates unique RTCM3 frames with a real message number and a running counter in the payload."""

    def __init__(self):
        """Starts the counter at 0."""
        self.counter = 0

    def make(self, message_type: int, payload_length: int) -> bytes:
        """
        Builds one unique, valid RTCM3 frame.

        :param message_type: RTCM message number (1001-1299 or 4001-4095)
        :param payload_length: Payload bytes, at least 6
        :return: Complete frame
        """
        payload_length = max(6, min(1023, payload_length))
        self.counter += 1
        head = bytes([(message_type >> 4) & 0xFF, (message_type << 4) & 0xF0]) + self.counter.to_bytes(4, "big")
        body = bytes((self.counter * 31 + i) & 0xFF for i in range(payload_length - len(head)))
        return rtk.build_rtcm3_frame(head + body)

    def epoch(self) -> list:
        """
        :return: One typical base epoch (1005, four MSM4 messages, 1230) as unique frames
        """
        return [self.make(t, n) for t, n in EPOCH]


class Ledger:
    """Thread safe record of every frame sent and received."""

    def __init__(self):
        """Creates an empty ledger."""
        self.lock = threading.Lock()
        self.sent = {}          # frame -> (scenario, send time)
        self.received = {}      # frame -> receive time
        self.unexpected = []    # (time, frame) of frames that were never sent intact
        self.duplicates = 0

    def add_sent(self, scenario: str, frames: list) -> None:
        """
        Records frames as sent now.

        :param scenario: Scenario name
        :param frames: Frames just written
        """
        now = time.monotonic()
        with self.lock:
            for f in frames:
                self.sent[f] = (scenario, now)

    def add_received(self, frame: bytes) -> None:
        """
        Records a frame that came out at the AIR side.

        :param frame: Complete RTCM3 frame (valid CRC)
        """
        now = time.monotonic()
        with self.lock:
            if frame not in self.sent:
                self.unexpected.append((now, frame))
            elif frame in self.received:
                self.duplicates += 1
            else:
                self.received[frame] = now

    def summary(self, scenario: str) -> dict:
        """
        :param scenario: Scenario name
        :return: sent, delivered, lost and latency statistics of that scenario
        """
        with self.lock:
            mine = [(f, t) for f, (s, t) in self.sent.items() if s == scenario]
            lat = sorted((self.received[f] - t) * 1000.0 for f, t in mine if f in self.received)
        sent = len(mine)
        out = {"scenario": scenario, "sent": sent, "delivered": len(lat), "lost": sent - len(lat)}
        if lat:
            out.update(p50=round(lat[len(lat) // 2], 1), p95=round(lat[min(len(lat) - 1, int(len(lat) * 0.95))], 1),
                       max=round(lat[-1], 1))
        return out


class AirReceiver(threading.Thread):
    """Reads GPS_RTCM_DATA from the AIR unit's UART, rebuilds RTCM3 frames and records them in the ledger."""

    def __init__(self, port: str, baud: int, ledger: Ledger):
        """
        :param port: Serial port of the USB-UART adapter on the AIR unit's UART
        :param baud: AIR UART baud
        :param ledger: Ledger to record frames in
        """
        super().__init__(daemon=True)
        self.conn = mavutil.mavlink_connection(port, baud=baud)
        self.ledger = ledger
        self.stop = threading.Event()
        self.reassembler = rtk.RtcmReassembler()
        self.parser = rtk.Rtcm3Parser()
        self.messages = 0
        self.max_backlog = 0    # largest number of unread bytes seen in the PC's serial buffer

    def run(self) -> None:
        """Receive loop until stopped."""
        while not self.stop.is_set():
            try:
                self.max_backlog = max(self.max_backlog, self.conn.port.in_waiting)
            except Exception:
                pass
            msg = self.conn.recv_match(type="GPS_RTCM_DATA", blocking=True, timeout=0.2)
            if msg is None:
                continue
            self.messages += 1
            for frame in self.parser.feed(self.reassembler.add(msg.flags, bytes(msg.data[:msg.len]))):
                self.ledger.add_received(frame)


class AirLogWatcher(threading.Thread):
    """Watches the AIR unit's USB log for reboots and warnings. Also used to reset the AIR unit."""

    PATTERNS = {"reboot": re.compile(r"ESP-ROM|rst:0x|app_main finished"),
                "queue_full": re.compile(r"db_uart_write_queue failed|send queue.*fail", re.I),
                "seq_gap": re.compile(r"sequence gap", re.I),
                "auth_fail": re.compile(r"AUTH_FAILED"),
                "error": re.compile(r"^E \(")}

    def __init__(self, port: str):
        """
        :param port: AIR unit's USB/JTAG port (log output)
        """
        super().__init__(daemon=True)
        self.port_name = port
        self.port = None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.counts = {k: 0 for k in self.PATTERNS}
        self.lines = []

    def _open(self) -> None:
        """Opens the log port without toggling the reset lines."""
        p = serial.Serial()
        p.port, p.baudrate, p.timeout = self.port_name, 115200, 0.2
        p.dtr, p.rts = False, False
        p.open()
        self.port = p

    def run(self) -> None:
        """Read loop, reopens the port after a USB re-enumeration."""
        buf = b""
        while not self.stop.is_set():
            try:
                if self.port is None:
                    self._open()
                buf += self.port.read(4096)
            except (serial.SerialException, OSError):
                self.port = None
                time.sleep(0.2)
                continue
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").strip()
                with self.lock:
                    for key, pat in self.PATTERNS.items():
                        if pat.search(line):
                            self.counts[key] += 1
                            self.lines.append(line[:160])

    def snapshot(self) -> dict:
        """
        :return: Copy of the counters
        """
        with self.lock:
            return dict(self.counts)

    def reset_air(self) -> None:
        """Resets the AIR unit through the USB/JTAG reset lines (RTS pulse with DTR low)."""
        with self.lock:
            p = self.port
        if p is None:
            return
        p.dtr = False
        p.rts = True
        time.sleep(0.1)
        p.rts = False


class GndWriter:
    """Writes raw bytes to the GND serial port and survives the port disappearing (GND reboot)."""

    def __init__(self, port: str, baud: int):
        """
        :param port: GND serial port
        :param baud: Baud (ignored for USB/JTAG)
        """
        self.port_name, self.baud = port, baud
        self.port = None
        self.disconnects = 0
        self.max_out_waiting = 0    # largest number of bytes the PC still had queued for the GND after a write
        self._open()

    def _open(self) -> bool:
        """
        Opens the port without toggling the reset lines.

        :return: True if open
        """
        try:
            p = serial.Serial()
            p.port, p.baudrate, p.write_timeout = self.port_name, self.baud, 2
            p.dtr, p.rts = False, False
            p.open()
            self.port = p
            return True
        except (serial.SerialException, OSError):
            self.port = None
            return False

    def write(self, data: bytes) -> bool:
        """
        Writes data, reopening the port once if it vanished.

        :param data: Bytes to write
        :return: True if written
        """
        for _ in range(2):
            try:
                if self.port is None and not self._open():
                    return False
                self.port.write(data)
                try:
                    self.max_out_waiting = max(self.max_out_waiting, self.port.out_waiting)
                except (serial.SerialException, OSError):
                    pass
                return True
            except (serial.SerialException, OSError):
                self.disconnects += 1
                try:
                    self.port.close()
                except Exception:
                    pass
                self.port = None
                time.sleep(0.05)
        return False

    def reset_gnd(self) -> None:
        """Resets the GND through the USB/JTAG reset lines."""
        if self.port is None:
            return
        try:
            self.port.dtr = False
            self.port.rts = True
            time.sleep(0.1)
            self.port.rts = False
        except (serial.SerialException, OSError):
            pass


def junk(rng: random.Random, factory: FrameFactory) -> bytes:
    """
    Builds a piece of junk the way a broken or mixed serial line could produce it.

    :param rng: Random generator
    :param factory: Used for corrupted and truncated copies of real looking frames (never sent intact)
    :return: Junk bytes
    """
    kind = rng.randrange(7)
    if kind == 0:
        return bytes(rng.getrandbits(8) for _ in range(rng.randrange(1, 120)))
    if kind == 1:
        return b"$GNGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76\r\n"
    if kind == 2:
        return b"\xb5\x62\x01\x07\x5c\x00" + bytes(rng.getrandbits(8) for _ in range(94))
    if kind == 3:   # false header with a plausible type claiming a long frame
        length = rng.randrange(20, 1024)
        return bytes([0xD3, (length >> 8) & 3, length & 0xFF, 0x3E, 0xD0]) + bytes(rng.getrandbits(8) for _ in range(rng.randrange(0, 40)))
    if kind == 4:   # corrupted frame: one bit flipped, must never come out
        f = bytearray(factory.make(1077, rng.randrange(20, 400)))
        f[rng.randrange(len(f))] ^= 1 << rng.randrange(8)
        return bytes(f)
    if kind == 5:   # truncated frame (sender died mid-frame)
        f = factory.make(1087, rng.randrange(40, 400))
        return f[:rng.randrange(1, len(f) - 1)]
    return b"\xd3" * rng.randrange(1, 6)


def settle(seconds: float = 2.5) -> None:
    """
    Waits for in-flight frames to arrive.

    :param seconds: Wait time
    """
    time.sleep(seconds)


def sc_baseline(w, led, fac, rng, duration=30):
    """Typical base: one epoch (6 messages) per second, written back to back."""
    for _ in range(duration):
        e = fac.epoch()
        led.add_sent("baseline", e)
        w.write(b"".join(e))
        time.sleep(1.0)


def sc_trickle(w, led, fac, rng, frames=40):
    """Frames written in 1-25 byte pieces with random pauses of 0-45 ms, often longer than the 20 ms timeout."""
    for _ in range(frames):
        f = fac.make(rng.choice([1005, 1074, 1124, 4072]), rng.randrange(10, 300))
        led.add_sent("trickle", [f])
        pos = 0
        while pos < len(f):
            n = rng.randrange(1, 26)
            w.write(f[pos:pos + n])
            pos += n
            time.sleep(rng.uniform(0, 0.045))


def sc_garbage(w, led, fac, rng, duration=40):
    """Every epoch interleaved with junk: noise, NMEA, UBX, false headers, corrupted and truncated frames."""
    for _ in range(duration):
        out = b""
        sent = []
        for f in fac.epoch():
            out += b"".join(junk(rng, fac) for _ in range(rng.randrange(0, 3))) + f
            sent.append(f)
        led.add_sent("garbage", sent)
        w.write(out)
        time.sleep(1.0)


def sc_flood(w, led, fac, rng):
    """64 KB of random noise at full speed, then normal epochs. All epochs after the flood must arrive."""
    for _ in range(16):
        w.write(bytes(rng.getrandbits(8) for _ in range(4096)))
    time.sleep(0.5)
    for _ in range(15):
        e = fac.epoch()
        led.add_sent("flood_recovery", e)
        w.write(b"".join(e))
        time.sleep(1.0)


def sc_crafted(w, led, fac, rng, duration=15):
    """Worst case CPU input (headers with a valid type claiming 1023 bytes, every 5 bytes) mixed with real epochs."""
    pattern = bytes([0xD3, 0x03, 0xFF, 0x3E, 0xD0]) * 400   # 2000 bytes
    for _ in range(duration):
        e = fac.epoch()
        led.add_sent("crafted_flood", e)
        w.write(pattern + b"".join(e) + pattern[:1000])
        time.sleep(1.0)


def sc_rate_ramp(w, led, fac, rng, rates=(1, 2, 4, 6, 8, 10, 12), seconds=8):
    """Evenly paced frames of ~186 bytes at rising data rates (kB/s) to find where the link saturates."""
    for kbs in rates:
        name = f"rate_{kbs}kBps"
        frame_len = 186
        interval = frame_len / (kbs * 1000.0)
        end = time.monotonic() + seconds
        nxt = time.monotonic()
        while time.monotonic() < end:
            f = fac.make(1074, frame_len - 6)
            led.add_sent(name, [f])
            w.write(f)
            nxt += interval
            time.sleep(max(0.0, nxt - time.monotonic()))
        settle(3)


def sc_bursts(w, led, fac, rng, sizes=(2048, 4096, 8192)):
    """Single writes of 2, 4 and 8 KB of frames at USB speed (a real UART base cannot burst like this)."""
    for size in sizes:
        name = f"burst_{size // 1024}KB"
        frames = []
        total = 0
        while total < size:
            f = fac.make(1084, 180)
            frames.append(f)
            total += len(f)
        led.add_sent(name, frames)
        w.write(b"".join(frames))
        settle(4)


def sc_gnd_reset(w, led, fac, rng, duration=25, reset_at=6):
    """Epochs every 0.5 s; the GND is reset mid-stream. Frames written while it reboots are lost, the rest must arrive."""
    for i in range(duration * 2):
        if i == reset_at * 2:
            w.reset_gnd()
        e = fac.epoch()
        led.add_sent("gnd_reset", e)
        w.write(b"".join(e))
        time.sleep(0.5)


def sc_air_reset(w, led, fac, rng, watcher, duration=25, reset_at=6):
    """Epochs every 0.5 s; the AIR unit is reset mid-stream."""
    for i in range(duration * 2):
        if i == reset_at * 2 and watcher is not None:
            watcher.reset_air()
        e = fac.epoch()
        led.add_sent("air_reset", e)
        w.write(b"".join(e))
        time.sleep(0.5)


def sc_soak(w, led, fac, rng, duration):
    """Long run: one epoch per second with light junk."""
    for _ in range(duration):
        out = b""
        sent = []
        for f in fac.epoch():
            if rng.random() < 0.1:
                out += junk(rng, fac)
            out += f
            sent.append(f)
        led.add_sent("soak", sent)
        w.write(out)
        time.sleep(1.0)


def main() -> int:
    """
    Runs the selected scenarios and prints a result table.

    :return: 0 if no frame was ever delivered that was not sent intact, 1 otherwise
    """
    ap = argparse.ArgumentParser(description="Stress test the DroneBridge RTCM -> MAVLink ground mode.")
    ap.add_argument("--gnd", required=True)
    ap.add_argument("--gnd-baud", type=int, default=115200)
    ap.add_argument("--air", required=True)
    ap.add_argument("--air-baud", type=int, default=115200)
    ap.add_argument("--air-log", help="AIR unit USB port: watch its log and allow the AIR reset scenario")
    ap.add_argument("--only", help="comma separated scenario names")
    ap.add_argument("--soak", type=int, default=120, help="soak duration in seconds (0 = skip)")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--json", help="write results to this file")
    args = ap.parse_args()

    rng = random.Random(args.seed)
    led, fac = Ledger(), FrameFactory()
    rx = AirReceiver(args.air, args.air_baud, led)
    rx.start()
    watcher = None
    if args.air_log:
        watcher = AirLogWatcher(args.air_log)
        watcher.start()
        time.sleep(3.0)     # opening the USB/JTAG port can reset the AIR unit once; let it boot first
        if watcher:
            with watcher.lock:
                watcher.counts = {k: 0 for k in watcher.counts}
    w = GndWriter(args.gnd, args.gnd_baud)
    time.sleep(1.0)

    plan = [("baseline", lambda: sc_baseline(w, led, fac, rng)),
            ("trickle", lambda: sc_trickle(w, led, fac, rng)),
            ("garbage", lambda: sc_garbage(w, led, fac, rng)),
            ("flood", lambda: sc_flood(w, led, fac, rng)),
            ("crafted", lambda: sc_crafted(w, led, fac, rng)),
            ("rate", lambda: sc_rate_ramp(w, led, fac, rng)),
            ("bursts", lambda: sc_bursts(w, led, fac, rng)),
            ("gnd_reset", lambda: sc_gnd_reset(w, led, fac, rng)),
            ("air_reset", lambda: sc_air_reset(w, led, fac, rng, watcher)),
            ("soak", lambda: sc_soak(w, led, fac, rng, args.soak))]
    only = set(args.only.split(",")) if args.only else None
    results = []
    for name, fn in plan:
        if only and name not in only:
            continue
        if name == "soak" and args.soak <= 0:
            continue
        if name == "air_reset" and watcher is None:
            continue
        rx.max_backlog = 0
        w.max_out_waiting = 0
        before_log = watcher.snapshot() if watcher else {}
        before_unexp = len(led.unexpected)
        before_disc = w.disconnects
        t0 = time.monotonic()
        print(f"--- {name} ...", flush=True)
        fn()
        settle()
        names = sorted({s for (s, _) in led.sent.values()} - {r["scenario"] for r in results})
        for s in names:
            r = led.summary(s)
            r["unexpected"] = len(led.unexpected) - before_unexp
            r["gnd_disconnects"] = w.disconnects - before_disc
            if watcher:
                after = watcher.snapshot()
                r["air_log"] = {k: after[k] - before_log.get(k, 0) for k in after if after[k] - before_log.get(k, 0)}
            r["seconds"] = round(time.monotonic() - t0, 1)
            r["pc_rx_backlog_max"] = rx.max_backlog
            r["pc_tx_queued_max"] = w.max_out_waiting
            results.append(r)
            print("    " + json.dumps(r), flush=True)

    rx.stop.set()
    if watcher:
        watcher.stop.set()
    print("\n%-16s %6s %9s %5s %6s %8s %8s %8s  %s" % ("scenario", "sent", "delivered", "lost", "unexp", "p50 ms", "p95 ms",
                                                    "max ms", "notes"))
    for r in results:
        notes = []
        if r.get("gnd_disconnects"):
            notes.append(f"GND port dropped {r['gnd_disconnects']}x")
        if r.get("air_log"):
            notes.append("AIR log " + ", ".join(f"{k}={v}" for k, v in r["air_log"].items()))
        print("%-16s %6d %9d %5d %6d %8s %8s %8s  %s" % (r["scenario"], r["sent"], r["delivered"], r["lost"],
                                                        r["unexpected"], r.get("p50", "-"), r.get("p95", "-"),
                                                        r.get("max", "-"), "; ".join(notes)))
    total_unexpected = len(led.unexpected)
    print(f"\nFrames delivered that were never sent intact: {total_unexpected}   duplicates: {led.duplicates}   "
          f"#233 messages received: {rx.messages}")
    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"results": results, "unexpected": total_unexpected, "duplicates": led.duplicates,
                       "air_log_lines": watcher.lines[-200:] if watcher else []}, fh, indent=1)
    return 0 if total_unexpected == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
