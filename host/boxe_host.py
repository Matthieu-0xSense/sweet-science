#!/usr/bin/env python3
"""Boxe_AI host hub.

Bridges the two wrist nodes (BLE) -- or a built-in simulator -- to:
  - a JSONL session log on disk (datalogging)
  - a WebSocket broadcast for the web debug panel (web/index.html)

Usage:
  python boxe_host.py --sim              # no hardware needed
  python boxe_host.py --ble              # scan + connect BOXE-L / BOXE-R
  python boxe_host.py --sim --no-log     # panel only, no file
  python boxe_host.py --ble --raw        # + raw 1 kHz capture for sweep.py

A reconnect starts a new raw capture file: the node's microsecond clock is the
only timebase in the stream and it does not survive the gap, so splicing two
sides of a dropout into one file would fabricate continuity that was not
there.

Requires: pip install -r requirements.txt (websockets; bleak for --ble)
"""

import argparse
import asyncio
import json
import math
import random
import struct
import sys
import time
from pathlib import Path

import fsr_calib

WS_HOST, WS_PORT = "127.0.0.1", 8765
LOG_DIR = Path(__file__).parent.parent / "logs"

SERVICE_UUID = "6f8e0001-b5a3-4f39-b0c4-2ae94a2c5e01"
CHAR_IMU = "6f8e0002-b5a3-4f39-b0c4-2ae94a2c5e01"
CHAR_EVENT = "6f8e0003-b5a3-4f39-b0c4-2ae94a2c5e01"
CHAR_RAW = "6f8e0004-b5a3-4f39-b0c4-2ae94a2c5e01"
CHAR_STATUS = "6f8e0005-b5a3-4f39-b0c4-2ae94a2c5e01"

RAW_BATCH = 20  # must match RAW_BATCH in firmware/src/boxe.h

ADXL375_G_PER_LSB = 0.049  # 49 mg/LSB

# Filled from host/fsr_calib.json when it exists; force_n stays None otherwise.
# See calibrate_fsr.py.
FSR_CAL = fsr_calib.load()


class Hub:
    """Fan-out: every record goes to the log file and all panel clients."""

    def __init__(self, log_enabled: bool):
        self.clients: set = set()
        self.log_file = None
        if log_enabled:
            LOG_DIR.mkdir(exist_ok=True)
            name = time.strftime("session_%Y%m%d_%H%M%S.jsonl")
            self.log_file = open(LOG_DIR / name, "w", encoding="utf-8")
            print(f"[hub] logging to logs/{name}")

    def publish(self, record: dict):
        record.setdefault("t", time.time())
        line = json.dumps(record, separators=(",", ":"))
        if self.log_file:
            self.log_file.write(line + "\n")
        dead = set()
        for ws in self.clients:
            try:
                ws._send_queue.put_nowait(line)
            except Exception:
                dead.add(ws)
        self.clients -= dead

    def log(self, level: str, msg: str):
        print(f"[{level}] {msg}")
        self.publish({"node": "-", "type": "log", "level": level, "msg": msg})

    def flush(self):
        if self.log_file:
            self.log_file.flush()


async def ws_server(hub: Hub):
    import websockets

    async def handler(ws):
        ws._send_queue = asyncio.Queue(maxsize=2000)
        hub.clients.add(ws)
        hub.log("info", f"panel connected ({len(hub.clients)} client(s))")
        try:
            while True:
                line = await ws._send_queue.get()
                await ws.send(line)
        except Exception:
            pass
        finally:
            hub.clients.discard(ws)
            hub.log("info", "panel disconnected")

    async with websockets.serve(handler, WS_HOST, WS_PORT):
        print(f"[hub] panel feed on ws://{WS_HOST}:{WS_PORT}")
        await asyncio.Future()


# ---------------------------------------------------------------- simulator

async def sim_node(hub: Hub, node: str):
    """Fake node: 50 Hz IMU stream, a punch every 2-5 s, 1 Hz status."""
    t_us = 0
    seq = 0
    next_punch = time.time() + random.uniform(1.5, 4.0)
    punch_until = 0.0
    punch_peak = 0.0
    last_status = 0.0
    events = 0

    while True:
        now = time.time()
        t_us += 20_000

        # baseline motion noise
        hg = abs(random.gauss(0, 0.3))
        gx = random.gauss(0, 20)

        # punch envelope: 150 ms swing, sharp hg spike at contact
        if now >= next_punch and now > punch_until:
            punch_until = now + 0.15
            punch_peak = random.uniform(15, 90)
            next_punch = now + random.uniform(2.0, 5.0)
        in_punch = now < punch_until
        if in_punch:
            phase = 1 - (punch_until - now) / 0.15
            hg = punch_peak * math.exp(-((phase - 0.85) ** 2) / 0.01)
            gx = 400 * math.sin(phase * math.pi)

        hub.publish({
            "node": node, "type": "imu", "t_us": t_us,
            "ax": round(random.gauss(0, 80) + (900 if in_punch else 0)),
            "ay": round(random.gauss(980, 30)),
            "az": round(random.gauss(0, 60)),
            "gx": round(gx), "gy": round(random.gauss(0, 15)),
            "gz": round(random.gauss(0, 10)),
            "hg": round(hg, 2),
        })

        # emit the event at the end of the punch window
        if punch_until and now >= punch_until and punch_peak:
            seq += 1
            events += 1
            contact = random.random() > 0.25  # some punches miss
            f0 = random.randint(800, 4095) if contact else 0
            hub.publish({
                "node": node, "type": "event", "t_us": t_us,
                "contact": contact, "sat": punch_peak > 85,
                "peak_g": round(punch_peak, 1),
                "f0": f0, "f1": int(f0 * random.uniform(0.3, 0.9)),
                "width_ms": round(random.uniform(8, 25), 1) if contact else 0,
                "impulse": int(f0 * random.uniform(8, 25)) if contact else 0,
                "seq": seq,
                "exec_ms": round(random.gauss(160, 30), 1),
                "retract_ms": round(random.gauss(220, 50), 1),
                "force_n": None,
            })
            punch_peak = 0.0

        if now - last_status >= 1.0:
            last_status = now
            hub.publish({
                "node": node, "type": "status",
                "uptime_s": int(now) % 100000,
                "batt_mv": random.randint(3900, 4050),
                "loop_hz": random.randint(998, 1002),
                "dropped": 0, "events": events,
                "rssi": random.randint(-70, -45),
            })
            hub.flush()

        await asyncio.sleep(0.02)


# ---------------------------------------------------------------- BLE source

def parse_imu(data: bytes, node: str):
    (t_us,) = struct.unpack_from("<I", data, 0)
    out = []
    for i in range(5):
        off = 4 + i * 22
        ax, ay, az, gx, gy, gz, hx, hy, hz, f0, f1 =             struct.unpack_from("<9h2H", data, off)
        hg = math.sqrt(hx * hx + hy * hy + hz * hz) * ADXL375_G_PER_LSB
        out.append({
            "node": node, "type": "imu", "t_us": t_us + i * 10_000,
            "ax": ax, "ay": ay, "az": az,
            "gx": gx / 10, "gy": gy / 10, "gz": gz / 10,
            "hg": round(hg, 2), "f0": f0, "f1": f1,
        })
    return out


def force_n(f0: int, f1: int):
    """Peak force across both channels, or None while uncalibrated.

    The two FSRs sit at different spots in the glove, so a punch loads them
    unequally; the peak is the meaningful single number, not the sum.
    """
    vals = [v for v in (FSR_CAL.newtons(f0), FSR_CAL.newtons(f1)) if v is not None]
    return round(max(vals), 1) if vals else None


def parse_event(data: bytes, node: str):
    t_us, flags, _, peak_hg, f0, f1, width, impulse, seq, t_start_us, retract = \
        struct.unpack("<IBBHHHHIIIH", data)
    return {
        "node": node, "type": "event", "t_us": t_us,
        "contact": bool(flags & 1), "sat": bool(flags & 2),
        "peak_g": round(peak_hg * ADXL375_G_PER_LSB, 1),
        "f0": f0, "f1": f1, "width_ms": width / 10,
        "impulse": impulse, "seq": seq,
        "exec_ms": round((t_us - t_start_us) / 1000, 1),
        "retract_ms": retract / 10,
        "force_n": force_n(f0, f1),
    }


def parse_status(data: bytes, node: str):
    uptime, batt, loop_hz, dropped, events = struct.unpack("<IHHHH", data)
    return {
        "node": node, "type": "status", "uptime_s": uptime,
        "batt_mv": batt, "loop_hz": loop_hz,
        "dropped": dropped, "events": events, "rssi": None,
    }


class RawCapture:
    """Writes one node's raw 1 kHz stream to logs/raw_<node>_<time>.bin.

    Subscribing to the characteristic is what starts the node capturing, so
    the file is created up front and the node is told last: the first packet
    arrives within a connection interval of the subscribe.
    """

    def __init__(self, hub: Hub, node: str):
        import rawlog

        LOG_DIR.mkdir(exist_ok=True)
        name = time.strftime(f"raw_{node}_%Y%m%d_%H%M%S.bin")
        self.hub = hub
        self.node = node
        self.writer = rawlog.RawWriter(LOG_DIR / name, node, time.time(),
                                       batch=RAW_BATCH)
        self.short = 0
        self.gaps = 0
        self.queue_lost = 0
        self.prev_seq = None
        self.last_report = time.time()
        hub.log("info", f"raw capture -> logs/{name}")

    def on_packet(self, data: bytes):
        now = time.time()
        if not self.writer.append(now, data):
            self.short += 1
            if self.short == 1:
                self.hub.log("warn", f"{self.node}: raw packet is "
                                     f"{len(data)} B, expected "
                                     f"{self.writer.packet_size} — RAW_BATCH "
                                     f"mismatch between host and firmware?")
            return

        seq, lost = struct.unpack_from("<HH", data, 4)
        self.queue_lost += lost
        if self.prev_seq is not None and (seq - self.prev_seq) & 0xFFFF != 1:
            self.gaps += 1
        self.prev_seq = seq

        if now - self.last_report >= 5.0:
            self.last_report = now
            self.writer.flush()
            kb = self.writer.bytes / 1024
            self.hub.log("info",
                         f"{self.node}: raw {self.writer.packets} packets, "
                         f"{kb:.0f} kB, {self.queue_lost} queue-dropped, "
                         f"{self.gaps} radio gaps")

    def close(self):
        self.writer.close()
        self.hub.log("info", f"{self.node}: raw capture closed, "
                             f"{self.writer.packets} packets, "
                             f"{self.queue_lost + self.gaps} lost")


async def ble_node(hub: Hub, name: str, node: str, raw: bool = False):
    """Connect one node, resubscribe forever."""
    from bleak import BleakClient, BleakScanner

    while True:
        hub.log("info", f"scanning for {name}...")
        dev = await BleakScanner.find_device_by_name(name, timeout=10.0)
        if dev is None:
            await asyncio.sleep(3)
            continue
        capture = None
        try:
            async with BleakClient(dev) as client:
                hub.log("info", f"{name} connected")

                def on_imu(_, data):
                    for rec in parse_imu(bytes(data), node):
                        hub.publish(rec)

                def on_event(_, data):
                    hub.publish(parse_event(bytes(data), node))

                def on_status(_, data):
                    hub.publish(parse_status(bytes(data), node))
                    hub.flush()

                await client.start_notify(CHAR_IMU, on_imu)
                await client.start_notify(CHAR_EVENT, on_event)
                await client.start_notify(CHAR_STATUS, on_status)

                if raw:
                    capture = RawCapture(hub, node)

                    def on_raw(_, data):
                        capture.on_packet(bytes(data))

                    # subscribing is the enable — the node starts sampling
                    # into its queue the moment this returns
                    await client.start_notify(CHAR_RAW, on_raw)

                while client.is_connected:
                    await asyncio.sleep(1)
        except Exception as e:
            hub.log("warn", f"{name}: {e}")
        finally:
            if capture is not None:
                capture.close()
        hub.log("warn", f"{name} disconnected, retrying")
        await asyncio.sleep(2)


# ---------------------------------------------------------------- main

async def main():
    ap = argparse.ArgumentParser(description=__doc__)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sim", action="store_true", help="simulated nodes")
    src.add_argument("--ble", action="store_true", help="real BLE nodes")
    ap.add_argument("--no-log", action="store_true", help="disable JSONL log")
    ap.add_argument("--raw", action="store_true",
                    help="also capture the raw 1 kHz stream to "
                         "logs/raw_<node>_*.bin (for sweep.py). BLE only; "
                         "adds ~10 kB/s per node")
    args = ap.parse_args()

    if args.raw and args.sim:
        ap.error("--raw needs real nodes: the simulator has no 1 kHz stream")

    hub = Hub(log_enabled=not args.no_log)
    tasks = [ws_server(hub)]
    if args.sim:
        tasks += [sim_node(hub, "L"), sim_node(hub, "R")]
    else:
        tasks += [ble_node(hub, "BOXE-L", "L", args.raw),
                  ble_node(hub, "BOXE-R", "R", args.raw)]
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
