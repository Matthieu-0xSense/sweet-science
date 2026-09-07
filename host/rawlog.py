#!/usr/bin/env python3
"""Container for raw 1 kHz captures (BLE characteristic 0004).

A capture is a flat binary file: a 24-byte header, then one record per BLE
packet. Records are the packet verbatim, prefixed with the host arrival time,
which is what lets a hand-typed label file be lined up with node timestamps
later -- the node counts microseconds since its own boot and knows nothing
about wall-clock time.

    header   magic "BOXERAW1", node, sample rate, batch size, start time
    record   f64 t_host | u32 t_us_base | u16 seq | u16 lost | batch samples
    sample   i16 hgx, hgy, hgz | u16 f0, f1

Two loss channels, deliberately distinct:

  * `lost` counts packets the node's queue refused -- those never got a seq,
    so without this field the gap would close up silently.
  * a jump in `seq` means the packet was built but never made it over the air.

Both matter when fitting thresholds: a sweep run over a capture with holes in
it fits the holes.

CLI:
    python rawlog.py stats  capture.bin
    python rawlog.py jsonl  capture.bin [-o out.jsonl]   # view in the panel
"""

import argparse
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

MAGIC = b"BOXERAW1"
HEADER_FMT = "<8scxHH2xd"
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 24
SAMPLE_FMT = "<3h2H"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)  # 10
PACKET_HEADER_FMT = "<IHH"
PACKET_HEADER_SIZE = struct.calcsize(PACKET_HEADER_FMT)  # 8

ADXL375_G_PER_LSB = 0.049


@dataclass
class Sample:
    t_us: int
    hgx: int
    hgy: int
    hgz: int
    f0: int
    f1: int

    @property
    def hg_g(self) -> float:
        return math.sqrt(self.hgx**2 + self.hgy**2 + self.hgz**2) * ADXL375_G_PER_LSB


@dataclass
class Packet:
    t_host: float
    t_us_base: int
    seq: int
    lost: int
    samples: list


class RawWriter:
    """Append-only capture writer. One file per node per session."""

    def __init__(self, path: Path, node: str, t_unix: float,
                 rate_hz: int = 1000, batch: int = 20):
        self.path = Path(path)
        self.batch = batch
        self.packet_size = PACKET_HEADER_SIZE + batch * SAMPLE_SIZE
        self.packets = 0
        self.bytes = 0
        self.f = open(self.path, "wb")
        self.f.write(struct.pack(HEADER_FMT, MAGIC, node.encode()[:1],
                                 rate_hz, batch, t_unix))

    def append(self, t_host: float, payload: bytes) -> bool:
        """Store one BLE notification. False if it is not the expected size."""
        if len(payload) != self.packet_size:
            return False
        self.f.write(struct.pack("<d", t_host))
        self.f.write(payload)
        self.packets += 1
        self.bytes += len(payload)
        return True

    def flush(self):
        self.f.flush()

    def close(self):
        self.f.close()


class RawReader:
    """Iterate a capture. Small enough to stream; captures run to tens of MB."""

    def __init__(self, path: Path):
        self.path = Path(path)
        with open(self.path, "rb") as f:
            head = f.read(HEADER_SIZE)
        if len(head) < HEADER_SIZE:
            raise ValueError(f"{self.path.name}: truncated header")
        magic, node, self.rate_hz, self.batch, self.t_unix = \
            struct.unpack(HEADER_FMT, head)
        if magic != MAGIC:
            raise ValueError(f"{self.path.name}: not a raw capture "
                             f"(magic {magic!r})")
        self.node = node.decode(errors="replace")
        self.sample_us = 1_000_000 // self.rate_hz
        self.packet_size = PACKET_HEADER_SIZE + self.batch * SAMPLE_SIZE
        self.record_size = 8 + self.packet_size

    def packets(self):
        with open(self.path, "rb") as f:
            f.seek(HEADER_SIZE)
            while True:
                rec = f.read(self.record_size)
                if len(rec) < self.record_size:
                    return          # trailing partial record: capture cut short
                (t_host,) = struct.unpack_from("<d", rec, 0)
                t_us_base, seq, lost = struct.unpack_from(PACKET_HEADER_FMT,
                                                          rec, 8)
                samples = []
                for i in range(self.batch):
                    off = 8 + PACKET_HEADER_SIZE + i * SAMPLE_SIZE
                    hgx, hgy, hgz, f0, f1 = struct.unpack_from(SAMPLE_FMT,
                                                               rec, off)
                    samples.append(Sample(
                        (t_us_base + i * self.sample_us) & 0xFFFFFFFF,
                        hgx, hgy, hgz, f0, f1))
                yield Packet(t_host, t_us_base, seq, lost, samples)

    def samples(self):
        for pkt in self.packets():
            yield from pkt.samples

    def clock_offset(self) -> float:
        """unix ≈ offset + t_us/1e6, from the median packet.

        The median, not the first packet: BLE delivery jitters by tens of
        milliseconds and the first packet after a subscribe is usually the
        worst one in the file.
        """
        deltas = sorted(p.t_host - p.t_us_base / 1e6 for p in self.packets())
        if not deltas:
            raise ValueError(f"{self.path.name}: no packets")
        return deltas[len(deltas) // 2]


def stats(path: Path) -> dict:
    r = RawReader(path)
    n_pkt = n_lost_queue = 0
    seq_gaps = seq_lost = 0
    prev_seq = None
    first_us = last_us = None
    hg_peak = 0.0
    f_max = 0

    for pkt in r.packets():
        n_pkt += 1
        n_lost_queue += pkt.lost
        if prev_seq is not None:
            step = (pkt.seq - prev_seq) & 0xFFFF
            if step != 1:
                seq_gaps += 1
                seq_lost += step - 1
        prev_seq = pkt.seq
        if first_us is None:
            first_us = pkt.t_us_base
        last_us = pkt.t_us_base
        for s in pkt.samples:
            hg_peak = max(hg_peak, s.hg_g)
            f_max = max(f_max, s.f0, s.f1)

    n_samples = n_pkt * r.batch
    span_s = ((last_us - first_us) & 0xFFFFFFFF) / 1e6 if n_pkt > 1 else 0.0
    expected = n_pkt + n_lost_queue + seq_lost
    return {
        "file": path.name,
        "node": r.node,
        "rate_hz": r.rate_hz,
        "packets": n_pkt,
        "samples": n_samples,
        "duration_s": round(span_s, 2),
        "queue_dropped_packets": n_lost_queue,
        "radio_dropped_packets": seq_lost,
        "seq_gaps": seq_gaps,
        "loss_pct": round(100 * (expected - n_pkt) / expected, 3) if expected else 0.0,
        "peak_g": round(hg_peak, 1),
        "max_fsr_counts": f_max,
    }


def to_jsonl(path: Path, out: Path):
    """Convert to the panel's record format so a capture can be eyeballed.

    Decimated to 100 Hz on the way out: the panel plots a few thousand points
    happily and a million points not at all. This is for looking, not fitting.
    """
    r = RawReader(path)
    offset = r.clock_offset()
    step = max(1, r.rate_hz // 100)
    n = 0
    with open(out, "w", encoding="utf-8") as f:
        for i, s in enumerate(r.samples()):
            if i % step:
                continue
            f.write(json.dumps({
                "t": round(offset + s.t_us / 1e6, 6),
                "node": r.node, "type": "imu", "t_us": s.t_us,
                "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0,
                "hg": round(s.hg_g, 2), "f0": s.f0, "f1": s.f1,
            }, separators=(",", ":")) + "\n")
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("stats", help="summarise a capture")
    p.add_argument("capture", type=Path)

    p = sub.add_parser("jsonl", help="convert to panel JSONL (100 Hz)")
    p.add_argument("capture", type=Path)
    p.add_argument("-o", "--out", type=Path)

    args = ap.parse_args()

    if args.cmd == "stats":
        for k, v in stats(args.capture).items():
            print(f"{k:24} {v}")
    else:
        out = args.out or args.capture.with_suffix(".jsonl")
        n = to_jsonl(args.capture, out)
        print(f"wrote {n} records to {out}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError) as exc:
        sys.exit(f"error: {exc}")
