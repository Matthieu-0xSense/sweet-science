# Boxe_AI — Protocol

Single source of truth for firmware ↔ host ↔ panel data formats.

## BLE GATT (node → host)

Device names: `BOXE-L`, `BOXE-R`.

Service UUID: `6f8e0001-b5a3-4f39-b0c4-2ae94a2c5e01`

| Characteristic | UUID (`6f8e....-b5a3-4f39-b0c4-2ae94a2c5e01`) | Props | Payload |
|---|---|---|---|
| IMU stream | `0002` | notify | `imu_packet` (below), 20 Hz, 5 samples/packet (decimated 100 Hz view of the 1 kHz loop) |
| Punch event | `0003` | notify | `event_packet`, one per detected punch |
| Raw capture | `0004` | notify | `raw_packet`, undecimated 1 kHz, 50 Hz packets. **Subscribing enables it**; unsubscribing or disconnecting stops it |
| Status | `0005` | read+notify | `status_packet`, 1 Hz |

All integers little-endian.

### imu_packet (5 samples × 22 B + 4 B header = 114 B)
```
u32  t_us_base            timestamp of first sample (node clock, µs)
5 × {
  i16 ax, ay, az          LSM6DS33/DS3TR-C, mg
  i16 gx, gy, gz          LSM6DS33/DS3TR-C, dps x10 (+/-2000 dps fits int16)
  i16 hg_x, hg_y, hg_z    ADXL375, raw LSB (49 mg/LSB)
  u16 f0, f1              FSR ADC counts (0..4095), peak-held
}
```
`f0`/`f1` are the **maximum** over the ten 1 kHz ticks the stream sample covers,
not a snapshot: a contact peak is a few ms wide and a 100 Hz snapshot walks past
it. `hg_*` remains a snapshot — the true impact peak is in `event_packet.peak_hg`,
computed at the full 1 kHz.

### event_packet (28 B)
```
u32  t_us         contact instant (FSR rise) or hg peak if no contact
u8   flags        bit0: contact (FSR fired)  bit1: saturated hg
u8   _pad
u16  peak_hg      ADXL375 peak, raw LSB
u16  f0_peak      FSR ch0 peak, ADC counts (0..4095)
u16  f1_peak      FSR ch1 peak
u16  width_ms     FSR pulse width ×10 (0.1 ms resolution)
u32  impulse      FSR integral, counts·ms
u32  seq          event counter since boot
u32  t_start_us   punch start (activity onset) — exec time = t_us - t_start_us
u16  retract_ms10 return-phase duration ×10 (contact end -> arm quiet).
                  v0 proxy; orientation-based guard-return is computed
                  host-side from the IMU stream.
```

### raw_packet (20 samples × 10 B + 8 B header = 208 B)
```
u32  t_us_base            timestamp of first sample (node clock, µs)
u16  seq                  packet counter, wraps at 2^16
u16  lost                 packets the node's queue refused since the last
                          one that got through
20 × {
  i16 hg_x, hg_y, hg_z    ADXL375, raw LSB (49 mg/LSB), no peak-hold
  u16 f0, f1              FSR ADC counts, raw — no peak-hold, no filter
}
```
The undecimated view of exactly what `punch_detect_feed()` consumes, with no
thresholding applied, so a threshold sweep can be replayed offline against the
same input the state machine saw. Sample *i* is at `t_us_base + i * 1000`.

Gyro and low-g accel are absent on purpose: they are not detector inputs and
would nearly triple the bandwidth. The 100 Hz stream keeps running alongside a
capture and still carries them.

There is no control characteristic. The CCC *is* the control: subscribing
starts capture, unsubscribing or dropping the link stops it, so no mode can
outlive the host that asked for it.

**Two distinct loss channels**, and a host reading a capture must handle both:

| Symptom | Meaning |
|---|---|
| `lost` > 0 | the node's 8-packet queue was full — the radio could not keep up. Those packets were never assigned a seq, so this counter is the only trace of them. |
| gap in `seq` | the packet was built and queued but the notify failed — dropped between the queue and the air. |

Bandwidth is ~10.4 kB/s per node. This needs the 2M PHY and a 15-30 ms
connection interval, both requested by the node on connect; a central that
refuses shows up as loss on these two channels, not as a failure.

### status_packet (12 B)
```
u32  uptime_s
u16  batt_mv
u16  loop_hz     measured sample loop rate (should be ~1000)
u16  dropped     samples dropped since boot (ring overflow)
u16  events      event count since boot
```

## Host WebSocket / JSONL (host → panel, host → disk)

Same JSON objects on both. One object per line in
`logs/session_<YYYYMMDD_HHMMSS>.jsonl`.

```json
{"t": 1752570000.123, "node": "L", "type": "imu",
 "t_us": 123456, "ax": -120, "ay": 980, "az": 40,
 "gx": 15, "gy": -8, "gz": 2, "hg": 1.2, "f0": 2100, "f1": 1450}
{"t": ..., "node": "L", "type": "event",
 "t_us": 125000, "contact": true, "sat": false,
 "peak_g": 38.5, "f0": 2100, "f1": 1450,
 "width_ms": 14.2, "impulse": 18000, "seq": 12,
 "exec_ms": 142.0, "retract_ms": 210.0, "force_n": null}
{"t": ..., "node": "R", "type": "status",
 "uptime_s": 320, "batt_mv": 3960, "loop_hz": 1001,
 "dropped": 0, "events": 12, "rssi": -61}
{"t": ..., "node": "-", "type": "log", "level": "info", "msg": "..."}
```

Units on the wire to the panel: `ax..az` mg, `gx..gz` dps, `hg` g (float,
already scaled by host), `peak_g` g, `force_n` null until the calibration
regression exists.

## Raw capture files (host → disk)

`logs/raw_<node>_<YYYYMMDD_HHMMSS>.bin`, written by `boxe_host.py --raw` and
read by `host/rawlog.py`. Not JSONL: at 1 kHz × 2 nodes the JSON overhead is
larger than the data.

```
header  24 B   "BOXERAW1" | node char | pad | u16 rate_hz | u16 batch | pad2 |
               f64 capture start (host unix time)
record         f64 t_host (arrival) | raw_packet verbatim (208 B)
```

`t_host` per record exists so a hand-typed label file can be aligned with node
timestamps: the node counts microseconds since its own boot and knows nothing
about wall-clock time. `RawReader.clock_offset()` takes the median of
`t_host - t_us/1e6` across the file rather than trusting the first packet,
which is usually the most delayed one in the capture.

A reconnect starts a new file. The node's clock does not survive the gap, so
splicing both sides into one file would fabricate continuity.

## Calibration artefacts (v1)

`calib/<node>.json` — per-node FSR divider value, FSR→N curve coefficients,
fused force regression `force_n = f(peak_g, f_peak, impulse, width_ms)`.
