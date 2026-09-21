# sweet-science

Wrist-worn punch telemetry. Two sensor nodes (nRF52840 + ADXL375 high-g accel
+ FlexiForce A201 in-glove force sensors) stream over BLE to a PC hub, with a
browser debug panel, JSONL datalogging and per-round session metrics.

## The idea

Boxing feedback is qualitative. A coach watches and says "drop your right",
"you're slowing in round 3". That judgement is good but unmeasured, so nothing
accumulates across sessions and nothing survives the coach leaving the gym.

Camera-based systems measure *pose*, which is the wrong end of a punch. The
quantities that matter — impact acceleration, contact force, contact duration,
execution and retraction time — happen in a 20 ms window at the fist, hidden
by the glove and often by the opponent's body. Two 1 kHz sensor nodes on the
wrists see all of it directly.

The bet: a hand-mounted high-g accelerometer plus a force sensor inside the
glove is enough to reconstruct punch quality, and pose estimation from video
becomes a *later fusion input* for what sensors cannot see (guard position,
footwork, distance), not the primary measurement.

## Objective

1. **Quantify a session without a camera.** Punch count and rate, hit rate,
   peak-g distribution, execution/retraction times, combos, fatigue decay
   across rounds. — *implemented, unvalidated*
2. **Calibrate force to newtons**, so numbers compare across sessions,
   fighters and hardware revisions. — *math and tooling done, never run*
3. **Classify punch type** (jab / cross / hook / uppercut) from the 1 kHz
   window. — *not started*
4. **Fuse with pose estimation** for guard, stance and distance. — *not
   started*

Success criterion for v1: hand a fighter a round-by-round report they
recognise as true, from a rig they forget they are wearing.

## Status — 2026-09

Hardware works. The pipeline is end-to-end. The numbers are not yet
trustworthy.

**Working, exercised on real hardware.** A long capture on 2026-09-01 ran both
nodes: 597 k IMU samples, 6 k status packets, 806 events, no drops.

- 1 kHz sampling loop on both nodes: FSR x2 (SAADC) + ADXL375, decimated
  100 Hz IMU stream, punch-detection state machine, 1 Hz status
- `adxl375.c` / `lsm6ds33.c`, register-level I2C drivers — Zephyr has no
  `adi,adxl375` binding and no LSM6DS33 driver
- BLE GATT service per `protocol.md`, dual-node connect with independent retry
  loops, advertising restart across host disconnects
- Python hub to JSONL + WebSocket; browser panel with live synced charts,
  zoom/pan, event table, node health; JSONL replay at 10x
- `metrics.py`: per-round, per-hand session report
- raw 1 kHz capture over BLE plus the offline threshold-fitting toolchain
  (`rawlog.py`, `punch_detect.py`, `sweep.py`, `mark.py`) — verified against a
  synthetic capture, not yet run on a recording from the bag
- simulator mode, so the whole host and panel stack runs with no hardware

**Not trustworthy yet.**

- **Detection thresholds are untuned.** They were guessed in `boxe.h` and
  never fitted to real punches. The 2026-09-01 events show it: `peak_g` of
  3-5 g on events that fired, contact widths of 400 ms. That is a desk and a
  loose glove, not a punch. Every downstream metric inherits this.
- **The first real hits on record are in `session_20260918_160344`** (right
  glove, hand inside): 51, 94 and 74 g with contact, 130-280 ms wide. The
  same log has 200 other events at 2-4 g with contact — hand moving in the
  glove, one every 0.55 s — which is what drove the detector change below.
  Three hits is not a fit.
- **Force is uncalibrated.** `force_n` is `null` in every record captured so
  far. The conductance regression and `calibrate_fsr.py` exist and the algebra
  is exact; the procedure has simply never been run with a scale and a puck.
- **`metrics.py` has never been checked against ground truth.** No manually
  counted round, no video cross-check. Combo detection and the guard-height
  proxy in particular are plausible, not verified.

## What is missing

Roughly in the order it blocks the next thing.

1. **Threshold fitting.** The tooling is in place (see below); the recording
   and the fit are not done. Until this lands the event stream is noise.
2. **Run the FSR calibration** on both nodes, in-glove, and check the fit
   holds at hand temperature (0.36 %/degC drift).
3. **Ground-truth validation.** One session, hand-counted and filmed, against
   `metrics.py` output. Anything that disagrees is a bug or a bad metric.
4. **Contact width.** Landed punches still run to the window cap on some
   events: the glove settles at a higher preload after impact and the FSR
   never falls back under `PUNCH_CONTACT_END_PCT` of its peak. Whether that
   is a pushed punch or a glove artefact needs bag data; `sweep.py` sweeps
   the percentage.
5. **Hand strap.** Left/right identity is a build flag (`CONFIG_BOXE_HAND_R`),
   so the two nodes carry different images. A GPIO strap read at boot would
   make them interchangeable.
6. **Mechanical.** No enclosure, no strap design, no cable strain relief. The
   FSR lead exiting the glove is the fragile part.
7. **Battery life.** Never measured. The 1 kHz loop with two I2C reads per
   tick and 20 Hz notifies is not power-tuned at all.
8. **Time sync between nodes.** Each node timestamps from its own clock; the
   host stamps arrival. Good enough for per-hand stats, not for anything
   comparing L and R within a few ms — which is exactly what combo timing
   wants.
9. **Punch classification**, then **pose fusion**. Both need 1-3 first.

## Layout

```
protocol.md         wire formats: BLE GATT packets, JSONL/WS, raw captures
host/
  boxe_host.py      hub — BLE (bleak) or simulator -> JSONL + WebSocket
  metrics.py        session -> per-round, per-hand report
  calibrate_fsr.py  FSR -> newtons, writes fsr_calib.json
  rawlog.py         raw capture container: read, stats, JSONL export
  punch_detect.py   port of punch_detect.c, thresholds exposed
  sweep.py          fit the thresholds offline against a capture
  mark.py           hand-label punches while recording
web/                debug panel — pure front end, no build step (uPlot vendored)
firmware/           Zephyr app for the Feather nRF52840 Sense nodes
logs/               created at runtime, not tracked
```

Hardware: 2x Adafruit Feather nRF52840 Sense, 2x ADXL375 (+/-200 g), 4x
FlexiForce A201 (two per glove), 47 kOhm divider resistors, LiPo per node,
one momentary push button per node (D6 to D5) for on/off.

## Quick start — no hardware needed

```
cd host
pip install -r requirements.txt
python boxe_host.py --sim
```

With real nodes, swap the flag: `python boxe_host.py --ble`. One of the two is
required — the hub has no default source. Each node gets its own retry loop, so
running with only one node powered is fine: the other keeps rescanning.

Then open `web/index.html` in a browser (`file://` is fine) and hit
**Connect**. You get live charts (high-g impact magnitude, FSR f0 and f1, swing
accel, gyro — L blue / R red), a punch event table, node status, a log console,
and every record written to `logs/session_*.jsonl`.

Wheel zooms toward the cursor, drag pans, all charts share one x window. The
first wheel or drag freezes the window so a waveform can be inspected while
data keeps arriving; double-click any chart or press **Live** to resume
following the tail. The cursor is synced across charts, so every legend reads
the same instant.

**Replay**: pick any past `.jsonl` in the "replay" file input — replays at 10x
into the same panel.

**Wrist attitude + punch path** (top strip): two animated gloves driven by the
100 Hz stream, each trailing the path of its last punch with a reach readout.
What a wrist IMU can and cannot give, and what the view does about it:

- *Absolute position in the room*: not recoverable. Acceleration integrated
  twice drifts to nonsense within a second and nothing external pulls it
  back. The view never claims it.
- *Orientation*: recoverable. Mahony-style filter — gyro integration
  corrected toward measured gravity, with the correction gain falling to zero
  more than 0.25 g away from 1 g, since mid-punch the accelerometer is not
  measuring gravity. Yaw has no anchor (no magnetometer in the stream) and
  drifts slowly; **Zero** re-anchors both gloves to the current guard. The
  first moment of stillness zeroes automatically.
- *Per-punch path*: recoverable with a zero-velocity reset. A punch is
  200-300 ms between two still moments; velocity and position reset at every
  still moment and integrate only during the swing, so drift gets no time to
  accumulate. Integration stops 0.8 s into any swing, where drift wins.

Verified against an analytic swing (0.9 g for 150 ms, then brake): 0.199 m
computed vs 0.199 m expected, 0.205 m on a 45° tilted mount. Not yet checked
against a real punch — no session on disk contains one.

## Threshold fitting

The detection constants in `firmware/src/boxe.h` were guessed and never
fitted. Changing one on the board costs a rebuild, a reflash of two nodes and
a fresh set of punches, so no two candidates are ever scored on the same data.
The fix is to record the full-rate stream once and sweep offline.

```
python boxe_host.py --ble --raw     # writes logs/raw_L_*.bin, raw_R_*.bin
python mark.py --stance orthodox    # someone else presses a key per punch
python sweep.py ../logs/raw_L_*.bin --labels ../logs/labels_*.txt
```

`sweep.py` prints a ranked table (precision, recall, F1, `hit_acc` = how
often the contact flag agrees with the label) and a `#define` block to paste
into `boxe.h`. Then reflash and re-record to confirm.

### Recording protocol

The labels are what turn a recording into a fit, and a timestamp alone
cannot say afterwards which hand threw, what it was, or whether it landed.
`mark.py` takes one key per punch, no Enter:

```
j c h u     landed jab / cross / hook / uppercut
J C H U     the same, missed
space / m   landed / missed, type unknown
l / r       hand for what follows (--hand sets the start; --stance orthodox
            makes jab=L and cross=R on their own)
s           sync mark
x           undo
```

and writes `<host time> <hand> <type> <hit|miss>` per line. `sweep.py` keeps
only the capture's own hand, so both nodes can be fitted from one file.

A session that yields something:

1. **Phone filming, nodes on, host running with `--raw`.** Start `mark.py`.
2. **Sync clap**: tap the two gloves together once in front of the camera,
   press `s` at the same time. One impact on both nodes, one frame on the
   video, one line in the labels — that is what lines the three clocks up
   afterwards, including the L/R offset that combo timing needs.
3. **Structured rounds** before free ones: a round of jabs only, a round of
   crosses only, then hooks, then free. The type label comes from the round
   and the labeller only has to press hit/miss, and the classifier gets
   clean classes for nothing.
4. **Some deliberate misses** (shadow, air) and **some glove-on / clench /
   push** moments with no punch, so `hit_acc` and the press rejection have
   something to score against.
5. **Nodes stay on through the rest** — the idle stretches are what the
   false-positive rate is measured on.
6. `rawlog.py stats` on both captures before anyone fits anything: a capture
   with holes fits the holes.

Whoever presses the keys is not the boxer. Their reaction time comes out as
`lag_ms`, and the sync clap gives that lag with no swing in front of it.

Two rules changed on the evidence of the captures, without a fit:

- **Only high-g opens an event** (`PUNCH_OPEN_ON_HG`). Every capture with a
  hand in the glove showed the FSR opening events on its own: the preload
  wanders by hundreds of counts with each clench, an event opens, stays
  active to the window cap because the baseline is frozen in `ACTIVE`, and
  the next opens as soon as `REFRACT` ends — one event every 0.55 s, at
  2-4 g, for minutes. Replayed over all 32 captures the change takes the
  event count from thousands to tens, and what is left has the high-g
  signature. A thrown punch, landed or not, always has one, so nothing worth
  counting is lost; what is lost is the "press" class, which `metrics.py`
  discarded anyway.
- **Contact ends** when the FSR falls under `PUNCH_CONTACT_END_PCT` of its
  peak in the event, not when it drops all the way back to
  `PUNCH_FSR_CONTACT` above a baseline that no longer applies. Widths on
  real hits were 250-400 ms; foam contact is 20-50 ms.

Two bugs fell out of the same replay. High-g impacts are 1-2 samples wide
above 5 g (45 g one millisecond, 2.4 g the next), so the 3 ms confirmation
now applies to FSR-opened runs only and high-g opens on its first sample.
And the sample that opens an event was not part of it, which did not matter
while the FSR opened events early but loses the whole punch when the impact
spike is the opener — 45 g on the capture, 10 g in the event.

One constant is already fitted this way: **`PUNCH_CONFIRM_MS` = 3** (mirrored
as `confirm_ms` in `host/punch_detect.py`). An event used to open on a single
active sample, and a gloved node lying still fired ~0.5–1 event/s: the FSR line
carries ~130 counts p-p of 50 Hz hum on top of the glove preload, plus 1–2 ms
spikes, and a spike on a hum crest clears `PUNCH_FSR_CONTACT` for one or two
samples. Replayed over every raw capture, 3 ms of sustained activity removes
all of those and keeps the real punches (28/28 on the 09-07 session); 5 ms
starts eating them. Checked on hardware as an A/B: new firmware 0 events in
47 s, old firmware 23, same table, same minute. Onset time (`t_start_us`) is
the first sample of the confirmed run, so `exec_ms` is not shortened.

- **`--raw` streams the undecimated 1 kHz view** of exactly what the detector
  consumes — high-g axes and both FSR channels, no thresholding — over GATT
  characteristic `0004`. Subscribing is what enables it, so there is no mode
  to get stuck in. ~10.4 kB/s per node, on top of the usual 100 Hz stream.
  Measured on both nodes at once: 1602 packets each over 32 s, zero loss.
- **The loop runs at 993.9 Hz, not 1000.** `K_USEC(1000)` against the 32768 Hz
  tick rounds 32.768 up to 33 ticks, so a sample is 1007 µs. Timestamps are
  self-consistent, so fitting is unaffected — `metrics.py` rates carry the
  0.7 % bias until the timer is fixed.
- **Capture continuously, not per event.** An event-triggered window can only
  ever record punches the current threshold already caught, so the false
  negatives — the thing the start threshold decides — stay invisible. That
  bias is why `0004` streams rather than dumping a buffer around each event.
- **`sweep.py` scores against ground truth**, F1 against a label file (best)
  or absolute count error against `--truth N`. With neither it falls back to a
  weak plausibility heuristic, which shortlists but does not fit.
- **`punch_detect.py` is a deliberate line-by-line port** of
  `punch_detect.c`, down to uint32 timestamp wrapping, C truncating division
  and the sqrt-free magnitude approximation. If it drifts from the C, the
  fitted numbers describe a detector that does not exist. Change one, change
  the other.
- `rawlog.py stats <capture>` reports packet loss — a sweep fitted on a
  capture with holes in it fits the holes. `rawlog.py jsonl <capture>`
  converts to panel format at 100 Hz to eyeball a recording.

## Session analytics

```
python metrics.py ../logs/session_XXXX.jsonl            # 3 min rounds default
python metrics.py session.jsonl --round 180 --rest 60 --json report.json
```

Per round and per hand: punch count and rate, hit%, peak-g mean/max, power
decay slope, execution time (guard to impact) and retraction medians, combo
detection (count, longest, favourite L/R patterns, intra-combo gap), movement
intensity (accel RMS), guard-height proxy (forearm pitch on quasi-static
samples), and a round-1 vs last-round fatigue summary.

Treat the output as a shape, not a measurement, until items 1-3 above are done.

## Force calibration

`force_n` stays `None` until the FSR is calibrated. The divider is not linear
in force, but nothing is lost by that: counts -> volts -> `R_fsr` is exact
algebra, and the FlexiForce's **conductance** is what is linear in force, so
the whole calibration is one constant `k` in `1/R_fsr = k * F`.

```
cd host
python calibrate_fsr.py --node L --channel f0
```

Sensor flat on a bathroom scale, **rigid puck** over the 9.53 mm sensing pad,
press through the puck and type each scale reading. The script conditions the
sensor first (the datasheet's repeatability and hysteresis figures assume a
conditioned sensor), takes a zero, then fits `k` and reports R2. It writes
`host/fsr_calib.json`, which `boxe_host.py` loads on start.

Two things that will quietly ruin a fit:

- **No puck.** Load spreads across the polyester substrate and only a fraction
  reaches the element — you end up calibrating your fingertip.
- **Glove preload.** `f0_peak` in an event is relative to the node's own EMA
  baseline, so the calibration stores the baseline it saw and adds it back. If
  the resting pressure in the glove differs a lot from calibration conditions,
  recalibrate in the glove.

Drift is 0.36 %/degC, so a calibration taken cold on a desk reads a few percent
off at hand temperature. Pass `--r-fixed` if the divider is not the default
47 kOhm.

## Firmware

`firmware/` is an upstream Zephyr (v4.4.x) application — no nRF Connect SDK,
no VS Code needed. Board target:

```
adafruit_feather_nrf52840/nrf52840/sense/uf2
```

Toolchain (one-off, all CLI):

```
pip install west pyocd cmake ninja
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v4.4.2 C:\zephyrproject
cd C:\zephyrproject && west update --narrow -o=--depth=1 && west zephyr-export
pip install -r zephyr\scripts\requirements-base.txt
west sdk install -t arm-zephyr-eabi --install-dir C:\zephyr-sdk
```

Build and flash:

```
west build -b adafruit_feather_nrf52840/nrf52840/sense/uf2 firmware
# double-tap RESET (or `dfu` on the USB shell) -> FTHRSNSBOOT drive appears -> copy the uf2:
copy build\zephyr\zephyr.uf2 E:\
```

Right-hand node: add `-- -DCONFIG_BOXE_HAND_R=y`.

With a Pi Debug Probe wired to SWD, swap the console for RTT and use the probe
for flash/debug:

```
west build -b adafruit_feather_nrf52840/nrf52840/sense/uf2 firmware -- -DEXTRA_CONF_FILE=rtt.conf
west flash -r pyocd
pyocd rtt -t nrf52840          # logs + Zephyr shell (i2c scan i2c0, ...)
pyocd gdbserver -t nrf52840    # step debug
```

### Things that cost real time here

- `prj.conf` sets `CONFIG_USE_DT_CODE_PARTITION=y`. The board does not, so
  without it the image links at 0x0 instead of the 0x26000 app partition and
  the UF2 bootloader never jumps to it. Silent failure — the board just sits
  there.
- `ble_service.c` restarts advertising from the `disconnected` callback. A
  connectable advertiser stops when a central connects and is not resumed for
  us, so without this a node advertises once per boot and goes invisible after
  the first host disconnect. The restart is deferred to the system work queue
  (`bt_le_adv_start()` must not run on the BT RX thread) and retried on
  `-ENOMEM`: the connection object is still held when `disconnected` fires, so
  a restart that runs too early finds no free conn. It is a race, so it shows
  on one board and not the other.
- A node missing from BLE scans is usually already connected — a connected
  peripheral does not advertise, and with `CONFIG_BT_MAX_CONN=1` a connectable
  advertiser then fails with `-ENOMEM`. Check for a running `boxe_host.py`
  before suspecting the firmware. The `ble` shell command reports name,
  `bt_is_ready`, connection count and last init failure.
- `f0`/`f1` in the IMU stream are **peak-held** over the ten 1 kHz ticks each
  100 Hz sample covers. A snapshot walks straight past a contact peak a few ms
  wide. This makes `imu_packet` 114 B rather than 94 B — fits, MTU is 247.
  `hg_*` is held the same way since the FIFO change (hardest tick, all three
  axes of it).
- **The ADXL375 runs at 800 Hz into its FIFO; the loop drains it each tick
  and keeps the hardest entry.** 1600 Hz was tried (400 Hz bandwidth smooths
  a 1-2 ms impact, the same punch read 20 % apart between trials) and did
  not survive the glove: over 400 kHz I2C the part is specified to 800 Hz
  only, and above it a still arm read 5-16 g spikes — torn samples, an axis
  near zero reading 0x00FF/0xFF00 — so guard position produced an event
  every few hundred ms while `loop_hz` fell from 992 to ~930. Faster needs
  SPI. An empty FIFO is never read (the data registers are mid-update) and
  samples beyond +/-4096 LSB are dropped; the `hg` shell command prints that
  drop count with the most FIFO entries ever found waiting and the overrun
  count — near 32 means the loop ran late and entries were lost.
- `lsm6ds33.c` accepts WHO_AM_I 0x69 (LSM6DS33) and 0x6A (LSM6DS3TR-C, fitted
  on later Feather Sense revisions) — same register map and sensitivities.
- `dfu` shell command reboots into the UF2 bootloader (GPREGRET magic 0x57),
  so reflashing needs no physical double-tap on RESET.
- **On/off is a momentary button between D6 (P0.07) and D5 (P1.08), not a
  switch on EN.** D5 is driven low as the button's return because the header
  has one GND pin and the FSR dividers already own it; real GND works too.
  Hold 1 s while running: the LED goes solid, let go and the node
  enters nRF System OFF. Press again to wake (a full reset). USB plug-in and
  RESET also wake it; the `off` shell command takes the same path for bench
  use. Before a strap button is wired, point the `pwrbtn` alias in the
  overlay at `&button0` (the on-board switch). Off draw is ~70-100 uA — the
  3.3 V regulator stays up, the ADXL375 and LSM6DS33 are put to
  standby/power-down first — against 0 uA for a switch on EN; months on a
  400 mAh pack. `src/power.c` has the two gotchas (the wake is a reset with
  the finger still on the button; System OFF with SENSE already true wakes
  immediately).
- the red LED (D13, `led0`) blinks 50 ms every 2 s while the node is running,
  so the on/off state is visible through the strap. Solid would cost ~2 mA
  (15-20 % of the node's draw); the blink averages ~50 uA.
- `fsr [samples]` prints raw SAADC counts and mV for both channels at 10 Hz.
  Otherwise the counts only surface inside an event packet, which needs a
  punch to fire — this is what sizes the divider's fixed resistor.
- Console and shell land on the **USB CDC port** (the `uf2` board variant
  wires `zephyr,console` there), so bring-up needs no debug probe.

## Debug access

No SSH — nodes are BLE MCUs. Three paths:

1. **The panel** — live data, events, node health over BLE
2. **RTT** (SWD probe) — firmware logs + Zephyr shell (`kernel threads`, ...)
3. **BLE sniffer** (nRF52840 dongle + Wireshark) — on-air traffic

## License

MIT, see `LICENSE`. uPlot is vendored under `web/vendor/` under its own MIT
license.
