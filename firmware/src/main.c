/*
 * Boxe_AI wrist node — main sampling pipeline.
 *
 *   k_timer 1 kHz ─▶ sample work: SAADC read (FSR x2) + ADXL375 read
 *                    ├─▶ punch_detect_feed()   (always, full rate)
 *                    └─▶ every IMU_DECIM ticks: LSM6DS33 read, batch into
 *                        imu_packet, notify when STREAM_BATCH full
 *   1 Hz status timer ─▶ battery ADC + counters -> status notify
 *
 * Both I2C sensors are driven register-level (adxl375.c / lsm6ds33.c): no
 * Zephyr driver exists for either part on this board.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <stdlib.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_power.h>
#include "boxe.h"

LOG_MODULE_REGISTER(boxe, LOG_LEVEL_INF);

/* Hand selection: build with -DCONFIG_BOXE_HAND_R=y for right node,
 * or read a GPIO strap at boot (v1). */
#ifdef CONFIG_BOXE_HAND_R
#define NODE_NAME "BOXE-R"
#else
#define NODE_NAME "BOXE-L"
#endif

static const struct adc_dt_spec fsr0 =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), fsr0);
static const struct adc_dt_spec fsr1 =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), fsr1);
static const struct adc_dt_spec vbat =
	ADC_DT_SPEC_GET_BY_NAME(DT_PATH(zephyr_user), vbat);

static struct imu_packet stream_pkt;
static uint8_t stream_fill;
static struct raw_packet raw_pkt;
static uint8_t raw_fill;
static uint32_t tick_count, dropped, event_count;
/* peak-hold for the FSR channels across one decimation window */
static uint16_t fsr0_hold, fsr1_hold;
static uint32_t ticks_this_second, measured_hz;
static uint32_t hg_down_s;      /* seconds the ADXL375 was not measuring */

static uint16_t adc_read_raw(const struct adc_dt_spec *spec)
{
	int16_t raw = 0;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};

	adc_sequence_init_dt(spec, &seq);
	if (adc_read_dt(spec, &seq) != 0) {
		return 0;
	}
	return raw < 0 ? 0 : (uint16_t)raw;
}

static void sample_fn(struct k_work *work)
{
	uint32_t t_us = (uint32_t)k_cyc_to_us_floor64(k_cycle_get_32());

	ticks_this_second++;

	/* --- FSR channels (fast path, every tick) ------------------- */
	uint16_t raw0 = adc_read_raw(&fsr0);
	uint16_t raw1 = adc_read_raw(&fsr1);

	if (raw0 > fsr0_hold) {
		fsr0_hold = raw0;
	}
	if (raw1 > fsr1_hold) {
		fsr1_hold = raw1;
	}

	/* --- ADXL375 (every tick — impact peaks are short) ----------- */
	int16_t hgx = 0, hgy = 0, hgz = 0;
	(void)adxl375_read(&hgx, &hgy, &hgz);

	punch_detect_feed(t_us, raw0, raw1, hgx, hgy, hgz);

	/* --- raw capture (only while a host is subscribed) ----------- */
	if (ble_raw_enabled()) {
		if (raw_fill == 0) {
			raw_pkt.t_us_base = t_us;
		}
		struct raw_sample *r = &raw_pkt.s[raw_fill];

		r->hgx = hgx; r->hgy = hgy; r->hgz = hgz;
		r->f0 = raw0; r->f1 = raw1;

		if (++raw_fill == RAW_BATCH) {
			raw_fill = 0;
			ble_raw_submit(&raw_pkt);
		}
	} else if (raw_fill) {
		/* capture stopped mid-packet: drop the partial batch rather
		 * than emit it later against a stale timebase */
		raw_fill = 0;
	}

	struct event_packet ev;

	while (punch_detect_pop(&ev)) {
		event_count++;
		ble_notify_event(&ev);
		LOG_INF("punch #%u peak=%u f0=%u width=%u.%u ms exec=%u ms",
			ev.seq, ev.peak_hg, ev.f0_peak,
			ev.width_ms10 / 10, ev.width_ms10 % 10,
			(ev.t_us - ev.t_start_us) / 1000);
	}

	/* --- decimated IMU stream ------------------------------------ */
	if (++tick_count % IMU_DECIM == 0) {
		int16_t a[3] = {0}, g[3] = {0};

		(void)lsm6ds33_read(a, g);

		if (stream_fill == 0) {
			stream_pkt.t_us_base = t_us;
		}
		struct imu_sample *s = &stream_pkt.s[stream_fill];

		s->ax = a[0]; s->ay = a[1]; s->az = a[2];
		s->gx = g[0]; s->gy = g[1]; s->gz = g[2];
		s->hgx = hgx; s->hgy = hgy; s->hgz = hgz;
		s->f0 = fsr0_hold; s->f1 = fsr1_hold;
		fsr0_hold = 0; fsr1_hold = 0;

		if (++stream_fill == STREAM_BATCH) {
			stream_fill = 0;
			ble_notify_imu(&stream_pkt);
		}
	}
}
K_WORK_DEFINE(sample_work, sample_fn);

static void sample_tick(struct k_timer *timer)
{
	/* keep the ISR minimal: hand off to the system workqueue */
	if (k_work_submit(&sample_work) < 0) {
		dropped++;
	}
}
K_TIMER_DEFINE(sample_timer, sample_tick, NULL);

static void status_fn(struct k_work *work)
{
	/* on-board 100k/100k divider: VBAT = 2 x measured */
	uint16_t vbat_raw = adc_read_raw(&vbat);
	int32_t mv = vbat_raw;

	if (adc_raw_to_millivolts_dt(&vbat, &mv) != 0) {
		mv = 0;
	}

	/*
	 * Throw away one FSR conversion before the sample loop takes its next
	 * one. The battery channel shares the SAADC with the FSR channels and
	 * uses a different input configuration, and the first fsr0 conversion
	 * after a vbat conversion reads about 160 counts high — one sample
	 * wide, on fsr0 only, because fsr1 is read second and has settled.
	 *
	 * That is above PUNCH_FSR_CONTACT, so it asserted contact for exactly
	 * one sample and manufactured a punch event once per second: 27 events
	 * in a 32 s capture of a node lying still, every one of them 1.000 s
	 * after the last.
	 */
	(void)adc_read_raw(&fsr0);

	measured_hz = ticks_this_second;
	ticks_this_second = 0;

	/* The high-g part is the punch detector's start trigger. If it drops
	 * into standby the node keeps streaming a plausible-looking stream of
	 * zeros and detects nothing at all, so check it every second. */
	bool measuring = false;

	(void)adxl375_health(&measuring);
	if (!measuring) {
		hg_down_s++;
	}

	struct status_packet st = {
		.uptime_s = k_uptime_get() / 1000,
		.batt_mv = (uint16_t)(mv * 2),
		.loop_hz = (uint16_t)measured_hz,
		.dropped = (uint16_t)dropped,
		.events = (uint16_t)event_count,
	};

	ble_notify_status(&st);
}
K_WORK_DEFINE(status_work, status_fn);

static void status_tick(struct k_timer *timer)
{
	k_work_submit(&status_work);
}
K_TIMER_DEFINE(status_timer, status_tick, NULL);

int main(void)
{
	LOG_INF("Boxe_AI node %s boot", NODE_NAME);

	if (!adc_is_ready_dt(&fsr0) || !adc_is_ready_dt(&fsr1) ||
	    !adc_is_ready_dt(&vbat)) {
		LOG_ERR("SAADC not ready");
	}
	(void)adc_channel_setup_dt(&fsr0);
	(void)adc_channel_setup_dt(&fsr1);
	(void)adc_channel_setup_dt(&vbat);

	if (adxl375_init() != 0) {
		LOG_ERR("adxl375 init failed — high-g is the punch start "
			"trigger, so detection will not work until it does");
	}
	(void)lsm6ds33_init();

	int err = ble_service_init(NODE_NAME);

	if (err) {
		LOG_ERR("BLE init failed (%d)", err);
		return err;
	}

	k_timer_start(&sample_timer, K_MSEC(1), K_USEC(1000000 / SAMPLE_HZ));
	k_timer_start(&status_timer, K_SECONDS(1), K_SECONDS(1));

	LOG_INF("pipeline running at %d Hz", SAMPLE_HZ);
	return 0;
}

/*
 * `dfu` shell command: reboot into the Adafruit UF2 bootloader without the
 * physical double-tap on RESET, so reflashing stays scriptable over the USB
 * console. 0x57 is the magic the bootloader looks for in GPREGRET.
 */
static int cmd_dfu(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "rebooting into the UF2 bootloader");
	nrf_power_gpregret_set(NRF_POWER, 0, 0x57);
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
SHELL_CMD_REGISTER(dfu, NULL, "Reboot into the UF2 bootloader", cmd_dfu);

/*
 * `fsr [n]` shell command: print raw SAADC counts for both FlexiForce
 * channels, n times at 10 Hz (default 20). Used to size the divider's fixed
 * resistor — the counts are otherwise only visible inside an event packet,
 * which needs a punch to fire.
 *
 * Scale: 12-bit, gain 1/6, 0.6 V internal reference -> 3.6 V full scale over
 * 4095 counts, i.e. 0.879 mV/count. A 3.3 V rail therefore tops out at 3754.
 */
static int cmd_fsr(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t n = 20;

	if (argc > 1) {
		n = strtoul(argv[1], NULL, 0);
		if (n == 0 || n > 1000) {
			shell_error(sh, "count must be 1..1000");
			return -EINVAL;
		}
	}

	/*
	 * Report the signed reading and the driver's error code, not the
	 * clamped value the sample loop uses. adc_read_raw() maps a failed
	 * conversion, a slightly negative reading at 0 V, and a genuine 0 V
	 * onto the same 0 — which is exactly the ambiguity that made a dead
	 * channel indistinguishable from an unloaded sensor.
	 */
	shell_print(sh, " f0raw err  f1raw err     mV0    mV1");
	for (uint32_t i = 0; i < n; i++) {
		int16_t v0 = 0, v1 = 0;
		struct adc_sequence s0 = { .buffer = &v0,
					   .buffer_size = sizeof(v0) };
		struct adc_sequence s1 = { .buffer = &v1,
					   .buffer_size = sizeof(v1) };

		adc_sequence_init_dt(&fsr0, &s0);
		int e0 = adc_read_dt(&fsr0, &s0);

		adc_sequence_init_dt(&fsr1, &s1);
		int e1 = adc_read_dt(&fsr1, &s1);

		shell_print(sh, "%6d %3d %6d %3d  %6d %6d", v0, e0, v1, e1,
			    (int)((int32_t)v0 * 3600 / 4095),
			    (int)((int32_t)v1 * 3600 / 4095));
		k_sleep(K_MSEC(100));
	}
	return 0;
}
SHELL_CMD_REGISTER(fsr, NULL, "Print raw FSR ADC counts: fsr [samples]", cmd_fsr);

/*
 * `hg` shell command: is the high-g part actually measuring, and what does it
 * read. Standby streams zeros, which on the wire is indistinguishable from a
 * perfectly still node — a whole session was recorded that way before this
 * existed. Check it before strapping the nodes on.
 */
static int cmd_hg(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t n = 10;

	if (argc > 1) {
		n = strtoul(argv[1], NULL, 0);
		if (n == 0 || n > 1000) {
			shell_error(sh, "count must be 1..1000");
			return -EINVAL;
		}
	}

	bool measuring = false;
	int err = adxl375_health(&measuring);

	shell_print(sh, "measuring : %s%s", measuring ? "yes" : "NO",
		    err ? " (i2c error)" : "");
	shell_print(sh, "recoveries: %u", adxl375_recoveries());
	shell_print(sh, "seconds down: %u", hg_down_s);
	if (!measuring) {
		shell_warn(sh, "high-g is the punch start trigger — "
			       "detection does nothing while it reads zero");
	}

	shell_print(sh, "    x      y      z       g");
	for (uint32_t i = 0; i < n; i++) {
		int16_t x = 0, y = 0, z = 0;

		if (adxl375_read(&x, &y, &z) != 0) {
			shell_error(sh, "read failed");
			return 0;
		}
		/* 49 mg/LSB; printed x100 to stay in integers */
		uint32_t mag = (uint32_t)((abs(x) > abs(y) ? abs(x) : abs(y)));

		mag = mag > (uint32_t)abs(z) ? mag : (uint32_t)abs(z);
		shell_print(sh, "%6d %6d %6d   %u.%02u", x, y, z,
			    (unsigned)(mag * 49 / 1000),
			    (unsigned)((mag * 49 / 10) % 100));
		k_sleep(K_MSEC(100));
	}
	return 0;
}
SHELL_CMD_REGISTER(hg, NULL, "ADXL375 state and live readings: hg [samples]", cmd_hg);
