/*
 * Punch detection state machine. Fed at SAMPLE_HZ, pops event packets.
 *
 * States: IDLE -> ACTIVE (ADXL375 or LSM6 magnitude above its start
 * threshold; with PUNCH_OPEN_ON_HG=0 also on FSR contact) -> collect peaks/impulse until
 * quiet for 30 ms or PUNCH_WINDOW_MS is up -> emit event -> REFRACTORY.
 *
 * All thresholds in boxe.h — fitted offline with host/sweep.py, which
 * replays host/punch_detect.py, a line-by-line port of this file. Change
 * one, change the other.
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <stdlib.h>
#include "boxe.h"

enum state { IDLE, ACTIVE, REFRACT };

static struct {
	enum state st;
	uint32_t t_start_us;
	uint32_t t_last_active_us;
	uint32_t t_contact_us;
	uint16_t peak_hg;
	uint16_t f0_peak, f1_peak;
	uint16_t f0_base, f1_base;   /* slow baseline (EMA) for contact detect */
	uint32_t impulse;
	uint32_t contact_start_us, contact_end_us;
	bool contact;
	uint32_t seq;
	bool primed;                 /* baseline seeded from the first sample */
	uint16_t run;                /* consecutive active samples while IDLE */
	uint32_t run_start_us;
	uint32_t lg_sq;              /* latest low-g |a|^2, mg^2 */
	uint32_t peak_lg_sq;
	bool swing;                  /* an accel crossed its start threshold */
} d;

static struct event_packet pending;
static bool pending_valid;

uint16_t punch_hg_mag(int16_t x, int16_t y, int16_t z)
{
	/* cheap magnitude approx: max + half min of |components| —
	 * avoids sqrt in the hot path, good enough for thresholding */
	uint16_t ax = abs(x), ay = abs(y), az = abs(z);
	uint16_t hi = MAX(ax, MAX(ay, az));
	uint16_t lo = MIN(ax, MIN(ay, az));
	return hi + lo / 2;
}

/*
 * Squared magnitude, no sqrt and no approximation: +/-16 g is 16000 mg per
 * axis, so the sum tops out at 7.7e8 and fits 32 bits.
 */
void punch_detect_lowg(const int16_t a_mg[3])
{
	d.lg_sq = (uint32_t)((int32_t)a_mg[0] * a_mg[0]) +
		  (uint32_t)((int32_t)a_mg[1] * a_mg[1]) +
		  (uint32_t)((int32_t)a_mg[2] * a_mg[2]);
}

static uint32_t isqrt32(uint32_t v)
{
	uint32_t r = 0, bit = 1UL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit) {
		if (v >= r + bit) {
			v -= r + bit;
			r = (r >> 1) + bit;
		} else {
			r >>= 1;
		}
		bit >>= 2;
	}
	return r;
}

void punch_detect_feed(uint32_t t_us, uint16_t f0, uint16_t f1,
		       int16_t hgx, int16_t hgy, int16_t hgz)
{
	uint16_t mag = punch_hg_mag(hgx, hgy, hgz);

	/*
	 * Seed the baseline from the first sample instead of walking up from
	 * zero. A glove holds the FSR at a resting preload of a couple of
	 * thousand counts, so the first sample of an unseeded detector reads
	 * thousands of counts above a baseline of 0 — far past
	 * PUNCH_FSR_CONTACT — and contact asserts on sample #1.
	 */
	if (!d.primed) {
		d.f0_base = f0;
		d.f1_base = f1;
		d.primed = true;
	}

	/*
	 * Track the baseline whenever we are not inside a punch (EMA, tau
	 * ~64 ms at 1 kHz).
	 *
	 * Tracking only in IDLE was self-defeating: any contact that asserted
	 * took the state machine out of IDLE, which froze the baseline, which
	 * kept contact asserted. The latch sustained itself for the rest of
	 * the session and pinned events to one per (PUNCH_WINDOW_MS +
	 * PUNCH_REFRACT_MS) — a measured 1.82 events/s on a node sitting
	 * still, matching 1/(0.400 + 0.150) exactly.
	 *
	 * REFRACT is included so the baseline can catch up with a preload that
	 * changed during the punch; ACTIVE is still excluded, or the baseline
	 * would climb into the pulse and clip its tail.
	 */
	if (d.st == IDLE || d.st == REFRACT) {
		d.f0_base += ((int32_t)f0 - d.f0_base) / 64;
		d.f1_base += ((int32_t)f1 - d.f1_base) / 64;
	}

	uint16_t f0_rel = f0 > d.f0_base ? f0 - d.f0_base : 0;
	uint16_t f1_rel = f1 > d.f1_base ? f1 - d.f1_base : 0;
	bool pressed = (f0_rel > PUNCH_FSR_CONTACT) ||
		       (f1_rel > PUNCH_FSR_CONTACT);
	/*
	 * Once contact has begun it only counts as continuing while the FSR
	 * stays above PUNCH_CONTACT_END_PCT of the event's peak. A glove that
	 * settles at a higher preload after the impact is not a fist still on
	 * the bag; without this, every landed punch ran to the window cap
	 * (250-400 ms widths on real hits; foam contact is 20-50 ms).
	 */
	bool contact_now = pressed;
	if (pressed && d.st == ACTIVE && d.contact && PUNCH_CONTACT_END_PCT) {
		uint32_t hold = (uint32_t)MAX(d.f0_peak, d.f1_peak) *
				PUNCH_CONTACT_END_PCT / 100;
		if (hold < PUNCH_FSR_CONTACT) hold = PUNCH_FSR_CONTACT;
		contact_now = (f0_rel > hold) || (f1_rel > hold);
	}
	/*
	 * Either accelerometer counts as a swing. The ADXL375 alone missed
	 * unloaded punches: shadow boxing peaks at 4-9 g on the wrist, the
	 * part carries 1-2 g of offset and noise, and 5 g is as low as its
	 * threshold goes before a still arm opens events. The LSM6 sees the
	 * same punches at 4-14 g against a 1.0 g floor; at 3 g it opened on
	 * every swing of the 21/09 sessions and on nothing in guard. It is
	 * sampled at 100 Hz, so a low-g open can be up to 10 ms late — the
	 * ADXL375 still gets there first on anything that hits.
	 */
	bool swing = mag > PUNCH_HG_START_LSB ||
		     (PUNCH_LG_START_MG &&
		      d.lg_sq > (uint32_t)PUNCH_LG_START_MG * PUNCH_LG_START_MG);
	/*
	 * Only an accelerometer opens an event. Every session with a hand in the glove
	 * showed why the FSR must not: the resting preload wanders by hundreds
	 * of counts with each clench and wrist movement, so the FSR opened an
	 * event, held it active to the window cap, and the baseline (frozen in
	 * ACTIVE) never caught up — one event every window+refract, 0.55 s,
	 * for minutes, at 2-4 g, which is a hand moving and no punch. A thrown
	 * punch, landed or not, always carries the high-g signature, so
	 * nothing worth counting is lost.
	 */
	bool open_now = PUNCH_OPEN_ON_HG ? swing : (swing || contact_now);
	bool active_now = swing || contact_now;

	switch (d.st) {
	case IDLE:
		/*
		 * An FSR-opened run needs PUNCH_CONFIRM_MS of uninterrupted
		 * contact. One sample used to be enough, and on a gloved node
		 * on USB that fired ~1 event/s with nothing moving: the FSR
		 * line carries ~130 counts p-p of 50 Hz hum plus 1-2 ms
		 * spikes, and a spike on a hum crest clears PUNCH_FSR_CONTACT
		 * for exactly one or two samples. Replayed over the raw
		 * captures, 3 ms removes every such event and keeps the real
		 * punches; 5 ms starts eating them.
		 *
		 * High-g has no such impostor and opens on its first sample:
		 * on the captures a real impact is 1-2 samples wide above 5 g
		 * (45 g one sample, 2.4 g the next), so waiting for a third
		 * would drop most of them.
		 */
		if (!open_now) {
			d.run = 0;
			break;
		}
		if (d.run == 0) {
			d.run_start_us = t_us;
		}
		d.run++;
		if (d.run < (swing ? 1 : PUNCH_CONFIRM_MS * (SAMPLE_HZ / 1000))) {
			break;
		}
		d.run = 0;
		d.peak_hg = 0;
		d.f0_peak = d.f1_peak = 0;
		d.impulse = 0;
		d.contact = false;
		d.contact_start_us = d.contact_end_us = 0;
		d.peak_lg_sq = 0;
		d.swing = false;
		d.st = ACTIVE;
		/* onset = first sample of the confirmed run, so exec_ms
		 * is not shortened by the confirmation delay */
		d.t_start_us = d.run_start_us;
		d.t_last_active_us = t_us;
		/* fall through: the sample that opened the event is part of
		 * it. Skipping it lost the whole punch when the impact spike
		 * was the opener — 45 g on the capture, 10 g in the event. */
		__fallthrough;

	case ACTIVE:
		if (mag > d.peak_hg) d.peak_hg = mag;
		if (d.lg_sq > d.peak_lg_sq) d.peak_lg_sq = d.lg_sq;
		if (swing) d.swing = true;
		if (f0_rel > d.f0_peak) d.f0_peak = f0_rel;
		if (f1_rel > d.f1_peak) d.f1_peak = f1_rel;
		if (contact_now) {
			if (!d.contact) {
				d.contact = true;
				d.contact_start_us = t_us;
				d.t_contact_us = t_us;
			}
			d.contact_end_us = t_us;
		}
		if (pressed) {
			d.impulse += (f0_rel + f1_rel) / (SAMPLE_HZ / 1000);
		}
		if (active_now) d.t_last_active_us = t_us;

		if ((t_us - d.t_last_active_us) > 30 * 1000 ||
		    (t_us - d.t_start_us) > PUNCH_WINDOW_MS * 1000) {
			/* quiet or window over -> emit */
			pending.t_us = d.contact ? d.t_contact_us
						 : d.t_start_us;
			pending.flags = (d.contact ? 1 : 0) |
					(d.peak_hg >= 4000 ? 2 : 0) |
					(d.swing ? 4 : 0);
			/* mg -> g x10; 16 g full scale reads 160, fits u8 */
			pending.peak_lg10 = (uint8_t)MIN(isqrt32(d.peak_lg_sq) / 100, 255);
			pending.peak_hg = d.peak_hg;
			pending.f0_peak = d.f0_peak;
			pending.f1_peak = d.f1_peak;
			pending.width_ms10 = d.contact
				? (d.contact_end_us - d.contact_start_us) / 100
				: 0;
			pending.impulse = d.impulse;
			pending.seq = ++d.seq;
			pending.t_start_us = d.t_start_us;
			pending.retract_ms10 = d.contact
				? (t_us - d.contact_end_us) / 100
				: 0;
			pending_valid = true;
			d.st = REFRACT;
			d.t_last_active_us = t_us;
		}
		break;

	case REFRACT:
		if ((t_us - d.t_last_active_us) > PUNCH_REFRACT_MS * 1000) {
			d.st = IDLE;
		}
		break;
	}
}

bool punch_detect_pop(struct event_packet *out)
{
	if (!pending_valid) {
		return false;
	}
	*out = pending;
	pending_valid = false;
	return true;
}
