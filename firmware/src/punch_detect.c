/*
 * Punch detection state machine. Fed at SAMPLE_HZ, pops event packets.
 *
 * States: IDLE -> ACTIVE (hg magnitude above start threshold or FSR
 * contact) -> collect peaks/impulse until quiet for PUNCH_WINDOW_MS ->
 * emit event -> REFRACTORY.
 *
 * All thresholds in boxe.h — tune against real data during calibration.
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
} d;

static struct event_packet pending;
static bool pending_valid;

static uint16_t hg_mag(int16_t x, int16_t y, int16_t z)
{
	/* cheap magnitude approx: max + half min of |components| —
	 * avoids sqrt in the hot path, good enough for thresholding */
	uint16_t ax = abs(x), ay = abs(y), az = abs(z);
	uint16_t hi = MAX(ax, MAX(ay, az));
	uint16_t lo = MIN(ax, MIN(ay, az));
	return hi + lo / 2;
}

void punch_detect_feed(uint32_t t_us, uint16_t f0, uint16_t f1,
		       int16_t hgx, int16_t hgy, int16_t hgz)
{
	uint16_t mag = hg_mag(hgx, hgy, hgz);

	/* baseline tracking only while idle (EMA, tau ~1 s) */
	if (d.st == IDLE) {
		d.f0_base += ((int32_t)f0 - d.f0_base) / 64;
		d.f1_base += ((int32_t)f1 - d.f1_base) / 64;
	}

	uint16_t f0_rel = f0 > d.f0_base ? f0 - d.f0_base : 0;
	uint16_t f1_rel = f1 > d.f1_base ? f1 - d.f1_base : 0;
	bool contact_now = (f0_rel > PUNCH_FSR_CONTACT) ||
			   (f1_rel > PUNCH_FSR_CONTACT);
	bool active_now = (mag > PUNCH_HG_START_LSB) || contact_now;

	switch (d.st) {
	case IDLE:
		if (active_now) {
			d.peak_hg = 0;
			d.f0_peak = d.f1_peak = 0;
			d.impulse = 0;
			d.contact = false;
			d.contact_start_us = d.contact_end_us = 0;
			d.st = ACTIVE;
			d.t_start_us = t_us;
			d.t_last_active_us = t_us;
		}
		break;

	case ACTIVE:
		if (mag > d.peak_hg) d.peak_hg = mag;
		if (f0_rel > d.f0_peak) d.f0_peak = f0_rel;
		if (f1_rel > d.f1_peak) d.f1_peak = f1_rel;
		if (contact_now) {
			if (!d.contact) {
				d.contact = true;
				d.contact_start_us = t_us;
				d.t_contact_us = t_us;
			}
			d.contact_end_us = t_us;
			d.impulse += (f0_rel + f1_rel) / (SAMPLE_HZ / 1000);
		}
		if (active_now) d.t_last_active_us = t_us;

		if ((t_us - d.t_last_active_us) > 30 * 1000 ||
		    (t_us - d.t_start_us) > PUNCH_WINDOW_MS * 1000) {
			/* quiet or window over -> emit */
			pending.t_us = d.contact ? d.t_contact_us
						 : d.t_start_us;
			pending.flags = (d.contact ? 1 : 0) |
					(d.peak_hg >= 4000 ? 2 : 0);
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
