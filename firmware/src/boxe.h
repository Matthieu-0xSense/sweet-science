/* Boxe_AI node — shared definitions. Wire formats must match protocol.md. */

#ifndef BOXE_H
#define BOXE_H

#include <stdint.h>
#include <stdbool.h>

#define SAMPLE_HZ        1000       /* FSR SAADC loop */
#define IMU_DECIM        10         /* 1 kHz -> 100 Hz stream view */
#define STREAM_BATCH     5          /* samples per BLE imu packet */
#define RAW_BATCH        20         /* samples per raw packet = 20 ms */

/* punch detection thresholds — tune during calibration */
#define PUNCH_HG_START_LSB   102    /* ~5 g on ADXL375 (49 mg/LSB) */
#define PUNCH_LG_START_MG    3000   /* LSM6 |a| that also opens an event; 0 = off.
				     * An unloaded punch peaks at 4-9 g on the
				     * wrist, right on the ADXL375 threshold and
				     * inside its offset + noise; the low-g part
				     * resolves that range cleanly */
#define PUNCH_FSR_CONTACT    150    /* ADC counts above baseline */
#define PUNCH_WINDOW_MS      400
#define PUNCH_REFRACT_MS     150
#define PUNCH_QUIET_MS       100    /* event ends after this long under every
				     * threshold. Was 30: a punch is a drive, a dip
				     * and a harder stop at extension 50-90 ms on,
				     * so 30 ms closed the event in the dip and the
				     * stop fell in the refractory or became a second
				     * event — 13 of 25 events held the punch's real
				     * peak; at 100 ms, 23 of 23 */
#define PUNCH_CONFIRM_MS     3      /* sustained FSR contact needed to open an event */
#define PUNCH_OPEN_ON_HG     1      /* 1: only high-g opens an event; the FSR is
				     * read inside it for contact. 0: FSR contact
				     * opens one too (original rule) */
#define PUNCH_CONTACT_END_PCT 50    /* contact is over once the FSR falls below
				     * this share of its peak in the event; 0
				     * keeps it while f_rel > PUNCH_FSR_CONTACT */

struct __packed imu_sample {
	int16_t ax, ay, az;         /* LSM6DS33, mg */
	int16_t gx, gy, gz;         /* dps x10 (+/-2000 dps fits int16) */
	int16_t hgx, hgy, hgz;      /* ADXL375 raw LSB, 49 mg/LSB — the hardest
				     * tick of the decimation window, same reason */
	uint16_t f0, f1;            /* FSR ADC counts, peak over the decimation
				     * window — a plain snapshot at 100 Hz walks
				     * straight past a contact peak a few ms wide */
};

struct __packed imu_packet {
	uint32_t t_us_base;
	struct imu_sample s[STREAM_BATCH];
};

struct __packed event_packet {
	uint32_t t_us;
	uint8_t  flags;             /* bit0 contact, bit1 hg saturated,
				     * bit2 swing (either accel over its start
				     * threshold during the event) */
	uint8_t  peak_lg10;         /* LSM6 |a| peak, g x10 (100 Hz samples) */
	uint16_t peak_hg;           /* raw LSB */
	uint16_t f0_peak;           /* ADC counts */
	uint16_t f1_peak;
	uint16_t width_ms10;        /* pulse width x10 */
	uint32_t impulse;           /* counts*ms */
	uint32_t seq;
	uint32_t t_start_us;        /* activity onset — exec = t_us - t_start_us */
	uint16_t retract_ms10;      /* return phase x10 (contact end -> quiet) */
};

/*
 * Raw capture: the undecimated 1 kHz view the detector actually consumes,
 * with no thresholding applied. Only what the punch detector reads is
 * carried — high-g axes and both FSR channels — so a threshold sweep can be
 * replayed offline against the exact input the state machine saw.
 *
 * The gyro and the low-g accel are deliberately absent: they are not detector
 * inputs, they would nearly triple the bandwidth, and the 100 Hz stream still
 * carries them alongside a raw capture.
 *
 * Sample timestamps are implicit: sample i is t_us_base + i * (1e6/SAMPLE_HZ).
 * `lost` counts packets the queue could not take since the last one that got
 * through, so a gap is visible in the capture instead of silently closing up.
 */
struct __packed raw_sample {
	int16_t hgx, hgy, hgz;      /* ADXL375 raw LSB, 49 mg/LSB: the hardest
				     * FIFO entry of the tick, what the detector saw */
	uint16_t f0, f1;            /* FSR ADC counts, no peak-hold, no filter */
};

struct __packed raw_packet {
	uint32_t t_us_base;
	uint16_t seq;               /* packet counter, wraps */
	uint16_t lost;              /* packets dropped since the previous one */
	struct raw_sample s[RAW_BATCH];
};

struct __packed status_packet {
	uint32_t uptime_s;
	uint16_t batt_mv;
	uint16_t loop_hz;
	uint16_t dropped;
	uint16_t events;
};

/* adxl375.c — high-g impact accelerometer, raw LSB out. The part buffers in
 * its FIFO; adxl375_read_peak() pops everything since the last tick and
 * returns the hardest entry (the last good sample if nothing new arrived),
 * so no sample is skipped between two loop ticks. */
int adxl375_init(void);
int adxl375_read_peak(int16_t *x, int16_t *y, int16_t *z);
void adxl375_fifo_stats(uint32_t *overruns, uint8_t *peak_entries,
			uint32_t *bad);
/* Liveness: the part streams zeros in standby, which is indistinguishable
 * from a still glove. Checked at 1 Hz; re-initialises if found asleep. */
int adxl375_health(bool *measuring);
uint32_t adxl375_recoveries(void);
int adxl375_standby(void);

/* lsm6ds33.c — on-board accel/gyro, mg and dps x10 out */
int lsm6ds33_init(void);
int lsm6ds33_read(int16_t *a, int16_t *g);
int lsm6ds33_power_down(void);

/* main.c — stop the sample/status/LED timers and leave the LED off, so a
 * System OFF entry does not race the pipeline for the I2C bus or freeze the
 * LED on (GPIO state is retained through System OFF: a lit LED would cost
 * 2 mA for the whole "off" period). */
void pipeline_stop(void);

/* power.c — momentary button as an on/off toggle via nRF System OFF.
 * Hold to power off; the same pin's GPIO SENSE wakes (resets) the chip. */
int power_button_init(void);

/* punch_detect.c — fed at SAMPLE_HZ from the sampling loop */
uint16_t punch_hg_mag(int16_t x, int16_t y, int16_t z);
void punch_detect_feed(uint32_t t_us, uint16_t f0, uint16_t f1,
		       int16_t hgx, int16_t hgy, int16_t hgz);
/* latest low-g sample, mg; held until the next one (the LSM6 is read at
 * SAMPLE_HZ / IMU_DECIM) */
void punch_detect_lowg(const int16_t a_mg[3]);
bool punch_detect_pop(struct event_packet *out);

/* ble_service.c */
int  ble_service_init(const char *name);
void ble_notify_imu(const struct imu_packet *pkt);
void ble_notify_event(const struct event_packet *pkt);
void ble_notify_status(const struct status_packet *pkt);

/* Raw capture. Enabled by the host subscribing to the raw characteristic;
 * there is no separate control write and therefore no mode to get stuck in —
 * a dropped connection clears the CCC and capture stops on its own.
 * ble_raw_submit() fills seq/lost and hands the packet to the tx thread; it
 * never blocks, so it is safe to call from the 1 kHz sample path. */
bool ble_raw_enabled(void);
void ble_raw_submit(struct raw_packet *pkt);

#endif /* BOXE_H */
