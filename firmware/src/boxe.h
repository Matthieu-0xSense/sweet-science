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
#define PUNCH_FSR_CONTACT    150    /* ADC counts above baseline */
#define PUNCH_WINDOW_MS      400
#define PUNCH_REFRACT_MS     150

struct __packed imu_sample {
	int16_t ax, ay, az;         /* LSM6DS33, mg */
	int16_t gx, gy, gz;         /* dps x10 (+/-2000 dps fits int16) */
	int16_t hgx, hgy, hgz;      /* ADXL375 raw LSB, 49 mg/LSB */
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
	uint8_t  flags;             /* bit0 contact, bit1 hg saturated */
	uint8_t  _pad;
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
	int16_t hgx, hgy, hgz;      /* ADXL375 raw LSB, 49 mg/LSB */
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

/* adxl375.c — high-g impact accelerometer, raw LSB out */
int adxl375_init(void);
int adxl375_read(int16_t *x, int16_t *y, int16_t *z);
/* Liveness: the part streams zeros in standby, which is indistinguishable
 * from a still glove. Checked at 1 Hz; re-initialises if found asleep. */
int adxl375_health(bool *measuring);
uint32_t adxl375_recoveries(void);

/* lsm6ds33.c — on-board accel/gyro, mg and dps x10 out */
int lsm6ds33_init(void);
int lsm6ds33_read(int16_t *a, int16_t *g);

/* punch_detect.c — fed at SAMPLE_HZ from the sampling loop */
void punch_detect_feed(uint32_t t_us, uint16_t f0, uint16_t f1,
		       int16_t hgx, int16_t hgy, int16_t hgz);
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
