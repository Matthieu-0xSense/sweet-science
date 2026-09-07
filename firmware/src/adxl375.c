/*
 * ADXL375 (+/-200 g) minimal I2C driver.
 *
 * Zephyr has no adi,adxl375 binding. The part shares the ADXL345 register map
 * and DEVID (0xE5) but has a fixed 49 mg/LSB scale, so a direct register
 * driver is both simpler and cheaper than going through the sensor API — the
 * BLE protocol carries raw LSB anyway (protocol.md).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "boxe.h"

LOG_MODULE_DECLARE(boxe);

#define ADXL375_ADDR        0x53   /* ALT ADDRESS low — Adafruit breakout default */

#define REG_DEVID           0x00
#define REG_BW_RATE         0x2C
#define REG_POWER_CTL       0x2D
#define REG_DATA_FORMAT     0x31
#define REG_DATAX0          0x32
#define REG_FIFO_CTL        0x38

#define DEVID_ADXL375       0xE5
#define BW_RATE_800HZ       0x0D   /* output data rate, 800 Hz */
#define POWER_CTL_MEASURE   0x08
#define DATA_FORMAT_INIT    0x0B   /* full resolution, right justified */
#define FIFO_CTL_BYPASS     0x00

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/*
 * The breakout is powered from the board's 3V3 rail, so unplugging USB from a
 * node running on USB alone power-cycles the sensor at the same moment the MCU
 * boots. The MCU wins that race: the DEVID read here failed on a sensor that
 * had not finished powering up, main() discarded the error, and the node then
 * streamed all-zero high-g for a whole session — no start trigger, no punch
 * detection, and nothing on the wire saying so. Observed on BOXE-R after a
 * cable swap: DEVID read back 0xe5 once the shell got to it, but POWER_CTL was
 * still 0x00, the power-on default.
 *
 * So: retry the probe, and verify the configuration took rather than trusting
 * the writes. adxl375_health() then re-runs this if the part is ever found
 * back in standby.
 */
#define INIT_RETRIES        10
#define INIT_RETRY_MS       5

static uint32_t recoveries;

/*
 * The register writes only, with no probe and no sleeping. Split out because
 * adxl375_health() runs on the system work queue at 1 Hz, which is the same
 * queue the 1 kHz sample work sits on: a retry loop that slept there would
 * stall sampling for as long as it slept.
 */
static int adxl375_configure(void)
{
	int err;

	/* Order matters: configure while standby, then enable measurement. */
	i2c_reg_write_byte(i2c, ADXL375_ADDR, REG_POWER_CTL, 0x00);
	i2c_reg_write_byte(i2c, ADXL375_ADDR, REG_DATA_FORMAT, DATA_FORMAT_INIT);
	i2c_reg_write_byte(i2c, ADXL375_ADDR, REG_BW_RATE, BW_RATE_800HZ);
	i2c_reg_write_byte(i2c, ADXL375_ADDR, REG_FIFO_CTL, FIFO_CTL_BYPASS);
	err = i2c_reg_write_byte(i2c, ADXL375_ADDR, REG_POWER_CTL,
				 POWER_CTL_MEASURE);
	if (err) {
		LOG_ERR("adxl375: measure enable failed (%d)", err);
		return err;
	}

	/* Read it back. A write that is ACKed by a part that is still coming
	 * up does not necessarily stick, and standby is indistinguishable from
	 * a perfectly still glove on the wire. */
	uint8_t pwr = 0;

	err = i2c_reg_read_byte(i2c, ADXL375_ADDR, REG_POWER_CTL, &pwr);
	if (err || pwr != POWER_CTL_MEASURE) {
		LOG_ERR("adxl375: POWER_CTL reads 0x%02x, expected 0x%02x "
			"(err %d)", pwr, POWER_CTL_MEASURE, err);
		return err ? err : -EIO;
	}

	LOG_INF("adxl375: ready (800 Hz, +/-200 g, 49 mg/LSB)");
	return 0;
}

int adxl375_init(void)
{
	uint8_t devid = 0;
	int err = -EIO;

	if (!device_is_ready(i2c)) {
		LOG_ERR("adxl375: i2c0 not ready");
		return -ENODEV;
	}

	for (int i = 0; i < INIT_RETRIES; i++) {
		err = i2c_reg_read_byte(i2c, ADXL375_ADDR, REG_DEVID, &devid);
		if (err == 0) {
			break;
		}
		k_sleep(K_MSEC(INIT_RETRY_MS));
	}
	if (err) {
		LOG_ERR("adxl375: no ACK at 0x%02x after %d tries (%d) — "
			"check STEMMA cable", ADXL375_ADDR, INIT_RETRIES, err);
		return err;
	}
	if (devid != DEVID_ADXL375) {
		LOG_ERR("adxl375: DEVID 0x%02x, expected 0x%02x", devid,
			DEVID_ADXL375);
		return -EINVAL;
	}

	return adxl375_configure();
}

/*
 * Cheap 1 Hz liveness check: one register read, and a reconfigure without
 * sleeping if the part is found asleep. Standby produces a stream of zeros
 * that looks exactly like a node nobody is wearing, so this is the only thing
 * standing between a dead sensor and a wasted session.
 */
int adxl375_health(bool *measuring)
{
	uint8_t pwr = 0;
	int err = i2c_reg_read_byte(i2c, ADXL375_ADDR, REG_POWER_CTL, &pwr);

	if (err) {
		*measuring = false;
		return err;
	}
	*measuring = (pwr == POWER_CTL_MEASURE);
	if (!*measuring) {
		LOG_WRN("adxl375: found in standby (POWER_CTL 0x%02x), "
			"reconfiguring", pwr);
		if (adxl375_configure() == 0) {
			recoveries++;
			*measuring = true;
		}
	}
	return 0;
}

uint32_t adxl375_recoveries(void)
{
	return recoveries;
}

int adxl375_read(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t b[6];
	int err = i2c_burst_read(i2c, ADXL375_ADDR, REG_DATAX0, b, sizeof(b));

	if (err) {
		return err;
	}
	*x = (int16_t)((b[1] << 8) | b[0]);
	*y = (int16_t)((b[3] << 8) | b[2]);
	*z = (int16_t)((b[5] << 8) | b[4]);
	return 0;
}
