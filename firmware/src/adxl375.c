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

int adxl375_init(void)
{
	uint8_t devid = 0;
	int err;

	if (!device_is_ready(i2c)) {
		LOG_ERR("adxl375: i2c0 not ready");
		return -ENODEV;
	}

	err = i2c_reg_read_byte(i2c, ADXL375_ADDR, REG_DEVID, &devid);
	if (err) {
		LOG_ERR("adxl375: no ACK at 0x%02x (%d) — check STEMMA cable",
			ADXL375_ADDR, err);
		return err;
	}
	if (devid != DEVID_ADXL375) {
		LOG_ERR("adxl375: DEVID 0x%02x, expected 0x%02x", devid,
			DEVID_ADXL375);
		return -EINVAL;
	}

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

	LOG_INF("adxl375: ready (800 Hz, +/-200 g, 49 mg/LSB)");
	return 0;
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
