/*
 * LSM6DS33 (Feather Sense on-board accel + gyro) minimal I2C driver.
 *
 * Zephyr ships lsm6dsl / lsm6dso / lsm6dsv* drivers; none of them matches the
 * WHO_AM_I of the part Adafruit fits, and the board devicetree declares no
 * node for it. Register-level access it is — 12 bytes per read, once per 10
 * ticks.
 *
 * Two parts answer at this address depending on board revision: the original
 * LSM6DS33 (0x69) and the LSM6DS3TR-C (0x6A) on later Feather Sense boards.
 * Everything used here — CTRL1_XL/CTRL2_G/CTRL3_C, OUTX_L_G at 0x22, the FS
 * encodings and the 0.488 mg / 70 mdps sensitivities — is identical, so both
 * are accepted and driven the same way.
 *
 * Output units follow protocol.md: accel mg, gyro dps x10.
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "boxe.h"

LOG_MODULE_DECLARE(boxe);

#define LSM6_ADDR         0x6A   /* SA0 tied low on the Feather Sense */

#define REG_WHO_AM_I      0x0F
#define REG_CTRL1_XL      0x10
#define REG_CTRL2_G       0x11
#define REG_CTRL3_C       0x12
#define REG_OUTX_L_G      0x22   /* 12 bytes: gyro XYZ then accel XYZ */

#define WHO_AM_I_DS33     0x69   /* LSM6DS33 */
#define WHO_AM_I_DS3TRC   0x6A   /* LSM6DS3TR-C, later Feather Sense revs */
#define CTRL1_XL_416HZ_16G  0x64 /* ODR 416 Hz, FS +/-16 g  (0.488 mg/LSB) */
#define CTRL2_G_416HZ_2000  0x6C /* ODR 416 Hz, FS +/-2000 dps (70 mdps/LSB) */
#define CTRL3_C_BDU_IFINC   0x44 /* block data update + auto address increment */

static const struct device *const i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));

int lsm6ds33_init(void)
{
	uint8_t who = 0;
	int err;

	if (!device_is_ready(i2c)) {
		LOG_ERR("lsm6ds33: i2c0 not ready");
		return -ENODEV;
	}

	err = i2c_reg_read_byte(i2c, LSM6_ADDR, REG_WHO_AM_I, &who);
	if (err) {
		LOG_ERR("lsm6ds33: no ACK at 0x%02x (%d)", LSM6_ADDR, err);
		return err;
	}
	if (who != WHO_AM_I_DS33 && who != WHO_AM_I_DS3TRC) {
		LOG_ERR("lsm6ds33: WHO_AM_I 0x%02x, expected 0x%02x or 0x%02x",
			who, WHO_AM_I_DS33, WHO_AM_I_DS3TRC);
		return -EINVAL;
	}
	LOG_INF("lsm6ds33: WHO_AM_I 0x%02x (%s)", who,
		who == WHO_AM_I_DS33 ? "LSM6DS33" : "LSM6DS3TR-C");

	i2c_reg_write_byte(i2c, LSM6_ADDR, REG_CTRL3_C, CTRL3_C_BDU_IFINC);
	i2c_reg_write_byte(i2c, LSM6_ADDR, REG_CTRL1_XL, CTRL1_XL_416HZ_16G);
	err = i2c_reg_write_byte(i2c, LSM6_ADDR, REG_CTRL2_G, CTRL2_G_416HZ_2000);
	if (err) {
		LOG_ERR("lsm6ds33: config failed (%d)", err);
		return err;
	}

	LOG_INF("lsm6ds33: ready (416 Hz, +/-16 g, +/-2000 dps)");
	return 0;
}

int lsm6ds33_read(int16_t *a, int16_t *g)
{
	uint8_t b[12];
	int err = i2c_burst_read(i2c, LSM6_ADDR, REG_OUTX_L_G, b, sizeof(b));

	if (err) {
		return err;
	}

	for (int i = 0; i < 3; i++) {
		int32_t raw = (int16_t)((b[i * 2 + 1] << 8) | b[i * 2]);
		/* 70 mdps/LSB -> dps x10: raw * 0.7 */
		g[i] = (int16_t)((raw * 7) / 10);
	}
	for (int i = 0; i < 3; i++) {
		int32_t raw = (int16_t)((b[6 + i * 2 + 1] << 8) | b[6 + i * 2]);
		/* 0.488 mg/LSB -> mg */
		a[i] = (int16_t)((raw * 488) / 1000);
	}
	return 0;
}
