/*
 * power.c — momentary push button as an on/off toggle.
 *
 * The strap button is normally open and not latched, so it cannot sit on the
 * regulator's EN pin the way the original slide switch did. The chip does the
 * latching instead: hold the button PWR_HOLD_MS while running and the node
 * enters nRF System OFF (~1 uA for the chip); press it again and the pin's
 * GPIO SENSE wakes the chip, which is a full reset into a normal boot.
 *
 * Two details make this more than "poweroff on press":
 *
 *  - The wake is a reset, and the finger is still on the button when main()
 *    runs. A press seen at boot is ignored until the first release, or the
 *    node powers itself straight back off.
 *  - System OFF with the SENSE condition already true wakes immediately, so
 *    the entry waits for the release. The LED goes solid once the hold is
 *    long enough ("you can let go now") and off on release, right before
 *    System OFF.
 *
 * The 3.3 V rail stays up through System OFF (EN is left alone), so what
 * remains is the regulator's quiescent draw (AP2112K, ~55 uA) plus whatever
 * the sensors take in the state they were left in — hence the standby and
 * power-down writes before the entry. ~70-100 uA total, months on a 400 mAh
 * pack, against 0 uA for the old switch. A USB plug-in (VBUS) and the RESET
 * button also wake from System OFF, so flashing needs no button dance.
 *
 * BLE is not torn down: bt_disable() blocks on work that may need the system
 * workqueue this runs on, and System OFF kills the radio anyway. The host
 * sees a supervision timeout, exactly as it did when the switch cut power.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/atomic.h>
#include <hal/nrf_power.h>
#include "boxe.h"

LOG_MODULE_DECLARE(boxe);

#define PWR_POLL_MS       50
#define PWR_HOLD_MS       1000      /* a glove bump is far shorter */
#define PWR_HOLD_TICKS    (PWR_HOLD_MS / PWR_POLL_MS)
#define PWR_RELEASE_TICKS 2         /* 100 ms of released before System OFF */

static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(pwrbtn), gpios);
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
/*
 * The button's other terminal. The Feather header has one GND pin and the
 * FSR dividers already sit on it, so a spare GPIO driven low stands in: the
 * pull-up's ~0.25 mA is nothing for the pin, and GPIO configuration is
 * retained through System OFF, so the return is still low when the wake
 * press comes. A button wired to real GND works unchanged.
 */
static const struct gpio_dt_spec btn_gnd =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pwrgnd_gpios);

enum pwr_state {
	PWR_BOOT,          /* ignore the button until its first release */
	PWR_ARMED,         /* counting the hold */
	PWR_WAIT_RELEASE,  /* pipeline stopped, LED solid, waiting to let go */
};

static enum pwr_state state = PWR_BOOT;
static uint32_t held, released;
static atomic_t force_off;         /* `off` shell command */

static void enter_system_off(void)
{
	(void)gpio_pin_set_dt(&led, 0);

	/*
	 * Written here and not when the hold was detected: a status_work item
	 * already queued at that moment would have run adxl375_health() after
	 * the standby write and woken the part again. By now the timers have
	 * been stopped for at least PWR_RELEASE_TICKS polls, so the queue has
	 * drained.
	 */
	(void)adxl375_standby();
	(void)lsm6ds33_power_down();

	/* Level interrupt on the nRF GPIO driver is implemented with SENSE,
	 * which is what wakes System OFF. */
	int err = gpio_pin_interrupt_configure_dt(&btn, GPIO_INT_LEVEL_ACTIVE);

	if (err) {
		LOG_ERR("power: wake sense config failed (%d), staying on", err);
		return;
	}

	LOG_INF("power: System OFF");
	sys_poweroff();
}

static void poll_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	bool pressed = gpio_pin_get_dt(&btn) > 0;

	switch (state) {
	case PWR_BOOT:
		if (!pressed) {
			state = PWR_ARMED;
		}
		break;

	case PWR_ARMED:
		held = pressed ? held + 1 : 0;
		if (held >= PWR_HOLD_TICKS || atomic_get(&force_off)) {
			pipeline_stop();
			(void)gpio_pin_set_dt(&led, 1);
			released = 0;
			state = PWR_WAIT_RELEASE;
			LOG_INF("power: hold detected, release to power off");
		}
		break;

	case PWR_WAIT_RELEASE:
		released = pressed ? 0 : released + 1;
		if (released >= PWR_RELEASE_TICKS) {
			enter_system_off();
			/* only reached if the SENSE config failed */
			state = PWR_ARMED;
			held = 0;
			atomic_clear(&force_off);
		}
		break;
	}
}
K_WORK_DEFINE(poll_work, poll_fn);

static void poll_tick(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	(void)k_work_submit(&poll_work);
}
K_TIMER_DEFINE(poll_timer, poll_tick, NULL);

int power_button_init(void)
{
	/* RESETREAS is sticky across resets; clear it so the next boot's
	 * reason is its own. Only visible over RTT — the USB console does not
	 * exist yet when this logs. */
	uint32_t reas = nrf_power_resetreas_get(NRF_POWER);

	nrf_power_resetreas_clear(NRF_POWER, reas);
	LOG_INF("power: reset reason 0x%08x%s", reas,
		(reas & NRF_POWER_RESETREAS_OFF_MASK) ? " (button wake)" : "");

	if (!gpio_is_ready_dt(&btn)) {
		LOG_ERR("power: button GPIO not ready — no way to power off");
		return -ENODEV;
	}

	/* return pin first, so the input never sees a floating other side */
	int err = gpio_pin_configure_dt(&btn_gnd, GPIO_OUTPUT_INACTIVE);

	if (err) {
		LOG_ERR("power: button return config failed (%d)", err);
		return err;
	}

	err = gpio_pin_configure_dt(&btn, GPIO_INPUT);
	if (err) {
		LOG_ERR("power: button config failed (%d)", err);
		return err;
	}

	k_timer_start(&poll_timer, K_MSEC(PWR_POLL_MS), K_MSEC(PWR_POLL_MS));
	return 0;
}

/* `off` shell command: same path as the hold, for bench use before a button
 * is wired. The board wakes on the button, USB plug-in or RESET. */
static int cmd_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "entering System OFF; button, USB or RESET wakes");
	atomic_set(&force_off, 1);
	return 0;
}
SHELL_CMD_REGISTER(off, NULL, "Power off (nRF System OFF)", cmd_off);
