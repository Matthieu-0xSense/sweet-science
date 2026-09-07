/*
 * Boxe_AI BLE GATT service. UUIDs and payloads per protocol.md.
 * Notify-only characteristics: IMU stream, punch events, status.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/shell/shell.h>
#include <string.h>
#include "boxe.h"

LOG_MODULE_REGISTER(boxe_ble, LOG_LEVEL_INF);

#define UUID_BASE(n) BT_UUID_128_ENCODE(0x6f8e000##n, 0xb5a3, 0x4f39, \
					0xb0c4, 0x2ae94a2c5e01)

static const struct bt_uuid_128 uuid_svc = BT_UUID_INIT_128(UUID_BASE(1));
static const struct bt_uuid_128 uuid_imu = BT_UUID_INIT_128(UUID_BASE(2));
static const struct bt_uuid_128 uuid_evt = BT_UUID_INIT_128(UUID_BASE(3));
static const struct bt_uuid_128 uuid_sta = BT_UUID_INIT_128(UUID_BASE(5));

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("ccc %u", value);
}

BT_GATT_SERVICE_DEFINE(boxe_svc,
	BT_GATT_PRIMARY_SERVICE(&uuid_svc),
	/* [1] IMU stream */
	BT_GATT_CHARACTERISTIC(&uuid_imu.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* [4] punch events */
	BT_GATT_CHARACTERISTIC(&uuid_evt.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* [7] status */
	BT_GATT_CHARACTERISTIC(&uuid_sta.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

#define ATTR_IMU (&boxe_svc.attrs[1])
#define ATTR_EVT (&boxe_svc.attrs[4])
#define ATTR_STA (&boxe_svc.attrs[7])

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, UUID_BASE(1)),
};

/*
 * The name lives in a static buffer because the scan-response data is rebuilt
 * on every advertising restart, not just at init.
 */
static char node_name[16];
static struct bt_data sd[1];

/*
 * A connectable advertiser stops the moment a central connects and is not
 * resumed for us — the implicit restart the old BT_LE_ADV_OPT_ONE_TIME
 * semantics gave us is gone. Without an explicit restart the node advertises
 * exactly once per boot and goes invisible after the first host disconnects.
 *
 * Deferred to the work queue for two reasons. bt_le_adv_start() must not run
 * from the BT RX thread that delivers the disconnected callback; and the
 * connection object is still held when that callback fires, so a connectable
 * advertiser started too early gets -ENOMEM — there is no free conn for it to
 * be connected on. That is a race, not a permanent failure: it fires on one
 * board and not another. Retry with backoff instead of giving up, or the node
 * goes dark for the rest of the session.
 */
#define ADV_RETRY_MS      100
#define ADV_RETRY_MAX     20

static int adv_retries;

static void adv_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	if (err == 0 || err == -EALREADY) {
		adv_retries = 0;
		LOG_INF("advertising as %s", node_name);
		return;
	}

	if (err == -ENOMEM && ++adv_retries <= ADV_RETRY_MAX) {
		/* conn not released yet — come back shortly */
		k_work_schedule(k_work_delayable_from_work(work),
				K_MSEC(ADV_RETRY_MS));
		return;
	}

	LOG_ERR("advertising restart failed (%d) after %d retries",
		err, adv_retries);
	adv_retries = 0;
}

static K_WORK_DELAYABLE_DEFINE(adv_work, adv_work_handler);

static void connected(struct bt_conn *conn, uint8_t err)
{
	LOG_INF("host connected (err %u)", err);
	/* request 2M PHY + short interval for stream throughput (v1) */
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("host disconnected (0x%02x)", reason);
	adv_retries = 0;
	k_work_schedule(&adv_work, K_MSEC(ADV_RETRY_MS));
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* Last failure from init/restart, reported by the `ble` shell command. The
 * boot log is unreachable on this board: the console is USB CDC, which only
 * exists after enumeration, so anything logged during init is already gone by
 * the time a terminal attaches. */
static int last_err;
static const char *last_stage = "not started";

int ble_service_init(const char *name)
{
	int err = bt_enable(NULL);
	if (err) {
		last_err = err;
		last_stage = "bt_enable";
		return err;
	}
	bt_set_name(name);

	strncpy(node_name, name, sizeof(node_name) - 1);
	sd[0] = (struct bt_data)BT_DATA(BT_DATA_NAME_COMPLETE, node_name,
					strlen(node_name));

	/* FAST_1: 30-60 ms advertising interval — the node should be picked up
	 * quickly when the athlete straps it on. */
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		last_err = err;
		last_stage = "bt_le_adv_start";
		return err;
	}
	last_stage = "advertising";
	LOG_INF("advertising as %s", node_name);
	return 0;
}

/*
 * `ble` shell command: report what init managed and retry advertising. The
 * retry matters as much as the report — if bt_enable() succeeded but the
 * advertiser did not start, this recovers the node without a reflash.
 */
static void count_conn(struct bt_conn *conn, void *data)
{
	ARG_UNUSED(conn);
	(*(int *)data)++;
}

static int cmd_ble(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int conns = 0;

	bt_conn_foreach(BT_CONN_TYPE_LE, count_conn, &conns);

	shell_print(sh, "name       : %s", node_name[0] ? node_name : "(unset)");
	shell_print(sh, "bt_is_ready: %s", bt_is_ready() ? "yes" : "no");
	shell_print(sh, "connections: %d (max %d)", conns, CONFIG_BT_MAX_CONN);
	shell_print(sh, "last stage : %s", last_stage);
	shell_print(sh, "last err   : %d", last_err);

	if (!bt_is_ready()) {
		int err = bt_enable(NULL);

		shell_print(sh, "bt_enable retry -> %d", err);
		if (err) {
			return 0;
		}
		bt_set_name(node_name);
	}

	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
				  sd, ARRAY_SIZE(sd));
	shell_print(sh, "bt_le_adv_start -> %d%s", err,
		    err == -EALREADY ? " (already advertising)"
		    : err == -ENOMEM ? " (no free conn — still connected?)" : "");
	if (!err || err == -EALREADY) {
		last_stage = "advertising";
	}
	return 0;
}
SHELL_CMD_REGISTER(ble, NULL, "Report BLE state and retry advertising", cmd_ble);

void ble_notify_imu(const struct imu_packet *pkt)
{
	(void)bt_gatt_notify(NULL, ATTR_IMU, pkt, sizeof(*pkt));
}

void ble_notify_event(const struct event_packet *pkt)
{
	(void)bt_gatt_notify(NULL, ATTR_EVT, pkt, sizeof(*pkt));
}

void ble_notify_status(const struct status_packet *pkt)
{
	(void)bt_gatt_notify(NULL, ATTR_STA, pkt, sizeof(*pkt));
}
