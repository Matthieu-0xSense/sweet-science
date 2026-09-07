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
static const struct bt_uuid_128 uuid_raw = BT_UUID_INIT_128(UUID_BASE(4));
static const struct bt_uuid_128 uuid_sta = BT_UUID_INIT_128(UUID_BASE(5));

static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	LOG_INF("ccc %u", value);
}

/* --- raw capture ------------------------------------------------------- */

/* 32 packets = 640 ms of slack between the 1 kHz producer and the radio.
 * Eight (160 ms) was not enough in practice: with both nodes streaming, the
 * Windows adapter interleaves the two links unevenly and one of them stalls
 * for hundreds of milliseconds at a time, which cost 30 % of a capture. The
 * buffer is 6.6 kB of a 256 kB part, and a host that stops draining entirely
 * is still reported as loss rather than absorbed as latency. */
#define RAW_QUEUE_DEPTH   32
#define RAW_TX_RETRIES    50
#define RAW_TX_RETRY_MS   2

/* A notification carries at most MTU-3 bytes. Exceeding it does not fail
 * loudly, it silently truncates every packet in the capture, so catch a
 * raised RAW_BATCH here rather than in a week-old recording. */
BUILD_ASSERT(sizeof(struct raw_packet) <= CONFIG_BT_L2CAP_TX_MTU - 3,
	     "raw_packet exceeds the ATT notification payload — lower "
	     "RAW_BATCH or raise CONFIG_BT_L2CAP_TX_MTU");

K_MSGQ_DEFINE(raw_q, sizeof(struct raw_packet), RAW_QUEUE_DEPTH, 4);

static atomic_t raw_on;
static uint16_t raw_seq;
static uint16_t raw_pending_lost;   /* dropped since the last packet queued */
static uint32_t raw_sent, raw_lost;

static void raw_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool on = (value == BT_GATT_CCC_NOTIFY);

	if (on == (atomic_get(&raw_on) != 0)) {
		return;
	}
	if (on) {
		/* start clean: stale packets from a previous capture would
		 * land at the head of the new file with the old timebase */
		k_msgq_purge(&raw_q);
		raw_seq = 0;
		raw_pending_lost = 0;
		raw_sent = raw_lost = 0;
	}
	atomic_set(&raw_on, on ? 1 : 0);
	LOG_INF("raw capture %s", on ? "ON" : "off");
}

bool ble_raw_enabled(void)
{
	return atomic_get(&raw_on) != 0;
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
	/* [10] raw 1 kHz capture — subscribing to it is what turns it on.
	 * Appended after status rather than in UUID order so the existing
	 * attribute indices below stay put. */
	BT_GATT_CHARACTERISTIC(&uuid_raw.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(raw_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

#define ATTR_IMU (&boxe_svc.attrs[1])
#define ATTR_EVT (&boxe_svc.attrs[4])
#define ATTR_STA (&boxe_svc.attrs[7])
#define ATTR_RAW (&boxe_svc.attrs[10])

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
	if (err) {
		return;
	}

	/*
	 * Raw capture needs ~10.4 kB/s per node, which the 1M PHY at a default
	 * 30-50 ms interval will not carry. Ask for 2M and a 15-30 ms interval
	 * with room for several packets per event. Both are requests: the
	 * central is free to refuse, so nothing here is load-bearing — a
	 * refusal shows up as loss on the `raw` counters, not as a failure.
	 */
	(void)bt_conn_le_phy_update(conn, BT_CONN_LE_PHY_PARAM_2M);
	(void)bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(12, 24, 0, 400));
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("host disconnected (0x%02x)", reason);
	/* the CCC is cleared for an unbonded peer anyway; be explicit so a
	 * capture cannot survive the host that asked for it */
	atomic_set(&raw_on, 0);
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

/*
 * Raw capture producer side. Called from the 1 kHz sample path, so it must
 * not block: a full queue drops the packet and the loss is folded into the
 * next one that fits, which keeps the gap visible in the capture file rather
 * than letting the sample loop stall behind the radio.
 */
void ble_raw_submit(struct raw_packet *pkt)
{
	pkt->seq = raw_seq++;
	pkt->lost = raw_pending_lost;

	if (k_msgq_put(&raw_q, pkt, K_NO_WAIT) != 0) {
		raw_seq--;               /* this packet never existed */
		if (raw_pending_lost < UINT16_MAX) {
			raw_pending_lost++;
		}
		raw_lost++;
		return;
	}
	raw_pending_lost = 0;
}

/*
 * Consumer side, on its own thread rather than the system workqueue: a notify
 * that has to wait for buffers would otherwise block the 1 kHz sample work
 * queued behind it, and a stalled sample loop is worse than a dropped packet.
 *
 * bt_gatt_notify() returns -ENOMEM when the controller's tx buffers are full,
 * which at 50 packets/s is normal back-pressure rather than an error — retry
 * across a few connection events before giving up on the packet.
 */
static void raw_tx_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	struct raw_packet pkt;

	while (1) {
		k_msgq_get(&raw_q, &pkt, K_FOREVER);

		int err = bt_gatt_notify(NULL, ATTR_RAW, &pkt, sizeof(pkt));

		/* Retry only while nothing is waiting behind this packet. Once
		 * the queue is backing up, spending 100 ms on one packet just
		 * converts a single hole into several. */
		for (int i = 0; i < RAW_TX_RETRIES &&
				(err == -ENOMEM || err == -EAGAIN) &&
				k_msgq_num_used_get(&raw_q) == 0; i++) {
			k_sleep(K_MSEC(RAW_TX_RETRY_MS));
			err = bt_gatt_notify(NULL, ATTR_RAW, &pkt, sizeof(pkt));
		}

		if (err == 0) {
			raw_sent++;
		} else {
			/* No counter to bump on this side: the packet already
			 * carries a seq, so the host sees the hole as a gap in
			 * the sequence. raw_pending_lost belongs to the
			 * producer thread alone and stays there. */
			raw_lost++;
		}
	}
}
K_THREAD_DEFINE(raw_tx_tid, 1024, raw_tx_thread, NULL, NULL, NULL,
		K_PRIO_PREEMPT(5), 0, 0);

/*
 * `raw` shell command: is capture running, and is the radio keeping up. The
 * loss counter is the one that matters — a sweep fitted on a capture with
 * holes in it fits the holes.
 */
static int cmd_raw(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "capture   : %s", ble_raw_enabled() ? "ON" : "off");
	shell_print(sh, "packets   : %u sent, %u lost", raw_sent, raw_lost);
	shell_print(sh, "queued    : %u/%u", k_msgq_num_used_get(&raw_q),
		    RAW_QUEUE_DEPTH);
	shell_print(sh, "rate      : %d B/packet, %d packets/s when running",
		    (int)sizeof(struct raw_packet), SAMPLE_HZ / RAW_BATCH);
	shell_print(sh, "enable by subscribing to the raw characteristic "
			"(boxe_host.py --raw)");
	return 0;
}
SHELL_CMD_REGISTER(raw, NULL, "Report raw 1 kHz capture state", cmd_raw);
