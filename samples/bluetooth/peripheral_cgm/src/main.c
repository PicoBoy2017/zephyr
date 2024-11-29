/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/cgms.h>

#include "database.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_CGMS_VAL))};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected: %s", addr);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);

	if (bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd))) {
		LOG_WRN("Advertising failed to start");
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

/*
 * This function is called every CONFIG_BT_CGMS_INTERVAL minutes.
 *
 * The CGMS thread will request for a glucose sample to be recorded to
 * data store as well as sent as notification to central.
 *
 * A user-app may query the RACP to retrieve stored glucose samples.
 */
static int get_glucose(uint16_t *glucose_conc)
{
	if (!glucose_conc) {
		return -EINVAL;
	}

	/* Placholder value */
	static uint16_t glucose_concentration = 0xBEEF;

	*glucose_conc = glucose_concentration;

	return 0;
}

static int init_bt(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return -1;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		if (settings_load()) {
			return -1;
		}
	}

	if (bt_conn_auth_cb_register(&auth_cb_display)) {
		return -1;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_WRN("Advertising failed to start (err %d)", err);
		return -1;
	}

	return 0;
}

int main(void)
{
	int err;

	LOG_INF("Starting CGM Service Sample");

	if (init_bt()) {
		LOG_ERR("Bluetooth init failed");
		return -1;
	}

	/* Register cgm callback functions */
	struct bt_cgms_callbacks cgms_callbacks = {
		.get_glucose_conc_cb = get_glucose,
		.add_record_cb = db_add_record,
		.get_record_cb = db_get_record,
	};

	/* Launch cgm service */
	err = bt_cgms_init(cgms_callbacks);
	if (err) {
		LOG_ERR("Unable to initialize CGMS");
		return -1;
	}

	return 0;
}
