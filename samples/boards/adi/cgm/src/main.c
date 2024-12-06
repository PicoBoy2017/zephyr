/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/services/cgms.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

#include "services/accs.h"
#include "services/ecs_monitor.h"
#include "led_ind/led_ind.h"
#include "database.h"

LOG_MODULE_REGISTER(adi_cgm, LOG_LEVEL_INF);

/* Next glucose reading */
static uint16_t glucose_concentration;

/* Bluetooth declarations */
#define FIXED_PASSKEY 555555
static struct bt_conn *active_conn;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_CGMS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection failed (err %u)", err);
		return;
	}

	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Connected: %s", addr);

	active_conn = bt_conn_ref(conn);

	bt_conn_set_security(conn, BT_SECURITY_L4);
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason %u)", reason);

	set_led_state(LED_IND_SEARCHING);

	bt_conn_unref(active_conn);
	active_conn = NULL;
}

static void on_recycled(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_WRN("Advertising failed to start (err %d)", err);
	}
}

/* Callback structs/definitions */
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = on_connected,
	.disconnected = on_disconnected,
	.recycled = on_recycled,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Auth cancelled %s", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.cancel = auth_cancel,
};

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	set_led_state(LED_IND_CONNECTED);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);

	LOG_INF("Pairing failed with %s (reason %u)", addr, reason);
}

static struct bt_conn_auth_info_cb auth_cb_info = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

/* Glucose Concetration Getter Callback */
static int get_glucose(uint16_t *glucose_conc)
{
	*glucose_conc = glucose_concentration;
	return 0;
}

static int init_bt(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return err;
	}

#ifdef CONFIG_BT_FIXED_PASSKEY
	err = bt_passkey_set(FIXED_PASSKEY);
	if (err) {
		LOG_ERR("Failed to set fixed passkey!");
		return err;
	}
#endif

	err = bt_conn_auth_cb_register(&auth_cb_display);
	if (err) {
		LOG_ERR("Failed to register auth display callback!");
		return err;
	}

	err = bt_conn_auth_info_cb_register(&auth_cb_info);
	if (err) {
		LOG_ERR("Failed to register auth info callback!");
		return err;
	}

	err = bt_set_name(CONFIG_BT_DEVICE_NAME);
	if (err) {
		LOG_ERR("Unable to set device name!");
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

	return 0;
}

static int init_sensors(void)
{
	int err;

	err = device_is_ready(adxl367_dev);
	if (!err) {
		LOG_INF("sensor: adxl367_dev not ready. (err %d)", err);
		return -1;
	}

	err = device_is_ready(max30123_dev);
	if (!err) {
		LOG_INF("sensor: max30123_dev not ready. (err %d)", err);
		return -1;
	}

	return 0;
}

static int init_peripherals(void)
{
	int err;

	err = init_sensors();
	if (err) {
		return err;
	}

	err = init_bt();
	if (err) {
		return err;
	}

	return 0;
}

int main(void)
{
	int err;

	err = k_thread_name_set(k_current_get(), "main");
	if (err) {
		LOG_ERR("Failed to set thread name");
		return err;
	}

	/* 1. Initialize peripherals */
	err = init_peripherals();
	if (err) {
		LOG_ERR("Failed initializing pheripherals");
		return -1;
	}
	set_led_state(LED_IND_DEV_READY);

	/* 2. Initialize services */
	init_ecs_monitor_thread(&glucose_concentration);

	struct bt_cgms_callbacks cgms_callbacks = {
		.get_glucose_conc_cb = get_glucose,
		.add_record_cb = db_add_record,
		.get_record_cb = db_get_record,
	};

	err = bt_cgms_init(cgms_callbacks);
	if (err) {
		LOG_ERR("Unable to initialize CGMS (err %d)", err);
		return err;
	}

	/* 3. Set LED indicator to search */
	set_led_state(LED_IND_SEARCHING);
}
