/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include "accs.h"

LOG_MODULE_REGISTER(accs, LOG_LEVEL_INF);

/* Thread definitions */
#define ACCS_MONITOR_INTERVAL_MS 250
#define ACCS_THREAD_PRIO         3
#define ACCS_THREAD_STACK_SIZE   1200
struct k_thread accs_thread;
K_THREAD_STACK_DEFINE(accs_stack, ACCS_THREAD_STACK_SIZE);
K_SEM_DEFINE(accs_sem, 0, 1);

static void interval_timer_expire(struct k_timer *timer)
{
	k_sem_give(&accs_sem);
}

K_TIMER_DEFINE(accs_timer, interval_timer_expire, NULL);

struct accelerometer {
	double x;
	double y;
	double z;
};

static bool notify;
static struct accelerometer accel;

static void update_accel(void)
{
	/* Val for XYZ-axis */
	struct sensor_value val[3];

	if (sensor_sample_fetch(adxl367_dev)) {
		LOG_ERR("Failed to fetch data");
		return;
	}

	if (sensor_channel_get(adxl367_dev, SENSOR_CHAN_ACCEL_XYZ, val)) {
		LOG_ERR("Failed to get data");
		return;
	}

	accel.x = sensor_value_to_double(&val[0]);
	accel.y = sensor_value_to_double(&val[1]);
	accel.z = sensor_value_to_double(&val[2]);
}

static ssize_t handle_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset);
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
BT_GATT_SERVICE_DEFINE(accs_svc, BT_GATT_PRIMARY_SERVICE(BT_ACCS),
		       BT_GATT_CHARACTERISTIC(BT_ACCS_VAL, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
					      BT_GATT_PERM_READ_ENCRYPT, handle_read, NULL, NULL),
		       BT_GATT_CCC(ccc_cfg_changed,
				   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT));

static ssize_t handle_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			   uint16_t len, uint16_t offset)
{
	update_accel();
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &accel, sizeof(accel));
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notify = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
	LOG_DBG("Notifications %s", notify ? "enabled" : "disabled");
}

static void accs_monitor(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_timer_start(&accs_timer, K_MSEC(ACCS_MONITOR_INTERVAL_MS),
		      K_MSEC(ACCS_MONITOR_INTERVAL_MS));

	while (1) {

		k_sem_take(&accs_sem, K_FOREVER);

		if (notify) {

			/* Get sensor data */
			update_accel();

			/* Notify central*/
			bt_gatt_notify(NULL, &accs_svc.attrs[1], &accel, sizeof(accel));

			LOG_INF("X: %f, Y: %f, Z: %f", accel.x, accel.y, accel.z);
		}
	}
}

int init_accs_monitor_thread(void)
{
	k_thread_create(&accs_thread, accs_stack, ACCS_THREAD_STACK_SIZE, accs_monitor, NULL, NULL,
			NULL, ACCS_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&accs_thread, "accs_monitor");

	return 0;
}

SYS_INIT(init_accs_monitor_thread, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
