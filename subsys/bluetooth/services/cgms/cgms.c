/** @file
 *  @brief CGM Service
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/cgms.h>

#include "cgms_db.h"
#include "cgms_racp.h"
#include "cgms_socp.h"
#include "cgms_internal.h"

LOG_MODULE_REGISTER(cgms, CONFIG_BT_CGMS_LOG_LEVEL);

static uint64_t cgm_start_time_ms; /* Internally used for calculate time-offset of measurements */
static uint16_t cgm_expected_runtime = CONFIG_BT_CGMS_EXPECTED_RUNTIME;
const static cgm_feature_t cgm_feature = {
	.cgm_feature_field = {0},
	.cgm_type_sample_location_field =
		(CONFIG_BT_CGMS_TYPE << 4) | CONFIG_BT_CGMS_SAMPLE_LOCATION,
	.e2e_crc = 0xFFFF, /* When CRC not supported, default is 0xFFFF */
};

static cgm_session_start_time_t session_start_time;

/* Notification and indication variables */
static bool cgm_notify;
static bool racp_indicate;
static bool socp_indicate;
static bool racp_indicating;
static bool socp_indicating;

static struct k_thread cgms_thread_data;
K_THREAD_STACK_DEFINE(cgms_stack_area, CONFIG_BT_CGMS_STACK_SIZE);
K_SEM_DEFINE(cgms_interval_sem, 0, 1);

static void interval_timer_expire(struct k_timer *timer)
{
	k_sem_give(&cgms_interval_sem);
}

K_TIMER_DEFINE(cgm_interval_timer, interval_timer_expire, NULL);
static uint8_t cgm_interval = CONFIG_BT_CGMS_INTERVAL;

struct bt_cgms_callbacks cgms_cb;

/* Glucose Service Declaration */
static void cgms_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void racp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void cgms_socp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t cgms_feature_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset);
static ssize_t cgms_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset);
static ssize_t cgms_start_time_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    void *buf, uint16_t len, uint16_t offset);
static ssize_t cgms_start_time_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				     const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t cgms_runtime_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset);
static ssize_t cgm_racp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t cgms_socp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
BT_GATT_SERVICE_DEFINE(
	cgms_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_CGMS),
	BT_GATT_CHARACTERISTIC(BT_UUID_CGM_MEASUREMENT, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,
			       NULL, NULL, NULL),
	BT_GATT_CCC(cgms_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_CGM_FEATURE, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_AUTHEN,
			       cgms_feature_read, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_CGM_STATUS, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_AUTHEN,
			       cgms_status_read, NULL, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_CGM_SESSION_START_TIME,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN,
			       cgms_start_time_read, cgms_start_time_write, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_CGM_SESSION_RUN_TIME, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ_AUTHEN, cgms_runtime_read, NULL,
			       &cgm_expected_runtime),
	BT_GATT_CHARACTERISTIC(BT_UUID_RECORD_ACCESS_CONTROL_POINT,
			       BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE_AUTHEN, NULL, cgm_racp_write, NULL),
	BT_GATT_CCC(racp_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN),
	BT_GATT_CHARACTERISTIC(BT_UUID_CGM_SPECIFIC_OPS_CONTROL_POINT,
			       BT_GATT_CHRC_INDICATE | BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE_AUTHEN, NULL, cgms_socp_write, NULL),
	BT_GATT_CCC(cgms_socp_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_AUTHEN));

static uint16_t get_cgm_uptime(void)
{
	return (uint16_t)((k_uptime_get() - cgm_start_time_ms) / MSEC_PER_MIN);
}

int notify_measurement(const cgm_measurement_t *record)
{
	int err;

	if (cgm_notify) {

		err = bt_gatt_notify(NULL, &cgms_svc.attrs[1], record, sizeof(cgm_measurement_t));

		if (err) {
			LOG_ERR("Unable to transmit CGM notification (err: %d)", err);
			return err;
		}
	}

	return 0;
}

static int record_measurement(const cgm_measurement_t *record)
{
	int err;

	LOG_DBG("Recording CGM Measurement - Sequence: 0x%x - Conc: %dmg/dL", record->time_offset,
		record->glucose_conc);

	err = add_record(record);
	if (err) {
		LOG_ERR("Error adding record to DB. Not notifying!");
		return err;
	}

	err = notify_measurement(record);
	if (err) {
		return err;
	}

	return 0;
}

/* Can be called for manual measurement recording */
int bt_cgms_record_measurement(uint16_t glucose_conc)
{
	int err;

	const cgm_measurement_t cgm_packet = {
		.size = 6,
		.flags = 0x00,
		.glucose_conc = glucose_conc,
		.time_offset = get_cgm_uptime(),
	};

	err = record_measurement(&cgm_packet);
	if (err) {
		return err;
	}

	return 0;
}

/* Periodic thread to take measurment and record result */
static void bt_cgms_interval_thread(void)
{
	int err;
	uint16_t glucose_conc;

	/* set initial timer to expire every cgm_interval minutes*/
	set_cgms_interval(cgm_interval);

	while (1) {

		/* Wait until timer expiry */
		k_sem_take(&cgms_interval_sem, K_FOREVER);

		/* Get measurement from sensor */
		err = cgms_cb.get_glucose_conc_cb(&glucose_conc);
		if (err) {
			LOG_ERR("Failed to retrieve glucose concentration");
			continue;
		}

		const cgm_measurement_t cgm_packet = {
			.size = 6,
			.flags = 0x00,
			.glucose_conc = glucose_conc,
			.time_offset = get_cgm_uptime(),
		};

		record_measurement(&cgm_packet);
	}
}

static ssize_t cgms_feature_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &cgm_feature, sizeof(cgm_feature));
}

static ssize_t cgms_status_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
{
	const cgm_status_t status_packet = {
		.time_offset = get_cgm_uptime(), .cgm_status = {0}
		/* CGM status mirror SSA which is not supported */
	};

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &status_packet,
				 sizeof(status_packet));
}

static ssize_t cgms_start_time_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != sizeof(session_start_time)) {
		LOG_ERR("Malformed Session Start Time Packet Received. len: %d != "
			"sizeof(session_start_time): %d",
			len, sizeof(session_start_time));
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&session_start_time, buf, sizeof(session_start_time));

	return len;
}

static ssize_t cgms_start_time_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				    void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &session_start_time,
				 sizeof(session_start_time));
}

static ssize_t cgms_runtime_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
				 sizeof(attr->user_data));
}

static void racp_indicate_destroy(struct bt_gatt_indicate_params *params)
{
	racp_indicating = false;
}

static void socp_indicate_destroy(struct bt_gatt_indicate_params *params)
{
	socp_indicating = false;
}

int racp_indicate_conn(struct bt_conn *conn, const struct bt_gatt_attr *attr, uint8_t *buf,
		       uint16_t len)
{
	static struct bt_gatt_indicate_params ind_params;

	if (racp_indicate) {

		if (racp_indicating) {
			return 0;
		}

		ind_params.attr = attr;
		ind_params.destroy = racp_indicate_destroy;
		ind_params.data = buf;
		ind_params.len = len;

		if (bt_gatt_indicate(conn, &ind_params) == 0) {
			racp_indicating = true;
		}
	}

	return 0;
}

static ssize_t cgm_racp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len < RACP_MIN_PACKET_SIZE) {
		LOG_ERR("Malformed RACP Packet Received. len: %d < RACP_MIN_PACKET_SIZE: %d", len,
			RACP_MIN_PACKET_SIZE);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	return handle_racp_request(conn, attr, buf, len);
}

int socp_indicate_conn(struct bt_conn *conn, const struct bt_gatt_attr *attr, uint8_t *buf,
		       uint16_t len)
{
	static struct bt_gatt_indicate_params ind_params;

	if (socp_indicate) {

		if (socp_indicating) {
			return 0;
		}

		ind_params.attr = attr;
		ind_params.destroy = socp_indicate_destroy;
		ind_params.data = buf;
		ind_params.len = len;

		if (bt_gatt_indicate(conn, &ind_params) == 0) {
			socp_indicating = true;
		}
	}

	return 0;
}

static ssize_t cgms_socp_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len < SOCP_MIN_PACKET_SIZE) {
		LOG_ERR("Malformed SOCP Packet Received. len: %d < SOCP_MIN_PACKET_SIZE: %d", len,
			SOCP_MIN_PACKET_SIZE);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	return handle_socp_request(conn, attr, buf, len);
}

static void cgms_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_DBG("CGMS notifications %s", notif_enabled ? "enabled" : "disabled");

	cgm_notify = notif_enabled ? 1 : 0;
}

static void racp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool indicate_enabled = (value == BT_GATT_CCC_INDICATE);

	LOG_DBG("RACP indications %s", indicate_enabled ? "enabled" : "disabled");

	racp_indicate = indicate_enabled ? 1 : 0;
}

static void cgms_socp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	bool indicate_enabled = (value == BT_GATT_CCC_INDICATE);

	LOG_DBG("CGMS SOCP indications %s", indicate_enabled ? "enabled" : "disabled");

	socp_indicate = indicate_enabled ? 1 : 0;
}

static void cgms_notify_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg3);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg1);

	bt_cgms_interval_thread();
}

void set_cgms_interval(uint8_t interval)
{
	cgm_interval = interval;
	k_timer_start(&cgm_interval_timer, K_MINUTES(interval), K_MINUTES(interval));
}

uint8_t get_cgms_interval(void)
{
	return cgm_interval;
}

int bt_cgms_register_callbacks(struct bt_cgms_callbacks callbacks)
{
	if ((callbacks.get_glucose_conc_cb == NULL) || (callbacks.add_record_cb == NULL) ||
	    (callbacks.get_record_cb == NULL)) {
		LOG_ERR("required callback not set");
		return -EINVAL;
	}
	cgms_cb.get_glucose_conc_cb = callbacks.get_glucose_conc_cb;
	cgms_cb.add_record_cb = callbacks.add_record_cb;
	cgms_cb.get_record_cb = callbacks.get_record_cb;

	return 0;
}

int bt_cgms_init(struct bt_cgms_callbacks callbacks)
{
	int err;

	cgm_start_time_ms = k_uptime_get();

	/* Time unknown at init */
	struct cgm_time time_start = {0};

	session_start_time.session_start = time_start;
	session_start_time.dst_offset = UNKNOWN_DST_OFFSET;
	session_start_time.time_zone = UNKNOWN_TIME_ZONE;
	racp_indicating = false;
	socp_indicating = false;

	err = bt_cgms_register_callbacks(callbacks);

	if (err) {
		LOG_ERR("Unable to register callbacks");
		return err;
	}

	racp_init();

	k_thread_create(&cgms_thread_data, cgms_stack_area, K_THREAD_STACK_SIZEOF(cgms_stack_area),
			cgms_notify_thread, NULL, NULL, NULL, CONFIG_BT_CGMS_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&cgms_thread_data, "CGMS");

	return 0;
}
