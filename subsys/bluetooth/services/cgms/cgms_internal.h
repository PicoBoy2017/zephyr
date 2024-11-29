/** @file
 *  @brief CGM Internal header file
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_CGMS_INTERNAL_H_
#define BT_CGMS_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/cgms.h>

#define UNKNOWN_TIME_ZONE  -128
#define UNKNOWN_DST_OFFSET 255

typedef struct __packed cgm_feature_t {
	uint8_t cgm_feature_field[3];
	uint8_t cgm_type_sample_location_field;
	uint16_t e2e_crc;
} cgm_feature_t;

typedef struct __packed cgm_status_t {
	/* Time Offset in minutes since Session Start Time - UINT16 */
	uint16_t time_offset;
	/* CGM status */
	uint8_t cgm_status[3];
} cgm_status_t;

struct __packed cgm_time {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
};

typedef struct __packed cgm_session_start_time_t {
	/* Session start timestamp */
	struct cgm_time session_start;
	/* Time zone*/
	uint8_t time_zone;
	/* Daylight savings time offset */
	uint8_t dst_offset;
} cgm_session_start_time_t;

extern struct bt_cgms_callbacks cgms_cb;

int notify_measurement(const cgm_measurement_t *record);
int racp_indicate_conn(struct bt_conn *conn, const struct bt_gatt_attr *attr, uint8_t *buf,
		       uint16_t len);
int socp_indicate_conn(struct bt_conn *conn, const struct bt_gatt_attr *attr, uint8_t *buf,
		       uint16_t len);
uint8_t get_cgms_interval(void);
void set_cgms_interval(uint8_t new_interval);

#ifdef __cplusplus
}
#endif

#endif /* BT_CGMS_INTERNAL_H_ */
