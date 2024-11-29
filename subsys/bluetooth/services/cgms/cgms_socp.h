/** @file
 *  @brief CGM Specific Ops Control Point Header File
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_CGMS_SOCP_H_
#define BT_CGMS_SOCP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#define SOCP_MIN_PACKET_SIZE 1

int handle_socp_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *req_buf,
			uint16_t req_len);

#ifdef __cplusplus
}
#endif

#endif /* BT_CGMS_SOCP_H_ */
