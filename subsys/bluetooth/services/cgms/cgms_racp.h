/** @file
 *  @brief CGM Record Access Control Point Characterstic Header File
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_CGMS_RACP_H_
#define BT_CGMS_RACP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#define RACP_MIN_PACKET_SIZE 2
#define RACP_MAX_PACKET_SIZE 4

#define ABORT_OPERATION_SIZE           2
#define REPORT_NUMBER_RECORDS_MIN_SIZE 2
#define REPORT_NUMBER_RECORDS_MAX_SIZE 4
#define REPORT_STORED_RECORDS_MIN_SIZE 2
#define REPORT_STORED_RECORDS_MAX_SIZE 4

typedef struct __packed racp_response_t {
	uint8_t response_code;
	uint8_t oper;
	uint16_t data;
} racp_response_t;

int handle_racp_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *req_buf,
			uint16_t req_len);
void racp_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BT_CGMS_RACP_H_ */
