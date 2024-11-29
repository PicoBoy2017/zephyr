/** @file
 *  @brief CGM Specific Ops Control Point Header File
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "cgms_socp.h"
#include "cgms_internal.h"

LOG_MODULE_REGISTER(bt_cgms_socp, CONFIG_BT_CGMS_LOG_LEVEL);

#define SOCP_OPCODE_POS   0
#define SOCP_INTERVAL_POS 1

#define SET_CGM_INTERVAL_SIZE 2
#define GET_CGM_INTERVAL_SIZE 1

typedef enum socp_opcode_t {
	SET_CGM_INTERVAL = 0x01,
	GET_CGM_INTERVAL = 0x02,
	CGM_INTERVAL_RESPONSE = 0x03,
	RESPONSE_CODE = 0x1C,
} socp_opcode_t;

typedef enum socp_response_code_t {
	RESP_SUCCESS = 0x01,
	RESP_OP_NOT_SUPPORTED = 0x02,
	RESP_INVALID_OPERAND = 0x03,
	RESP_PROC_NOT_COMPLETED = 0x04,
	RESP_PARAM_OUT_OF_RANGE = 0x05,
} socp_response_code_t;

typedef struct __packed socp_response_t {
	uint8_t response_code;
	uint16_t data;
} socp_response_t;

static void build_response(socp_opcode_t opcode, socp_response_code_t code,
			   socp_response_t *response, uint16_t *response_len)
{
	response->response_code = RESPONSE_CODE;
	response->data = (opcode << 8) | code;
	*response_len = sizeof(socp_response_t);
}

static void build_interval_response(uint8_t interval, socp_response_t *response,
				    uint16_t *response_len)
{
	response->response_code = CGM_INTERVAL_RESPONSE;
	response->data = interval;
	*response_len = 2;
}

int handle_socp_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *req_buf,
			uint16_t req_len)
{
	if (req_len < SOCP_MIN_PACKET_SIZE) {
		LOG_ERR("Request buffer of invalid size");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	socp_response_t response;
	uint16_t response_len = 0;
	uint8_t *request = (uint8_t *)req_buf;
	uint8_t opcode = request[SOCP_OPCODE_POS];

	switch (opcode) {
	case SET_CGM_INTERVAL:
		if (req_len != SET_CGM_INTERVAL_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, &response, &response_len);
			break;
		}

		uint8_t new_interval = request[SOCP_INTERVAL_POS];

		set_cgms_interval(new_interval);
		build_response(opcode, RESP_SUCCESS, &response, &response_len);
		break;

	case GET_CGM_INTERVAL:
		if (req_len != GET_CGM_INTERVAL_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, &response, &response_len);
			break;
		}
		build_interval_response(get_cgms_interval(), &response, &response_len);
		break;

	default:
		build_response(opcode, RESP_OP_NOT_SUPPORTED, &response, &response_len);
		break;
	};

	return socp_indicate_conn(NULL, attr, (uint8_t *)&response, response_len);
}
