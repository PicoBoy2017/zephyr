/** @file
 *  @brief CGM Record Access Control Point Characterstic
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/services/cgms.h>

#include "cgms_racp.h"
#include "cgms_db.h"
#include "cgms_internal.h"

LOG_MODULE_REGISTER(bt_cgms_racp, CONFIG_BT_CGMS_LOG_LEVEL);

/* RACP Protocol Macros */
#define RACP_OPCODE_POS      0
#define RACP_OPERATOR_POS    1
#define RACP_FILTER_TYPE_POS 2
#define RACP_FILTER_VAL_POS  3

/* RACP Worker Configuration */
void worker_work_fn(struct k_work *work);
K_WORK_DEFINE(racp_work, worker_work_fn);
K_THREAD_STACK_DEFINE(worker_workq_stack, CONFIG_BT_CGMS_RACP_STACK_SIZE);
static struct k_work_q racp_work_q;
K_SEM_DEFINE(worker_kill_sem, 0, 1);
K_SEM_DEFINE(worker_timeout_sem, 0, 1);

static void worker_timeout_handler(struct k_timer *timer)
{
	k_sem_give(&worker_timeout_sem);
}

K_TIMER_DEFINE(racp_worker_timer, worker_timeout_handler, NULL);

/* Holds active RACP worker request */
static struct bt_gatt_attr *active_attr;
static uint8_t active_request[RACP_MAX_PACKET_SIZE];
static uint16_t active_request_len;

typedef enum racp_opcode_t {
	REPORT_STORED_RECORDS = 0x01,
	ABORT_OPERATION = 0x03,
	REPORT_NUMBER_RECORDS = 0x04,
	NUMBER_RECORDS_RESPONSE = 0x05,
	RESPONSE_CODE = 0x06,
} racp_opcode_t;

typedef enum racp_operator_filter_t {
	NULL_OPERATOR = 0x00,
	ALL_RECORDS = 0x01,
	GREATER_EQUAL = 0x03,
} racp_operator_filter_t;

typedef enum racp_filter_type_t {
	TIME_OFFSET = 0x01,
} racp_filter_type_t;

typedef enum racp_response_code_t {
	RESP_SUCCESS = 0x01,
	RESP_OP_NOT_SUPPORTED = 0x02,
	RESP_INVALID_OPERATOR = 0x03,
	RESP_OPERATOR_NOT_SUPPORTED = 0x04,
	RESP_INVALID_OPERAND = 0x05,
	RESP_NO_RECORDS_FOUND = 0x06,
	RESP_ABORT_UNSUCCESSFUL = 0x07,
	RESP_PROC_NOT_COMPLETED = 0x08,
	RESP_OPERAND_NOT_SUPPORTED = 0x09,
	RESP_SERVER_BUSY = 0x0A,
} racp_response_code_t;

static void build_response(racp_opcode_t opcode, racp_response_code_t code,
			   racp_response_t *response)
{
	response->response_code = RESPONSE_CODE;
	response->oper = NULL_OPERATOR;
	response->data = (opcode << 8) | code;
}

static void transmit_queried_records(uint8_t opcode, uint8_t oper, uint16_t idx, uint16_t len,
				     racp_response_t *response)
{
	int rc;
	cgm_measurement_t record;

	for (int i = idx; i < idx + len; i++) {

		/* Check if worker has been killed */
		if (!k_sem_take(&worker_kill_sem, K_NO_WAIT)) {
			k_sem_give(&worker_timeout_sem);
			build_response(opcode, RESP_PROC_NOT_COMPLETED, response);
			return;
		}

		rc = get_record(&record, i);
		if (rc) {
			LOG_ERR("Failed to retrieve record at index %d", i);
			build_response(opcode, RESP_NO_RECORDS_FOUND, response);
			return;
		}

		rc = notify_measurement(&record);
		if (rc) {
			LOG_ERR("Failed to notify central of record at index %d", i);
			build_response(opcode, RESP_PROC_NOT_COMPLETED, response);
			return;
		}
	}

	build_response(opcode, RESP_SUCCESS, response);
}

/*
 * To transmit records to the central, we first determine what index we want to start transmitting
 * from, then transmit the records one by one. This is to simplify the interface between the user
 * defined record database and the RACP characteristic.
 */
static void report_stored_records(uint8_t *request, uint16_t req_len, racp_response_t *response)
{
	int err = 0;
	uint8_t opcode = request[RACP_OPCODE_POS];
	uint8_t oper = request[RACP_OPERATOR_POS];
	uint16_t idx;
	uint16_t len;

	/* Based on the request, fill the starting and ending indices for record lookup */
	switch (oper) {
	case ALL_RECORDS:
		if (req_len != REPORT_NUMBER_RECORDS_MIN_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, response);
			err = -1;
			break;
		}

		/* Start from record idx=0 and transmit the entire database */
		idx = 0;
		len = get_number_of_records();

		break;

	case GREATER_EQUAL:
		if (req_len != REPORT_NUMBER_RECORDS_MAX_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, response);
			err = -1;
			break;
		}

		uint8_t filter_type = request[RACP_FILTER_TYPE_POS];

		if (filter_type != TIME_OFFSET) {
			build_response(opcode, RESP_OPERAND_NOT_SUPPORTED, response);
			err = -1;
			break;
		}

		uint8_t filter_val = request[RACP_FILTER_VAL_POS];

		err = get_index_greater_or_equal_to_timestamp(filter_val, &idx, &len);
		if (err) {
			build_response(opcode, RESP_NO_RECORDS_FOUND, response);
			break;
		}

		break;

	default:
		build_response(opcode, RESP_OPERATOR_NOT_SUPPORTED, response);
		err = -1;
		break;
	}

	/* If query successful, transmit the records to central */
	if (!err) {
		transmit_queried_records(opcode, oper, idx, len, response);
	}

	/* Transmit filled response */
	racp_indicate_conn(NULL, active_attr, (uint8_t *)response, sizeof(racp_response_t));
}

static void report_number_of_records(uint8_t *request, uint16_t req_len, racp_response_t *response)
{
	int rc;
	uint8_t opcode = request[RACP_OPCODE_POS];
	uint8_t oper = request[RACP_OPERATOR_POS];
	uint16_t idx;
	uint16_t len;

	switch (oper) {
	case ALL_RECORDS:
		if (req_len != REPORT_NUMBER_RECORDS_MIN_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, response);
			return;
		}

		len = get_number_of_records();

		break;

	case GREATER_EQUAL:
		if (req_len != REPORT_NUMBER_RECORDS_MAX_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, response);
			return;
		}

		uint8_t filter_type = request[RACP_FILTER_TYPE_POS];

		if (filter_type != TIME_OFFSET) {
			build_response(opcode, RESP_OPERAND_NOT_SUPPORTED, response);
			return;
		}

		uint8_t filter_val = request[RACP_FILTER_VAL_POS];

		rc = get_index_greater_or_equal_to_timestamp(filter_val, &idx, &len);
		if (rc) {
			build_response(opcode, RESP_NO_RECORDS_FOUND, response);
			return;
		}

		break;

	default:
		build_response(opcode, RESP_OPERATOR_NOT_SUPPORTED, response);
		return;
	}

	/* Fill response */
	response->response_code = NUMBER_RECORDS_RESPONSE;
	response->oper = NULL_OPERATOR;
	response->data = len;
}

void worker_work_fn(struct k_work *work)
{
	racp_response_t response;

	report_stored_records(active_request, active_request_len, &response);
}

int handle_racp_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *req_buf,
			uint16_t req_len)
{
	if (req_len < RACP_MIN_PACKET_SIZE) {
		LOG_ERR("Request buffer of invalid size");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	racp_response_t response;
	uint8_t *request = (uint8_t *)req_buf;
	uint8_t opcode = request[RACP_OPCODE_POS];

	switch (opcode) {
	case REPORT_STORED_RECORDS:
		if (req_len < REPORT_STORED_RECORDS_MIN_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, &response);
			break;
		}

		/* Check if worker is running */
		bool busy = k_work_busy_get(&racp_work);

		if (busy) {
			build_response(opcode, RESP_SERVER_BUSY, &response);
			break;
		}

		active_attr = (struct bt_gatt_attr *)attr;
		memcpy(&active_request, request, req_len);
		active_request_len = req_len;

		/*
		 * Launch worker and immediately return. The worker
		 * is responsible for tranmitting response code
		 */
		k_work_submit_to_queue(&racp_work_q, &racp_work);

		return 0;

	case ABORT_OPERATION:
		if (req_len != ABORT_OPERATION_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, &response);
			break;
		}

		/* Check if worker is already idling */
		bool working = k_work_is_pending(&racp_work);

		if (!working) {
			build_response(opcode, RESP_ABORT_UNSUCCESSFUL, &response);
			break;
		}

		/* Signal worker to close */
		k_sem_give(&worker_kill_sem);

		/* Start timeout timer */
		k_timer_start(&racp_worker_timer,
			      K_MSEC(CONFIG_BT_CGMS_RACP_WORKER_ABORT_TIMEOUT_MS), K_NO_WAIT);

		/* Wait for timeout to trigger or worker to signal closed */
		k_sem_take(&worker_timeout_sem, K_FOREVER);

		k_timer_stop(&racp_worker_timer);

		/* Check if worker received kill signal */
		int aborted = k_sem_take(&worker_kill_sem, K_NO_WAIT);

		if (aborted) {
			build_response(opcode, RESP_SUCCESS, &response);
			break;
		}

		/* Abort the thread and reset resource */
		k_work_cancel(&racp_work);
		build_response(opcode, RESP_ABORT_UNSUCCESSFUL, &response);
		break;

	case REPORT_NUMBER_RECORDS:
		if (req_len < REPORT_NUMBER_RECORDS_MIN_SIZE) {
			build_response(opcode, RESP_INVALID_OPERAND, &response);
			break;
		}
		report_number_of_records(request, req_len, &response);
		break;

	default:
		build_response(opcode, RESP_OP_NOT_SUPPORTED, &response);
		break;
	};

	/* Send command response */
	return racp_indicate_conn(NULL, attr, (uint8_t *)&response, sizeof(racp_response_t));
}

void racp_init(void)
{
	k_work_queue_init(&racp_work_q);
	const struct k_work_queue_config cfg = {
		.name = "CGMS RACP WQ",
	};
	k_work_queue_start(&racp_work_q, worker_workq_stack,
			   K_THREAD_STACK_SIZEOF(worker_workq_stack),
			   CONFIG_BT_CGMS_RACP_WORKER_PRIO, &cfg);
	k_work_init(&racp_work, worker_work_fn);
}
