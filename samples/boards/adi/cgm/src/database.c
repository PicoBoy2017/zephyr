/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "database.h"

LOG_MODULE_REGISTER(database, CONFIG_BT_SERVICE_LOG_LEVEL);

#define MAX_NUM_RECORDS 400

static cgm_measurement_t database[MAX_NUM_RECORDS] = {0};
static uint16_t curr_idx;

int db_add_record(const cgm_measurement_t record)
{
	if (curr_idx >= MAX_NUM_RECORDS) {
		LOG_ERR("Database full, cannot add record");
		return -ENOMEM;
	}

	database[curr_idx] = record;
	curr_idx++;

	return 0;
}

int db_get_record(cgm_measurement_t *record, uint16_t record_idx)
{
	if (record_idx > curr_idx) {
		LOG_ERR("Record index: %d out of bounds > %d", record_idx, curr_idx);
		return -EINVAL;
	}

	*record = database[record_idx];

	return 0;
}
