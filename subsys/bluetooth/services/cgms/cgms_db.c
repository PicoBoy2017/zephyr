/** @file
 *  @brief CGM Database
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/services/cgms.h>

#include "cgms_db.h"
#include "cgms_internal.h"

LOG_MODULE_REGISTER(bt_cgms_db, CONFIG_BT_CGMS_LOG_LEVEL);

K_MUTEX_DEFINE(db_mutex);

static uint16_t num_records;

int add_record(const cgm_measurement_t *record)
{
	k_mutex_lock(&db_mutex, K_FOREVER);

	int ret = cgms_cb.add_record_cb(*record);

	if (!ret) {
		num_records++;
	}

	k_mutex_unlock(&db_mutex);

	return ret;
}

int get_record(cgm_measurement_t *record, uint16_t record_idx)
{
	if (record_idx > num_records) {
		return -EINVAL;
	}

	k_mutex_lock(&db_mutex, K_FOREVER);

	int ret = cgms_cb.get_record_cb(record, record_idx);

	k_mutex_unlock(&db_mutex);

	return ret;
}

int get_index_greater_or_equal_to_timestamp(uint16_t timestamp, uint16_t *idx_start,
					    uint16_t *list_len)
{
	k_mutex_lock(&db_mutex, K_FOREVER);

	cgm_measurement_t temp_record;

	for (int16_t i = 0; i < num_records; i++) {
		get_record(&temp_record, i);
		if (temp_record.time_offset >= timestamp) {
			*idx_start = (uint16_t)i;
			*list_len = num_records - i;

			k_mutex_unlock(&db_mutex);
			return 0;
		}
	}

	/* Return error code if no records match filter */
	k_mutex_unlock(&db_mutex);
	return -ENODATA;
}

uint16_t get_number_of_records(void)
{
	return num_records;
}
