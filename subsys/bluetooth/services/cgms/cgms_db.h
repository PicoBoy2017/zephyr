/** @file
 *  @brief CGM Database Header File
 */

/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_CGMS_DB_H_
#define BT_CGMS_DB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

#include "cgms_internal.h"

int add_record(const cgm_measurement_t *record);
int get_record(cgm_measurement_t *record, uint16_t record_idx);
int get_index_greater_or_equal_to_timestamp(uint16_t timestamp, uint16_t *idx_start,
					    uint16_t *list_len);
uint16_t get_number_of_records(void);

#ifdef __cplusplus
}
#endif

#endif /* BT_CGMS_DB_H_ */
