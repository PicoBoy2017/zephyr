/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DATABASE_H_
#define DATABASE_H_

/*
 * Example implementation of a user-defined database to store CGM measurements.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/services/cgms.h>

int db_add_record(const cgm_measurement_t record);
int db_get_record(cgm_measurement_t *record, uint16_t record_idx);

#endif /* DATABASE_H_ */
