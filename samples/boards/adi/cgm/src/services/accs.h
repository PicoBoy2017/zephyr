/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_ACCEL_SERVICE_H_
#define BT_ACCEL_SERVICE_H_

/** @file
 *  @brief Accelerometer Service (ACCS) sample
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/uuid.h>

#define ADXL367_NODE DT_NODELABEL(adxl367)
static const struct device *const adxl367_dev = DEVICE_DT_GET(ADXL367_NODE);

/* 128 bit UUIDs for GATT service and characteristics */
#define BT_UUID_ACCS     BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0xadadadadad02)
#define BT_UUID_ACCS_VAL BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0xadadadadad03)

#define BT_ACCS     BT_UUID_DECLARE_128(BT_UUID_ACCS)
#define BT_ACCS_VAL BT_UUID_DECLARE_128(BT_UUID_ACCS_VAL)

#ifdef __cplusplus
}
#endif

#endif /* BT_ACCEL_SERVICE_H_ */
