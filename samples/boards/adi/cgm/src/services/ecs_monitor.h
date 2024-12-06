/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ECS_MONITOR_H_
#define ECS_MONITOR_H_

/** @file
 *  @brief Electrochemical Monitor Thread
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#define MAX30123_NODE DT_NODELABEL(max30123)
static const struct device *const max30123_dev = DEVICE_DT_GET(MAX30123_NODE);

void init_ecs_monitor_thread(uint16_t *glucose_concentration);

#ifdef __cplusplus
}
#endif

#endif /* ECS_MONITOR_H_ */
