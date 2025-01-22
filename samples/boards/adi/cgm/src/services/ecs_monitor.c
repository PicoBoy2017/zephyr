/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/max30123.h>

#include "ecs_monitor.h"

LOG_MODULE_REGISTER(ecs_monitor, LOG_LEVEL_INF);

/*
 * This is the thread where you will define your process of analyzing and
 * making determinations based on data returned from the electrochemical sensor.
 *
 * Saving a value to the glucose_concentration pointer will allow the CGM Service
 * to send the data to the connected client.
 */

#define ECS_MONITOR_INTERVAL_MS 3000
#define ECS_MONITOR_PRIO        3
#define ECS_STACK_SIZE          600
struct k_thread ecs_monitor_thread;
K_THREAD_STACK_DEFINE(ecs_monitor_stack_area, ECS_STACK_SIZE);
K_SEM_DEFINE(ecs_sem, 0, 1);

static void interval_timer_expire(struct k_timer *timer)
{
	k_sem_give(&ecs_sem);
}

K_TIMER_DEFINE(ecs_timer, interval_timer_expire, NULL);

/* This is where you will calculate the glucose concentration */
static void calculate_glocose_concentration(uint16_t *glucose_concentration,
					    struct sensor_value val[2])
{
	/* Arbitrary example */
	static uint16_t baseline = 5;
	uint16_t base = (uint16_t)(val[0].val1 + val[1].val1) / 2;
	uint16_t scale = base / 4200;

	*glucose_concentration = (uint16_t)(baseline * scale);
}

static void monitor_ecs(void *p1, void *p2, void *p3)
{
	int err;
	struct sensor_value val[2];
	uint16_t *glucose_concentration = (uint16_t *)p1;

	k_timer_start(&ecs_timer, K_MSEC(ECS_MONITOR_INTERVAL_MS), K_MSEC(ECS_MONITOR_INTERVAL_MS));

	while (1) {

		k_sem_take(&ecs_sem, K_FOREVER);

		err = sensor_sample_fetch(max30123_dev);
		if (err) {
			LOG_ERR("Failed to fetch (err: %d)", err);
		}

		err = sensor_channel_get(max30123_dev, SENSOR_CHAN_ECS_SEQUENCER_1, &val[0]);
		if (err) {
			LOG_ERR("Failed to get sequencer 1 data (err: %d)", err);
		}

		err = sensor_channel_get(max30123_dev, SENSOR_CHAN_ECS_SEQUENCER_2, &val[1]);
		if (err) {
			LOG_ERR("Failed to get sequencer 2 data (err: %d)", err);
		}

		LOG_DBG("Sequencer 1: %d", val[0].val1);
		LOG_DBG("Sequencer 2: %d", val[1].val1);

		calculate_glocose_concentration(glucose_concentration, val);
	}
}

void init_ecs_monitor_thread(uint16_t *glucose_concentration)
{
	k_thread_create(&ecs_monitor_thread, ecs_monitor_stack_area, ECS_STACK_SIZE, monitor_ecs,
			glucose_concentration, NULL, NULL, ECS_MONITOR_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&ecs_monitor_thread, "ecs_monitor");
}
