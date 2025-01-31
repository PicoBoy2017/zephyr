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
static void calculate_glucose_concentration(uint16_t *glucose_concentration,
					    struct sensor_value val)
{
	/* Arbitrary example */
	uint16_t current = (uint16_t)((val.val1 * 128) /
				      65536); /* Current calculation from MAX30123 datasheet */

	static const uint16_t min_in = 56; /* Minimum input in nA */
	static const uint16_t max_in = 60; /* Maximum input in nA */
	static const uint16_t input_range = max_in - min_in;

	static const uint16_t min_out = 72;  /* Minimum output in mg/dL */
	static const uint16_t max_out = 100; /* Maximum output in mg/dL */
	static const uint16_t output_range = max_out - min_out;

	/* Scale the current to in-range glucose values */
	uint16_t measurement = min_out + ((current - min_in) * output_range) / input_range;

	LOG_DBG("glucose_concentration: %dmg/dL", measurement);

	*glucose_concentration = measurement;
}

static void monitor_ecs(void *p1, void *p2, void *p3)
{
	int err;
	struct sensor_value val;
	uint16_t *glucose_concentration = (uint16_t *)p1;

	k_timer_start(&ecs_timer, K_MSEC(ECS_MONITOR_INTERVAL_MS), K_MSEC(ECS_MONITOR_INTERVAL_MS));

	while (1) {

		k_sem_take(&ecs_sem, K_FOREVER);

		err = sensor_sample_fetch(max30123_dev);
		if (err) {
			LOG_ERR("Failed to fetch (err: %d)", err);
		}

		err = sensor_channel_get(max30123_dev, SENSOR_CHAN_ECS_SEQUENCER_1, &val);
		if (err) {
			LOG_ERR("Failed to get sequencer 1 data (err: %d)", err);
		}

		calculate_glucose_concentration(glucose_concentration, val);
	}
}

void init_ecs_monitor_thread(uint16_t *glucose_concentration)
{
	k_thread_create(&ecs_monitor_thread, ecs_monitor_stack_area, ECS_STACK_SIZE, monitor_ecs,
			glucose_concentration, NULL, NULL, ECS_MONITOR_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&ecs_monitor_thread, "ecs_monitor");
}
