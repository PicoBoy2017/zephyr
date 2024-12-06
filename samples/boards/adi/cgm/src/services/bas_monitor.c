/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/services/bas.h>

LOG_MODULE_REGISTER(bas_monitor, LOG_LEVEL_INF);

/* Thread definitions */
#define BAS_MONITOR_INTERVAL_MS 60000
#define BAS_THREAD_PRIO         11
#define BAS_THREAD_STACK_SIZE   700
struct k_thread bas_thread;
K_THREAD_STACK_DEFINE(bas_stack, BAS_THREAD_STACK_SIZE);
K_SEM_DEFINE(bas_sem, 0, 1);

static void interval_timer_expire(struct k_timer *timer)
{
	k_sem_give(&bas_sem);
}

K_TIMER_DEFINE(bas_timer, interval_timer_expire, NULL);

static uint8_t bas_level = 100;

static void bas_monitor(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_timer_start(&bas_timer, K_MSEC(BAS_MONITOR_INTERVAL_MS), K_MSEC(BAS_MONITOR_INTERVAL_MS));

	while (1) {

		k_sem_take(&bas_sem, K_FOREVER);

		bas_level--;
		if (bas_level == 0) {
			bas_level = 100;
		}

		if (bt_bas_set_battery_level(bas_level)) {
			LOG_ERR("Failed to update battery level");
		}
	}
}

int init_bas_monitor_thread(void)
{
	k_thread_create(&bas_thread, bas_stack, BAS_THREAD_STACK_SIZE, bas_monitor, NULL, NULL,
			NULL, BAS_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&bas_thread, "bas_monitor");

	return 0;
}

SYS_INIT(init_bas_monitor_thread, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
