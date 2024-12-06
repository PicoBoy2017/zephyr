/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "led_ind.h"

#define LED_NODE DT_ALIAS(led)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

LOG_MODULE_REGISTER(adi_cgm_led_ind, LOG_LEVEL_INF);

/* Thread definitions */
#define POLL_SLEEP_MS     1000
#define THREAD_PRIO       5
#define THREAD_STACK_SIZE 300
struct k_thread led_thread;
K_THREAD_STACK_DEFINE(stack_area, THREAD_STACK_SIZE);

#define QUICK_BLINK_DELAY       40
#define DOUBLE_BLINK_DELAY      80
#define DOUBLE_BLINK_DELAY_LONG 900

static enum led_state state;

void set_led_state(enum led_state new_state)
{
	state = new_state;
}

static void led_state_off(void)
{
	if (gpio_pin_set_dt(&led, 0)) {
		LOG_ERR("Error setting LED");
	}
}

static void led_state_dev_ready(void)
{
	if (gpio_pin_set_dt(&led, 0)) {
		LOG_ERR("Error setting LED");
	}
}

static void led_state_searching(void)
{
	/* Double blink routine */
	if (gpio_pin_toggle_dt(&led)) {
		LOG_ERR("Error setting LED");
	}
	k_sleep(K_MSEC(DOUBLE_BLINK_DELAY));
	if (gpio_pin_toggle_dt(&led)) {
		LOG_ERR("Error setting LED");
	}
	k_sleep(K_MSEC(DOUBLE_BLINK_DELAY));
	if (gpio_pin_toggle_dt(&led)) {
		LOG_ERR("Error setting LED");
	}
	k_sleep(K_MSEC(DOUBLE_BLINK_DELAY));
	if (gpio_pin_toggle_dt(&led)) {
		LOG_ERR("Error setting LED");
	}
	k_sleep(K_MSEC(DOUBLE_BLINK_DELAY_LONG));
}

static void led_state_connected(void)
{
	if (gpio_pin_set_dt(&led, 1)) {
		LOG_ERR("Error setting LED");
	}
}

static void led_state_sleeping(void)
{
	if (gpio_pin_set_dt(&led, 0)) {
		LOG_ERR("Error setting LED");
	}
	k_sleep(K_MSEC(DOUBLE_BLINK_DELAY_LONG));
}

static void led_startup(void)
{
	for (int i = 0; i < 12; i++) {
		if (gpio_pin_toggle_dt(&led)) {
			LOG_ERR("Error setting LED");
		}
		k_sleep(K_MSEC(QUICK_BLINK_DELAY));
	}
}

static void run_led(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	enum led_state curr_state = state;

	while (1) {

		/* If we switch LED states, reset LED before starting new subroutine */
		if (curr_state != state) {
			if (gpio_pin_set_dt(&led, 0)) {
				LOG_ERR("Error setting LED");
			}
		}

		switch (state) {
		case LED_IND_OFF:
			led_state_off();
			break;
		case LED_IND_DEV_READY:
			led_state_dev_ready();
			break;
		case LED_IND_SEARCHING:
			led_state_searching();
			break;
		case LED_IND_CONNECTED:
			led_state_connected();
			break;
		case LED_IND_SLEEP:
			led_state_sleeping();
			break;
		default:
			led_state_off();
			break;
		}
		k_sleep(K_MSEC(POLL_SLEEP_MS));
	}
}

int init_led_thread(void)
{
	if (!gpio_is_ready_dt(&led)) {
		LOG_ERR("led not ready!");
		return -1;
	}

	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE)) {
		LOG_ERR("Error configuring LED");
		return -1;
	}

	if (gpio_pin_set_dt(&led, 0)) {
		LOG_ERR("Error setting LED");
		return -1;
	}

	/* Startup LED sequence */
	led_startup();

	k_thread_create(&led_thread, stack_area, THREAD_STACK_SIZE, run_led, NULL, NULL, NULL,
			THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&led_thread, "led_ind");

	return 0;
}

SYS_INIT(init_led_thread, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
