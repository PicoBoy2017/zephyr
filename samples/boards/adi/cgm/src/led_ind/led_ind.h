/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LED_INDICATION_H_
#define LED_INDICATION_H_

/** @file
 *  @brief LED indicator process
 */

#ifdef __cplusplus
extern "C" {
#endif

enum led_state {
	LED_IND_OFF,
	LED_IND_DEV_READY,
	LED_IND_SEARCHING,
	LED_IND_CONNECTED,
	LED_IND_SLEEP
};

void set_led_state(enum led_state new_state);

#ifdef __cplusplus
}
#endif

#endif /* LED_INDICATION_H_ */
