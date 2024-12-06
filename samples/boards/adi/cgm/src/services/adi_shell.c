/*
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdlib.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(adi_shell, LOG_LEVEL_DBG);

#define NUM_LEDS 1

#define ADI_SHELL_PEEK_NUM_ARGS  4
#define ADI_SHELL_POKE_NUM_ARGS  5
#define ADI_SHELL_POWER_NUM_ARGS 3
#define ADI_SHELL_LED_NUM_ARGS   3
#define ADI_SHELL_MAX_ARGS       5

#define ADI_SHELL_PEEK_HELP                                                                        \
	"Read the value of a peripheral register\n"                                                \
	"    Usage: peek <spi|i2c> <interface-id (int)> <register-addr (hex)>"
#define ADI_SHELL_POKE_HELP                                                                        \
	"Set the value of a peripheral register\n"                                                 \
	"    Usage: poke <spi|i2c> <interface-id (int)> <register-addr (hex)> <value (8-bit hex)>"
#define ADI_SHELL_POWER_HELP                                                                       \
	"Set the power mode of device\n"                                                           \
	"    Usage: power <power-mode> <duration-in-seconds>"
#define ADI_SHELL_LED_HELP                                                                         \
	"Set the state of an LED\n"                                                                \
	"    Usage: led <led-num> <on|off>"

#define ADI_SHELL_MAX_RESPONSE_SIZE 512
char ret_buf[ADI_SHELL_MAX_RESPONSE_SIZE];
K_MUTEX_DEFINE(ret_buf_mutex);

static const struct spi_dt_spec spi_dev =
	SPI_DT_SPEC_GET(DT_NODELABEL(max30123), SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);
static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(DT_NODELABEL(adxl367));

#define I2C_READ         0x01u
#define I2C_REG_READ(x)  (((x & 0xFF) << 1) | I2C_READ)
#define I2C_REG_WRITE(x) ((x & 0xFF) << 1)

#define ERR_RESPONSE         "ERROR: ADI Shell error processing command"
#define LED_OUT_OF_RANGE     "ERROR: LED parameter out of range"
#define LED_STATE_ERROR      "ERROR: LED state must be 'on' or 'off'"
#define POWER_MODE_ERROR     "ERROR: Chosen power mode invalid"
#define POWER_DURATION_ERROR "ERROR: Power duration out of range"
#define P_INTERFACE_ERROR    "ERROR: Interface must be 'spi' or 'i2c'"
#define P_ID_ERROR           "ERROR: Interface ID out of range"
#define P_REG_ERROR          "ERROR: Register address out of range"

#define MAX_SLEEP_DURATION_SEC 60

static struct gpio_dt_spec leds[NUM_LEDS] = {GPIO_DT_SPEC_GET(DT_ALIAS(led), gpios)};

static int spi_access(const struct spi_buf *buf, uint8_t len, bool read)
{
	struct spi_buf_set tx = {.buffers = buf};

	if (read) {
		const struct spi_buf_set rx = {.buffers = buf, .count = len};

		tx.count = len - 1;
		return spi_transceive_dt(&spi_dev, &tx, &rx);
	}

	tx.count = len;
	return spi_write_dt(&spi_dev, &tx);
}

static int peek_spi(uint8_t spi_id, uint8_t reg, uint8_t *ret)
{
	LOG_INF("%s dev: %d, reg: 0x%x", __func__, spi_id, reg);

	uint8_t max30123_spi_read = 0x80u;
	uint8_t max30123_spi_len = 3;

	/* Allocate spi buffer struct */
	const struct spi_buf buf[3] = {{.buf = &reg, .len = 1},
				       {.buf = &max30123_spi_read, .len = 1},
				       {.buf = ret, .len = 1}};

	return spi_access(buf, max30123_spi_len, true);
}

static int poke_spi(uint8_t spi_dev, uint8_t reg, uint8_t val)
{
	LOG_INF("%s dev: %d, reg: 0x%x, val: %d", __func__, spi_dev, reg, val);

	uint8_t max30123_spi_write = 0x00u;
	uint8_t max30123_spi_len = 3;

	/* Allocate spi buffer struct */
	const struct spi_buf buf[3] = {{.buf = &reg, .len = 1},
				       {.buf = &max30123_spi_write, .len = 1},
				       {.buf = &val, .len = 1}};

	return spi_access(buf, max30123_spi_len, false);
}

static int i2c_bus_access(uint8_t reg, void *data, size_t length)
{
	if ((reg & I2C_READ) != 0) {
		return i2c_burst_read_dt(&i2c_dev, reg >> 1, (uint8_t *)data, length);
	}

	if (length != 1) {
		return -EINVAL;
	}

	return i2c_reg_write_byte_dt(&i2c_dev, reg >> 1, *(uint8_t *)data);
}

static int peek_i2c(uint8_t i2c_dev, uint8_t reg, uint8_t *ret)
{
	LOG_INF("%s dev: %d, reg: 0x%x", __func__, i2c_dev, reg);

	return i2c_bus_access(I2C_REG_READ(reg), ret, 1);
}

static int poke_i2c(uint8_t i2c_dev, uint8_t reg, uint8_t val)
{
	LOG_INF("%s dev: %d, reg: 0x%x, val: %d", __func__, i2c_dev, reg, val);

	return i2c_bus_access(I2C_REG_WRITE(reg), &val, 1);
}

int adi_shell_peek(char **argv, char resp[ADI_SHELL_MAX_RESPONSE_SIZE])
{
	uint8_t interface_id = 0;
	uint8_t reg_addr = 0;
	uint8_t val = 0;

	/* Interface_id must be below uint8_t*/
	if (atoi(argv[2]) > 255) {
		strncpy(resp, P_ID_ERROR, strlen(P_ID_ERROR) + 1);
		return 0;
	}

	interface_id = (uint8_t)atoi(argv[2]);

	/* Register address must be below uint8_t */
	if (strtol(argv[3], NULL, 16) > 255) {
		strncpy(resp, P_REG_ERROR, strlen(P_REG_ERROR) + 1);
		return 0;
	}

	reg_addr = (uint8_t)strtol(argv[3], NULL, 16);

	/* Run the peek */
	if (!strcmp(argv[1], "i2c")) {
		if (peek_i2c(interface_id, reg_addr, &val)) {
			strncpy(resp, ERR_RESPONSE, strlen(ERR_RESPONSE) + 1);
			return 0;
		}
	} else if (!strcmp(argv[1], "spi")) {
		if (peek_spi(interface_id, reg_addr, &val)) {
			strncpy(resp, ERR_RESPONSE, strlen(ERR_RESPONSE) + 1);
			return 0;
		}
	} else {
		strncpy(resp, P_INTERFACE_ERROR, strlen(P_INTERFACE_ERROR) + 1);
		return 0;
	}

	/* Success response */
	snprintf(ret_buf, ADI_SHELL_MAX_RESPONSE_SIZE, "%s@%d[0x%02X] = 0x%02X", argv[1],
		 interface_id, reg_addr, val);

	return 0;
}

int adi_shell_poke(char **argv, char resp[ADI_SHELL_MAX_RESPONSE_SIZE])
{
	uint8_t interface_id = 0;
	uint8_t reg_addr = 0;
	uint8_t val = 0;

	if (atoi(argv[2]) > 255) {
		strncpy(resp, P_ID_ERROR, strlen(P_ID_ERROR) + 1);
		return 0;
	}

	interface_id = (uint8_t)atoi(argv[2]);

	if (strtol(argv[3], NULL, 16) > 255) {
		strncpy(resp, P_REG_ERROR, strlen(P_REG_ERROR) + 1);
		return 0;
	}

	reg_addr = (uint8_t)strtol(argv[3], NULL, 16);

	if (strtol(argv[4], NULL, 16) > 255) {
		strncpy(resp, P_REG_ERROR, strlen(P_REG_ERROR) + 1);
		return 0;
	}

	val = (uint8_t)strtol(argv[4], NULL, 16);

	/* Run the poke */
	if (!strcmp(argv[1], "i2c")) {
		if (poke_i2c(interface_id, reg_addr, val)) {
			strncpy(resp, ERR_RESPONSE, strlen(ERR_RESPONSE) + 1);
			return 0;
		}
	} else if (!strcmp(argv[1], "spi")) {
		if (poke_spi(interface_id, reg_addr, val)) {
			strncpy(resp, ERR_RESPONSE, strlen(ERR_RESPONSE) + 1);
			return 0;
		}
	} else {
		strncpy(resp, P_INTERFACE_ERROR, strlen(P_INTERFACE_ERROR) + 1);
		return 0;
	}

	/* Success response */
	snprintf(ret_buf, ADI_SHELL_MAX_RESPONSE_SIZE, "%s@%d[0x%02X] updated to 0x%02X", argv[1],
		 interface_id, reg_addr, val);

	return 0;
}

int adi_shell_power(char **argv, char resp[ADI_SHELL_MAX_RESPONSE_SIZE])
{
	if (strcmp(argv[1], "sleep")) {
		strncpy(resp, POWER_MODE_ERROR, strlen(POWER_MODE_ERROR) + 1);
		return 0;
	}

	int duration = atoi(argv[2]);

	if (duration < -1 || duration > MAX_SLEEP_DURATION_SEC) {
		strncpy(resp, POWER_DURATION_ERROR, strlen(POWER_DURATION_ERROR) + 1);
		return 0;
	}

	/* Success response */
	if (duration == -1) {
		snprintf(ret_buf, ADI_SHELL_MAX_RESPONSE_SIZE, "Entering %s indefinitely...",
			 argv[1]);
	} else {
		snprintf(ret_buf, ADI_SHELL_MAX_RESPONSE_SIZE, "Entering %s for %d seconds...",
			 argv[1], duration);
	}

	/* TODO: Trigger deep-sleep */

	return 0;
}

int adi_shell_led(char **argv, char resp[ADI_SHELL_MAX_RESPONSE_SIZE])
{
	int led = atoi(argv[1]);

	if (led < 0 || led >= NUM_LEDS) {
		strncpy(resp, LED_OUT_OF_RANGE, strlen(LED_OUT_OF_RANGE) + 1);
		return 0;
	}

	bool state;

	if (!strcmp(argv[2], "on")) {
		state = true;
	} else if (!strcmp(argv[2], "off")) {
		state = false;
	} else {
		strncpy(resp, LED_STATE_ERROR, strlen(LED_STATE_ERROR) + 1);
		return 0;
	}

	gpio_pin_set_dt(&leds[led], state);

	/* Success response */
	snprintf(ret_buf, ADI_SHELL_MAX_RESPONSE_SIZE, "led-%d: %s", led, state ? "on" : "off");

	return 0;
}

static int adi_shell_init(void)
{
	int err;

	/* Initialize LEDs */
	for (int ledn = 0; ledn < NUM_LEDS; ledn++) {
		struct gpio_dt_spec led = leds[ledn];

		if (!gpio_is_ready_dt(&led)) {
			LOG_ERR("ERRPR: led-%d not ready", ledn);
			return -1;
		}

		err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		if (err) {
			LOG_ERR("ERROR: led-%d failed to configure", ledn);
			return err;
		}

		err = gpio_pin_toggle_dt(&led);
		if (err) {
			LOG_ERR("ERROR: led-%d failed to toggle", ledn);
			return err;
		}

		err = (!device_is_ready(spi_dev.bus));
		if (err) {
			LOG_ERR("ERROR: spi not ready (err %d)", err);
		}

		err = (!device_is_ready(i2c_dev.bus));
		if (err) {
			LOG_ERR("ERROR: i2c not ready (err %d)", err);
		}
	}

	return 0;
}

static int cmd_peek(const struct shell *sh, size_t argc, char **argv)
{
	k_mutex_lock(&ret_buf_mutex, K_FOREVER);
	if (adi_shell_peek(argv, ret_buf)) {
		shell_print(sh, "Error: Peek command failed");
		k_mutex_unlock(&ret_buf_mutex);
		return -1;
	}

	shell_print(sh, "%s", ret_buf);
	k_mutex_unlock(&ret_buf_mutex);
	return 0;
}

static int cmd_poke(const struct shell *sh, size_t argc, char **argv)
{
	k_mutex_lock(&ret_buf_mutex, K_FOREVER);
	if (adi_shell_poke(argv, ret_buf)) {
		shell_print(sh, "Error: Poke command failed");
		k_mutex_unlock(&ret_buf_mutex);
		return -1;
	}

	shell_print(sh, "%s", ret_buf);
	k_mutex_unlock(&ret_buf_mutex);
	return 0;
}

static int cmd_power(const struct shell *sh, size_t argc, char **argv)
{
	k_mutex_lock(&ret_buf_mutex, K_FOREVER);
	if (adi_shell_power(argv, ret_buf)) {
		shell_print(sh, "Error: Power command failed");
		k_mutex_unlock(&ret_buf_mutex);
		return -1;
	}

	shell_print(sh, "%s", ret_buf);
	k_mutex_unlock(&ret_buf_mutex);
	return 0;
}

static int cmd_led(const struct shell *sh, size_t argc, char **argv)
{
	k_mutex_lock(&ret_buf_mutex, K_FOREVER);
	if (adi_shell_led(argv, ret_buf)) {
		shell_print(sh, "Error: LED command failed");
		k_mutex_unlock(&ret_buf_mutex);
		return -1;
	}

	shell_print(sh, "%s", ret_buf);
	k_mutex_unlock(&ret_buf_mutex);
	return 0;
}

/* Register adi-shell commands */
SHELL_CMD_ARG_REGISTER(peek, NULL, ADI_SHELL_PEEK_HELP, cmd_peek, ADI_SHELL_PEEK_NUM_ARGS, 0);
SHELL_CMD_ARG_REGISTER(poke, NULL, ADI_SHELL_POKE_HELP, cmd_poke, ADI_SHELL_POKE_NUM_ARGS, 0);
SHELL_CMD_ARG_REGISTER(power, NULL, ADI_SHELL_POWER_HELP, cmd_power, ADI_SHELL_POWER_NUM_ARGS, 0);
SHELL_CMD_ARG_REGISTER(led, NULL, ADI_SHELL_LED_HELP, cmd_led, ADI_SHELL_LED_NUM_ARGS, 0);

SYS_INIT(adi_shell_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
