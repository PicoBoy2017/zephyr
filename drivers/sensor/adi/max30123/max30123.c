/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max30123

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

#include "max30123.h"

LOG_MODULE_REGISTER(max30123, CONFIG_SENSOR_LOG_LEVEL);

/*
 * Direct SPI interaction functions
 */
static int max30123_register_access(const struct device *dev, uint8_t addr_reg, uint8_t *data,
				    bool read, size_t length)
{
	const struct max30123_dev_config *config = dev->config;
	uint8_t rw_reg;

	/* Allocate spi buffer struct */
	const struct spi_buf buf[3] = {{.buf = &addr_reg, .len = 1},
				       {.buf = &rw_reg, .len = 1},
				       {.buf = data, .len = length}};

	struct spi_buf_set tx = {.buffers = buf};

	if (read) {
		rw_reg = MAX30123_SPI_READ;

		const struct spi_buf_set rx = {.buffers = buf, .count = ARRAY_SIZE(buf)};

		tx.count = 2;

		return spi_transceive_dt(&config->spi, &tx, &rx);
	}

	rw_reg = MAX30123_SPI_WRITE;
	tx.count = 3;

	return spi_write_dt(&config->spi, &tx);
}

static int max30123_reg_read(const struct device *dev, uint8_t reg_addr, uint8_t *val)
{
	return max30123_register_access(dev, reg_addr, val, true, 1);
}

static int max30123_reg_read_multiple(const struct device *dev, uint8_t reg_addr, uint8_t *val,
				      uint16_t count)
{
	return max30123_register_access(dev, reg_addr, val, true, count);
}

static int max30123_reg_write(const struct device *dev, uint8_t reg_addr, uint8_t val)
{
	return max30123_register_access(dev, reg_addr, &val, false, 1);
}

static int max30123_reg_write_multiple(const struct device *dev, uint8_t reg_addr, uint8_t *val,
				       uint16_t count)
{
	return max30123_register_access(dev, reg_addr, val, false, count);
}

static int max30123_set_reg_field(const struct device *dev, uint8_t reg_addr, uint8_t mask,
				  uint8_t value)
{
	int err;
	uint8_t tmp;

	err = max30123_reg_read(dev, reg_addr, &tmp);
	if (err) {
		return err;
	}

	tmp = tmp & ~mask;
	tmp |= FIELD_PREP(mask, value);

	err = max30123_reg_write(dev, reg_addr, tmp);
	if (err) {
		LOG_ERR("Unable to set register field at reg_addr: %d", reg_addr);
		return err;
	}

	return 0;
}

static int max30123_fifo_read(const struct device *dev, uint8_t buffer[FIFO_BYTES])
{
	const struct max30123_dev_config *config = dev->config;
	uint8_t addr = FIFO_DATA_REGISTER;
	uint8_t rw_reg = MAX30123_SPI_READ;

	/* Allocate spi buffer struct */
	const struct spi_buf buf[3] = {{.buf = &addr, .len = sizeof(addr)},
				       {.buf = &rw_reg, .len = sizeof(rw_reg)},
				       {.buf = buffer, .len = FIFO_BYTES}};

	struct spi_buf_set tx = {.buffers = buf};

	const struct spi_buf_set rx = {.buffers = buf, .count = ARRAY_SIZE(buf)};

	tx.count = 2;

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

/*
 * Internal driver helper functions
 */
static int max30123_parse_fifo_data(const struct device *dev, uint8_t fifo_data[FIFO_BYTES])
{
	uint32_t data = sys_get_be24(fifo_data);

	/* Skip markers */
	if (data == USER_MARKER || data == INVALID_DATA) {
		return 0;
	}

	struct max30123_data *const device_data = dev->data;

	uint8_t sequence = (uint8_t)FIELD_GET(DATA_MEASURE_MASK, data);
	uint16_t adc_data = (uint16_t)FIELD_GET(DATA_DATA_MASK, data);

	/* Save data */
	if (sequence >= NUM_SEQUENCERS) {
		LOG_ERR("Invalid sequence number");
		return -EBADMSG;
	}

	device_data->sequencer[sequence] = adc_data;

	return 0;
}

static int max30123_get_fifo_data_count(const struct device *dev, uint16_t *count)
{
	int ret;
	uint8_t tmp[2];

	ret = max30123_reg_read_multiple(dev, FIFO_COUNTER_1, tmp, 2);
	if (ret) {
		return ret;
	}

	*count = sys_get_be16(tmp);

	return 0;
}

static int max30123_fetch_fifo_data(const struct device *dev)
{
	int ret;
	uint16_t count;

	ret = max30123_get_fifo_data_count(dev, &count);
	if (ret) {
		LOG_ERR("Error in getting fifo data count");
		return ret;
	}

	uint8_t data[FIFO_BYTES];

	for (uint16_t i = 0; i < count; i++) {
		ret = max30123_fifo_read(dev, data);
		if (ret) {
			LOG_ERR("Error in getting fifo data");
			return ret;
		}

		ret = max30123_parse_fifo_data(dev, data);
		if (ret) {
			LOG_ERR("Error in parsing fifo data");
			return ret;
		}
	}

	return 0;
}

static int max30123_check_status_registers(const struct device *dev)
{
	uint8_t tmp;
	uint8_t status;

	/* Check for valid configurations */
	if (max30123_reg_read(dev, STATUS_1, &tmp)) {
		return -EIO;
	}

	status = tmp & INVALID_CFG_MASK;
	if (status) {
		LOG_ERR("MAX30123 invalid configuration set");
		return -EIO;
	}

	status = tmp & PWR_RDY_MASK;
	if (status) {
		LOG_ERR("MAX30123 VDD below POR threshold");
		return -EIO;
	}

	return 0;
}

static int max30123_save_serial_id(const struct device *dev)
{
	struct max30123_data *const data = dev->data;
	uint8_t id[8] = {0};

	if (max30123_reg_read_multiple(dev, SERIAL_ID_1, id, 5)) {
		return -EIO;
	}

	data->serial_id = sys_get_be40(id);

	if (!data->serial_id) {
		LOG_ERR("Failed to read MAX30123 serial ID");
		return -EIO;
	}

	return 0;
}

static int set_chrono_config(const struct device *dev, enum chrono_sel chrono_sel,
			     chrono_conf_t chrono_config)
{
	int err;
	uint8_t base_reg;

	if (chrono_sel == CHRONO_A) {
		base_reg = CHRONO_A_CONFIG_2;
	} else {
		base_reg = CHRONO_B_CONFIG_2;
	}

	/* Set the chrono config */
	err = max30123_set_reg_field(dev, base_reg, FULL_MASK, chrono_config.step_smp);
	if (err) {
		return err;
	}

	/* rec_smp is 1 register after base_reg */
	err = max30123_set_reg_field(dev, base_reg + 1, FULL_MASK, chrono_config.rec_smp);
	if (err) {
		return err;
	}

	/* offset_sel and adc_fs are 4 registers after base_reg */
	err = max30123_set_reg_field(dev, base_reg + 4, CHRONO_A_OFFSET_SEL_MASK,
				     chrono_config.chrono_offset_sel);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, base_reg + 4, ADC_FS_CHRONO_A_MASK,
				     chrono_config.adc_fs_chrono);
	if (err) {
		return err;
	}

	return err;
}

static uint8_t get_base_seq_conf_reg(enum sequencer_sel sequencer)
{
	switch (sequencer) {
	case SEQUENCER_1:
		return M0_CONFIG;
	case SEQUENCER_2:
		return M1_CONFIG;
	case SEQUENCER_3:
		return M2_CONFIG;
	case SEQUENCER_4:
		return M3_CONFIG;
	case SEQUENCER_5:
		return M4_CONFIG;
	case SEQUENCER_6:
		return M5_CONFIG;
	default:
		return 0;
	}
}

static uint8_t get_base_seq_adc_reg(enum sequencer_sel sequencer)
{
	switch (sequencer) {
	case SEQUENCER_1:
		return M0_ADC_CONFIG_1;
	case SEQUENCER_2:
		return M1_ADC_CONFIG_1;
	case SEQUENCER_3:
		return M2_ADC_CONFIG_1;
	case SEQUENCER_4:
		return M3_ADC_CONFIG_1;
	case SEQUENCER_5:
		return M4_ADC_CONFIG_1;
	case SEQUENCER_6:
		return M5_ADC_CONFIG_1;
	default:
		return 0;
	}
}

static int set_sequencer_config(const struct device *dev)
{
	int err;
	const struct max30123_dev_config *config = dev->config;

	for (enum sequencer_sel i = 0; i < NUM_SEQUENCERS; i++) {

		sequencer_conf_t sequencer_conf = config->sequencer_conf[i];

		/* Get sequencer base reg */
		uint8_t seq_conf_base = get_base_seq_conf_reg(i);
		uint8_t seq_adc_base = get_base_seq_adc_reg(i);

		/* Sequncer conf */
		err = max30123_set_reg_field(dev, seq_conf_base, MX_MODE_MASK, sequencer_conf.mode);
		if (err) {
			return err;
		}
		/* Sequencers 1, 5 and 6 do not have SRD configuration */
		if (i != SEQUENCER_1 && i != SEQUENCER_5 && i != SEQUENCER_6) {
			err = max30123_set_reg_field(dev, seq_conf_base, MX_SRD_MASK,
						     sequencer_conf.srd);
			if (err) {
				return err;
			}
		}
		/* delay reg is directly after mode */
		err = max30123_set_reg_field(dev, seq_conf_base + 1, FULL_MASK, sequencer_conf.dly);
		if (err) {
			return err;
		}

		/* Sequencer ADC conf */
		err = max30123_set_reg_field(dev, seq_adc_base, MX_I_CONV_TYPE_MASK,
					     sequencer_conf.i_conv_type);
		if (err) {
			return err;
		}
		err = max30123_set_reg_field(dev, seq_adc_base, MX_V_CONV_TYPE_MASK,
					     sequencer_conf.v_conv_type);
		if (err) {
			return err;
		}
		err = max30123_set_reg_field(dev, seq_adc_base, MX_CONV_TIME_MASK,
					     sequencer_conf.conv_time);
		if (err) {
			return err;
		}
	}

	return 0;
}

static int max30123_write_config(const struct device *dev)
{
	int err;
	const struct max30123_dev_config *config = dev->config;

	/* Sensor Clock Config */
	uint8_t auto_clk_div[3];
	uint8_t chrono_clk_div[2];

	sys_put_be24(config->auto_clk_div, auto_clk_div);
	err = max30123_reg_write_multiple(dev, AUTO_CLK_DIVIDER_HIGH, auto_clk_div,
					  sizeof(auto_clk_div));
	if (err) {
		return err;
	}

	sys_put_be16(config->chrono_clk_div, chrono_clk_div);
	err = max30123_reg_write_multiple(dev, CHRONO_CLK_DIVIDER_HIGH, chrono_clk_div,
					  sizeof(chrono_clk_div));
	if (err) {
		return err;
	}

	/* Set DACs Config */
	err = max30123_set_reg_field(dev, DACA_MSB_CODE, FULL_MASK,
				     (uint8_t)(config->daca_code >> DAC_MSB_SHIFT));
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, DACAB_LSB_CODE, DACA_LSB_CODE_MASK,
				     (uint8_t)(config->daca_code << DACA_LSB_SHIFT));
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, DACB_MSB_CODE, FULL_MASK,
				     (uint8_t)(config->dacb_code >> DAC_MSB_SHIFT));
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, DACAB_LSB_CODE, DACB_LSB_CODE_MASK,
				     (uint8_t)(config->dacb_code));
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, DAC_CONTROL_1, DAC_PEDESTAL_MASK, config->dac_pedestal);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, DAC_CONTROL_1, DAC_ENABLE_MASK, config->dac_enable);
	if (err) {
		return err;
	}

	/* Electrode and Guard Ring Configuration */
	err = max30123_set_reg_field(dev, WE1_CONFIG_1, WE1_AMP_EN_MASK, config->we1_amp_enable);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_1, WE1_DAC_MX_MASK, config->we1_dac_mx);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_1, WE1_RS_MASK, config->we1_rs);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_1, WE1_SWA_MASK, config->we1_swa);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_1, WE1_SWB_MASK, config->we1_swb);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_1, WE1_SRA_MASK, config->we1_sra);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_2, WE1_IOS_MODE_MASK, config->we1_ios_mode);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_2, WE1_OFFSET_SEL_MASK,
				     config->we1_offset_sel);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, WE1_CONFIG_2, ADC_FS_WE1_MASK, config->adc_fs_we1);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, CE1_CONFIG, CE1_AMP_EN_MASK, config->ce1_amp_enable);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, CE1_CONFIG, CE1_DAC_MX_MASK, config->ce1_dac_mx);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, CE1_CONFIG, CE1_SG_MASK, config->ce1_sg);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, CE1_CONFIG, CE1_SC_MASK, config->ce1_sc);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, CE1_CONFIG, CE1_SRB_MASK, config->ce1_srb);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, GUARD_CONFIG, GR1_MX_MASK, config->gr1_mx);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, GUARD_CONFIG, GR1_EN_MASK, config->gr1_en);
	if (err) {
		return err;
	}

	/* Chrono Config */
	err = max30123_set_reg_field(dev, CHRONO_AMPLITUDE, CHRONO_AMPLITUDE_MASK,
				     config->chrono_amplitude);
	if (err) {
		return err;
	}
	err = set_chrono_config(dev, CHRONO_A, config->chrono_a_config);
	if (err) {
		return err;
	}
	err = set_chrono_config(dev, CHRONO_B, config->chrono_b_config);
	if (err) {
		return err;
	}

	/* AP Config */
	err = max30123_set_reg_field(dev, AP_CONFIG_10, AP_OFFSET_SEL_MASK, config->ap_offset_sel);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, AP_CONFIG_10, ADC_FS_AP_MASK, config->adc_fs_ap);
	if (err) {
		return err;
	}

	/* IO Config */
	err = max30123_set_reg_field(dev, GPIO_SETUP_1, LOCK_OCFG_MASK, config->lock_ocfg);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, GPIO_SETUP_1, GPIO2_OCFG_MASK, config->gpio2_ocfg);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, GPIO_SETUP_1, GPIO1_OCFG_MASK, config->gpio1_ocfg);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, GPIO_SETUP_2, LOCK_PO_MASK, config->lock_po);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, GPIO_SETUP_2, GPIO2_PO_MASK, config->gpio2_po);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, GPIO_SETUP_2, GPIO1_PO_MASK, config->gpio1_po);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, FIFO_CONFIGURATION_1, FULL_MASK, config->fifo_a_full);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, FIFO_CONFIGURATION_2, FIFO_RO_MASK, config->fifo_ro);
	if (err) {
		return err;
	}

	/* Sequencer Config */
	err = set_sequencer_config(dev);
	if (err) {
		return err;
	}

	/* Set the auto and convert fields lastly to begin conversions */
	err = max30123_set_reg_field(dev, CONVERT_MODE, AUTO_MASK, config->auto_convert);
	if (err) {
		return err;
	}
	err = max30123_set_reg_field(dev, CONVERT_MODE, CONVERT_MASK, config->convert);
	if (err) {
		return err;
	}

	return 0;
}

static int max30123_electrochemical_init(const struct device *dev)
{
	const struct max30123_dev_config *const config = dev->config;

	if (!spi_is_ready_dt(&config->spi)) {
		return -ENODEV;
	}

	/* Write config from devicetree and Kconfig */
	if (max30123_write_config(dev)) {
		return -EIO;
	}

	/* Check if the written configuration is valid */
	if (max30123_check_status_registers(dev)) {
		return -EIO;
	}

	/* Save serial ID of device to memory*/
	if (max30123_save_serial_id(dev)) {
		return -EIO;
	}

	LOG_INF("MAX30123 Initialized");

	return 0;
}

/*
 * Zephyr Sensor API functions
 */

static int max30123_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
	struct max30123_data *const data = dev->data;

	val->val1 = data->sequencer[chan - SENSOR_CHAN_PRIV_START];

	return 0;
}

static int max30123_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret = -ENOTSUP;

	/* Due to FIFO data buffer, all data must be read at once. Cannot target specific chanenl */
	if (chan == SENSOR_CHAN_ALL) {
		ret = max30123_fetch_fifo_data(dev);
		if (ret) {
			return ret;
		}
	}

	return ret;
}

static const struct sensor_driver_api max30123_api_funcs = {.sample_fetch = max30123_sample_fetch,
							    .channel_get = max30123_channel_get};

#define MAX30123_DEFINE(inst)                                                                      \
	static struct max30123_data max30123_data_##inst;                                          \
                                                                                                   \
	static const struct max30123_dev_config max30123_dev_config_##inst = {                     \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),          \
                                                                                                   \
		/* Device Tree Controlled Config */                                                \
		.fifo_a_full = DT_INST_PROP(inst, fifo_a_full),                                    \
		.fifo_ro = DT_INST_PROP(inst, fifo_ro),                                            \
                                                                                                   \
		.auto_clk_div = DT_INST_PROP(inst, auto_clk_div),                                  \
		.chrono_clk_div = DT_INST_PROP(inst, chrono_clk_div),                              \
		.daca_code = DT_INST_PROP(inst, daca_code),                                        \
		.dacb_code = DT_INST_PROP(inst, dacb_code),                                        \
		.dac_pedestal = DT_INST_PROP(inst, dac_pedestal),                                  \
		.dac_enable = DT_INST_PROP(inst, dac_enable),                                      \
		.we1_amp_enable = DT_INST_PROP(inst, we1_amp_enable),                              \
		.we1_dac_mx = DT_INST_PROP(inst, we1_dac_mx),                                      \
		.we1_rs = DT_INST_PROP(inst, we1_rs),                                              \
		.we1_swa = DT_INST_PROP(inst, we1_swa),                                            \
		.we1_swb = DT_INST_PROP(inst, we1_swb),                                            \
		.we1_sra = DT_INST_PROP(inst, we1_sra),                                            \
		.we1_ios_mode = DT_INST_PROP(inst, we1_ios_mode),                                  \
		.we1_offset_sel = DT_INST_PROP(inst, we1_offset_sel),                              \
		.adc_fs_we1 = DT_INST_PROP(inst, adc_fs_we1),                                      \
		.ce1_amp_enable = DT_INST_PROP(inst, ce1_amp_enable),                              \
		.ce1_dac_mx = DT_INST_PROP(inst, ce1_dac_mx),                                      \
		.ce1_sg = DT_INST_PROP(inst, ce1_sg),                                              \
		.ce1_sc = DT_INST_PROP(inst, ce1_sc),                                              \
		.ce1_srb = DT_INST_PROP(inst, ce1_srb),                                            \
		.gr1_mx = DT_INST_PROP(inst, gr1_mx),                                              \
		.gr1_en = DT_INST_PROP(inst, gr1_enable),                                          \
		.chrono_amplitude = DT_INST_PROP(inst, chrono_amplitude),                          \
		.chrono_a_config.step_smp = DT_INST_PROP(inst, step_smp_a),                        \
		.chrono_a_config.rec_smp = DT_INST_PROP(inst, rec_smp_a),                          \
		.chrono_a_config.chrono_offset_sel = DT_INST_PROP(inst, chrono_a_offset_sel),      \
		.chrono_a_config.adc_fs_chrono = DT_INST_PROP(inst, adc_fs_chrono_a),              \
		.chrono_b_config.step_smp = DT_INST_PROP(inst, step_smp_b),                        \
		.chrono_b_config.rec_smp = DT_INST_PROP(inst, rec_smp_b),                          \
		.chrono_b_config.chrono_offset_sel = DT_INST_PROP(inst, chrono_b_offset_sel),      \
		.chrono_b_config.adc_fs_chrono = DT_INST_PROP(inst, adc_fs_chrono_b),              \
		.ap_offset_sel = DT_INST_PROP(inst, ap_offset_sel),                                \
		.adc_fs_ap = DT_INST_PROP(inst, adc_fs_ap),                                        \
		.auto_convert = DT_INST_PROP(inst, auto),                                          \
		.convert = DT_INST_PROP(inst, convert),                                            \
                                                                                                   \
		.sequencer_conf[SEQUENCER_1].mode = DT_INST_PROP(inst, m0_mode),                   \
		.sequencer_conf[SEQUENCER_1].dly = DT_INST_PROP(inst, m0_dly),                     \
		.sequencer_conf[SEQUENCER_1].i_conv_type = DT_INST_PROP(inst, m0_i_conv_type),     \
		.sequencer_conf[SEQUENCER_1].v_conv_type = DT_INST_PROP(inst, m0_v_conv_type),     \
		.sequencer_conf[SEQUENCER_1].conv_time = DT_INST_PROP(inst, m0_conv_time),         \
                                                                                                   \
		.sequencer_conf[SEQUENCER_2].mode = DT_INST_PROP(inst, m1_mode),                   \
		.sequencer_conf[SEQUENCER_2].srd = DT_INST_PROP(inst, m1_srd),                     \
		.sequencer_conf[SEQUENCER_2].dly = DT_INST_PROP(inst, m1_dly),                     \
		.sequencer_conf[SEQUENCER_2].i_conv_type = DT_INST_PROP(inst, m1_i_conv_type),     \
		.sequencer_conf[SEQUENCER_2].v_conv_type = DT_INST_PROP(inst, m1_v_conv_type),     \
		.sequencer_conf[SEQUENCER_2].conv_time = DT_INST_PROP(inst, m1_conv_time),         \
                                                                                                   \
		.sequencer_conf[SEQUENCER_3].mode = DT_INST_PROP(inst, m2_mode),                   \
		.sequencer_conf[SEQUENCER_3].srd = DT_INST_PROP(inst, m2_srd),                     \
		.sequencer_conf[SEQUENCER_3].dly = DT_INST_PROP(inst, m2_dly),                     \
		.sequencer_conf[SEQUENCER_3].i_conv_type = DT_INST_PROP(inst, m2_i_conv_type),     \
		.sequencer_conf[SEQUENCER_3].v_conv_type = DT_INST_PROP(inst, m2_v_conv_type),     \
		.sequencer_conf[SEQUENCER_3].conv_time = DT_INST_PROP(inst, m2_conv_time),         \
                                                                                                   \
		.sequencer_conf[SEQUENCER_4].mode = DT_INST_PROP(inst, m3_mode),                   \
		.sequencer_conf[SEQUENCER_4].srd = DT_INST_PROP(inst, m3_srd),                     \
		.sequencer_conf[SEQUENCER_4].dly = DT_INST_PROP(inst, m3_dly),                     \
		.sequencer_conf[SEQUENCER_4].i_conv_type = DT_INST_PROP(inst, m3_i_conv_type),     \
		.sequencer_conf[SEQUENCER_4].v_conv_type = DT_INST_PROP(inst, m3_v_conv_type),     \
		.sequencer_conf[SEQUENCER_4].conv_time = DT_INST_PROP(inst, m3_conv_time),         \
                                                                                                   \
		.sequencer_conf[SEQUENCER_5].mode = DT_INST_PROP(inst, m4_mode),                   \
		.sequencer_conf[SEQUENCER_5].dly = DT_INST_PROP(inst, m4_dly),                     \
		.sequencer_conf[SEQUENCER_5].i_conv_type = DT_INST_PROP(inst, m4_i_conv_type),     \
		.sequencer_conf[SEQUENCER_5].v_conv_type = DT_INST_PROP(inst, m4_v_conv_type),     \
		.sequencer_conf[SEQUENCER_5].conv_time = DT_INST_PROP(inst, m4_conv_time),         \
                                                                                                   \
		.sequencer_conf[SEQUENCER_6].mode = DT_INST_PROP(inst, m5_mode),                   \
		.sequencer_conf[SEQUENCER_6].dly = DT_INST_PROP(inst, m5_dly),                     \
		.sequencer_conf[SEQUENCER_6].i_conv_type = DT_INST_PROP(inst, m5_i_conv_type),     \
		.sequencer_conf[SEQUENCER_6].v_conv_type = DT_INST_PROP(inst, m5_v_conv_type),     \
		.sequencer_conf[SEQUENCER_6].conv_time = DT_INST_PROP(inst, m5_conv_time),         \
                                                                                                   \
		.lock_ocfg = DT_INST_PROP(inst, lock_ocfg),                                        \
		.gpio2_ocfg = DT_INST_PROP(inst, gpio2_ocfg),                                      \
		.gpio1_ocfg = DT_INST_PROP(inst, gpio1_ocfg),                                      \
		.lock_po = DT_INST_PROP(inst, lock_po),                                            \
		.gpio2_po = DT_INST_PROP(inst, gpio2_po),                                          \
		.gpio1_po = DT_INST_PROP(inst, gpio1_po),                                          \
	};                                                                                         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, max30123_electrochemical_init, NULL,                    \
				     &max30123_data_##inst, &max30123_dev_config_##inst,           \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                     \
				     &max30123_api_funcs)

DT_INST_FOREACH_STATUS_OKAY(MAX30123_DEFINE)
