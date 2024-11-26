/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MAX30123_H_
#define ZEPHYR_DRIVERS_SENSOR_MAX30123_H_

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor/max30123.h>

#define NUM_SEQUENCERS 6
#define FIFO_BYTES     3

enum sequencer_sel {
	SEQUENCER_1 = 0x00u,
	SEQUENCER_2 = 0x01u,
	SEQUENCER_3 = 0x02u,
	SEQUENCER_4 = 0x03u,
	SEQUENCER_5 = 0x04u,
	SEQUENCER_6 = 0x05u,
};

enum chrono_sel {
	CHRONO_A,
	CHRONO_B,
};

/* Note: M0, M5 and M6 do not have srd option */
typedef struct {
	uint8_t mode;
	uint8_t srd;
	uint8_t dly;
	uint8_t i_conv_type;
	uint8_t v_conv_type;
	uint8_t conv_time;
} sequencer_conf_t;

typedef struct {
	uint8_t step_smp;
	uint8_t rec_smp;
	uint8_t chrono_offset_sel;
	uint8_t adc_fs_chrono;
} chrono_conf_t;

/* Register addresses */
enum register_addr {
	STATUS_1 = 0x00u,
	STATUS_2 = 0x01u,
	INTERRUPT_ENABLE_1 = 0x04u,
	INTERRUPT_ENABLE_2 = 0x05u,
	FIFO_WRITE_POINTER = 0x07u,
	FIFO_READ_POINTER = 0x08u,
	FIFO_COUNTER_1 = 0x09u,
	FIFO_COUNTER_2 = 0x0Au,
	FIFO_DATA_REGISTER = 0x0Bu,
	FIFO_CONFIGURATION_1 = 0x0Cu,
	FIFO_CONFIGURATION_2 = 0x0Du,
	SYSTEM_CONTROL_1 = 0x0Eu,
	SYSTEM_CONTROL_2 = 0x0Fu,
	SR_CLK_FINE_TUNE = 0x11u,
	ADC_CLK_FINE_TUNE = 0x12u,
	CLK_V_SPI_COUNT_MSB = 0x13u,
	CLK_V_SPI_COUT_LSB = 0x14u,
	AUTO_CLK_DIVIDER_HIGH = 0x15u,
	AUTO_CLK_DIVIDER_MID = 0x16u,
	AUTO_CLK_DIVIDER_LOW = 0x17u,
	CHRONO_CLK_DIVIDER_HIGH = 0x18u,
	CHRONO_CLK_DIVIDER_LOW = 0x19u,
	DACA_MSB_CODE = 0x1Au,
	DACB_MSB_CODE = 0x1Bu,
	DACAB_LSB_CODE = 0x1Cu,
	DAC_CONTROL_1 = 0x20u,
	PSTAT_CONFIG = 0x22u,
	WE1_CONFIG_1 = 0x23u,
	WE1_CONFIG_2 = 0x24u,
	CE1_CONFIG = 0x29u,
	GUARD_CONFIG = 0x31u,
	INTEGRITY_CHECK_RESISTOR = 0x33u,
	CHRONO_AMPLITUDE = 0x35u,
	CHRONO_SAMPLE_HOLD = 0x36u,
	CHRONO_A_CONFIG_1 = 0x37u,
	CHRONO_A_CONFIG_2 = 0x38u,
	CHRONO_A_CONFIG_3 = 0x39u,
	CHRONO_A_CONFIG_4 = 0x3Au,
	CHRONO_A_CONFIG_5 = 0x3Bu,
	CHRONO_A_CONFIG_6 = 0x3Cu,
	CHRONO_B_CONFIG_1 = 0x3Du,
	CHRONO_B_CONFIG_2 = 0x3Eu,
	CHRONO_B_CONFIG_3 = 0x3Fu,
	CHRONO_B_CONFIG_4 = 0x40u,
	CHRONO_B_CONFIG_5 = 0x41u,
	CHRONO_B_CONFIG_6 = 0x42u,
	AP_CONFIG_1 = 0x56u,
	AP_CONFIG_2 = 0x57u,
	AP_CONFIG_3 = 0x58u,
	AP_CONFIG_4 = 0x59u,
	AP_CONFIG_5 = 0x5Au,
	AP_CONFIG_6 = 0x5Bu,
	AP_CONFIG_7 = 0x5Cu,
	AP_CONFIG_8 = 0x5Du,
	AP_CONFIG_9 = 0x5Eu,
	AP_CONFIG_10 = 0x5Fu,
	CONVERT_MODE = 0x60u,
	SEQUENCE_COUNTER = 0x61u,
	M0_CONFIG = 0x62u,
	M0_DELAY = 0x63u,
	M1_CONFIG = 0x64u,
	M1_DELAY = 0x65u,
	M2_CONFIG = 0x66u,
	M2_DELAY = 0x67u,
	M3_CONFIG = 0x68u,
	M3_DELAY = 0x69u,
	M4_CONFIG = 0x6Au,
	M4_DELAY = 0x6Bu,
	M5_CONFIG = 0x6Cu,
	M5_DELAY = 0x6Du,
	M6_CONFIG = 0x6Eu,
	M6_DELAY = 0x6Fu,
	M0_ADC_CONFIG_1 = 0x70u,
	M0_ADC_CONFIG_2 = 0x71u,
	M1_ADC_CONFIG_1 = 0x72u,
	M1_ADC_CONFIG_2 = 0x73u,
	M2_ADC_CONFIG_1 = 0x74u,
	M2_ADC_CONFIG_2 = 0x75u,
	M3_ADC_CONFIG_1 = 0x76u,
	M3_ADC_CONFIG_2 = 0x77u,
	M4_ADC_CONFIG_1 = 0x78u,
	M4_ADC_CONFIG_2 = 0x79u,
	M5_ADC_CONFIG_1 = 0x7Au,
	M5_ADC_CONFIG_2 = 0x7Bu,
	M6_ADC_CONFIG_1 = 0x7Cu,
	M6_ADC_CONFIG_2 = 0x7Du,
	WE1_CURRENT_OFFSET_MSB = 0x80u,
	WE1_CURRENT_OFFSET_LSB = 0x81u,
	TEMPERATURE_SETUP = 0x84u,
	VOLTAGE_SELECT_1 = 0x85u,
	VOLTAGE_SELECT_2 = 0x86u,
	WE1_ALARM_LOW_SETUP = 0x90u,
	WE1_ALARM_HIGH_SETUP = 0x91u,
	WE1_ALARM_LOW_MSB = 0x92u,
	WE1_ALARM_LOW_LSB = 0x93u,
	WE1_ALARM_HIGH_MSB = 0x94u,
	WE1_ALARM_HIGH_LSB = 0x95u,
	V2I_ALARM_LOW_SETUP = 0xA0u,
	V2I_ALARM_HIGH_SETUP = 0xA1u,
	V2I_ALARM_LOW_MSB = 0xA2u,
	V2I_ALARM_LOW_LSB = 0xA3u,
	V2I_ALARM_HIGH_MSB = 0xA4u,
	V2I_ALARM_HIGH_LSB = 0xA5u,
	V2I_ALARM_SEL_1 = 0xA6u,
	GPIO_SETUP_1 = 0xA8u,
	GPIO_SETUP_2 = 0xA9u,
	SERIAL_ID_1 = 0xAAu,
	SERIAL_ID_2 = 0xABu,
	SERIAL_ID_3 = 0xACu,
	SERIAL_ID_4 = 0xADu,
	SERIAL_ID_5 = 0xAEu
};

/* Register Masks */
enum register_masks {
	FULL_MASK = 0xFFu,

	PWR_RDY_MASK = 0x01u,
	INVALID_CFG_MASK = 0x04u,
	POWER_MODE_MASK = 0x07u,
	DACA_LSB_CODE_MASK = 0x0Cu,
	DACB_LSB_CODE_MASK = 0x03u,
	DAC_PEDESTAL_MASK = 0x06u,
	DAC_ENABLE_MASK = 0x01u,
	WE1_AMP_EN_MASK = 0x80u,
	WE1_DAC_MX_MASK = 0x30u,
	WE1_RS_MASK = 0x08u,
	WE1_SWA_MASK = 0x04u,
	WE1_SWB_MASK = 0x02u,
	WE1_SRA_MASK = 0x01u,
	WE1_IOS_MODE_MASK = 0x80u,
	WE1_OFFSET_SEL_MASK = 0x60u,
	ADC_FS_WE1_MASK = 0x07u,
	CE1_AMP_EN_MASK = 0x80u,
	CE1_DAC_MX_MASK = 0x30u,
	CE1_SG_MASK = 0x04u,
	CE1_SC_MASK = 0x02u,
	CE1_SRB_MASK = 0x01u,
	GR1_MX_MASK = 0x06u,
	GR1_EN_MASK = 0x01u,
	CHRONO_AMPLITUDE_MASK = 0x1Fu,
	CHRONO_A_OFFSET_SEL_MASK = 0x60u,
	ADC_FS_CHRONO_A_MASK = 0x07u,
	CHRONO_B_OFFSET_SEL_MASK = 0x60u,
	ADC_FS_CHRONO_B_MASK = 0x07u,
	AP_OFFSET_SEL_MASK = 0x60u,
	ADC_FS_AP_MASK = 0x07u,
	LOCK_OCFG_MASK = 0xC0u,
	GPIO2_OCFG_MASK = 0x0Cu,
	GPIO1_OCFG_MASK = 0x03u,
	LOCK_PO_MASK = 0x04u,
	GPIO2_PO_MASK = 0x02u,
	GPIO1_PO_MASK = 0x01u,
	FIFO_RO_MASK = 0x02u,
	MX_MODE_MASK = 0xE0u,
	MX_SRD_MASK = 0x1F,
	MX_I_CONV_TYPE_MASK = 0xC0u,
	MX_V_CONV_TYPE_MASK = 0x10u,
	MX_CONV_TIME_MASK = 0x07u,
	AUTO_MASK = 0x02u,
	CONVERT_MASK = 0x01u,

	/* FIFO Data masks */
	DATA_MEASURE_MASK = 0xE00000u,
	DATA_TYPE_MASK = 0x1F0000u,
	DATA_DATA_MASK = 0xFFFFu,

	/* Data masks (for multi-byte data) */
	AUTO_CLK_DIV_HIGH_MASK = 0xFF0000u,
	AUTO_CLK_DIV_MID_MASK = 0xFF00u,
	AUTO_CLK_DIV_LOW_MASK = 0xFFu,
	CHRONO_CLK_DIV_HIGH_MASK = 0xFF00u,
	CHRONO_CLK_DIV_LOW_MASK = 0xFFu,
	DACA_CODE_LSB_MASK = 0x3u,
	DACB_CODE_LSB_MASK = 0x3u,
};

/* Required shifts */
enum dac_shifts {
	DAC_MSB_SHIFT = 2u,
	DACA_LSB_SHIFT = 2u,
};

/* FIFO Data Types */
enum data_type {
	WE1_OFFSET_CURRENT = 0x0u,
	WE1_PSTAT_CURRENT = 0x1u,
	WE1_PRE = 0x2u,
	WE1_STEP = 0x3u,
	WE1_POST = 0x4u,
	OFFSET_VOLTAGE = 0x10u,
	WE1_VOLTAGE = 0x11u,
	RE1_VOLTAGE = 0x13u,
	CE1_VOLTAGE = 0x15u,
	GR1_VOLTAGE = 0x17u,
	GPIO1_VOLTAGE = 0x19u,
	GPIO2_VOLTAGE = 0x1Au,
	VBAT_VOLTAGE = 0x1Bu,
	VREF_VOLTAGE = 0x1Cu,
	VDD_VOLTAGE = 0x1Du,
	TEMPERATURE = 0x1Eu,

	USER_MARKER = 0xFFFFFEu,
	INVALID_DATA = 0xFFFFFFu,
} data_type;

/* Measurement modes */
enum measurement_mode {
	M_MODE_DISABLED = 0,
	M_MODE_PSTAT = 1,
	M_MODE_CHRONO_A = 2,
	M_MODE_CHRONO_B = 3,
	M_MODE_AP = 4,
	M_MODE_TEMPERATURE = 6,
	M_MODE_SYS_VOLTAGE = 7,
};

/* MAX30123 SPI protocol specific */
enum max30123_spi_instr {
	MAX30123_SPI_WRITE = 0x00u,
	MAX30123_SPI_READ = 0x80u
};

/* Zephyr Sensor API Structs */
struct max30123_data {
	/* Scientific data by measurement */
	uint16_t sequencer[6];

	/* Serial identifier of device */
	uint64_t serial_id;
};

struct max30123_dev_config {
	struct spi_dt_spec spi;

	uint32_t auto_clk_div;
	uint16_t chrono_clk_div;
	uint16_t daca_code;
	uint16_t dacb_code;
	uint8_t dac_pedestal;
	uint8_t dac_enable;
	uint8_t we1_amp_enable;
	uint8_t we1_dac_mx;
	uint8_t we1_rs;
	uint8_t we1_swa;
	uint8_t we1_swb;
	uint8_t we1_sra;
	uint8_t we1_ios_mode;
	uint8_t we1_offset_sel;
	uint8_t adc_fs_we1;
	uint8_t ce1_amp_enable;
	uint8_t ce1_dac_mx;
	uint8_t ce1_sg;
	uint8_t ce1_sc;
	uint8_t ce1_srb;
	uint8_t gr1_mx;
	uint8_t gr1_en;

	uint8_t chrono_amplitude;
	chrono_conf_t chrono_a_config;
	chrono_conf_t chrono_b_config;

	uint8_t ap_offset_sel;
	uint8_t adc_fs_ap;
	uint8_t lock_ocfg;
	uint8_t gpio2_ocfg;
	uint8_t gpio1_ocfg;
	uint8_t lock_po;
	uint8_t gpio2_po;
	uint8_t gpio1_po;
	uint8_t fifo_a_full;
	uint8_t fifo_ro;

	/* Sequencer Configuration */
	sequencer_conf_t sequencer_conf[6];

	uint8_t m0_mode;
	uint8_t m0_dly;
	uint8_t m1_mode;
	uint8_t m1_srd;
	uint8_t m1_dly;
	uint8_t m2_mode;
	uint8_t m2_srd;
	uint8_t m2_dly;
	uint8_t m3_mode;
	uint8_t m3_srd;
	uint8_t m3_dly;
	uint8_t m4_mode;
	uint8_t m4_srd;
	uint8_t m4_dly;
	uint8_t m5_mode;
	uint8_t m5_dly;
	uint8_t m6_mode;
	uint8_t m6_dly;

	uint8_t m0_i_conv_type;
	uint8_t m0_v_conv_type;
	uint8_t m0_conv_time;
	uint8_t m1_i_conv_type;
	uint8_t m1_v_conv_type;
	uint8_t m1_conv_time;
	uint8_t m2_i_conv_type;
	uint8_t m2_v_conv_type;
	uint8_t m2_conv_time;
	uint8_t m3_i_conv_type;
	uint8_t m3_v_conv_type;
	uint8_t m3_conv_time;
	uint8_t m4_i_conv_type;
	uint8_t m4_v_conv_type;
	uint8_t m4_conv_time;
	uint8_t m5_i_conv_type;
	uint8_t m5_v_conv_type;
	uint8_t m5_conv_time;
	uint8_t m6_i_conv_type;
	uint8_t m6_v_conv_type;
	uint8_t m6_conv_time;

	uint8_t auto_convert;
	uint8_t convert;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_MAX30123_H_ */
