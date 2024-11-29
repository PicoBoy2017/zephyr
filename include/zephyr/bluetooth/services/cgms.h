/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CGMS_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CGMS_H_

/**
 * @brief Continuous Glucose Measurement Service (CGMS)
 * @defgroup bt_cgms Continuous Glucose Measurement Service (CGMS)
 * @ingroup bluetooth
 * @{
 *
 * For more details on the Bluetooth CGM specification, refer to the official documentation:
 * @see <a
 * href="https://www.bluetooth.com/specifications/specs/continuous-glucose-monitoring-service-1-0-2/">Continuous
 * Glucose Monitoring Service 1.0.2</a>
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

/**
 * @struct cgm_measurement
 * @brief Structure representing a Continuous Glucose Monitoring (CGM) measurement.
 *
 * Holds data for a single CGM measurement, including glucose concentration, time offset,
 * and additional flags. It is packed to so can be transmitted over BLE in expected structure.
 */
typedef struct __packed cgm_measurement_t {
	/**
	 * @brief Size field.
	 *
	 * Indicates the size of the measurement data. This field is an 8-bit unsigned integer
	 * with a minimum size of 6 octets (48 bits).
	 */
	uint8_t size;

	/**
	 * @brief Flags field.
	 *
	 * An 8-bit unsigned integer used to store various flags associated with the measurement.
	 */
	uint8_t flags;

	/**
	 * @brief CGM glucose concentration.
	 *
	 * The glucose concentration in an SFLOAT format, which uses a 16-bit representation:
	 * a 4-bit integer and a 12-bit mantissa. Milligram per deciliter (mg/dL).
	 */
	uint16_t glucose_conc;

	/**
	 * @brief Time offset.
	 *
	 * The time offset in minutes since the session start time. Represented as a 16-bit
	 * unsigned integer.
	 */
	uint16_t time_offset;
} cgm_measurement_t;

/**
 * @struct bt_cgms_callbacks
 * @brief Callbacks for Continuous Glucose Monitoring Service (CGMS).
 *
 * This structure provides callback functions for the Continuous Glucose Monitoring Service
 * (CGMS). Each callback function allows interaction with glucose concentration
 * data and records.
 *
 * All callbacks are mandatory and must be implemented by the application.
 *
 * A return value of 0 indicates success, while a negative error code indicates failure.
 */
struct bt_cgms_callbacks {
	/**
	 * @brief Callback to get the current glucose concentration.
	 *
	 * This callback function retrieves the uint16_t glucose concentration calculated
	 * by the system.
	 *
	 * @param[out] concentration Pointer to store the latest glucose concentration value.
	 *
	 * @return 0 Success, negative error code for failure.
	 * @return -EINVAL in case cgm_measurement_t pointer is NULL
	 */
	int (*get_glucose_conc_cb)(uint16_t *glucose_conc);

	/**
	 * @brief Callback to add a new glucose measurement record to database.
	 *
	 * This callback function stores a record to the system database for later retrieval.
	 *
	 * @param[in] measurement The glucose measurement data to be added.
	 *
	 * @return 0 Success, negative error code for failure.
	 * @return -ENOMEM database full.
	 */
	int (*add_record_cb)(cgm_measurement_t measurement);

	/**
	 * @brief Callback to retrieve a specific glucose measurement record.
	 *
	 * This callback function retrieves a glucose measurement record by its index in
	 * the database. Index should not be construed with time offset.
	 *
	 * @param[out] measurement Pointer to store the retrieved glucose measurement data.
	 * @param[in] index Index of the record to retrieve.
	 *
	 * @return 0 Success, negative error code for failure.
	 * @return -EINVAL if index out of bounds.
	 */
	int (*get_record_cb)(cgm_measurement_t *measurement, uint16_t index);
};

/**
 * @brief Manually records a measurement to CGM Service
 *
 * This function will notifies the central of a new measurement and adds
 * it to the CGM data using the registered callback.
 *
 * @param[in] glucose_conc The glucose concentration value.
 *
 * @return 0 Success, negative errno code on failure.
 */
int bt_cgms_record_measurement(uint16_t glucose_conc);

/**
 * @brief Register CGM Service callback functions
 *
 * Registers the callbacks to be used during periodic glucose measurement.
 *
 * @param[in] callbacks Callback struct to fill internal callbacks.
 *
 * @return 0 Success, negative errno code on failure.
 * @return -EINVAL if callbacks invalid.
 */
int bt_cgms_register_callbacks(struct bt_cgms_callbacks callbacks);

/**
 * @brief Initialize CGM Service
 *
 * Initializes the CGM service.
 *
 * @param[in] callbacks Callback struct to fill internal callbacks.
 *
 * @return 0 Success, negative errno code on failure.
 */
int bt_cgms_init(struct bt_cgms_callbacks callbacks);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CGMS_H_ */
