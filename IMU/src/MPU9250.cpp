/*
 * MPU9250.cpp
 *
 *  Created on: 13. 12. 2016
 *      Author: michp
 */

#include "MPU9250.h"

MPU9250* MPU9250::pInstance = NULL;

MPU9250* MPU9250::Instance() {
	if (MPU9250::pInstance == NULL)
		pInstance = new MPU9250();

	return pInstance;
}

void MPU9250::IT_Init() {
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef exti;

	exti.Pin = GPIO_PIN_1;
	exti.Mode = GPIO_MODE_IT_RISING;
	exti.Pull = GPIO_NOPULL;
	exti.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &exti);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

uint8_t MPU9250::SelfTest()
{
	int result;
	long gyro[3], accel[3];

	// run MPU6050 (or MPU9250) self-test with debugging turned off
	result = mpu_run_6500_self_test(gyro, accel, 0);

	if (result == 0x7) {
		/*printf("Passed!\n");
		printf("accel: %7.4f %7.4f %7.4f\n",
			accel[0] / 65536.f,
			accel[1] / 65536.f,
			accel[2] / 65536.f);
		printf("gyro: %7.4f %7.4f %7.4f\n",
			gyro[0] / 65536.f,
			gyro[1] / 65536.f,
			gyro[2] / 65536.f);*/

		unsigned short accel_sens;
		float gyro_sens;

		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		inv_set_accel_bias(accel, 3);
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);
		inv_set_gyro_bias(gyro, 3);
	}
	else {
		/*if (!(result & 0x1))
			printf("Gyro failed.\n");
		if (!(result & 0x2))
			printf("Accel failed.\n");
		if (!(result & 0x4))
			printf("Compass failed.\n");*/

		return 0;
	}

	inv_accel_was_turned_off();
	inv_gyro_was_turned_off();
	inv_compass_was_turned_off();

	return 1;
}

uint8_t MPU9250::Init(I2C_HandleTypeDef *hi2c) {
	I2cMaster_Init(hi2c);
	IT_Init();
	struct int_param_s int_param;
	unsigned char accel_fsr, new_temp = 0;
	unsigned short gyro_rate, gyro_fsr, compass_fsr;

	if (mpu_init(&int_param))
		return 1;

	if (inv_init_mpl())
		return 2;

	/* Compute 6-axis and 9-axis quaternions. */
	if (inv_enable_quaternion())
		return 3;
	if (inv_enable_9x_sensor_fusion())
		return 4;

	/* Update gyro biases when not in motion.
	* WARNING: These algorithms are mutually exclusive.
	*/
	if (inv_enable_fast_nomot())
		return 5;
	/* inv_enable_motion_no_motion(); */
	/* inv_set_no_motion_time(1000); */

	/* Update gyro biases when temperature changes. */
	if (inv_enable_gyro_tc())
		return 6;

	if (inv_enable_vector_compass_cal())
		return 7;
	if (inv_enable_magnetic_disturbance() /*|| inv_enable_heading_from_gyro()*/)
		return 8;

	/* Allows use of the MPL APIs in read_from_mpl. */
	if (inv_enable_eMPL_outputs())
		return 9;

	if (inv_start_mpl())
		return 10;

	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS) || mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) || mpu_set_sample_rate(this->GYRO_SAMPLE_RATE) || mpu_set_compass_sample_rate(10))
		return 11;
	/* Read back configuration in case it was set improperly. */
	if (mpu_get_sample_rate(&gyro_rate) || mpu_get_gyro_fsr(&gyro_fsr) || mpu_get_accel_fsr(&accel_fsr) || mpu_get_compass_fsr(&compass_fsr))
		return 12;

	/* Sync driver configuration with MPL. */
	/* Sample rate expected in microseconds. */
	inv_set_gyro_sample_rate(1000000L / gyro_rate);
	inv_set_accel_sample_rate(1000000L / gyro_rate);

	inv_set_compass_sample_rate(100 * 1000L);

	/* Set chip-to-body orientation matrix.
	* Set hardware units to dps/g's/degrees scaling factor.
	*/
	inv_set_gyro_orientation_and_scale(
		inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
		(long)gyro_fsr << 15);
	inv_set_accel_orientation_and_scale(
		inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
		(long)accel_fsr << 15);

	inv_set_compass_orientation_and_scale(
		inv_orientation_matrix_to_scalar(compass_pdata.orientation),
		(long)compass_fsr << 15);

	if (dmp_load_motion_driver_firmware())
		return 13;
	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation)))
		return 14;

	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL))
		return 15;
	if (dmp_set_fifo_rate(this->GYRO_SAMPLE_RATE))
		return 16;
	if (mpu_set_dmp_state(1))
		return 17;

	return 0;
}

uint8_t MPU9250::CheckNewData(float *euler, uint8_t *accur)
{
	bool new_data = false;
	bool new_compass = false;
	unsigned long sensor_timestamp;

	timestamp = HAL_GetTick();

	if ((timestamp > this->next_compass_ms) && dataReady) {
		this->next_compass_ms = timestamp + COMPASS_READ_MS;
		new_compass = 1;
	}

	if (timestamp > next_temp_ms) {
		next_temp_ms = timestamp + 500;
		new_temp = true;
	}

	if (dataReady) {
		short gyro[3], accel_short[3], sensors;
		unsigned char more;
		long accel[3], quat[4], temperature;

		int state = dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);

		// FIFO overflow
		if (state == -2)
			return 2;
		// another error, data not ready e. g.
		else if (state != 0)
			return 3;

		if (!more)
			dataReady = false;
		if (sensors & INV_XYZ_GYRO) {
			/* Push the new data to the MPL. */
			inv_build_gyro(gyro, sensor_timestamp);
			new_data = true;
			if (new_temp) {
				new_temp = false;
				/* Temperature only used for gyro temp comp. */
				if (mpu_get_temperature(&temperature, &sensor_timestamp))
					return 2;
				inv_build_temp(temperature, sensor_timestamp);
			}
		}
		if (sensors & INV_XYZ_ACCEL) {
			accel[0] = (long)accel_short[0];
			accel[1] = (long)accel_short[1];
			accel[2] = (long)accel_short[2];
			inv_build_accel(accel, 0, (inv_time_t)sensor_timestamp);
			new_data = true;
		}
		if (sensors & INV_WXYZ_QUAT) {
			inv_build_quat(quat, 0, sensor_timestamp);
			new_data = true;
		}
	}

	if (new_compass) {
		short compass_short[3];
		long compass[3];
		new_compass = 0;

		if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
			compass[0] = (long)compass_short[0];
			compass[1] = (long)compass_short[1];
			compass[2] = (long)compass_short[2];

			inv_build_compass(compass, 0, sensor_timestamp);
		}
		new_data = 1;
	}

	if (new_data) {
		if (inv_execute_on_data())
			return 2;

		long msg, data[9];
		int8_t accuracy;
		unsigned long timestamp;

		if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t*)&timestamp)) {
			euler[0] = data[0] / 65536.f;
			euler[1] = data[1] / 65536.f;
			euler[2] = data[2] / 65536.f;
			(*accur) = accuracy;
		}

		return 1;
	}

	return 0;
}

extern "C" void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
