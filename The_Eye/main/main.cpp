#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "Motors_Controller.h"
#include "MPU9250.h"
#include "MPU6050.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"
#include "WiFi_Controller.h"
#include "CRC.h"
#include "LEDs.h"

using namespace flyhero;

Motors_Controller& motors_controller = Motors_Controller::Instance();
QueueHandle_t euler_queue;

void wifi_task(void *args);
void imu_task(void *args);

void wifi_parser(uint8_t *buffer, uint8_t received_length);

extern "C" void app_main(void) {
	LEDs::Init();

	euler_queue = xQueueCreate(20, sizeof(IMU::Euler_Angles));

	xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 1);

	while (true);
}

void imu_task(void *args) {
	IMU::Sensor_Data accel, gyro;
	IMU::Euler_Angles euler;

	//MPU9250& mpu = MPU9250::Instance();
	MPU6050& mpu = MPU6050::Instance();
	//Mahony_Filter mahony(2, 0.1f, 1000);
	Complementary_Filter complementary(0.995f, 1000);

	motors_controller.Init();
	mpu.Init();

	uint8_t i = 0;

	while (true) {
		if (mpu.Data_Ready()) {
			mpu.Read_Data(accel, gyro);

			//mahony.Compute(accel, gyro, euler);
			complementary.Compute(accel, gyro, euler);

			i++;

			if (i == 100) {
				xQueueSend(euler_queue, &euler, 0);
				i = 0;
			}


			motors_controller.Update_Motors(euler);
		}
	}
}

void wifi_task(void *args) {
	const uint8_t BUFFER_SIZE = 200;

	WiFi_Controller& wifi = WiFi_Controller::Instance();
	uint8_t buffer[BUFFER_SIZE];
	uint8_t received_length;

	IMU::Euler_Angles euler;

	wifi.Init();

	while (true) {
		if (wifi.Receive(buffer, BUFFER_SIZE, received_length)) {
			// wrong length
			if (buffer[0] != received_length)
				continue;

			uint16_t crc;
			crc = buffer[received_length - 1] << 8;
			crc |= buffer[received_length - 2];

			// wrong crc
			if (crc != CRC::CRC16(buffer, received_length - 2))
                continue;

			wifi_parser(buffer, received_length);
		}

		if (xQueueReceive(euler_queue, &euler, 0) == pdTRUE) {
			uint8_t *roll, *pitch, *yaw;
			roll = (uint8_t*)&euler.roll;
			pitch = (uint8_t*)&euler.pitch;
			yaw = (uint8_t*)&euler.yaw;

			uint8_t data[16];

            data[0] = 16;
            data[1] = 0x00;
			data[2] = roll[3];
			data[3] = roll[2];
			data[4] = roll[1];
			data[5] = roll[0];
			data[6] = pitch[3];
			data[7] = pitch[2];
			data[8] = pitch[1];
			data[9] = pitch[0];
			data[10] = yaw[3];
			data[11] = yaw[2];
			data[12] = yaw[1];
			data[13] = yaw[0];

			uint16_t crc = CRC::CRC16(data, 14);

            data[14] = crc & 0xFF;
			data[15] = crc >> 8;

			wifi.Send(data, 16);
		}
	}
}

void wifi_parser(uint8_t *buffer, uint8_t received_length) {
	switch (buffer[1]) {
	case 0x00:
		uint16_t throttle;
		throttle = buffer[3] << 8;
		throttle |= buffer[2];

		//std::cout << "t: " << throttle << std::endl;

		motors_controller.Set_Throttle(throttle);
		motors_controller.Set_PID_Constants(Roll, 1, 0, 0);
		motors_controller.Set_PID_Constants(Pitch, 1, 0, 0);

		break;
	}
}
