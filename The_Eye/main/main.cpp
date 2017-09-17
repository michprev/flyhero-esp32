#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "Motors_Controller.h"
#include "MPU9250.h"
#include "Mahony_Filter.h"
#include "WiFi_Controller.h"
#include "CRC.h"
#include "LEDs.h"

using namespace flyhero;

Motors_Controller& motors_controller = Motors_Controller::Instance();
QueueHandle_t euler_queue;

void wifi_task(void *args);
void imu_task(void *args);

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

	MPU9250& mpu = MPU9250::Instance();
	Mahony_Filter mahony(2, 0.1f, 1000);

	motors_controller.Init();
	mpu.Init();

	uint8_t i = 0;

	while (true) {
		if (mpu.Data_Ready()) {
			mpu.Read_Data(accel, gyro);

			mahony.Compute(accel, gyro, euler);

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
			if (buffer[0] != received_length) {
				std::cout << "wrong length\n";
				continue;
			}

			uint16_t crc = buffer[received_length - 2] << 8;
			crc |= buffer[received_length - 1];

			if (crc != CRC::CRC16(buffer, received_length - 2)) {
				std::cout << "wrong crc\n";
				continue;
			}

			uint16_t throttle = buffer[1] << 8;
			throttle |= buffer[2];

			std::cout << "t: " << throttle << std::endl;
		}

		if (xQueueReceive(euler_queue, &euler, 0) == pdTRUE) {
			uint8_t *roll, *pitch, *yaw;
			roll = (uint8_t*)&euler.roll;
			pitch = (uint8_t*)&euler.pitch;
			yaw = (uint8_t*)&euler.yaw;

			uint8_t data[15];
			data[0] = 15;
			data[1] = roll[0];
			data[2] = roll[1];
			data[3] = roll[2];
			data[4] = roll[3];
			data[5] = pitch[0];
			data[6] = pitch[1];
			data[7] = pitch[2];
			data[8] = pitch[3];
			data[9] = yaw[0];
			data[10] = yaw[1];
			data[11] = yaw[2];
			data[12] = yaw[3];

			uint16_t crc = CRC::CRC16(data, 13);

			data[13] = crc >> 8;
			data[14] = crc & 0xFF;

			wifi.Send(data, 15);
		}
	}
}
