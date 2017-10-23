#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <nvs_flash.h>

#include "Motors_Controller.h"
#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"
#include "WiFi_Controller.h"
#include "CRC.h"
#include "LEDs.h"
#include "../../IMU/main/IMU_Detector.h"

using namespace flyhero;

Motors_Controller& motors_controller = Motors_Controller::Instance();
QueueHandle_t euler_queue;

void wifi_task(void *args);
void imu_task(void *args);

void wifi_parser(uint8_t *buffer, uint8_t received_length);

const uint8_t FUSION_ALGORITHMS_USED = 2;

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_status = nvs_flash_init();
    }

    ESP_ERROR_CHECK(nvs_status);

	LEDs::Init();

	euler_queue = xQueueCreate(20, sizeof(IMU::Euler_Angles[FUSION_ALGORITHMS_USED]));

	xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 1);

	while (true);
}

void imu_task(void *args) {
	IMU::Sensor_Data accel, gyro;
	IMU::Euler_Angles mahony_euler, complementary_euler;

    IMU *imu;
    ESP_ERROR_CHECK(IMU_Detector::Detect_IMU(&imu));

	Mahony_Filter mahony(2, 0.1f, 1000);
	Complementary_Filter complementary(0.995f, 1000);

	motors_controller.Init();
	imu->Init();

    IMU::Euler_Angles queue_data[FUSION_ALGORITHMS_USED];
	uint8_t i = 0;

	while (true) {
		if (imu->Data_Ready()) {
			imu->Read_Data(accel, gyro);

			mahony.Compute(accel, gyro, mahony_euler);
			complementary.Compute(accel, gyro, complementary_euler);

			i++;

			if (i == 100) {
                queue_data[0] = mahony_euler;
                queue_data[1] = complementary_euler;

				xQueueSend(euler_queue, &queue_data, 0);
				i = 0;
			}


			motors_controller.Update_Motors(mahony_euler);
		}
	}
}

void wifi_task(void *args) {
	const uint8_t BUFFER_SIZE = 200;

	WiFi_Controller& wifi = WiFi_Controller::Instance();
	uint8_t buffer[BUFFER_SIZE];
	uint8_t received_length;

	IMU::Euler_Angles euler[FUSION_ALGORITHMS_USED];

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

            // 1 (data size) + 1 (datagram type) + 12 * #_fusion_algorithms_used + 2 (crc)
            const uint8_t DATA_SIZE = 4 + 12 * FUSION_ALGORITHMS_USED;
            uint8_t data[DATA_SIZE];

            data[0] = DATA_SIZE;
            data[1] = 0x00;

            for (uint8_t i = 0; i < FUSION_ALGORITHMS_USED; i++) {
                roll = (uint8_t *) &(euler[i].roll);
                pitch = (uint8_t *) &(euler[i].pitch);
                yaw = (uint8_t *) &(euler[i].yaw);

                data[2 + i * 12] = roll[3];
                data[3 + i * 12] = roll[2];
                data[4 + i * 12] = roll[1];
                data[5 + i * 12] = roll[0];
                data[6 + i * 12] = pitch[3];
                data[7 + i * 12] = pitch[2];
                data[8 + i * 12] = pitch[1];
                data[9 + i * 12] = pitch[0];
                data[10 + i * 12] = yaw[3];
                data[11 + i * 12] = yaw[2];
                data[12 + i * 12] = yaw[1];
                data[13 + i * 12] = yaw[0];
            }

			uint16_t crc = CRC::CRC16(data, DATA_SIZE - 2);

            data[DATA_SIZE - 2] = crc & 0xFF;
			data[DATA_SIZE - 1] = crc >> 8;

			wifi.Send(data, DATA_SIZE);
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
