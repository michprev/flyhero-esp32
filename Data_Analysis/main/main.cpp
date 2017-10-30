#include <nvs_flash.h>

#include "IMU_Detector.h"
#include "CRC.h"
#include "WiFi_Controller.h"
#include "Motors_Controller.h"

using namespace flyhero;


void imu_task(void *args);
void wifi_task(void *args);

Motors_Controller& motors_controller = Motors_Controller::Instance();

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_status = nvs_flash_init();
    }

    ESP_ERROR_CHECK(nvs_status);

    LEDs::Init();

    motors_controller.Init();

    xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 1);

}

void imu_task(void *args) {
    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles euler;
    euler.roll = 0;
    euler.pitch = 0;
    euler.yaw = 0;

    IMU *imu;
    ESP_ERROR_CHECK(IMU_Detector::Detect_IMU(&imu));

    imu->Init();

    while (true) {
        if (imu->Data_Ready()) {
            imu->Read_Data(accel, gyro);

            motors_controller.Update_Motors(euler);

            printf("%f %f %f %f %f %f\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
        }
    }
}

void wifi_task(void *args) {
    const uint8_t BUFFER_SIZE = 200;

    WiFi_Controller& wifi = WiFi_Controller::Instance();
    uint8_t buffer[BUFFER_SIZE];
    uint8_t received_length;

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

            uint16_t throttle;
            throttle = buffer[2] << 8;
            throttle |= buffer[1];

            motors_controller.Set_Throttle(throttle);
            motors_controller.Set_PID_Constants(Roll, 1, 0, 0);
            motors_controller.Set_PID_Constants(Pitch, 1, 0, 0);
        }
    }
}