#include <nvs_flash.h>

#include "IMU_Detector.h"
#include "CRC.h"
#include "WiFi_Controller.h"
#include "Motors_Controller.h"
#include "Complementary_Filter.h"
#include "Mahony_Filter.h"


using namespace flyhero;


void imu_task(void *args);
void wifi_task(void *args);

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    LEDs::Init();

    Motors_Controller::Instance().Init();

    xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 0);
}

void imu_task(void *args)
{
    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles euler;
    euler.roll = 0;
    euler.pitch = 0;
    euler.yaw = 0;

    IMU::Euler_Angles complementary_euler, mahony_euler;
    Complementary_Filter complementary_filter(0.97);
    Mahony_Filter mahony_filter(25, 0);

    Motors_Controller& motors_controller = Motors_Controller::Instance();
    IMU& imu = IMU_Detector::Detect_IMU();

    imu.Init();

    while (!imu.Start())
        imu.Calibrate();

    while (true)
    {
        if (imu.Data_Ready())
        {
            imu.Read_Data(accel, gyro);

            complementary_filter.Compute(accel, gyro, complementary_euler);
            mahony_filter.Compute(accel, gyro, mahony_euler);

            motors_controller.Update_Motors(euler, gyro);

            printf("%f %f %f %f %f %f %f %f %f\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z,
                   (float)mahony_euler.roll, (float)mahony_euler.pitch, (float)mahony_euler.yaw);
        }
    }
}

void wifi_task(void *args)
{
    const uint8_t TCP_BUFFER_LENGTH = 50;
    char TCP_buffer[TCP_BUFFER_LENGTH];
    uint8_t received_length = 0;
    bool process_tcp = true;

    WiFi_Controller &wifi = WiFi_Controller::Instance();
    Motors_Controller& motors_controller = Motors_Controller::Instance();
    wifi.Init();

    ESP_ERROR_CHECK(wifi.TCP_Server_Start());
    ESP_ERROR_CHECK(wifi.TCP_Wait_For_Client());

    while (process_tcp)
    {
        if (wifi.TCP_Receive(TCP_buffer, TCP_BUFFER_LENGTH, &received_length))
        {
            if (strncmp((const char*)TCP_buffer, "start", 5) == 0)
            {
                wifi.TCP_Send("yup", 3);
                process_tcp = false;
            } else
                wifi.TCP_Send("nah", 3);
        }
    }

    ESP_ERROR_CHECK(wifi.TCP_Server_Stop());
    ESP_ERROR_CHECK(wifi.UDP_Server_Start());

    WiFi_Controller::In_Datagram_Data datagram_data;

    while (true)
    {
        if (wifi.UDP_Receive(datagram_data))
        {
            motors_controller.Set_Throttle(datagram_data.throttle);

            double parameters[3][3] = {
                    { datagram_data.roll_kp * 0.01, datagram_data.roll_ki * 0.01, 0 },
                    { datagram_data.pitch_kp * 0.01, datagram_data.pitch_ki * 0.01, 0 },
                    { datagram_data.yaw_kp * 0.01, datagram_data.yaw_ki * 0.01, 0 }
            };

            motors_controller.Set_PID_Constants(Motors_Controller::RATE, parameters);
        }
    }
}