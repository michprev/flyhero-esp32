#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <nvs_flash.h>
#include <esp_task_wdt.h>
#include <driver/periph_ctrl.h>
#include <soc/timer_group_struct.h>

#include "Motors_Controller.h"
#include "IMU_Detector.h"
#include "Mahony_Filter.h"
#include "Complementary_Filter.h"
#include "WiFi_Controller.h"


using namespace flyhero;

Motors_Controller & motors_controller = Motors_Controller::Instance();

void wifi_task(void * args);

void imu_task(void * args);

void gpio_isr_task(void * args)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3));

    vTaskDelete(NULL);
}

void TCP_process(const char * command);

extern "C" void app_main(void)
{
    // Initialize NVS
    esp_err_t nvs_status = nvs_flash_init();

    if (nvs_status == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    xTaskCreatePinnedToCore(gpio_isr_task, "GPIO ISR register task", 2048, NULL, 2, NULL, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    LEDs::Init();
    motors_controller.Init();

    IMU & imu = IMU_Detector::Detect_IMU();
    imu.Init();

    xTaskCreatePinnedToCore(wifi_task, "WiFi task", 4096, NULL, 2, NULL, 0);
}

void imu_task(void * args)
{
    IMU::Sensor_Data accel, gyro;
    IMU::Euler_Angles complementary_euler;
    IMU::Read_Data_Type data_type;

    IMU & imu = IMU_Detector::Detect_IMU();
    Complementary_Filter complementary(0.97f);

    // some dummy reads to heat up SPI/I2C drivers
    imu.Read_Data(accel, gyro);
    imu.Read_Data(accel, gyro);
    imu.Read_Data(accel, gyro);

    periph_module_enable(PERIPH_TIMG1_MODULE);  // enable TIMG1 watchdog clock
    TIMERG1.wdt_wprotect = 0x50d83aa1;          // disable write protection
    TIMERG1.wdt_config0.sys_reset_length = 7;
    TIMERG1.wdt_config0.cpu_reset_length = 7;
    TIMERG1.wdt_config0.level_int_en = 0;       // disable level interrupt
    TIMERG1.wdt_config0.edge_int_en = 0;        // disable edge interrupt
    TIMERG1.wdt_config0.stg0 = 3;               // reset chip when reaching stage 0 limit
    TIMERG1.wdt_config0.stg1 = 0;               //
    TIMERG1.wdt_config0.stg2 = 0;               // disable other stages
    TIMERG1.wdt_config0.stg3 = 0;               //
    TIMERG1.wdt_config1.clk_prescale = 80;      // tick every 1 us
    TIMERG1.wdt_config2 = 1100;                 // wait 1100 ticks (1100 microseconds) before resetting chip
    TIMERG1.wdt_config0.en = 1;
    TIMERG1.wdt_feed = 1;
    TIMERG1.wdt_wprotect = 0;                   // enable write protection

    while (true)
    {
        if (imu.Data_Ready())
        {
            TIMERG1.wdt_wprotect = 0x50d83aa1;
            TIMERG1.wdt_feed = 1;
            TIMERG1.wdt_wprotect = 0;

            data_type = imu.Read_Data(accel, gyro);

            complementary.Compute(accel, gyro, complementary_euler);

            if (data_type == IMU::Read_Data_Type::ACCEL_GYRO)
            {
                motors_controller.Feed_Stab_PIDs(complementary_euler);
                motors_controller.Feed_Rate_PIDs(gyro);
            } else
                motors_controller.Feed_Rate_PIDs(gyro);
        }
    }
}

void wifi_task(void * args)
{
    WiFi_Controller & wifi = WiFi_Controller::Instance();
    WiFi_Controller::In_Datagram_Data in_datagram_data;

    const char * received_data;
    size_t received_length;
    bool process_tcp, process_udp;

    wifi.Init();

    ESP_ERROR_CHECK(wifi.UDP_Server_Start());
    ESP_ERROR_CHECK(wifi.TCP_Server_Start());
    ESP_ERROR_CHECK(wifi.TCP_Wait_For_Client());

    while (true)
    {
        // TCP phase

        process_tcp = true;

        while (process_tcp)
        {
            if (wifi.TCP_Receive(received_data, received_length))
            {
                if (strncmp(received_data, "start", strlen("start")) == 0)
                {
                    IMU_Detector::Detect_IMU().Gyro_Calibrate();
                    if (IMU_Detector::Detect_IMU().Start())
                    {
                        motors_controller.Start();
                        wifi.TCP_Send("yup", 3);
                        process_tcp = false;
                    } else
                        wifi.TCP_Send("nah", 3);
                } else
                    TCP_process(received_data);
            }
            vTaskDelay(20);
        }

        TaskHandle_t imu_task_handle;
        assert(xTaskCreatePinnedToCore(imu_task, "IMU task", 4096, NULL, 20, &imu_task_handle, 1) == pdTRUE);

        periph_module_enable(PERIPH_TIMG0_MODULE);  // enable TIMG0 watchdog clock
        TIMERG0.wdt_wprotect = 0x50d83aa1;          // disable write protection
        TIMERG0.wdt_config0.sys_reset_length = 7;
        TIMERG0.wdt_config0.cpu_reset_length = 7;
        TIMERG0.wdt_config0.level_int_en = 0;       // disable level interrupt
        TIMERG0.wdt_config0.edge_int_en = 0;        // disable edge interrupt
        TIMERG0.wdt_config0.stg0 = 3;               // reset chip when reaching stage 0 limit
        TIMERG0.wdt_config0.stg1 = 0;               //
        TIMERG0.wdt_config0.stg2 = 0;               // disable other stages
        TIMERG0.wdt_config0.stg3 = 0;               //
        TIMERG0.wdt_config1.clk_prescale = 40000;   // tick every 500 us
        TIMERG0.wdt_config2 = 2000;                 // wait 2000 ticks (1 second) before resetting chip
        TIMERG0.wdt_config0.en = 1;
        TIMERG0.wdt_feed = 1;
        TIMERG0.wdt_wprotect = 0;                   // enable write protection

        // UDP phase

        process_udp = true;

        while (process_udp)
        {
            if (wifi.UDP_Receive(in_datagram_data))
            {
                TIMERG0.wdt_wprotect = 0x50d83aa1;
                TIMERG0.wdt_feed = 1;
                TIMERG0.wdt_wprotect = 0;

                float rate_parameters[3][3] = {
                        { in_datagram_data.rate_roll_kp * 0.01f,  0, 0 },
                        { in_datagram_data.rate_pitch_kp * 0.01f, 0, 0 },
                        { in_datagram_data.rate_yaw_kp * 0.01f,   0, 0 }
                };

                float stab_parameters[3][3] = {
                        { in_datagram_data.stab_roll_kp * 0.01f,  in_datagram_data.stab_roll_ki * 0.01f,  0 },
                        { in_datagram_data.stab_pitch_kp * 0.01f, in_datagram_data.stab_pitch_ki * 0.01f, 0 },
                        { 0,                                      0,                                      0 }
                };

                motors_controller.Set_Throttle(in_datagram_data.throttle);
                motors_controller.Set_PID_Constants(Motors_Controller::RATE, rate_parameters);
                motors_controller.Set_PID_Constants(Motors_Controller::STABILIZE, stab_parameters);
            }
            if (wifi.TCP_Receive(received_data, received_length))
            {
                if (strncmp(received_data, "stop", strlen("stop")) == 0)
                {
                    motors_controller.Stop();

                    // motors disabled, safe to disable both watchdogs
                    TIMERG0.wdt_wprotect = 0x50d83aa1;
                    TIMERG0.wdt_config0.en = 0;
                    TIMERG0.wdt_wprotect = 0;

                    TIMERG1.wdt_wprotect = 0x50d83aa1;
                    TIMERG1.wdt_config0.en = 0;
                    TIMERG1.wdt_wprotect = 0;

                    vTaskDelete(imu_task_handle);
                    IMU_Detector::Detect_IMU().Stop();
                    process_udp = false;

                    wifi.TCP_Send("yup", 3);
                } else
                    wifi.TCP_Send("nah", 3);
            }
            vTaskDelay(20);
        }
    }
}

void TCP_process(const char * command)
{
    WiFi_Controller & wifi = WiFi_Controller::Instance();

    if (strncmp(command, "calibrate", strlen("calibrate")) == 0)
    {
        IMU_Detector::Detect_IMU().Accel_Calibrate();
        wifi.TCP_Send("yup", 3);
    } else
        wifi.TCP_Send("nah", 3);
}