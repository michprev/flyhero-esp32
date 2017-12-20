//
// Created by michal on 22.10.17.
//

#pragma once

#include <driver/i2c.h>
#include <driver/spi_master.h>

#include "IMU.h"
#include "MPU6050.h"
#include "MPU9250.h"
#include "MPU6000.h"


namespace flyhero
{

class IMU_Detector
{
private:
    static spi_device_handle_t spi;
    static IMU *imu;
    static bool detected;

    static bool i2c_init();
    static void i2c_deinit();
    static bool spi_init();
    static void spi_deinit();

    static bool try_i2c_imu(const uint8_t DEVICE_ADDRESS_WRITE, const uint8_t DEVICE_ADDRESS_READ,
                            const uint8_t WHO_AM_I_REGISTER, const uint8_t EXPECTED_VALUE);
    static bool try_spi_imu(const uint8_t WHO_AM_I_REGISTER, const uint8_t EXPECTED_VALUE, const gpio_num_t CS_NUM);

public:
    static IMU& Detect_IMU();

};

}
