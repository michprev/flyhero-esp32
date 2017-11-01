/*
 * MPU9250.h
 *
 *  Created on: 7. 9. 2017
 *      Author: michp
 */

#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/task.h>
#include <cmath>

#include "Biquad_Filter.h"
#include "IMU.h"


extern "C" {

#include <esp_heap_caps.h>

}

// debug
#include <iostream>
#include <sys/time.h>
#include <unistd.h>


namespace flyhero
{

class MPU9250 : public IMU
{
private:
    /* Singleton begin */
    MPU9250();
    MPU9250(MPU9250 const &);
    MPU9250 &operator=(MPU9250 const &);
    /*Singleton end */

    enum gyro_fsr
    {
        GYRO_FSR_250 = 0x00,
        GYRO_FSR_500 = 0x08,
        GYRO_FSR_1000 = 0x10,
        GYRO_FSR_2000 = 0x18,
        GYRO_FSR_NOT_SET = 0xFF
    };

    enum accel_fsr
    {
        ACCEL_FSR_2 = 0x00,
        ACCEL_FSR_4 = 0x08,
        ACCEL_FSR_8 = 0x10,
        ACCEL_FSR_16 = 0x18,
        ACCEL_FSR_NOT_SET = 0xFF
    };

    enum gyro_lpf
    {
        // 32 kHz
                GYRO_LPF_8800HZ = 0x00,
        // 32 kHz
                GYRO_LPF_3600Hz = 0x00,
        // should be 8 kHz, measured 32 kHz
                GYRO_LPF_250HZ = 0x00,
        // 1 kHz
                GYRO_LPF_184HZ = 0x01,
        GYRO_LPF_92HZ = 0x02,
        GYRO_LPF_41HZ = 0x03,
        GYRO_LPF_20HZ = 0x04,
        GYRO_LPF_10HZ = 0x05,
        GYRO_LPF_5HZ = 0x06,
        GYRO_LPF_NOT_SET = 0xFF
    };

    enum accel_lpf
    {
        // 4 kHz
                ACCEL_LPF_1046HZ = 0x00,
        // 1 kHz
                ACCEL_LPF_218HZ = 0x00,
        ACCEL_LPF_99HZ = 0x02,
        ACCEL_LPF_45HZ = 0x03,
        ACCEL_LPF_21HZ = 0x04,
        ACCEL_LPF_10HZ = 0x05,
        ACCEL_LPF_5HZ = 0x06,
        // do not use - outputs strange values
                ACCEL_LPF_420HZ = 0x07,
        ACCEL_LPF_NOT_SET = 0xFF
    };

    const struct
    {
        uint8_t SMPLRT_DIV = 25;
        uint8_t CONFIG = 26;
        uint8_t GYRO_CONFIG = 27;
        uint8_t ACCEL_CONFIG = 28;
        uint8_t ACCEL_CONFIG2 = 29;
        uint8_t INT_PIN_CFG = 55;
        uint8_t INT_ENABLE = 56;
        uint8_t ACCEL_XOUT_H = 59;
        uint8_t SIGNAL_PATH_RESET = 104;
        uint8_t USER_CTRL = 106;
        uint8_t PWR_MGMT_1 = 107;
        uint8_t PWR_MGMT_2 = 108;
        uint8_t WHO_AM_I = 117;
    } REGISTERS;

    const uint8_t ADC_BITS = 16;

    Biquad_Filter accel_x_filter, accel_y_filter, accel_z_filter;
    Biquad_Filter gyro_x_filter, gyro_y_filter, gyro_z_filter;
    gyro_fsr g_fsr;
    accel_fsr a_fsr;
    gyro_lpf g_lpf;
    accel_lpf a_lpf;
    float g_mult, a_mult;
    uint16_t sample_rate;
    spi_device_handle_t spi;
    bool data_ready;
    bool ready;
    float accel_offsets[3];
    float gyro_offsets[3];

    esp_err_t spi_init();
    esp_err_t int_init();
    esp_err_t spi_reg_read(uint8_t reg, uint8_t &data);
    esp_err_t spi_reg_write(uint8_t reg, uint8_t data);

    esp_err_t set_gyro_fsr(gyro_fsr fsr);
    esp_err_t set_accel_fsr(accel_fsr fsr);
    esp_err_t set_gyro_lpf(gyro_lpf lpf);
    esp_err_t set_accel_lpf(accel_lpf lpf);
    esp_err_t set_sample_rate(uint16_t rate);
    esp_err_t set_interrupt(bool enable);

public:
    static MPU9250 &Instance();

    void Init() override;
    void Calibrate() override;
    void Read_Raw(Raw_Data &raw_accel, Raw_Data &raw_gyro) override;
    void Read_Data(Sensor_Data &accel, Sensor_Data &gyro) override;
    void Data_Ready_Callback() override;
    bool Data_Ready() override;
};

} /* namespace flyhero */
