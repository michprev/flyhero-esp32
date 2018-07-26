/*
 * MPU6050.h
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#pragma once

#include <cstdint>
#include <nvs_flash.h>
#include <driver/i2c.h>

#include "Biquad_Filter.h"
#include "IMU.h"
#include "Counting_Median_Finder.h"


namespace flyhero
{

class MPU6050 : public IMU
{
private:
    MPU6050();

    MPU6050(MPU6050 const &);

    MPU6050 &operator=(MPU6050 const &);

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

    enum lpf_bandwidth
    {
        // 8 kHz
        LPF_256HZ = 0x00,
        // 1 kHz
        LPF_188HZ = 0x01,
        LPF_98HZ = 0x02,
        LPF_42HZ = 0x03,
        LPF_20HZ = 0x04,
        LPF_10HZ = 0x05,
        LPF_5HZ = 0x06,
        LPF_NOT_SET = 0xFF
    };

    const float GYRO_SAMPLE_RATE = 1000;
    const float ACCEL_SAMPLE_RATE = 1000;
    const uint8_t SAMPLE_RATES_RATIO = 1;
    const uint8_t ADC_BITS = 16;
    const uint8_t I2C_ADDRESS_WRITE = (0x68 << 1);
    const uint8_t I2C_ADDRESS_READ = ((0x68 << 1) | 0x01);
    const uint16_t I2C_TIMEOUT = 500;

    const struct
    {
        uint8_t ACCEL_X_OFFSET = 0x06;
        uint8_t GYRO_X_OFFSET = 0x13;
        uint8_t SMPRT_DIV = 0x19;
        uint8_t CONFIG = 0x1A;
        uint8_t GYRO_CONFIG = 0x1B;
        uint8_t ACCEL_CONFIG = 0x1C;
        uint8_t FIFO_EN = 0x23;
        uint8_t INT_PIN_CFG = 0x37;
        uint8_t INT_ENABLE = 0x38;
        uint8_t INT_STATUS = 0x3A;
        uint8_t ACCEL_XOUT_H = 0x3B;
        uint8_t GYRO_XOUT_H = 0x43;
        uint8_t SIGNAL_PATH_RESET = 0x68;
        uint8_t USER_CTRL = 0x6A;
        uint8_t PWR_MGMT_1 = 0x6B;
        uint8_t PWR_MGMT_2 = 0x6C;
        uint8_t FIFO_COUNT_H = 0x72;
        uint8_t FIFO_COUNT_L = 0x73;
        uint8_t FIFO_R_W = 0x74;
        uint8_t WHO_AM_I = 0x75;
    } REGISTERS;

#if CONFIG_FLYHERO_IMU_USE_SOFT_LPF
    Biquad_Filter accel_x_filter, accel_y_filter, accel_z_filter;
    Biquad_Filter gyro_x_filter, gyro_y_filter, gyro_z_filter;
#endif
#if CONFIG_FLYHERO_IMU_USE_NOTCH
    Biquad_Filter gyro_x_notch_filter, gyro_y_notch_filter, gyro_z_notch_filter;
#endif

#if CONFIG_FLYHERO_IMU_GYRO_FSR_250
    const gyro_fsr TARGET_GYRO_FSR = GYRO_FSR_250;
#elif CONFIG_FLYHERO_IMU_GYRO_FSR_500
    const gyro_fsr TARGET_GYRO_FSR = GYRO_FSR_500;
#elif CONFIG_FLYHERO_IMU_GYRO_FSR_1000
    const gyro_fsr TARGET_GYRO_FSR = GYRO_FSR_1000;
#elif CONFIG_FLYHERO_IMU_GYRO_FSR_2000
    const gyro_fsr TARGET_GYRO_FSR = GYRO_FSR_2000;
#else
#error "Gyro FSR not set"
#endif

#if CONFIG_FLYHERO_IMU_ACCEL_FSR_2
    const accel_fsr TARGET_ACCEL_FSR = ACCEL_FSR_2;
#elif CONFIG_FLYHERO_IMU_ACCEL_FSR_4
    const accel_fsr TARGET_ACCEL_FSR = ACCEL_FSR_4;
#elif CONFIG_FLYHERO_IMU_ACCEL_FSR_8
    const accel_fsr TARGET_ACCEL_FSR = ACCEL_FSR_8;
#elif CONFIG_FLYHERO_IMU_ACCEL_FSR_16
    const accel_fsr TARGET_ACCEL_FSR = ACCEL_FSR_16;
#else
#error "Accel FSR not set"
#endif

#if CONFIG_FLYHERO_IMU_HARD_LPF_256HZ
    const lpf_bandwidth TARGET_LPF = LPF_256HZ;
#elif CONFIG_FLYHERO_IMU_HARD_LPF_188HZ
    const lpf_bandwidth TARGET_LPF = LPF_188HZ;
#elif CONFIG_FLYHERO_IMU_HARD_LPF_98HZ
    const lpf_bandwidth TARGET_LPF = LPF_98HZ;
#elif CONFIG_FLYHERO_IMU_HARD_LPF_42HZ
    const lpf_bandwidth TARGET_LPF = LPF_42HZ;
#elif CONFIG_FLYHERO_IMU_HARD_LPF_20HZ
    const lpf_bandwidth TARGET_LPF = LPF_20HZ;
#elif CONFIG_FLYHERO_IMU_HARD_LPF_10HZ
    const lpf_bandwidth TARGET_LPF = LPF_10HZ;
#elif CONFIG_FLYHERO_IMU_HARD_LPF_5HZ
    const lpf_bandwidth TARGET_LPF = LPF_5HZ;
#else
#error "Gyro hardware LPF not set"
#endif

    gyro_fsr g_fsr;
    float g_mult;
    float a_mult;
    accel_fsr a_fsr;
    lpf_bandwidth lpf;
    bool sample_rate_divider_set;
    uint8_t sample_rate_divider;
    int16_t accel_offsets[3];
    int16_t gyro_offsets[3];
    volatile bool data_ready;

    esp_err_t i2c_init();

    esp_err_t int_init();

    esp_err_t i2c_write(uint8_t reg, uint8_t data);

    esp_err_t i2c_write(uint8_t reg, uint8_t *data, uint8_t data_size);

    esp_err_t i2c_read(uint8_t reg, uint8_t *data);

    esp_err_t i2c_read(uint8_t reg, uint8_t *data, uint8_t data_size);

    esp_err_t set_gyro_fsr(gyro_fsr fsr);

    esp_err_t set_accel_fsr(accel_fsr fsr);

    esp_err_t set_lpf(lpf_bandwidth lpf);

    esp_err_t set_sample_rate_divider(uint8_t divider);

    esp_err_t set_interrupt(bool enable);

    esp_err_t load_accel_offsets();

public:
    static MPU6050 &Instance();

    void Init() override;

    bool Start() override;

    void Stop() override;

    void Accel_Calibrate() override;

    void Gyro_Calibrate() override;

    float Get_Accel_Sample_Rate() override;

    float Get_Gyro_Sample_Rate() override;

    uint8_t Get_Sample_Rates_Ratio() override;

    void Read_Raw(Raw_Data &raw_accel, Raw_Data &raw_gyro) override;

    IMU::Read_Data_Type Read_Data(Sensor_Data &accel, Sensor_Data &gyro) override;

    void Data_Ready_Callback() override;

    bool Data_Ready() override;
};

} /* namespace The_Eye */
