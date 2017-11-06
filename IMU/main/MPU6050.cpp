/*
 * MPU6050.cpp
 *
 *  Created on: 24. 4. 2017
 *      Author: michp
 */

#include "MPU6050.h"


namespace flyhero
{

static void int_handler(void *arg)
{
    MPU6050::Instance().Data_Ready_Callback();
}

MPU6050 &MPU6050::Instance()
{
    static MPU6050 instance;

    return instance;
}

MPU6050::MPU6050()
        : accel_x_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 10),
          accel_y_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 10),
          accel_z_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 10),
          gyro_x_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 60),
          gyro_y_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 60),
          gyro_z_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 60)
{
    this->g_fsr = GYRO_FSR_NOT_SET;
    this->g_mult = 0;
    this->a_mult = 0;
    this->a_fsr = ACCEL_FSR_NOT_SET;
    this->lpf = LPF_NOT_SET;
    this->sample_rate = 0;
    this->accel_offsets[0] = 0;
    this->accel_offsets[1] = 0;
    this->accel_offsets[2] = 0;
    this->gyro_offsets[0] = 0;
    this->gyro_offsets[1] = 0;
    this->gyro_offsets[2] = 0;
    this->ready = false;
    this->data_ready = false;
}

esp_err_t MPU6050::i2c_init()
{
    esp_err_t state;

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_5;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = GPIO_NUM_18;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;

    if ((state = i2c_param_config(I2C_NUM_0, &conf)))
        return state;
    if ((state = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0)))
        return state;

    return ESP_OK;
}

esp_err_t MPU6050::int_init()
{
    esp_err_t state;

    gpio_config_t conf;
    conf.pin_bit_mask = GPIO_SEL_4;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_POSEDGE;

    if ((state = gpio_config(&conf)))
        return state;

    if ((state = gpio_install_isr_service(0)))
        return state;
    if ((state = gpio_isr_handler_add(GPIO_NUM_4, int_handler, NULL)))
        return state;

    return ESP_OK;
}

esp_err_t MPU6050::i2c_write(uint8_t reg, uint8_t data)
{
    esp_err_t state;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if ((state = i2c_master_start(cmd)))
        return state;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        return state;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        return state;
    if ((state = i2c_master_write_byte(cmd, data, true)))
        return state;
    if ((state = i2c_master_stop(cmd)))
        return state;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
    return state;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t MPU6050::i2c_write(uint8_t reg, uint8_t *data, uint8_t data_size)
{
    esp_err_t state;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if ((state = i2c_master_start(cmd)))
        return state;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        return state;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        return state;
    if ((state = i2c_master_write(cmd, data, data_size, true)))
        return state;
    if ((state = i2c_master_stop(cmd)))
        return state;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
    return state;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t MPU6050::i2c_read(uint8_t reg, uint8_t *data)
{
    esp_err_t state;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if ((state = i2c_master_start(cmd)))
        return state;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        return state;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        return state;

    if ((state = i2c_master_start(cmd)))
        return state;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_READ, true)))
        return state;
    if ((state = i2c_master_read_byte(cmd, data, 0x01)))
        return state;
    if ((state = i2c_master_stop(cmd)))
        return state;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
    return state;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t MPU6050::i2c_read(uint8_t reg, uint8_t *data, uint8_t data_size)
{
    esp_err_t state;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if ((state = i2c_master_start(cmd)))
        return state;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        return state;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        return state;

    if ((state = i2c_master_start(cmd)))
        return state;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_READ, true)))
        return state;

    if ((state = i2c_master_read(cmd, data, data_size - 1, 0x00)))
        return state;

    if ((state = i2c_master_read_byte(cmd, data + data_size - 1, 0x01)))
        return state;

    if ((state = i2c_master_stop(cmd)))
        return state;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
    return state;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t MPU6050::set_gyro_fsr(gyro_fsr fsr)
{
    if (fsr == GYRO_FSR_NOT_SET)
        return ESP_FAIL;

    if (this->g_fsr == fsr)
        return ESP_OK;

    if (this->i2c_write(this->REGISTERS.GYRO_CONFIG, fsr) == ESP_OK)
    {
        this->g_fsr = fsr;

        switch (this->g_fsr)
        {
            case GYRO_FSR_250:
                this->g_mult = 250;
                break;
            case GYRO_FSR_500:
                this->g_mult = 500;
                break;
            case GYRO_FSR_1000:
                this->g_mult = 1000;
                break;
            case GYRO_FSR_2000:
                this->g_mult = 2000;
                break;
            case GYRO_FSR_NOT_SET:
                return ESP_FAIL;
        }

        this->g_mult /= std::pow(2, this->ADC_BITS - 1);

        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t MPU6050::set_accel_fsr(accel_fsr fsr)
{
    if (fsr == ACCEL_FSR_NOT_SET)
        return ESP_FAIL;

    if (this->a_fsr == fsr)
        return ESP_OK;

    if (this->i2c_write(this->REGISTERS.ACCEL_CONFIG, fsr) == ESP_OK)
    {
        this->a_fsr = fsr;

        switch (this->a_fsr)
        {
            case ACCEL_FSR_2:
                this->a_mult = 2;
                break;
            case ACCEL_FSR_4:
                this->a_mult = 4;
                break;
            case ACCEL_FSR_8:
                this->a_mult = 8;
                break;
            case ACCEL_FSR_16:
                this->a_mult = 16;
                break;
            case ACCEL_FSR_NOT_SET:
                return ESP_FAIL;
        }

        this->a_mult /= std::pow(2, this->ADC_BITS - 1);

        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t MPU6050::set_lpf(lpf_bandwidth lpf)
{
    if (lpf == LPF_NOT_SET)
        return ESP_FAIL;

    if (this->lpf == lpf)
        return ESP_OK;

    if (this->i2c_write(this->REGISTERS.CONFIG, lpf) == ESP_OK)
    {
        this->lpf = lpf;
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t MPU6050::set_sample_rate(uint16_t rate)
{
    if (this->sample_rate == rate)
        return ESP_OK;

    uint8_t val = (1000 - rate) / rate;

    if (this->i2c_write(this->REGISTERS.SMPRT_DIV, val) == ESP_OK)
    {
        this->sample_rate = rate;
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t MPU6050::set_interrupt(bool enable)
{
    return this->i2c_write(this->REGISTERS.INT_ENABLE, enable ? 0x01 : 0x00);
}

void MPU6050::Init()
{
    // init I2C bus including DMA peripheral
    ESP_ERROR_CHECK(this->i2c_init());

    // init INT pin on ESP32
    ESP_ERROR_CHECK(this->int_init());

    // reset device
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x80));

    // wait until reset done
    uint8_t tmp;
    do
    {
        this->i2c_read(this->REGISTERS.PWR_MGMT_1, &tmp);
    } while (tmp & 0x80);

    // reset analog devices - should not be needed
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.SIGNAL_PATH_RESET, 0x07));

    vTaskDelay(100 / portTICK_RATE_MS);

    // wake up, set clock source PLL with Z gyro axis
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x03));

    vTaskDelay(50 / portTICK_RATE_MS);

    // do not disable any sensor
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.PWR_MGMT_2, 0x00));

    // check I2C connection
    uint8_t who_am_i;
    ESP_ERROR_CHECK(this->i2c_read(this->REGISTERS.WHO_AM_I, &who_am_i));

    if (who_am_i != 0x68)
        ESP_ERROR_CHECK(ESP_FAIL);

    // disable interrupt
    ESP_ERROR_CHECK(this->set_interrupt(false));

    // set INT pin active high, push-pull; don't use latched mode, fsync nor I2C master aux
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.INT_PIN_CFG, 0x00));

    // disable I2C master aux
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.USER_CTRL, 0x20));

    // set gyro full scale range
    ESP_ERROR_CHECK(this->set_gyro_fsr(GYRO_FSR_2000));

    // set accel full scale range
    ESP_ERROR_CHECK(this->set_accel_fsr(ACCEL_FSR_16));

    // set low pass filter to 188 Hz (both acc and gyro sample at 1 kHz)
    ESP_ERROR_CHECK(this->set_lpf(LPF_188HZ));

    // set sample rate
    ESP_ERROR_CHECK(this->set_sample_rate(this->SAMPLE_RATE));

    ESP_ERROR_CHECK(this->set_interrupt(true));

    this->Calibrate();

    this->ready = true;
}

void MPU6050::Calibrate()
{
    gyro_fsr prev_g_fsr = this->g_fsr;
    accel_fsr prev_a_fsr = this->a_fsr;

    this->set_gyro_fsr(GYRO_FSR_1000);
    this->set_accel_fsr(ACCEL_FSR_16);

    uint8_t offset_data[6] = {0x00};

    // gyro offsets should be already zeroed
    // for accel we need to read factory values and preserve bit 0 of LSB for each axis
    // http://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers
    ESP_ERROR_CHECK(this->i2c_read(this->REGISTERS.ACCEL_X_OFFSET, offset_data, 6));

    int16_t accel_offsets[3];
    accel_offsets[0] = (offset_data[0] << 8) | offset_data[1];
    accel_offsets[1] = (offset_data[2] << 8) | offset_data[3];
    accel_offsets[2] = (offset_data[4] << 8) | offset_data[5];

    int32_t offsets[6] = {0};
    Raw_Data gyro, accel;

    // we want accel Z to be 2048 (+ 1g)

    for (uint16_t i = 0; i < 500; i++)
    {
        this->Read_Raw(accel, gyro);

        offsets[0] += accel.x;
        offsets[1] += accel.y;
        offsets[2] += accel.z - 2048;
        offsets[3] += gyro.x;
        offsets[4] += gyro.y;
        offsets[5] += gyro.z;
    }

    int16_t gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
    accel_x = -offsets[0] / 500;
    accel_y = -offsets[1] / 500;
    accel_z = -offsets[2] / 500;
    gyro_x = -offsets[3] / 500;
    gyro_y = -offsets[4] / 500;
    gyro_z = -offsets[5] / 500;

    accel_offsets[0] += accel_x;
    accel_offsets[1] += accel_y;
    accel_offsets[2] += accel_z;

    if (offset_data[1] & 0x01)
        accel_offsets[0] |= 0x01;
    else
        accel_offsets[0] &= 0xFFFE;

    if (offset_data[3] & 0x01)
        accel_offsets[1] |= 0x01;
    else
        accel_offsets[1] &= 0xFFFE;

    if (offset_data[5] & 0x01)
        accel_offsets[2] |= 0x01;
    else
        accel_offsets[2] &= 0xFFFE;

    offset_data[0] = accel_offsets[0] >> 8;
    offset_data[1] = accel_offsets[0] & 0xFF;
    offset_data[2] = accel_offsets[1] >> 8;
    offset_data[3] = accel_offsets[1] & 0xFF;
    offset_data[4] = accel_offsets[2] >> 8;
    offset_data[5] = accel_offsets[2] & 0xFF;

    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.ACCEL_X_OFFSET, offset_data, 6));

    offset_data[0] = gyro_x >> 8;
    offset_data[1] = gyro_x & 0xFF;
    offset_data[2] = gyro_y >> 8;
    offset_data[3] = gyro_y & 0xFF;
    offset_data[4] = gyro_z >> 8;
    offset_data[5] = gyro_z & 0xFF;

    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.GYRO_X_OFFSET, offset_data, 6));

    // set gyro & accel FSR to its original value
    this->set_gyro_fsr(prev_g_fsr);
    this->set_accel_fsr(prev_a_fsr);

    vTaskDelay(5000 / portTICK_RATE_MS);

    // lets measure offsets again to be applied on STM
    this->accel_offsets[0] = this->accel_offsets[1] = this->accel_offsets[2] = 0;
    this->gyro_offsets[0] = this->gyro_offsets[1] = this->gyro_offsets[2] = 0;

    for (uint16_t i = 0; i < 500; i++)
    {
        this->Read_Raw(accel, gyro);

        this->accel_offsets[0] += accel.x;
        this->accel_offsets[1] += accel.y;
        this->accel_offsets[2] += accel.z - 2048;
        this->gyro_offsets[0] += gyro.x;
        this->gyro_offsets[1] += gyro.y;
        this->gyro_offsets[2] += gyro.z;
    }

    this->accel_offsets[0] /= -500;
    this->accel_offsets[1] /= -500;
    this->accel_offsets[2] /= -500;
    this->gyro_offsets[0] /= -500;
    this->gyro_offsets[1] /= -500;
    this->gyro_offsets[2] /= -500;
}

uint16_t MPU6050::Get_Sample_Rate()
{
    return this->SAMPLE_RATE;
}

void MPU6050::Read_Raw(Raw_Data &accel, Raw_Data &gyro)
{
    uint8_t data[14];

    ESP_ERROR_CHECK(this->i2c_read(this->REGISTERS.ACCEL_XOUT_H, data, 14));

    accel.x = (data[2] << 8) | data[3];
    accel.y = (data[0] << 8) | data[1];
    accel.z = (data[4] << 8) | data[5];

    gyro.x = (data[10] << 8) | data[11];
    gyro.y = (data[8] << 8) | data[9];
    gyro.z = (data[12] << 8) | data[13];
}

void MPU6050::Read_Data(Sensor_Data &accel, Sensor_Data &gyro)
{
    uint8_t data[14];

    ESP_ERROR_CHECK(this->i2c_read(this->REGISTERS.ACCEL_XOUT_H, data, 14));

    Raw_Data raw_accel, raw_gyro;

    raw_accel.x = (data[2] << 8) | data[3];
    raw_accel.y = (data[0] << 8) | data[1];
    raw_accel.z = (data[4] << 8) | data[5];

    raw_gyro.x = (data[10] << 8) | data[11];
    raw_gyro.y = (data[8] << 8) | data[9];
    raw_gyro.z = (data[12] << 8) | data[13];

    accel.x = this->accel_x_filter.Apply_Filter((raw_accel.x + this->accel_offsets[0]) * this->a_mult);
    accel.y = this->accel_y_filter.Apply_Filter((raw_accel.y + this->accel_offsets[1]) * this->a_mult);
    accel.z = this->accel_z_filter.Apply_Filter((raw_accel.z + this->accel_offsets[2]) * this->a_mult);

    gyro.x = this->gyro_x_filter.Apply_Filter((raw_gyro.x + this->gyro_offsets[0]) * this->g_mult);
    gyro.y = this->gyro_y_filter.Apply_Filter((raw_gyro.y + this->gyro_offsets[1]) * this->g_mult);
    gyro.z = this->gyro_z_filter.Apply_Filter((raw_gyro.z + this->gyro_offsets[2]) * this->g_mult);
}

void MPU6050::Data_Ready_Callback()
{
    portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

    vTaskEnterCritical(&mutex);

    if (this->ready)
        this->data_ready = true;

    vTaskExitCritical(&mutex);
}

bool MPU6050::Data_Ready()
{
    bool tmp = this->data_ready;

    if (this->data_ready)
        this->data_ready = false;

    return tmp;
}

} /* namespace The_Eye */
