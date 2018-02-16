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
#if CONFIG_FLYHERO_IMU_USE_SOFT_LPF
        : accel_x_filter(Biquad_Filter::FILTER_LOW_PASS, this->ACCEL_SAMPLE_RATE, CONFIG_FLYHERO_IMU_ACCEL_SOFT_LPF),
          accel_y_filter(Biquad_Filter::FILTER_LOW_PASS, this->ACCEL_SAMPLE_RATE, CONFIG_FLYHERO_IMU_ACCEL_SOFT_LPF),
          accel_z_filter(Biquad_Filter::FILTER_LOW_PASS, this->ACCEL_SAMPLE_RATE, CONFIG_FLYHERO_IMU_ACCEL_SOFT_LPF),
          gyro_x_filter(Biquad_Filter::FILTER_LOW_PASS, this->GYRO_SAMPLE_RATE, CONFIG_FLYHERO_IMU_GYRO_SOFT_LPF),
          gyro_y_filter(Biquad_Filter::FILTER_LOW_PASS, this->GYRO_SAMPLE_RATE, CONFIG_FLYHERO_IMU_GYRO_SOFT_LPF),
          gyro_z_filter(Biquad_Filter::FILTER_LOW_PASS, this->GYRO_SAMPLE_RATE, CONFIG_FLYHERO_IMU_GYRO_SOFT_LPF)
#endif
#if CONFIG_FLYHERO_IMU_USE_NOTCH
    #if CONFIG_FLYHERO_IMU_USE_SOFT_LPF
          ,
    #endif
          gyro_x_notch_filter(Biquad_Filter::FILTER_NOTCH, this->GYRO_SAMPLE_RATE, CONFIG_FLYHERO_IMU_GYRO_NOTCH),
          gyro_y_notch_filter(Biquad_Filter::FILTER_NOTCH, this->GYRO_SAMPLE_RATE, CONFIG_FLYHERO_IMU_GYRO_NOTCH),
          gyro_z_notch_filter(Biquad_Filter::FILTER_NOTCH, this->GYRO_SAMPLE_RATE, CONFIG_FLYHERO_IMU_GYRO_NOTCH)
#endif
{
    this->g_fsr = GYRO_FSR_NOT_SET;
    this->g_mult = 0;
    this->a_mult = 0;
    this->a_fsr = ACCEL_FSR_NOT_SET;
    this->lpf = LPF_NOT_SET;
    this->sample_rate_divider_set = false;
    this->sample_rate_divider = 0;
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
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, data, true)))
        goto i2c_error;
    if ((state = i2c_master_stop(cmd)))
        goto i2c_error;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
        goto i2c_error;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;

    i2c_error:

    i2c_cmd_link_delete(cmd);
    return state;
}

esp_err_t MPU6050::i2c_write(uint8_t reg, uint8_t *data, uint8_t data_size)
{
    esp_err_t state;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if ((state = i2c_master_start(cmd)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        goto i2c_error;
    if ((state = i2c_master_write(cmd, data, data_size, true)))
        goto i2c_error;
    if ((state = i2c_master_stop(cmd)))
        goto i2c_error;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
        goto i2c_error;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;

    i2c_error:

    i2c_cmd_link_delete(cmd);
    return state;
}

esp_err_t MPU6050::i2c_read(uint8_t reg, uint8_t *data)
{
    esp_err_t state;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if ((state = i2c_master_start(cmd)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        goto i2c_error;

    if ((state = i2c_master_start(cmd)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_READ, true)))
        goto i2c_error;
    if ((state = i2c_master_read_byte(cmd, data, I2C_MASTER_NACK)))
        goto i2c_error;
    if ((state = i2c_master_stop(cmd)))
        goto i2c_error;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
        goto i2c_error;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;

    i2c_error:

    i2c_cmd_link_delete(cmd);
    return state;
}

esp_err_t MPU6050::i2c_read(uint8_t reg, uint8_t *data, uint8_t data_size)
{
    esp_err_t state;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if ((state = i2c_master_start(cmd)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_WRITE, true)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, reg, true)))
        goto i2c_error;

    if ((state = i2c_master_start(cmd)))
        goto i2c_error;
    if ((state = i2c_master_write_byte(cmd, this->I2C_ADDRESS_READ, true)))
        goto i2c_error;

    if ((state = i2c_master_read(cmd, data, data_size, I2C_MASTER_LAST_NACK)))
        goto i2c_error;

    if ((state = i2c_master_stop(cmd)))
        goto i2c_error;

    if ((state = i2c_master_cmd_begin(I2C_NUM_0, cmd, this->I2C_TIMEOUT / portTICK_RATE_MS)))
        goto i2c_error;

    i2c_cmd_link_delete(cmd);

    return ESP_OK;

    i2c_error:

    i2c_cmd_link_delete(cmd);
    return state;
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

esp_err_t MPU6050::set_sample_rate_divider(uint8_t divider)
{
    if (this->sample_rate_divider == divider && this->sample_rate_divider_set)
        return ESP_OK;

    if (this->i2c_write(this->REGISTERS.SMPRT_DIV, divider) == ESP_OK)
    {
        this->sample_rate_divider_set = true;
        this->sample_rate_divider = divider;
        return ESP_OK;
    }

    return ESP_FAIL;
}

esp_err_t MPU6050::set_interrupt(bool enable)
{
    return this->i2c_write(this->REGISTERS.INT_ENABLE, enable ? 0x01 : 0x00);
}

esp_err_t MPU6050::load_accel_offsets()
{
    esp_err_t ret;

    nvs_handle handle;

    if ((ret = nvs_open("MPU6050", NVS_READONLY, &handle)) != ESP_OK)
        return ret;

    union
    {
        uint32_t uint32_value;
        float float_value;
    } uint32_float_converter;

    if ((ret = nvs_get_u32(handle, "accel_x_offset", &(uint32_float_converter.uint32_value))) != ESP_OK)
        goto nvs_error;
    this->accel_offsets[0] = uint32_float_converter.float_value;

    if ((ret = nvs_get_u32(handle, "accel_y_offset", &(uint32_float_converter.uint32_value))) != ESP_OK)
        goto nvs_error;
    this->accel_offsets[1] = uint32_float_converter.float_value;

    if ((ret = nvs_get_u32(handle, "accel_z_offset", &(uint32_float_converter.uint32_value))) != ESP_OK)
        goto nvs_error;
    this->accel_offsets[2] = uint32_float_converter.float_value;

    nvs_error:

    nvs_close(handle);
    return ret;
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

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // reset analog devices - should not be needed
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.SIGNAL_PATH_RESET, 0x07));

    // wake up, set clock source PLL with Z gyro axis
    ESP_ERROR_CHECK(this->i2c_write(this->REGISTERS.PWR_MGMT_1, 0x03));

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
#if CONFIG_FLYHERO_IMU_GYRO_FSR_250
    ESP_ERROR_CHECK(this->set_gyro_fsr(GYRO_FSR_250));
#elif CONFIG_FLYHERO_IMU_GYRO_FSR_500
    ESP_ERROR_CHECK(this->set_gyro_fsr(GYRO_FSR_500));
#elif CONFIG_FLYHERO_IMU_GYRO_FSR_1000
    ESP_ERROR_CHECK(this->set_gyro_fsr(GYRO_FSR_1000));
#elif CONFIG_FLYHERO_IMU_GYRO_FSR_2000
    ESP_ERROR_CHECK(this->set_gyro_fsr(GYRO_FSR_2000));
#else
#error "Gyro FSR not set"
#endif

    // set accel full scale range
#if CONFIG_FLYHERO_IMU_ACCEL_FSR_2
    ESP_ERROR_CHECK(this->set_accel_fsr(ACCEL_FSR_2));
#elif CONFIG_FLYHERO_IMU_ACCEL_FSR_4
    ESP_ERROR_CHECK(this->set_accel_fsr(ACCEL_FSR_4));
#elif CONFIG_FLYHERO_IMU_ACCEL_FSR_8
    ESP_ERROR_CHECK(this->set_accel_fsr(ACCEL_FSR_8));
#elif CONFIG_FLYHERO_IMU_ACCEL_FSR_16
    ESP_ERROR_CHECK(this->set_accel_fsr(ACCEL_FSR_16));
#else
#error "Accel FSR not set"
#endif

    // set low pass filter
#if CONFIG_FLYHERO_IMU_HARD_LPF_256HZ
    ESP_ERROR_CHECK(this->set_lpf(LPF_256HZ));
#elif CONFIG_FLYHERO_IMU_HARD_LPF_188HZ
    ESP_ERROR_CHECK(this->set_lpf(LPF_188HZ));
#elif CONFIG_FLYHERO_IMU_HARD_LPF_98HZ
    ESP_ERROR_CHECK(this->set_lpf(LPF_98HZ));
#elif CONFIG_FLYHERO_IMU_HARD_LPF_42HZ
    ESP_ERROR_CHECK(this->set_lpf(LPF_42HZ));
#elif CONFIG_FLYHERO_IMU_HARD_LPF_20HZ
    ESP_ERROR_CHECK(this->set_lpf(LPF_20HZ));
#elif CONFIG_FLYHERO_IMU_HARD_LPF_10HZ
    ESP_ERROR_CHECK(this->set_lpf(LPF_10HZ));
#elif CONFIG_FLYHERO_IMU_HARD_LPF_5HZ
    ESP_ERROR_CHECK(this->set_lpf(LPF_5HZ));
#else
#error "Gyro hardware LPF not set"
#endif

    // set sample rate
#if CONFIG_FLYHERO_IMU_HARD_LPF_256HZ
    ESP_ERROR_CHECK(this->set_sample_rate_divider(7));
#else
    ESP_ERROR_CHECK(this->set_sample_rate_divider(0));
#endif

    vTaskDelay(100 / portTICK_PERIOD_MS);
}

bool MPU6050::Start()
{
    if (this->load_accel_offsets() != ESP_OK)
        return false;

    ESP_ERROR_CHECK(this->set_interrupt(true));
    this->ready = true;

    return true;
}

void MPU6050::Accel_Calibrate()
{
    Raw_Data accel, gyro;

    this->accel_offsets[0] = this->accel_offsets[1] = this->accel_offsets[2] = 0;

    int16_t accel_z_calibration_value;

    switch (this->a_fsr)
    {
        case ACCEL_FSR_2:
            accel_z_calibration_value = 16384;
            break;
        case ACCEL_FSR_4:
            accel_z_calibration_value = 8192;
            break;
        case ACCEL_FSR_8:
            accel_z_calibration_value = 4096;
            break;
        case ACCEL_FSR_16:
            accel_z_calibration_value = 2048;
            break;
        default:
            return;
    }

    for (uint16_t i = 0; i < 5000; i++)
    {
        this->Read_Raw(accel, gyro);

        this->accel_offsets[0] += accel.x;
        this->accel_offsets[1] += accel.y;
        this->accel_offsets[2] += accel.z - accel_z_calibration_value;
    }

    this->accel_offsets[0] /= -5000;
    this->accel_offsets[1] /= -5000;
    this->accel_offsets[2] /= -5000;

    nvs_handle handle;

    if (nvs_open("MPU6050", NVS_READWRITE, &handle) != ESP_OK)
        return;

    union
    {
        uint32_t uint32_value;
        float float_value;
    } float_uint32_converter;

    float_uint32_converter.float_value = this->accel_offsets[0];
    nvs_set_u32(handle, "accel_x_offset", float_uint32_converter.uint32_value);

    float_uint32_converter.float_value = this->accel_offsets[1];
    nvs_set_u32(handle, "accel_y_offset", float_uint32_converter.uint32_value);

    float_uint32_converter.float_value = this->accel_offsets[2];
    nvs_set_u32(handle, "accel_z_offset", float_uint32_converter.uint32_value);

    nvs_close(handle);
}

void MPU6050::Gyro_Calibrate()
{
    Raw_Data accel, gyro;

    this->gyro_offsets[0] = this->gyro_offsets[1] = this->gyro_offsets[2] = 0;

    for (uint16_t i = 0; i < 5000; i++)
    {
        this->Read_Raw(accel, gyro);

        this->gyro_offsets[0] += gyro.x;
        this->gyro_offsets[1] += gyro.y;
        this->gyro_offsets[2] += gyro.z;
    }

    this->gyro_offsets[0] /= -5000;
    this->gyro_offsets[1] /= -5000;
    this->gyro_offsets[2] /= -5000;
}

float MPU6050::Get_Accel_Sample_Rate()
{
    return this->ACCEL_SAMPLE_RATE;
}

float MPU6050::Get_Gyro_Sample_Rate()
{
    return this->GYRO_SAMPLE_RATE;
}

uint8_t MPU6050::Get_Sample_Rates_Ratio()
{
    return this->SAMPLE_RATES_RATIO;
}

void MPU6050::Read_Raw(Raw_Data &accel, Raw_Data &gyro)
{
    uint8_t data[14];

    ESP_ERROR_CHECK(this->i2c_read(this->REGISTERS.ACCEL_XOUT_H, data, 14));

    accel.x = (data[2] << 8) | data[3];
    accel.y = (data[0] << 8) | data[1];
    accel.z = (data[4] << 8) | data[5];

    gyro.x = -((data[8] << 8) | data[9]);
    gyro.y = -((data[10] << 8) | data[11]);
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

    raw_gyro.x = -((data[8] << 8) | data[9]);
    raw_gyro.y = -((data[10] << 8) | data[11]);
    raw_gyro.z = (data[12] << 8) | data[13];

    accel.x = (raw_accel.x + this->accel_offsets[0]) * this->a_mult;
    accel.y = (raw_accel.y + this->accel_offsets[1]) * this->a_mult;
    accel.z = (raw_accel.z + this->accel_offsets[2]) * this->a_mult;

    gyro.x = (raw_gyro.x + this->gyro_offsets[0]) * this->g_mult;
    gyro.y = (raw_gyro.y + this->gyro_offsets[1]) * this->g_mult;
    gyro.z = (raw_gyro.z + this->gyro_offsets[2]) * this->g_mult;

#if CONFIG_FLYHERO_IMU_USE_NOTCH
    gyro.x = this->gyro_x_notch_filter.Apply_Filter(gyro.x);
    gyro.y = this->gyro_y_notch_filter.Apply_Filter(gyro.y);
    gyro.z = this->gyro_z_notch_filter.Apply_Filter(gyro.z);
#endif
#if CONFIG_FLYHERO_IMU_USE_SOFT_LPF
    accel.x = this->accel_x_filter.Apply_Filter(accel.x);
    accel.y = this->accel_y_filter.Apply_Filter(accel.y);
    accel.z = this->accel_z_filter.Apply_Filter(accel.z);

    gyro.x = this->gyro_x_filter.Apply_Filter(gyro.x);
    gyro.y = this->gyro_y_filter.Apply_Filter(gyro.y);
    gyro.z = this->gyro_z_filter.Apply_Filter(gyro.z);
#endif
}

void MPU6050::Data_Ready_Callback()
{
    if (this->ready)
        this->data_ready = true;
}

bool MPU6050::Data_Ready()
{
    bool tmp = this->data_ready;

    if (this->data_ready)
        this->data_ready = false;

    return tmp;
}

} /* namespace The_Eye */
