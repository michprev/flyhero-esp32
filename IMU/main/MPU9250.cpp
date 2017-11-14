/*
 * MPU9250.cpp
 *
 *  Created on: 7. 9. 2017
 *      Author: michp
 */

#include "MPU9250.h"


namespace flyhero
{

static void IRAM_ATTR int_isr_handler(void *arg)
{
    MPU9250::Instance().Data_Ready_Callback();
}

MPU9250 &MPU9250::Instance()
{
    static MPU9250 instance;

    return instance;
}

MPU9250::MPU9250()
        : accel_x_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 10),
          accel_y_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 10),
          accel_z_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 10),
          gyro_x_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 60),
          gyro_y_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 60),
          gyro_z_filter(Biquad_Filter::FILTER_LOW_PASS, this->SAMPLE_RATE, 60)
{
    this->spi = NULL;
    this->a_fsr = ACCEL_FSR_NOT_SET;
    this->a_lpf = ACCEL_LPF_NOT_SET;
    this->a_mult = 0;
    this->g_fsr = GYRO_FSR_NOT_SET;
    this->g_lpf = GYRO_LPF_NOT_SET;
    this->g_mult = 0;
    this->sample_rate = 0;
    this->data_ready = false;
    this->ready = false;
    this->accel_offsets[0] = 0;
    this->accel_offsets[1] = 0;
    this->accel_offsets[2] = 0;
    this->gyro_offsets[0] = 0;
    this->gyro_offsets[1] = 0;
    this->gyro_offsets[2] = 0;
}

esp_err_t MPU9250::spi_init()
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
    buscfg.miso_io_num = GPIO_NUM_16;
    buscfg.mosi_io_num = GPIO_NUM_5;
    buscfg.sclk_io_num = GPIO_NUM_18;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0;

    spi_device_interface_config_t devcfg;
    devcfg.command_bits = 0;
    devcfg.address_bits = 8;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.duty_cycle_pos = 128;
    devcfg.cs_ena_pretrans = 0;
    devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz = 1000000;
    devcfg.spics_io_num = GPIO_NUM_13;
    devcfg.flags = 0;
    devcfg.queue_size = 7;
    devcfg.pre_cb = 0;
    devcfg.post_cb = 0;

    //Initialize the SPI bus
    if ((ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0)))
        return ret;

    if ((ret = spi_bus_add_device(HSPI_HOST, &devcfg, &this->spi)))
        return ret;

    return ESP_OK;
}

esp_err_t MPU9250::int_init()
{
    esp_err_t ret;

    // config INT pin
    gpio_config_t conf;
    conf.intr_type = GPIO_INTR_POSEDGE;
    conf.mode = GPIO_MODE_INPUT;
    conf.pin_bit_mask = GPIO_SEL_4;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;

    if ((ret = gpio_config(&conf)))
        return ret;

    if ((ret = gpio_install_isr_service(0)))
        return ret;

    if ((ret = gpio_isr_handler_add(GPIO_NUM_4, int_isr_handler, NULL)))
        return ret;

    return ESP_OK;
}

esp_err_t MPU9250::spi_reg_read(uint8_t reg, uint8_t &data)
{
    esp_err_t ret;

    spi_transaction_t trans;
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.cmd = 0;
    trans.addr = reg | 0x80;
    trans.length = 8;
    trans.rxlength = 8;
    trans.user = 0;
    trans.tx_data[0] = 0x00;

    if ((ret = spi_device_transmit(this->spi, &trans)))
        return ret;

    data = trans.rx_data[0];

    return ESP_OK;
}

esp_err_t MPU9250::spi_reg_write(uint8_t reg, uint8_t data)
{
    esp_err_t ret;

    spi_transaction_t trans;
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.cmd = 0;
    trans.addr = reg & 0x7F;
    trans.length = 8;
    trans.rxlength = 0;
    trans.user = 0;
    trans.tx_data[0] = data;
    trans.rx_buffer = NULL;

    if ((ret = spi_device_transmit(this->spi, &trans)))
        return ret;

    return ESP_OK;
}

esp_err_t MPU9250::set_gyro_fsr(gyro_fsr fsr)
{
    esp_err_t ret;

    if (fsr == GYRO_FSR_NOT_SET)
        return ESP_FAIL;

    if (fsr == this->g_fsr)
        return ESP_OK;


    // gyro FSR config shares the same register with DLPF config so we just update reg value
    uint8_t gyro_config;

    if ((ret = this->spi_reg_read(this->REGISTERS.GYRO_CONFIG, gyro_config)))
        return ret;
    if ((ret = this->spi_reg_write(this->REGISTERS.GYRO_CONFIG, gyro_config | fsr)))
        return ret;

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

esp_err_t MPU9250::set_accel_fsr(accel_fsr fsr)
{
    esp_err_t ret;

    if (fsr == ACCEL_FSR_NOT_SET)
        return ESP_FAIL;

    if (fsr == this->a_fsr)
        return ESP_OK;

    if ((ret = this->spi_reg_write(this->REGISTERS.ACCEL_CONFIG, fsr)))
        return ret;

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

esp_err_t MPU9250::set_gyro_lpf(gyro_lpf lpf)
{
    esp_err_t ret;

    if (lpf == GYRO_LPF_NOT_SET)
        return ESP_FAIL;

    if (this->g_lpf == lpf)
        return ESP_OK;

    uint8_t Fchoice_b;

    if (lpf == GYRO_LPF_8800HZ)
        Fchoice_b = 0x01;
    else if (lpf == GYRO_LPF_3600Hz)
        Fchoice_b = 0x02;
    else
        Fchoice_b = 0x00;

    // gyro DLPF config shares the same register with FSR config so we just update reg value
    uint8_t gyro_config;

    if ((ret = this->spi_reg_read(this->REGISTERS.GYRO_CONFIG, gyro_config)))
        return ret;
    gyro_config &= 0xFC;
    if ((ret = this->spi_reg_write(this->REGISTERS.GYRO_CONFIG, gyro_config | Fchoice_b)))
        return ret;

    if ((ret = this->spi_reg_write(this->REGISTERS.CONFIG, lpf)))
        return ret;

    this->g_lpf = lpf;

    return ESP_OK;
}

esp_err_t MPU9250::set_accel_lpf(accel_lpf lpf)
{
    esp_err_t ret;

    if (lpf == ACCEL_LPF_NOT_SET)
        return ESP_FAIL;

    if (lpf == this->a_lpf)
        return ESP_OK;

    uint8_t accel_config2 = lpf;

    if (lpf == ACCEL_LPF_1046HZ)
        accel_config2 |= 0x08;

    if ((ret = this->spi_reg_write(this->REGISTERS.ACCEL_CONFIG2, accel_config2)))
        return ret;

    this->a_lpf = lpf;

    return ESP_OK;
}

esp_err_t MPU9250::set_sample_rate(uint16_t rate)
{
    // setting SMPLRT_DIV won't be effective in cases:
    // 8800 Hz => sample at 32 kHz
    // 3600 Hz => sample at 32 kHz
    // 250 Hz => should sample at 8 kHz, measured 32 kHz
    if (this->g_lpf == GYRO_LPF_8800HZ || this->g_lpf == GYRO_LPF_3600Hz || this->g_lpf == GYRO_LPF_250HZ)
        return ESP_OK;

    uint8_t div = (1000 - rate) / rate;

    return this->spi_reg_write(this->REGISTERS.SMPLRT_DIV, div);
}

esp_err_t MPU9250::set_interrupt(bool enable)
{
    return this->spi_reg_write(this->REGISTERS.INT_ENABLE, (enable ? 0x01 : 0x00));
}

void MPU9250::Init()
{
    ESP_ERROR_CHECK(this->int_init());

    ESP_ERROR_CHECK(this->spi_init());

    // reset the device
    ESP_ERROR_CHECK(this->spi_reg_write(this->REGISTERS.PWR_MGMT_1, 0x80));

    // wait until reset done
    uint8_t tmp;
    do
    {
        this->spi_reg_read(this->REGISTERS.PWR_MGMT_1, tmp);
    } while (tmp & 0x80);

    vTaskDelay(100 / portTICK_PERIOD_MS);

    /* should not be needed
    this->spi_reg_write(this->REGISTERS.SIGNAL_PATH_RESET, 0x07);
    */

    // disable FIFO, I2C master; SPI mode only; reset all signal paths
    ESP_ERROR_CHECK(this->spi_reg_write(this->REGISTERS.USER_CTRL, 0x2F));

    // enable all sensors
    ESP_ERROR_CHECK(this->spi_reg_write(this->REGISTERS.PWR_MGMT_2, 0x00));

    // check device responding
    uint8_t who_am_i;

    ESP_ERROR_CHECK(this->spi_reg_read(this->REGISTERS.WHO_AM_I, who_am_i));


    if (who_am_i != 0x71 && who_am_i != 0x73)
        ESP_ERROR_CHECK(ESP_FAIL);

    ESP_ERROR_CHECK(this->set_interrupt(false));

    // set INT pin active high, push-pull; don't use latched mode, fsync nor I2C bypass
    ESP_ERROR_CHECK(this->spi_reg_write(this->REGISTERS.INT_PIN_CFG, 0x10));

    ESP_ERROR_CHECK(this->set_gyro_fsr(GYRO_FSR_2000));
    ESP_ERROR_CHECK(this->set_gyro_lpf(GYRO_LPF_184HZ));

    ESP_ERROR_CHECK(this->set_accel_fsr(ACCEL_FSR_16));
    ESP_ERROR_CHECK(this->set_accel_lpf(ACCEL_LPF_218HZ));

    ESP_ERROR_CHECK(this->set_sample_rate(this->SAMPLE_RATE));

    ESP_ERROR_CHECK(this->set_interrupt(true));

    // set SPI speed to 20 MHz

    ESP_ERROR_CHECK(spi_bus_remove_device(this->spi));

    spi_device_interface_config_t devcfg;
    devcfg.command_bits = 0;
    devcfg.address_bits = 8;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.duty_cycle_pos = 128;
    devcfg.cs_ena_pretrans = 0;
    devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz = 20000000;
    devcfg.spics_io_num = GPIO_NUM_13;
    devcfg.flags = 0;
    devcfg.queue_size = 7;
    devcfg.pre_cb = 0;
    devcfg.post_cb = 0;

    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &this->spi));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    this->Calibrate();

    this->ready = true;
}

// we won't use offset registers since we would have to switch back to 1 MHz SPI clock
void MPU9250::Calibrate()
{
    Raw_Data accel, gyro;

    for (uint16_t i = 0; i < 500; i++)
    {
        this->Read_Raw(accel, gyro);

        this->accel_offsets[0] += accel.x;
        this->accel_offsets[1] += accel.y;
        this->accel_offsets[2] += accel.z - 2048; // 2048 = 1G in +- 16G FSR
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

uint16_t MPU9250::Get_Sample_Rate()
{
    return this->SAMPLE_RATE;
}

void MPU9250::Read_Raw(Raw_Data &raw_accel, Raw_Data &raw_gyro)
{
    static const uint8_t tx_data[14] = {0x00};
    static uint8_t rx_data[14];

    spi_transaction_t trans;
    trans.flags = 0;
    trans.cmd = 0;
    trans.addr = this->REGISTERS.ACCEL_XOUT_H | 0x80;
    trans.length = 14 * 8;
    trans.rxlength = 14 * 8;
    trans.user = 0;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    // reading:
    // 75 us not 32b aligned, non DMA capable mem
    // 69 us 32b aligned, DMA capable mem
    ESP_ERROR_CHECK(spi_device_transmit(this->spi, &trans));

    raw_accel.x = (rx_data[2] << 8) | rx_data[3];
    raw_accel.y = (rx_data[0] << 8) | rx_data[1];
    raw_accel.z = (rx_data[4] << 8) | rx_data[5];

    raw_gyro.x = (rx_data[10] << 8) | rx_data[11];
    raw_gyro.y = (rx_data[8] << 8) | rx_data[9];
    raw_gyro.z = (rx_data[12] << 8) | rx_data[13];
}

void MPU9250::Read_Data(Sensor_Data &accel, Sensor_Data &gyro)
{
    static const uint8_t tx_data[14] = {0x00};
    static uint8_t rx_data[14];

    spi_transaction_t trans;
    trans.flags = 0;
    trans.cmd = 0;
    trans.addr = this->REGISTERS.ACCEL_XOUT_H | 0x80;
    trans.length = 14 * 8;
    trans.rxlength = 14 * 8;
    trans.user = 0;
    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;

    // reading:
    // 75 us not 32b aligned, non DMA capable mem
    // 69 us 32b aligned, DMA capable mem
    ESP_ERROR_CHECK(spi_device_transmit(this->spi, &trans));

    static Raw_Data raw_accel, raw_gyro;

    raw_accel.x = (rx_data[2] << 8) | rx_data[3];
    raw_accel.y = (rx_data[0] << 8) | rx_data[1];
    raw_accel.z = (rx_data[4] << 8) | rx_data[5];

    raw_gyro.x = (rx_data[10] << 8) | rx_data[11];
    raw_gyro.y = (rx_data[8] << 8) | rx_data[9];
    raw_gyro.z = (rx_data[12] << 8) | rx_data[13];

    accel.x = this->accel_x_filter.Apply_Filter((raw_accel.x + this->accel_offsets[0]) * this->a_mult);
    accel.y = this->accel_y_filter.Apply_Filter((raw_accel.y + this->accel_offsets[1]) * this->a_mult);
    accel.z = this->accel_z_filter.Apply_Filter((raw_accel.z + this->accel_offsets[2]) * this->a_mult);

    gyro.x = this->gyro_x_filter.Apply_Filter((raw_gyro.x + this->gyro_offsets[0]) * this->g_mult);
    gyro.y = this->gyro_y_filter.Apply_Filter((raw_gyro.y + this->gyro_offsets[1]) * this->g_mult);
    gyro.z = this->gyro_z_filter.Apply_Filter((raw_gyro.z + this->gyro_offsets[2]) * this->g_mult);
}

void MPU9250::Data_Ready_Callback()
{
    portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

    vTaskEnterCritical(&mutex);

    if (this->ready)
        this->data_ready = true;

    vTaskExitCritical(&mutex);
}

bool MPU9250::Data_Ready()
{
    bool tmp = this->data_ready;

    if (this->data_ready)
        this->data_ready = false;

    return tmp;
}

} /* namespace flyhero */
