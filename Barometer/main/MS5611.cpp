#include "MS5611.h"


namespace flyhero
{

MS5611 &MS5611::Instance()
{
    static MS5611 instance;

    return instance;
}

MS5611::MS5611()
{
    this->spi = NULL;
    std::memset(this->c, 0, 6);
    this->d1 = 0;
    this->d2 = 0;
}

esp_err_t MS5611::spi_init()
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
    buscfg.miso_io_num = GPIO_NUM_19;
    buscfg.mosi_io_num = GPIO_NUM_22;
    buscfg.sclk_io_num = GPIO_NUM_23;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0;

    spi_device_interface_config_t devcfg;
    devcfg.command_bits = 8;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.mode = 0;
    devcfg.duty_cycle_pos = 128;
    devcfg.cs_ena_pretrans = 0;
    devcfg.cs_ena_posttrans = 0;
    devcfg.clock_speed_hz = 20000000;
    devcfg.spics_io_num = GPIO_NUM_21;
    devcfg.flags = 0;
    devcfg.queue_size = 7;
    devcfg.pre_cb = 0;
    devcfg.post_cb = 0;

    //Initialize the SPI bus
    if ((ret = spi_bus_initialize(VSPI_HOST, &buscfg, 0)))
        return ret;

    if ((ret = spi_bus_add_device(VSPI_HOST, &devcfg, &this->spi)))
        return ret;

    return ESP_OK;
}

esp_err_t MS5611::reset()
{
    spi_transaction_t trans;
    trans.flags = 0;
    trans.cmd = 0x1E;
    trans.addr = 0;
    trans.length = 0;
    trans.rxlength = 0;
    trans.user = 0;
    trans.tx_buffer = NULL;
    trans.rx_buffer = NULL;

    return spi_device_transmit(this->spi, &trans);
}

esp_err_t MS5611::load_prom()
{
    esp_err_t ret;

    spi_transaction_t trans;
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.cmd = 0;
    trans.addr = 0;
    trans.length = 16;
    trans.rxlength = 16;
    trans.user = 0;
    trans.tx_buffer = NULL;

    for (uint8_t i = 0; i < 6; i++)
    {
        trans.cmd = 0xA2 + i * 2;

        if ((ret = spi_device_transmit(this->spi, &trans)))
            return ret;

        this->c[i] = trans.rx_data[0] << 8;
        this->c[i] |= trans.rx_data[1];
    }

    return ESP_OK;
}

esp_err_t MS5611::read_d1()
{
    esp_err_t ret;

    // convert D1 (OSR = 4096)
    spi_transaction_t trans;
    trans.flags = 0;
    trans.cmd = 0x48;
    trans.addr = 0;
    trans.length = 0;
    trans.rxlength = 0;
    trans.user = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;

    if ((ret = spi_device_transmit(this->spi, &trans)))
        return ret;

    // not supported yet
    //TickType_t current_ticks = xTaskGetTickCount();
    //vTaskDelayUntil(&current_ticks, 10 / portTICK_RATE_MS);

    // wait until conversion done
    vTaskDelay(10 / portTICK_RATE_MS);

    // read converted D1 value
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.cmd = 0x00;
    trans.addr = 0;
    trans.length = 24;
    trans.rxlength = 24;
    trans.user = 0;
    trans.tx_buffer = NULL;

    if ((ret = spi_device_transmit(this->spi, &trans)))
        return ret;

    this->d1 = trans.rx_data[0] << 16;
    this->d1 |= trans.rx_data[1] << 8;
    this->d1 |= trans.rx_data[2];

    return ESP_OK;
}

esp_err_t MS5611::read_d2()
{
    esp_err_t ret;

    // convert D2 (OSR = 4096)
    spi_transaction_t trans;
    trans.flags = 0;
    trans.cmd = 0x58;
    trans.addr = 0;
    trans.length = 0;
    trans.rxlength = 0;
    trans.user = 0;
    trans.rx_buffer = NULL;
    trans.tx_buffer = NULL;

    if ((ret = spi_device_transmit(this->spi, &trans)))
        return ret;

    // wait until conversion done
    vTaskDelay(10 / portTICK_RATE_MS);

    // not supported yet
    //TickType_t current_ticks = xTaskGetTickCount();
    //vTaskDelayUntil(&current_ticks, 10 / portTICK_RATE_MS);

    // read converted D2 value
    trans.flags = SPI_TRANS_USE_RXDATA;
    trans.cmd = 0x00;
    trans.addr = 0;
    trans.length = 24;
    trans.rxlength = 24;
    trans.user = 0;
    trans.tx_buffer = NULL;

    if ((ret = spi_device_transmit(this->spi, &trans)))
        return ret;

    this->d2 = trans.rx_data[0] << 16;
    this->d2 |= trans.rx_data[1] << 8;
    this->d2 |= trans.rx_data[2];

    return ESP_OK;
}

void MS5611::Init()
{

    ESP_ERROR_CHECK(this->spi_init());

    ESP_ERROR_CHECK(this->reset());

    // wait until reset done
    vTaskDelay(20 / portTICK_RATE_MS);

    ESP_ERROR_CHECK(this->load_prom());
}

void MS5611::Get_Data(int32_t &temperature, int32_t &pressure)
{
    ESP_ERROR_CHECK(this->read_d1());

    ESP_ERROR_CHECK(this->read_d2());

    int32_t d_t = this->d2 - (uint32_t) this->c[4] * 256;
    int32_t temp = 2000 + ((int64_t) d_t * this->c[5]) / 8388608;

    int64_t off = (int64_t) this->c[1] * 65536 + (int64_t) this->c[3] * d_t / 128;
    int64_t sens = (int64_t) this->c[0] * 32768 + (int64_t) this->c[2] * d_t / 256;

    int32_t temp_2 = 0;
    int64_t off_2 = 0;
    int64_t sens_2 = 0;

    if (temp < 2000)
    {
        temp_2 = ((int64_t) d_t * d_t) / 2147483648;
        off_2 = 5 * ((temp - 2000) * (temp - 2000)) / 2;
        sens_2 = 5 * ((temp - 2000) * (temp - 2000)) / 4;
    }
    if (temp < -1500)
    {
        off_2 = off_2 + 7 * (temp + 1500) * (temp + 1500);
        sens_2 = sens_2 + 11 * (temp + 1500) * (temp + 1500) / 2;
    }

    temp -= temp_2;
    off -= off_2;
    sens -= sens_2;

    int32_t p = (this->d1 * sens / 2097152 - off) / 32768;

    temperature = temp;
    pressure = p;
}

}
