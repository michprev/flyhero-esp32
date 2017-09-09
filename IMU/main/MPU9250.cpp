/*
 * MPU9250.cpp
 *
 *  Created on: 7. 9. 2017
 *      Author: michp
 */

#include "MPU9250.h"

namespace flyhero {

static void IRAM_ATTR int_isr_handler(void* arg)
{
	MPU9250::Instance().Data_Ready_Callback();
}

MPU9250& MPU9250::Instance() {
	static MPU9250 instance;

	return instance;
}

MPU9250::MPU9250() {
	this->spi = NULL;
	this->rx_buffer = NULL;
	this->a_fsr = ACCEL_FSR_NOT_SET;
	this->a_lpf = ACCEL_LPF_NOT_SET;
	this->a_mult = 0;
	this->g_fsr = GYRO_FSR_NOT_SET;
	this->g_lpf = GYRO_LPF_NOT_SET;
	this->g_mult = 0;
	this->sample_rate = 0;
	this->data_ready = false;
	this->ready = false;
}

esp_err_t MPU9250::spi_init() {
    esp_err_t ret;

    spi_bus_config_t buscfg;
    buscfg.miso_io_num = GPIO_NUM_25;
    buscfg.mosi_io_num = GPIO_NUM_23;
    buscfg.sclk_io_num = GPIO_NUM_19;
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
    devcfg.spics_io_num = GPIO_NUM_22;
    devcfg.flags = 0;
    devcfg.queue_size = 7;
    devcfg.pre_cb = 0;
    devcfg.post_cb = 0;

    //Initialize the SPI bus
    if ( (ret = spi_bus_initialize(HSPI_HOST, &buscfg, 0)) )
    	return ret;

    if ( (ret = spi_bus_add_device(HSPI_HOST, &devcfg, &this->spi)) )
    	return ret;

    return ESP_OK;
}

esp_err_t MPU9250::int_init() {
	esp_err_t ret;

	// config INT pin
	gpio_config_t conf;
	conf.intr_type = GPIO_INTR_POSEDGE;
	conf.mode = GPIO_MODE_INPUT;
	conf.pin_bit_mask = GPIO_SEL_13;
	conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	conf.pull_up_en = GPIO_PULLUP_DISABLE;

	if ( (ret = gpio_config(&conf)) )
		return ret;

	if ( (ret = gpio_install_isr_service(0)) )
		return ret;

	if ( (ret = gpio_isr_handler_add(GPIO_NUM_13, int_isr_handler, NULL)))
		return ret;

	return ESP_OK;
}

esp_err_t MPU9250::spi_reg_read(uint8_t reg, uint8_t& data) {
	esp_err_t ret;

	spi_transaction_t trans;
	trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
	trans.cmd = 0;
	trans.addr = reg | 0x80;
	trans.length = 8;
	trans.rxlength = 8;
	trans.user = 0;
	trans.tx_data[0] = 0x00;

	if ( (ret = spi_device_transmit(this->spi, &trans)) )
		return ret;

	data = trans.rx_data[0];

	return ESP_OK;
}

DRAM_ATTR static const uint8_t tx_dummy[40] = { 0x00 };

esp_err_t MPU9250::spi_regs_read(uint8_t first_reg, uint8_t count) {
	return ESP_ERR_NOT_SUPPORTED;

	/*count += 4 - (count % 4);

	esp_err_t ret;

	spi_transaction_t trans;
	trans.flags = 0;
	trans.cmd = 0;
	trans.addr = first_reg | 0x80;
	trans.length = count * 8;
	trans.rxlength = count * 8;
	trans.user = 0;
	trans.tx_buffer = tx_dummy;
	trans.rx_buffer = this->rx_buffer;

	if ( (ret = spi_device_transmit(this->spi, &trans)) )
		return ret;

	return ESP_OK;*/
}

esp_err_t MPU9250::spi_reg_write(uint8_t reg, uint8_t data) {
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

	if ( (ret = spi_device_transmit(this->spi, &trans)) )
		return ret;

	return ESP_OK;
}

void MPU9250::set_gyro_fsr(gyro_fsr fsr) {
	if (fsr == GYRO_FSR_NOT_SET)
		return;

	if (fsr == this->g_fsr)
		return;


	// gyro FSR config shares the same register with DLPF config so we just update reg value
	uint8_t gyro_config;

	this->spi_reg_read(this->REGISTERS.GYRO_CONFIG, gyro_config);
	this->spi_reg_write(this->REGISTERS.GYRO_CONFIG, gyro_config | fsr);

	this->g_fsr = fsr;

	switch (this->g_fsr) {
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
		return;
	}

	this->g_mult /= std::pow(2, this->ADC_BITS - 1);
}

void MPU9250::set_accel_fsr(accel_fsr fsr) {
	if (fsr == ACCEL_FSR_NOT_SET)
		return;

	if (fsr == this->a_fsr)
		return;

	this->spi_reg_write(this->REGISTERS.ACCEL_CONFIG, fsr);

	this->a_fsr = fsr;

	switch (this->a_fsr) {
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
		return;
	}

	this->a_mult /= std::pow(2, this->ADC_BITS - 1);
}

void MPU9250::set_gyro_lpf(gyro_lpf lpf) {
	if (lpf == GYRO_LPF_NOT_SET)
		return;

	if (this->g_lpf == lpf)
		return;

	uint8_t Fchoice_b;

	if (lpf == GYRO_LPF_8800HZ)
		Fchoice_b = 0x01;
	else if (lpf == GYRO_LPF_3600Hz)
		Fchoice_b = 0x02;
	else
		Fchoice_b = 0x00;

	// gyro DLPF config shares the same register with FSR config so we just update reg value
	uint8_t gyro_config;

	this->spi_reg_read(this->REGISTERS.GYRO_CONFIG, gyro_config);
	gyro_config &= 0xFC;
	this->spi_reg_write(this->REGISTERS.GYRO_CONFIG, gyro_config | Fchoice_b);

	this->spi_reg_write(this->REGISTERS.CONFIG, lpf);

	this->g_lpf = lpf;
}

void MPU9250::set_accel_lpf(accel_lpf lpf) {
	if (lpf == ACCEL_LPF_NOT_SET)
		return;

	if (lpf == this->a_lpf)
		return;

	uint8_t accel_config2 = lpf;

	if (lpf == ACCEL_LPF_1046HZ)
		accel_config2 |= 0x08;

	this->spi_reg_write(this->REGISTERS.ACCEL_CONFIG2, accel_config2);

	this->a_lpf = lpf;
}

void MPU9250::set_sample_rate(uint16_t rate) {
	// setting SMPLRT_DIV won't be effective in cases:
	// 8800 Hz => sample at 32 kHz
	// 3600 Hz => sample at 32 kHz
	// 250 Hz => should sample at 8 kHz, measured 32 kHz
	if (this->g_lpf == GYRO_LPF_8800HZ || this->g_lpf == GYRO_LPF_3600Hz || this->g_lpf == GYRO_LPF_250HZ)
		return;

	uint8_t div = (1000 - rate) / rate;

	this->spi_reg_write(this->REGISTERS.SMPLRT_DIV, div);
}

void MPU9250::set_interrupt(bool enable) {
	this->spi_reg_write(this->REGISTERS.INT_ENABLE, (enable ? 0x01 : 0x00));
}

void MPU9250::Init() {
	esp_err_t ret;

	/*this->rx_buffer = (uint8_t*)heap_caps_malloc(40, MALLOC_CAP_32BIT | MALLOC_CAP_DMA);

	if (this->rx_buffer == NULL)
		std::cout << "alloc error" << std::endl;*/

	if ( (ret = this->int_init()))
		return;

	if ( (ret = this->spi_init()) )
		return;

	// reset the device
	this->spi_reg_write(this->REGISTERS.PWR_MGMT_1, 0x80);

	// wait until reset done
	uint8_t tmp;
	do {
		this->spi_reg_read(this->REGISTERS.PWR_MGMT_1, tmp);
	} while (tmp & 0x80);

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	/* should not be needed
	this->spi_reg_write(this->REGISTERS.SIGNAL_PATH_RESET, 0x07);
	*/

	// disable FIFO, I2C master; SPI mode only; reset all signal paths
	this->spi_reg_write(this->REGISTERS.USER_CTRL, 0x2F);

	// enable all sensors
	this->spi_reg_write(this->REGISTERS.PWR_MGMT_2, 0x00);

	// check device responding
	uint8_t who_am_i;

	this->spi_reg_read(this->REGISTERS.WHO_AM_I, who_am_i);


	if (who_am_i != 0x71)
		return;

	this->set_interrupt(false);

	// set INT pin active high, push-pull; don't use latched mode, fsync nor I2C bypass
	this->spi_reg_write(this->REGISTERS.INT_PIN_CFG, 0x10);

	this->set_gyro_fsr(GYRO_FSR_2000);
	this->set_gyro_lpf(GYRO_LPF_184HZ);

	this->set_accel_fsr(ACCEL_FSR_16);
	this->set_accel_lpf(ACCEL_LPF_1046HZ);

	this->set_sample_rate(1000);

	this->set_interrupt(true);

	// set SPI speed to 20 MHz

	if ( (ret = spi_bus_remove_device(this->spi)) )
		return;

	spi_device_interface_config_t devcfg;
	devcfg.command_bits = 0;
	devcfg.address_bits = 8;
	devcfg.dummy_bits = 0;
	devcfg.mode = 0;
	devcfg.duty_cycle_pos = 128;
	devcfg.cs_ena_pretrans = 0;
	devcfg.cs_ena_posttrans = 0;
	devcfg.clock_speed_hz = 20000000;
	devcfg.spics_io_num = GPIO_NUM_22;
	devcfg.flags = 0;
	devcfg.queue_size = 7;
	devcfg.pre_cb = 0;
	devcfg.post_cb = 0;

	if ( (ret = spi_bus_add_device(HSPI_HOST, &devcfg, &this->spi)) )
		return;

	this->ready = true;
}

void MPU9250::Data_Ready_Callback() {
	portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

	vTaskEnterCritical(&mutex);

	if (this->ready)
		this->data_ready = true;

	vTaskExitCritical(&mutex);
}

bool MPU9250::Data_Ready() {
	bool tmp = this->data_ready;

	if (this->data_ready)
		this->data_ready = false;

	return tmp;
}

} /* namespace flyhero */
