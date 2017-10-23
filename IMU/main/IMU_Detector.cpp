//
// Created by michal on 22.10.17.
//

#include "IMU_Detector.h"

namespace flyhero {

spi_device_handle_t IMU_Detector::spi = NULL;

bool IMU_Detector::i2c_init() {
    // init i2c peripheral
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_5;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = GPIO_NUM_18;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;

    if (i2c_param_config(I2C_NUM_0, &conf) != ESP_OK)
        return false;
    if (i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0) != ESP_OK)
        return false;

    return true;
}

void IMU_Detector::i2c_deinit() {
    i2c_driver_delete(I2C_NUM_0);
}

bool IMU_Detector::spi_init() {
    //Initialize the SPI bus
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

    if (spi_bus_initialize(HSPI_HOST, &buscfg, 0) != ESP_OK)
        return false;

    if (spi_bus_add_device(HSPI_HOST, &devcfg, &IMU_Detector::spi) != ESP_OK)
        return false;

    return true;
}

void IMU_Detector::spi_deinit() {
    spi_bus_remove_device(IMU_Detector::spi);
    spi_bus_free(HSPI_HOST);
}

bool IMU_Detector::try_i2c_imu(const uint8_t DEVICE_ADDRESS_WRITE, const uint8_t DEVICE_ADDRESS_READ,
                 const uint8_t WHO_AM_I_REGISTER, const uint8_t EXPECTED_VALUE) {
    // read WHO_AM_I register
    uint8_t who_am_i;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (i2c_master_start(cmd) != ESP_OK)
        return false;
    if (i2c_master_write_byte(cmd, DEVICE_ADDRESS_WRITE, true) != ESP_OK)
        return false;
    if (i2c_master_write_byte(cmd, WHO_AM_I_REGISTER, true) != ESP_OK)
        return false;

    if (i2c_master_start(cmd) != ESP_OK)
        return false;
    if (i2c_master_write_byte(cmd, DEVICE_ADDRESS_READ, true) != ESP_OK)
        return false;
    if (i2c_master_read_byte(cmd, &who_am_i, 0x01) != ESP_OK)
        return false;
    if (i2c_master_stop(cmd) != ESP_OK)
        return false;

    if (i2c_master_cmd_begin(I2C_NUM_0, cmd, 500 / portTICK_RATE_MS) != ESP_OK)
        return false;

    i2c_cmd_link_delete(cmd);

    return who_am_i == EXPECTED_VALUE;
}

bool IMU_Detector::try_spi_imu(const uint8_t WHO_AM_I_REGISTER, const uint8_t EXPECTED_VALUE) {
    // read WHO_AM_I register
    uint8_t who_am_i;

    spi_transaction_t trans;
    trans.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    trans.cmd = 0;
    trans.addr = WHO_AM_I_REGISTER | 0x80;
    trans.length = 8;
    trans.rxlength = 8;
    trans.user = 0;
    trans.tx_data[0] = 0x00;

    if (spi_device_transmit(spi, &trans) != ESP_OK)
        return false;

    who_am_i = trans.rx_data[0];

    return who_am_i == EXPECTED_VALUE;
}

esp_err_t IMU_Detector::Detect_IMU(IMU **imu) {
    if (IMU_Detector::i2c_init()) {

        if (IMU_Detector::try_i2c_imu(0x68 << 1, (0x68 << 1) | 1, 0x75, 0x68)) {
            *imu = &MPU6050::Instance();
            IMU_Detector::i2c_deinit();

            return ESP_OK;
        }

        IMU_Detector::i2c_deinit();
    }

    if (IMU_Detector::spi_init()) {

        if (IMU_Detector::try_spi_imu(0x75, 0x71)) {
            *imu = &MPU9250::Instance();
            IMU_Detector::spi_deinit();

            return ESP_OK;
        }
        else if (IMU_Detector::try_spi_imu(0x75, 0x73)) {
            *imu = &MPU9250::Instance();
            IMU_Detector::spi_deinit();

            return ESP_OK;
        }

        IMU_Detector::spi_deinit();
    }

    return ESP_FAIL;
}

}