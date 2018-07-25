//
// Created by michal on 23.4.18.
//

#include <stdint.h>
#include <spi_master.h>
#include <soc/spi_struct.h>
#include <stdbool.h>
#include <math.h>
#include <esp_timer.h>
#include "../../../../flyhero-esp32-idf/components/soc/esp32/include/soc/spi_struct.h"

typedef struct
{
    bool wr_bit_order;
    bool rd_bit_order;
    bool wp;
    uint8_t mosi_delay_num;
    uint8_t mosi_delay_mode;
    uint8_t miso_delay_num;
    uint8_t miso_delay_mode;
    // bool cs_hold; TODO implement?
    // uint8_t hold_time; TODO implement?
    // uint8_t cs_hold_delay; TODO implement?
    // bool cs_setup; TODO works only in half duplex - probably won't implement
    // uint8_t setup_time; TODO works only in half duplex - probably won't implement
    bool clk_equ_sysclk;
    uint8_t clkdiv_pre;
    uint8_t clkcnt_n;
    uint8_t clkcnt_h;
    uint8_t clkcnt_l;
    bool ck_out_edge;
    bool cs_keep_active;
    bool ck_idle_edge;
    bool usr_dummy_idle;
    // bool ck_dis; TODO implement?
    // uint8_t master_ck_sel; TODO implement?
} SPI_Master_Saved_Config_t;

typedef struct
{
    spi_dev_t * device;
    SPI_Master_CS_t last_cs;
    SPI_Master_Saved_Config_t devices[3];
} SPI_Master_Handle_t;

/* ------------------------------------------------------------------------------------------------------------------ */

static SPI_Master_Handle_t spi_master_handles[4] =
{
    {
        .device = &SPI0,
        .last_cs = -1
    },
    {
        .device = &SPI1,
        .last_cs = -1
    },
    {
        .device = &SPI2,
        .last_cs = -1
    },
    {
        .device = &SPI0,
        .last_cs = -1
    }
};

// will be called when switching from one CS line to another to preserve custom config for every CS line
static void SPI_Master_Load_Config(SPI_Device_t spi_device, SPI_Master_CS_t cs)
{
    // should already be in lock here

    SPI_Master_Handle_t * target_handle = &spi_master_handles[spi_device];
    spi_dev_t * target_device = target_handle->device;
    const SPI_Master_Saved_Config_t * target_config = &(target_handle->devices[cs]);

    target_device->ctrl.wr_bit_order = target_config->wr_bit_order;
    target_device->ctrl.rd_bit_order = target_config->rd_bit_order;
    target_device->ctrl.wp = target_config->wp;

    target_device->ctrl2.mosi_delay_num = target_config->mosi_delay_num;
    target_device->ctrl2.mosi_delay_mode = target_config->mosi_delay_mode;
    target_device->ctrl2.miso_delay_num = target_config->miso_delay_num;
    target_device->ctrl2.miso_delay_mode = target_config->miso_delay_mode;

    target_device->clock.clk_equ_sysclk = target_config->clk_equ_sysclk;
    target_device->clock.clkdiv_pre = target_config->clkdiv_pre;
    target_device->clock.clkcnt_n = target_config->clkcnt_n;
    target_device->clock.clkcnt_h = target_config->clkcnt_h;
    target_device->clock.clkcnt_l = target_config->clkcnt_l;

    target_device->user.ck_out_edge = target_config->ck_out_edge;
    target_device->pin.ck_idle_edge = target_config->ck_idle_edge;

    target_device->pin.cs_keep_active = target_config->cs_keep_active;

    target_device->user.usr_dummy_idle = target_config->usr_dummy_idle;

    switch (cs)
    {
        case SPI_MASTER_CS_NONE:
            break;
        case SPI_MASTER_CS0:
            target_device->pin.cs0_dis = 0;
            target_device->pin.cs1_dis = 1;
            target_device->pin.cs2_dis = 1;
            break;
        case SPI_MASTER_CS1:
            target_device->pin.cs0_dis = 1;
            target_device->pin.cs1_dis = 0;
            target_device->pin.cs2_dis = 1;
            break;
        case SPI_MASTER_CS2:
            target_device->pin.cs0_dis = 1;
            target_device->pin.cs1_dis = 1;
            target_device->pin.cs2_dis = 0;
            break;
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */

/**
 *
 * @param device target SPI device
 * @param cs_line chip select line of target SPI device
 * @param config
 */
void SPI_Master_Config(SPI_Device_t device, SPI_Master_CS_t cs_line, const SPI_Master_Config_t * config)
{
    uint32_t apb_frequency = 80000000; // TODO Clock_Get_APB_Frequency();

    SPI_Master_Handle_t * target_handle = &spi_master_handles[device];

    SPI_Master_Saved_Config_t * target_config = &(target_handle->devices[cs_line]);

    target_config->cs_keep_active = config->flags & SPI_MASTER_FLAG_CS_KEEP_ENABLED;
    target_config->wp = !(config->flags & SPI_MASTER_FLAG_WP_IDLE_LOW);
    target_config->usr_dummy_idle = config->flags & SPI_MASTER_FLAG_DUMMY_CLOCK_DISABLE;

    // set clock dividers

    unsigned target_clock_divider = (unsigned) roundf((float) apb_frequency / config->frequency);

    if (target_clock_divider == 1)
        target_config->clk_equ_sysclk = 1;
    else if (apb_frequency / config->frequency <= 64)
    {
        target_config->clk_equ_sysclk = 0;
        target_config->clkdiv_pre = 0;
        target_config->clkcnt_n = target_clock_divider - 1;
        target_config->clkcnt_h = target_config->clkcnt_n / 2 - 1;
        // TODO CLKCNT_H for duty cycle
        target_config->clkcnt_l = target_config->clkcnt_n;
    } else
    {
        for (uint8_t i = 64; i > 0; i--)
            if (target_clock_divider % i == 0)
            {
                target_config->clk_equ_sysclk = 0;
                target_config->clkdiv_pre = target_clock_divider / i - 1;
                target_config->clkcnt_n = i - 1;
                target_config->clkcnt_h = target_config->clkcnt_n / 2 - 1;
                // TODO CLKCNT_H for duty cycle
                target_config->clkcnt_l = target_config->clkcnt_n;

                break;
            }
    }

    target_config->miso_delay_mode = 0;
    target_config->miso_delay_num = 0;

    target_config->mosi_delay_mode = 0;
    target_config->mosi_delay_num = 0;

    switch (config->clock_mode)
    {
        case SPI_CLOCK_MODE_0:

            target_config->ck_idle_edge = 0;
            target_config->ck_out_edge = 0;
            break;
        case SPI_CLOCK_MODE_1:
            target_config->ck_idle_edge = 0;
            target_config->ck_out_edge = 1;
            break;
        case SPI_CLOCK_MODE_2:
            target_config->ck_idle_edge = 1;
            target_config->ck_out_edge = 1;
            break;
        case SPI_CLOCK_MODE_3:
            target_config->ck_idle_edge = 1;
            target_config->ck_out_edge = 0;
            break;
    }

    switch (config->write_bit_order)
    {
        case SPI_BIT_ORDER_LSB_FIRST:
            target_config->wr_bit_order = 1;
            break;
        case SPI_BIT_ORDER_MSB_FIRST:
            target_config->wr_bit_order = 0;
            break;
    }

    switch (config->read_bit_order)
    {
        case SPI_BIT_ORDER_LSB_FIRST:
            target_config->rd_bit_order = 1;
            break;
        case SPI_BIT_ORDER_MSB_FIRST:
            target_config->rd_bit_order = 0;
            break;
    }
}

// TODO sending 4 bits using mosi behave differently than sending using command
void
SPI_Master_Transmit(SPI_Device_t spi_device, SPI_Master_CS_t cs_line, const SPI_Master_Transmission_t * transmission,
                    uint8_t * mosi_buffer, uint8_t * miso_buffer)
{
    SPI_Master_Handle_t * target_handle = &spi_master_handles[spi_device];

    spi_dev_t * target_device = target_handle->device;

    // load config for target slave if not already loaded
    //if (target_handle->last_cs != cs_line)
        SPI_Master_Load_Config(spi_device, cs_line);

    // set master mode
    target_device->slave.slave_mode = 0;

    switch (transmission->mode)
    {
        case SPI_MASTER_MODE_FULL_DUPLEX:
            // disable fast read mode
            target_device->ctrl.fastrd_mode = 0;

            // disable half duplex mode
            target_device->user.sio = 0;

            // disable all fast write modes
            target_device->user.fwrite_dio = 0;
            target_device->user.fwrite_dual = 0;
            target_device->user.fwrite_qio = 0;
            target_device->user.fwrite_quad = 0;

            // enable full duplex mode
            target_device->user.doutdin = 1;
            break;
        case SPI_MASTER_MODE_HALF_DUPLEX:
            // disable fast read mode
            target_device->ctrl.fastrd_mode = 0;

            // enable half duplex mode
            target_device->user.sio = 1;

            // disable all fast write modes
            target_device->user.fwrite_dio = 0;
            target_device->user.fwrite_dual = 0;
            target_device->user.fwrite_qio = 0;
            target_device->user.fwrite_quad = 0;

            // disable full duplex mode
            target_device->user.doutdin = 0;
            break;
        case SPI_MASTER_MODE_DIO:
            // enable QIO read mode
            target_device->ctrl.fastrd_mode = 1;
            target_device->ctrl.fread_qio = 0;
            target_device->ctrl.fread_dio = 1;
            target_device->ctrl.fread_quad = 0;
            target_device->ctrl.fread_dual = 0;

            // disable half duplex mode
            target_device->user.sio = 0;

            // enable QIO write mode
            target_device->user.fwrite_dio = 1;
            target_device->user.fwrite_dual = 0;
            target_device->user.fwrite_qio = 0;
            target_device->user.fwrite_quad = 0;

            // disable full duplex mode
            target_device->user.doutdin = 0;
            break;
        case SPI_MASTER_MODE_DOUT:
            // enable QIO read mode
            target_device->ctrl.fastrd_mode = 1;
            target_device->ctrl.fread_qio = 0;
            target_device->ctrl.fread_dio = 0;
            target_device->ctrl.fread_quad = 0;
            target_device->ctrl.fread_dual = 1;

            // disable half duplex mode
            target_device->user.sio = 0;

            // enable QIO write mode
            target_device->user.fwrite_dio = 0;
            target_device->user.fwrite_dual = 1;
            target_device->user.fwrite_qio = 0;
            target_device->user.fwrite_quad = 0;

            // disable full duplex mode
            target_device->user.doutdin = 0;
            break;
        case SPI_MASTER_MODE_QIO:
            // enable QIO read mode
            target_device->ctrl.fastrd_mode = 1;
            target_device->ctrl.fread_qio = 1;
            target_device->ctrl.fread_dio = 0;
            target_device->ctrl.fread_quad = 0;
            target_device->ctrl.fread_dual = 0;

            // disable half duplex mode
            target_device->user.sio = 0;

            // enable QIO write mode
            target_device->user.fwrite_dio = 0;
            target_device->user.fwrite_dual = 0;
            target_device->user.fwrite_qio = 1;
            target_device->user.fwrite_quad = 0;

            // disable full duplex mode
            target_device->user.doutdin = 0;
            break;
        case SPI_MASTER_MODE_QOUT:
            // enable QIO read mode
            target_device->ctrl.fastrd_mode = 1;
            target_device->ctrl.fread_qio = 0;
            target_device->ctrl.fread_dio = 0;
            target_device->ctrl.fread_quad = 1;
            target_device->ctrl.fread_dual = 0;

            // disable half duplex mode
            target_device->user.sio = 0;

            // enable QIO write mode
            target_device->user.fwrite_dio = 0;
            target_device->user.fwrite_dual = 0;
            target_device->user.fwrite_qio = 0;
            target_device->user.fwrite_quad = 1;

            // disable full duplex mode
            target_device->user.doutdin = 0;
            break;
    }

    if (transmission->command_bits > 0)
    {
        uint16_t tmp_command = transmission->command << (16 - transmission->command_bits);
        target_device->user2.usr_command_value = (tmp_command >> 8) | (tmp_command << 8);
        target_device->user2.usr_command_bitlen = transmission->command_bits - 1;
        target_device->user.usr_command = 1;
    } else
        target_device->user.usr_command = 0;

    if (transmission->address_bits > 0)
    {
        target_device->user.usr_addr = 1;
        target_device->user1.usr_addr_bitlen = transmission->address_bits - 1;

        if (transmission->address_bits > 32)
        {
            target_device->addr = transmission->address >> (transmission->address_bits - 32);
            target_device->slv_wr_status = transmission->address << (64 - transmission->address_bits);
        } else
            target_device->addr = transmission->address << (32 - transmission->address_bits);
    } else
        target_device->user.usr_addr = 0;

    if (transmission->dummy_bits > 0)
    {
        target_device->user1.usr_dummy_cyclelen = transmission->dummy_bits - 1;
        target_device->user.usr_dummy = 1;
    } else
        target_device->user.usr_dummy = 0;

    if (transmission->miso_bits > 0)
    {
        target_device->user.usr_miso_highpart = 0;
        target_device->miso_dlen.usr_miso_dbitlen = transmission->miso_bits - 1;
        target_device->user.usr_miso = 1;
    } else
        target_device->user.usr_miso = 0;

    if (transmission->mosi_bits > 0)
    {
        // in case that mosi_bits > 0 and mosi_bits < miso_bits SPI hardware generates clock only for MOSI bits
        // fix this by adding 0 bits TODO

        if (transmission->mosi_bits > 8 * 32)
            target_device->user.usr_mosi_highpart = 0;  // in this case data cannot be read
        else
            target_device->user.usr_mosi_highpart = 1;

        target_device->mosi_dlen.usr_mosi_dbitlen = transmission->mosi_bits - 1;
        target_device->user.usr_mosi = 1;

    } else
        target_device->user.usr_mosi = 0;


    uint8_t fifo_pos = (transmission->mosi_bits > 8 * 32 ? 0 : 8);
    uint8_t buffer_pos = 0;
    uint32_t copied_bits = 0;

    while (copied_bits < transmission->mosi_bits)
    {
        uint32_t tmp = 0;

        for (uint8_t i = 0; i < 4; i++)
        {
            if (copied_bits >= transmission->mosi_bits)
                break;
            tmp |= mosi_buffer[buffer_pos++] << 8 * i;
            copied_bits += 8;
        }
        target_device->data_buf[fifo_pos++] = tmp;
    }

    // disable trans_done interrupt
    target_device->slave.trans_inten = 0;

    // clear trans_done interrupt
    target_device->slave.trans_done = 0;

    *((volatile uint32_t *)0x3FF0017C) = 6;
    *((volatile uint32_t *)0x3FF00290) = 6;

    // enable trans_done interrupt
    target_device->slave.trans_inten = 1;

    // start user command
    target_device->cmd.usr = 1;

    while (!target_device->slave.trans_done);

    copied_bits = 0;
    fifo_pos = 0;
    buffer_pos = 0;

    while (copied_bits < transmission->miso_bits)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            if (copied_bits >= transmission->miso_bits)
                break;

            // when miso_bits is not multiple of 8 hardware will clear remaining bits itself
            miso_buffer[buffer_pos++] = (target_device->data_buf[fifo_pos] >> 8 * i) & 0xFF;

            copied_bits += 8;
        }
        fifo_pos++;
    }
}