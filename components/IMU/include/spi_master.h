//
// Created by michal on 23.4.18.
//

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**  example of full duplex transaction
 *            command                address                     write & read
 * MOSI   C3  C2  C1  C0  |  A5  A4  A3  A2  A1  A0  |  W3  W2  W1  W0  --  --  --  --
 * MISO   --  --  --  --  |  --  --  --  --  --  --  |  R7  R6  R5  R4  R3  R2  R1  R0
 */

/**  example of half duplex transaction
 *            command                address                 write              read
 * MOSI   C3  C2  C1  C0  |  A5  A4  A3  A2  A1  A0  |  W3  W2  W1  W0  |  R3  R2  R1  R0
 */

/**  example of DIO transaction
 *            command            address           write          read
 * MOSI   C3  C2  C1  C0  |  A6  A4  A2  A0  |  W4  W2  W0  |  R4  R2  R0
 * MISO   --  --  --  --  |  A7  A5  A3  A1  |  W5  W3  W1  |  R5  R3  R1
 */

/**  example of DOUT transaction
 *            command                    address                   write          read
 * MOSI   C3  C2  C1  C0  |  A7  A6  A5  A4  A3  A2  A1  A0  |  W4  W2  W0  |  R4  R2  R0
 * MISO   --  --  --  --  |  --  --  --  --  --  --  --  --  |  W5  W3  W1  |  R5  R3  R1
 */

typedef enum
{
    SPI_MASTER_MODE_FULL_DUPLEX,    /**< data are transmitted over MOSI and MISO at the same time */
    SPI_MASTER_MODE_HALF_DUPLEX,    /**< data are transmitted over MOSI only */
    SPI_MASTER_MODE_DIO,            /**< address and write and read data are sent over MOSI TODO */
    SPI_MASTER_MODE_DOUT,
    SPI_MASTER_MODE_QIO,
    SPI_MASTER_MODE_QOUT,
} SPI_Master_Mode_t;

/**  CS pre-transmission delay
 * Unless specified in setup_time (in that case it sums up with setup_time) // TODO
 * Setting setup_time works only in half duplex mode due to hardware bug
 *
 * MODE 0: half a cycle before clock goes high
 * MODE 1: at the same time as clock goes high
 * MODE 2: half a cycle before clock goes low
 * MODE 3: at the same time as clock goes low
 */

typedef enum
{
    SPI_MASTER_CS_NONE = -1,
    SPI_MASTER_CS0 = 0,
    SPI_MASTER_CS1,
    SPI_MASTER_CS2
} SPI_Master_CS_t;

#define SPI_MASTER_FLAG_CS_KEEP_ENABLED         0x0001u
#define SPI_MASTER_FLAG_WP_IDLE_LOW             0x0002u
#define SPI_MASTER_FLAG_DUMMY_CLOCK_DISABLE     0x0004u

typedef enum
{
    SPI_CLOCK_MODE_0,
    SPI_CLOCK_MODE_1,
    SPI_CLOCK_MODE_2,
    SPI_CLOCK_MODE_3
} SPI_Clock_Mode_t;

typedef enum
{
    SPI_BIT_ORDER_MSB_FIRST,
    SPI_BIT_ORDER_LSB_FIRST
} SPI_Bit_Order_t;

typedef enum
{
    SPI_DEVICE_0 = 0,
    SPI_DEVICE_1,
    SPI_DEVICE_HSPI,
    SPI_DEVICE_VSPI,
} SPI_Device_t;

typedef struct
{
    SPI_Clock_Mode_t clock_mode;
    SPI_Bit_Order_t read_bit_order;
    SPI_Bit_Order_t write_bit_order;
    uint32_t frequency;
    uint32_t flags;
} SPI_Master_Config_t;

typedef struct
{
    SPI_Master_Mode_t mode;
    uint16_t command;
    uint8_t command_bits;   // 0 - 16
    uint64_t address;
    uint8_t address_bits;   // 0 - 64
    uint8_t dummy_bits;     // 0 - 64
    uint32_t mosi_bits;
    uint32_t miso_bits;
} SPI_Master_Transmission_t;

void SPI_Master_Config(SPI_Device_t device, SPI_Master_CS_t cs_line, const SPI_Master_Config_t * config);

void SPI_Master_Transmit(SPI_Device_t device, SPI_Master_CS_t cs_line,
                                       const SPI_Master_Transmission_t * transmission, uint8_t * mosi_buffer,
                                       uint8_t * miso_buffer);

#ifdef __cplusplus
}
#endif