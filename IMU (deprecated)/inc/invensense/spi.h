/*
 * spi.h
 *
 *  Created on: 18. 2. 2017
 *      Author: michp
 */

#ifndef INVENSENSE_SPI_H_
#define INVENSENSE_SPI_H_

#include <stm32f4xx_hal.h>

void SLV4_INT_cb(void);
uint8_t Get_SLV4_ready();
void SPI_Master_Init(void);
void Set_SPI_Retry(unsigned short ml_sec);
unsigned short Get_SPI_Retry();

int Sensors_SPI_ReadRegister(unsigned char RegisterAddr,
	unsigned short RegisterLen, unsigned char *RegisterValue);
int Sensors_SPI_WriteRegister(unsigned char RegisterAddr,
	unsigned short RegisterLen, const unsigned char *RegisterValue);


#endif /* INVENSENSE_SPI_H_ */
