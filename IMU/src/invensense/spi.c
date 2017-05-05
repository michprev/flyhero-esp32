/*******************************************************************************
File    : i2c.c
Purpose : I2c 3 to communicate with the sensors
Author  :
********************************** Includes ***********************************/
#include "spi.h"

/********************************* Defines ************************************/

/********************************* Globals ************************************/

SPI_HandleTypeDef SPI_Handle;
volatile uint8_t SLV4_ready = 0;

/********************************* Prototypes *********************************/
unsigned long ST_Sensors_SPI_WriteRegister(unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue);
unsigned long ST_Sensors_SPI_ReadRegister(unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue);
/*******************************  Function ************************************/

void SLV4_INT_cb(void) {
	SLV4_ready = 1;
}

uint8_t Get_SLV4_ready() {
	return SLV4_ready;
}

void SPI_Master_Init(void)
{
	if (__GPIOA_IS_CLK_DISABLED())
		__GPIOA_CLK_ENABLE();

	if (__SPI1_IS_CLK_DISABLED())
		__SPI1_CLK_ENABLE();

	GPIO_InitTypeDef NSS;
	NSS.Pin = GPIO_PIN_4;
	NSS.Mode = GPIO_MODE_OUTPUT_PP;
	NSS.Speed = GPIO_SPEED_HIGH;
	NSS.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &NSS);

	// disable slave
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	SPI_Handle.Instance = SPI1;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.Direction = SPI_DIRECTION_2LINES;
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;
	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
	SPI_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPI_Handle.Init.CRCPolynomial = 10;

	HAL_SPI_Init(&SPI_Handle);
}

void SPI_Reset()
{

	/* The following code allows I2C error recovery and return to normal communication
	if the error source doesn’t still exist (ie. hardware issue..) */
	HAL_SPI_DeInit(&SPI_Handle);

	SPI_Handle.Instance = SPI1;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.Direction = SPI_DIRECTION_2LINES;
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;
	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_Handle.Init.TIMode = SPI_TIMODE_DISABLE;
	SPI_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SPI_Handle.Init.CRCPolynomial = 10;

	HAL_SPI_Init(&SPI_Handle);
}

int Sensors_SPI_WriteRegister(unsigned char reg_addr,
	unsigned short len,
	const unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_SPI_Retry();

tryWriteAgain:
	ret = ST_Sensors_SPI_WriteRegister(reg_addr, len, data_ptr);

	if (ret && retry_in_mlsec)
	{
		if (retries++ > 4)
			return ret;

		SPI_Reset();

		HAL_Delay(retry_in_mlsec);
		goto tryWriteAgain;
	}
	return ret;
}

int Sensors_SPI_ReadRegister(unsigned char reg_addr,
	unsigned short len,
	unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_SPI_Retry();

tryReadAgain:
	ret = ST_Sensors_SPI_ReadRegister(reg_addr, len, data_ptr);

	if (ret && retry_in_mlsec)
	{
		if (retries++ > 4)
			return ret;

		SPI_Reset();

		HAL_Delay(retry_in_mlsec);
		goto tryReadAgain;
	}
	return ret;
}


/**
* @brief  Writes a Byte to a given register to the sensors through the
control interface (I2C)
* @param  RegisterAddr: The address (location) of the register to be written.
* @param  RegisterValue: the Byte value to be written into destination register.
* @retval 0 if correct communication, else wrong communication
*/
unsigned long ST_Sensors_SPI_WriteRegister(unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(&SPI_Handle, &RegisterAddr, 1, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}
	if (HAL_SPI_Transmit(&SPI_Handle, RegisterValue, RegisterLen, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	/* Return the verifying value: 0 (Passed) or 1 (Failed) */
	return 0;
}

unsigned long ST_Sensors_SPI_ReadRegister(unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	RegisterAddr |= 0x80;

	if (HAL_SPI_Transmit(&SPI_Handle, &RegisterAddr, 1, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}
	if (HAL_SPI_Receive(&SPI_Handle, RegisterValue, RegisterLen, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		return 1;
	}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	/* Return the verifying value: 0 (Passed) or 1 (Failed) */
	return 0;
}

static unsigned short RETRY_IN_MLSEC = 55;

void Set_SPI_Retry(unsigned short ml_sec)
{
	RETRY_IN_MLSEC = ml_sec;
}

unsigned short Get_SPI_Retry()
{
	return RETRY_IN_MLSEC;
}
