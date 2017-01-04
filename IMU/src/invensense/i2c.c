/*******************************************************************************
File    : i2c.c
Purpose : I2c 3 to communicate with the sensors
Author  :
********************************** Includes ***********************************/
#include "i2c.h"

/********************************* Defines ************************************/



/********************************* Globals ************************************/
I2C_HandleTypeDef MPU_Handle;

/********************************* Prototypes *********************************/
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	const unsigned char *data_ptr);
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char *data_ptr);

/*******************************  Function ************************************/

void I2cMaster_Init(void)
{
	if (__GPIOB_IS_CLK_DISABLED())
		__GPIOB_CLK_ENABLE();

	if (__I2C1_IS_CLK_DISABLED())
		__I2C1_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	MPU_Handle.Instance = I2C1;
	MPU_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	MPU_Handle.Init.ClockSpeed = 400000;
	MPU_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	MPU_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	MPU_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	MPU_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	MPU_Handle.Init.OwnAddress1 = 0;
	MPU_Handle.Init.OwnAddress2 = 0;

	HAL_I2C_Init(&MPU_Handle);
}

/**
* @brief  Basic management of the timeout situation.
* @param  None.
* @retval None.
*/
static void I2C_Reset()
{

	/* The following code allows I2C error recovery and return to normal communication
	if the error source doesn’t still exist (ie. hardware issue..) */
	I2C_InitTypeDef I2C_InitStructure;
	HAL_I2C_DeInit(&MPU_Handle);

	MPU_Handle.Instance = I2C1;
	MPU_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	MPU_Handle.Init.ClockSpeed = 100000;
	MPU_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	MPU_Handle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	MPU_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	MPU_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	MPU_Handle.Init.OwnAddress1 = 0;
	MPU_Handle.Init.OwnAddress2 = 0;

	HAL_I2C_Init(&MPU_Handle);
}


int Sensors_I2C_WriteRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	const unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_I2C_Retry();

tryWriteAgain:
	ret = HAL_I2C_Mem_Write(&MPU_Handle, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, 500);

	if (ret)
		I2C_Reset();

	if (ret && retry_in_mlsec)
	{
		if (retries++ > 4)
			return ret;

		HAL_Delay(retry_in_mlsec);
		goto tryWriteAgain;
	}
	return ret;
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr,
	unsigned char reg_addr,
	unsigned short len,
	unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = Get_I2C_Retry();

tryReadAgain:
	ret = HAL_I2C_Mem_Read(&MPU_Handle, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, 500);

	if (ret)
		I2C_Reset();

	if (ret && retry_in_mlsec)
	{
		if (retries++ > 4)
			return ret;

		HAL_Delay(retry_in_mlsec);
		goto tryReadAgain;
	}
	return ret;
}

static unsigned short RETRY_IN_MLSEC = 55;

void Set_I2C_Retry(unsigned short ml_sec)
{
	RETRY_IN_MLSEC = ml_sec;
}

unsigned short Get_I2C_Retry()
{
	return RETRY_IN_MLSEC;
}
