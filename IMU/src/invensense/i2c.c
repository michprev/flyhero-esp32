/*******************************************************************************
File    : i2c.c
Purpose : I2c 3 to communicate with the sensors
Author  :
********************************** Includes ***********************************/
#include "i2c.h"

/********************************* Defines ************************************/



/********************************* Globals ************************************/
I2C_HandleTypeDef *I2C_Handle;

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

void I2cMaster_Init(I2C_HandleTypeDef *hi2c)
{
	I2C_Handle = hi2c;
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
	HAL_I2C_DeInit(I2C_Handle);

	I2C_Handle->Instance = I2C1;
	I2C_Handle->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2C_Handle->Init.ClockSpeed = 100000;
	I2C_Handle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2C_Handle->Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2C_Handle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2C_Handle->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	I2C_Handle->Init.OwnAddress1 = 0;
	I2C_Handle->Init.OwnAddress2 = 0;

	HAL_I2C_Init(I2C_Handle);
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
	ret = HAL_I2C_Mem_Write(I2C_Handle, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, 100);

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
	ret = HAL_I2C_Mem_Read(I2C_Handle, slave_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, 100);

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
