/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "PWM_Generator.h"
#include "ESP8266_UDP.h"
#include "MS5611.h"
#include "MPU9250.h"

#ifdef DEBUG
extern "C" void initialise_monitor_handles(void);
#endif

const uint32_t UART_BUFFER_SIZE = 2048;
unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";


ESP8266_UDP esp8266 = ESP8266_UDP(UART_BUFFER_SIZE);
PWM_Generator generator;
MPU9250 mpu;
MS5611 ms5611;

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	mpu.dataReady = true;
}

extern "C" void DMA1_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&esp8266.hdma_usart3_rx);
}

uint16_t prev1 = 0x00;
uint16_t prev2 = 0x00;
uint16_t prev3 = 0x00;
uint16_t prev4 = 0x00;

bool on = true;

void IPD_Callback(uint8_t *data, uint16_t length) {
	switch (length) {
	case 10:
		if (data[0] == 0x4D && data[9] == 0x4D) {
			uint16_t motor1, motor2, motor3, motor4;

			motor1 = data[1] << 8;
			motor1 |= data[2];
			motor2 = data[3] << 8;
			motor2 |= data[4];
			motor3 = data[5] << 8;
			motor3 |= data[6];
			motor4 = data[7] << 8;
			motor4 |= data[8];

			if (prev1 != motor1) {
				generator.SetPulse(motor1 == 0xFFFF ? 950 : motor1 + 1000, 0);
				prev1 = motor1;
			}
			if (prev2 != motor2) {
				generator.SetPulse(motor2 == 0xFFFF ? 950 : motor2 + 1000, 1);
				prev2 = motor2;
			}
			if (prev3 != motor3) {
				generator.SetPulse(motor3 == 0xFFFF ? 950 : motor3 + 1000, 2);
				prev3 = motor3;
			}
			if (prev4 != motor4) {
				generator.SetPulse(motor4 == 0xFFFF ? 950 : motor4 + 1000, 3);
				prev4 = motor4;
			}
		}
		break;
	case 3:
		if (length == 3) {
			if (on)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

			on = !on;
		}
		break;
	}
}

int main(void)
{
	HAL_Init();
#ifdef DEBUG
	initialise_monitor_handles();
#endif

	printf("init\n");

	__GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_5;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	IWDG_HandleTypeDef hiwdg;
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 2047;

	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		printf("Could not start watchdog\n");
	}

	generator.Init();
	generator.Arm();

	// reset gyro
	uint8_t result;
	if (result = mpu.Init()) {
		printf("Error %d\n", result);
	}
	else
		printf("Everything ok..\n");

	// reset barometer
	ms5611.Init();

	// reset ESP8266
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	while (!esp8266.ready)
		esp8266.WaitReady();
	esp8266.IPD_Callback = &IPD_Callback;
	esp8266.output = true;
	esp8266.Init();
	printf("Init complete\n");

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	int32_t press, temp;
	long data[3];
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	uint8_t accuracy;
	uint8_t udpD[20];
	uint32_t timestamp = HAL_GetTick();

	while (true) {
		HAL_IWDG_Refresh(&hiwdg);

		if (HAL_GetTick() - timestamp >= 1000) {
			ms5611.GetData(&temp, &press);
			mpu.CheckNewData(data, &accuracy);

			udpD[0] = (data[0] >> 24) & 0xFF;
			udpD[1] = (data[0] >> 16) & 0xFF;
			udpD[2] = (data[0] >> 8) & 0xFF;
			udpD[3] = data[0] & 0xFF;
			udpD[4] = (data[1] >> 24) & 0xFF;
			udpD[5] = (data[1] >> 16) & 0xFF;
			udpD[6] = (data[1] >> 8) & 0xFF;
			udpD[7] = data[1] & 0xFF;
			udpD[8] = (data[2] >> 24) & 0xFF;
			udpD[9] = (data[2] >> 16) & 0xFF;
			udpD[10] = (data[2] >> 8) & 0xFF;
			udpD[11] = data[2] & 0xFF;
			udpD[12] = (temp >> 24) & 0xFF;
			udpD[13] = (temp >> 16) & 0xFF;
			udpD[14] = (temp >> 8) & 0xFF;
			udpD[15] = temp & 0xFF;
			udpD[16] = (press >> 24) & 0xFF;
			udpD[17] = (press >> 16) & 0xFF;
			udpD[18] = (press >> 8) & 0xFF;
			udpD[19] = press & 0xFF;

			esp8266.SendUDP(udpD, 20);

			timestamp = HAL_GetTick();
		}

		esp8266.WaitReady(0);
	}
}
