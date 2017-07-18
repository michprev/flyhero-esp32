/*
 * Logger.h
 *
 *  Created on: 8. 2. 2017
 *      Author: michp
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "stm32f4xx_hal.h"
#include "MPU6050.h"
#include "ESP.h"
#include "Motors_Controller.h"

namespace flyhero {

class Logger {
public:
	enum Data_Type {
		Accel_X = 1 << 10, Accel_Y = 1 << 9, Accel_Z = 1 << 8, Gyro_X = 1 << 7, Gyro_Y = 1 << 6, Gyro_Z = 1 << 5,
		Temperature = 1 << 4, Roll = 1 << 3, Pitch = 1 << 2, Yaw = 1 << 1, Throttle = 1 << 0,
		Accel_All = Accel_X | Accel_Y | Accel_Z,
		Gyro_All = Gyro_X | Gyro_Y | Gyro_Z,
		Euler_All = Roll | Pitch | Yaw
	};

	enum Log_Type { WiFi, UART };

private:
	Logger();
	Logger(Logger const&){};
	Logger& operator=(Logger const&){};

	UART_HandleTypeDef huart;
	DMA_HandleTypeDef hdma_usart2_tx;
	Data_Type data_type;
	Log_Type log_type;
	uint8_t data_buffer[32];
	bool log;
	uint32_t last_ticks;

public:
	static Logger& Instance();
	HAL_StatusTypeDef Init();
	HAL_StatusTypeDef Print(uint8_t *data, uint16_t len);
	HAL_StatusTypeDef Set_Data_Type(Log_Type log_type, Data_Type data_type);
	HAL_StatusTypeDef Send_Data();

	DMA_HandleTypeDef* Get_DMA_Tx_Handle();
	UART_HandleTypeDef* Get_UART_Handle();
};

inline Logger::Data_Type operator|(Logger::Data_Type a, Logger::Data_Type b) {
		return static_cast<Logger::Data_Type>(static_cast<int>(a) | static_cast<int>(b));
}

}

#endif /* LOGGER_H_ */
