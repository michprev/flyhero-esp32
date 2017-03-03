/*
 * main.h
 *
 *  Created on: 3. 3. 2017
 *      Author: michp
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "PWM_Generator.h"
#include "ESP8266_UDP.h"
#include "MS5611.h"
#include "MPU9250.h"
#include "LEDs.h"
#include "NEO_M8N.h"
#include "PID.h"
#include "Logger.h"

extern ESP8266_UDP *esp8266;
extern PWM_Generator *pwm;
extern MPU9250 *mpu;
extern MS5611 *ms5611;
extern NEO_M8N *neo;
extern Logger *logger;

#endif /* MAIN_H_ */
