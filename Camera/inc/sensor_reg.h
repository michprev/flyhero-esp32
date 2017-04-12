/*
 * sensor_reg.h
 *
 *  Created on: 9. 4. 2017
 *      Author: michp
 */

#ifndef SENSOR_REG_H_
#define SENSOR_REG_H_

#include <stm32f4xx_hal.h>

struct sensor_reg {
	uint16_t reg;
	uint16_t val;
};


#endif /* SENSOR_REG_H_ */
