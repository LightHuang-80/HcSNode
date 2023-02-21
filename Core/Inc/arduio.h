/*
 * arduio.h
 *
 *  Created on: Mar 26, 2021
 *      Author: Administrator
 */

#ifndef INC_ARDUIO_H_
#define INC_ARDUIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef enum PIN_MODE_t{
	OUTPUT,
	INPUT_PULLDOWN,
	INPUT_PULLUP
}PIN_MODE;

#ifdef __cplusplus
extern "C" {
#endif

void pinMode(GPIO_TypeDef  *port, uint32_t pin, PIN_MODE mode);

#ifdef __cplusplus
}
#endif

#endif /* INC_ARDUIO_H_ */
