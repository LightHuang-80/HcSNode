/*
 * cmd.c
 *
 *  Created on: 2020年8月20日
 *      Author: Administrator
 */


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "usart.h"
#include "stepper.h"

#define BYE_CMD      0x1F
#define STEPPER_CMD  0x1E
#define DATA_CMD     0x1D
