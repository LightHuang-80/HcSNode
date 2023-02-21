/*
 * as5048a.h
 *
 *  Created on: 2020年8月19日
 *      Author: Administrator
 */

#ifndef SRC_ENCODER_AS5048A_H_
#define SRC_ENCODER_AS5048A_H_

#ifdef __cplusplus
extern "C" {
#endif

void AS5048A_Init();

uint16_t AS5048A_ReadAngle(uint16_t *angle);
uint16_t AS5048A_ZeroPoint();

#ifdef __cplusplus
}
#endif

#endif /* SRC_ENCODER_AS5048A_H_ */
