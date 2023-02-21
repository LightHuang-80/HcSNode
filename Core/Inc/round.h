/*
 * round.h
 *
 *  Created on: 2020年8月20日
 *      Author: Administrator
 */

#ifndef SRC_ENCODER_ROUND_H_
#define SRC_ENCODER_ROUND_H_

#ifdef __cplusplus
extern "C" {
#endif

void RND_Init(uint16_t angle);

void RND_Input(uint16_t angle);
int16_t  RND_GetRound();
uint16_t RND_GetHomeAngle();
int32_t  RND_GetCrossAngle(uint16_t angle);
int32_t  RND_GetIncSteps(uint16_t angle, float ms);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ENCODER_ROUND_H_ */
