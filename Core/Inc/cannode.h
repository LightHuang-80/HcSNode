/*
 * canode.h
 *
 *  Created on: 2020年9月3日
 *      Author: Administrator
 */

#ifndef SRC_CIA_CANODE_H_
#define SRC_CIA_CANODE_H_

#include <stdint.h>

#define CAN_LOOP_DUARTION   4
#define TMR_TASK_INTERVAL	1000

#ifdef __cplusplus
extern "C" {
#endif

uint8_t NODE_new();
uint8_t NODE_Init(void* canDevice, uint8_t nodeId);
void    NODE_process(uint16_t timeInterval, bool_t syncWas);

#ifdef __cplusplus
}
#endif
#endif /* SRC_CIA_CANODE_H_ */
