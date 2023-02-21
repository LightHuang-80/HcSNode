/*
 * fsusart.h
 *
 *  Created on: 2020年9月2日
 *      Author: Administrator
 */

#ifndef SRC_USARTS_FSUSART_H_
#define SRC_USARTS_FSUSART_H_

#include "message.h"

#ifdef __cplusplus
extern "C" {
#endif

void FUART_Init();
void FUART_sendMsg(Joint_Msg_t* msg, uint8_t* data, size_t len);
int  FUART_peekMsg(UART_HandleTypeDef* uart, uint8_t* buf);
void FUART_beginReceive(UART_HandleTypeDef* uart);
#ifdef __cplusplus
}
#endif

#endif /* SRC_USARTS_FSUSART_H_ */
