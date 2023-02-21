/*
 * fsusart.c
 *
 *  Created on: 2020年9月2日
 *      Author: Administrator
 */

/* 系统usart
 * 包括： 1.  RingBuffer
 *   2.系统中多个usart管理
 *   3.涉及相关的DMA等
 * */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "message.h"
#include "CO_fifo.h"
#include "fsusart.h"
//
// Bound Rx/Tx fifo buffer
//

#define RXFIFO_BUFSIZE 2048
char g_RxFifoBuf[RXFIFO_BUFSIZE];

#define TXFIFO_BUFSIZE 2048
char g_TxFifoBuf[TXFIFO_BUFSIZE];

#define UART_TXDMA_BUFSIZE 1024
unsigned char g_UARTTxDMABuf[UART_TXDMA_BUFSIZE];

/* @brief Receive buffer. */
#define UART_RXDMA_BUFSIZE 1024
unsigned char g_UARTRxDMABuf[UART_RXDMA_BUFSIZE];

CO_fifo_t g_rxFifo;
CO_fifo_t g_txFifo;
SemaphoreHandle_t g_txMutex = NULL;
SemaphoreHandle_t g_rxMutex = NULL;

void FUART_Init()
{
	CO_fifo_init(&g_rxFifo, g_RxFifoBuf, RXFIFO_BUFSIZE);
	CO_fifo_init(&g_txFifo, g_TxFifoBuf, TXFIFO_BUFSIZE);

	g_txMutex = xSemaphoreCreateMutex();
	g_rxMutex = xSemaphoreCreateMutex();
}

void FUART_beginReceive(UART_HandleTypeDef* uart)
{
	HAL_UART_Receive_DMA(uart, (uint8_t*)g_UARTRxDMABuf, UART_RXDMA_BUFSIZE);
}

void FUART_pushMsgToFifo(Joint_Msg_t *msg, uint8_t *data, size_t datalen, CO_fifo_t* fifo)
{
	// Enter critical section
	if (xSemaphoreTake(g_txMutex,  (TickType_t) 0)) {
		CO_fifo_write(fifo, (const char *)msg, JOINT_MSG_LEN, NULL);
		if (msg->len > 0){
			CO_fifo_write(fifo, (const char *)data, datalen, NULL);
		}
		xSemaphoreGive(g_txMutex);
	}
	// Exit critical
}

static size_t internalPeekMsgFromFifo(CO_fifo_t* fifo, uint8_t *buf)
{
	size_t len = CO_fifo_getOccupied(fifo);
	if (len < JOINT_MSG_LEN){
		return 0;
	}

	// read message hdr
	CO_fifo_altBegin(fifo, 0);
	CO_fifo_altRead(fifo, (char*)buf, JOINT_MSG_LEN);
	Joint_Msg_t* msg = (Joint_Msg_t*)buf;
	if (msg->len > 0){
		if (len < (uint16_t)(msg->len + JOINT_MSG_LEN)){
			// message not ready yet
			return 0;
		}

		// read message params from fifo
		CO_fifo_altBegin(fifo, JOINT_MSG_LEN);
		CO_fifo_altRead(fifo, (char*)(buf + JOINT_MSG_LEN), msg->len);
		CO_fifo_altFinish(fifo, NULL);
		len = msg->len + JOINT_MSG_LEN;
	}else {
		CO_fifo_altFinish(fifo, NULL);
		len = JOINT_MSG_LEN;
	}

	return len;
}

void FUART_sendMsg(Joint_Msg_t *msg, uint8_t *data, size_t len)
{
	// Check UART transmit busy
	if (huart1.gState != HAL_UART_STATE_READY || (huart1.Instance->CR3 & USART_CR3_DMAT)){
		FUART_pushMsgToFifo(msg, data, len, &g_txFifo);
		return;
	}

	// UART Transmit
	if (xSemaphoreTake(g_txMutex, (TickType_t)0)){
		memcpy(g_UARTTxDMABuf, msg, JOINT_MSG_LEN);
		if (msg->len > 0){
			memcpy(&g_UARTTxDMABuf[JOINT_MSG_LEN], data, len);
		}
		HAL_UART_Transmit_DMA(&huart1, g_UARTTxDMABuf, len + JOINT_MSG_LEN);
		xSemaphoreGive(g_txMutex);
	}
}

int FUART_peekMsg(UART_HandleTypeDef* uart, uint8_t* buf)
{
	// Peek msg
	size_t len = 0;

	if (xSemaphoreTake(g_rxMutex, (TickType_t)0)){
		len = internalPeekMsgFromFifo(&g_rxFifo, buf);
		if (len > 0){
			xSemaphoreGive(g_rxMutex);
			return len;
		}
		xSemaphoreGive(g_rxMutex);
	}

	// UART DMA rx check
	static uint32_t lastReadPos = 0;

	__disable_irq();
	uint32_t rxBufPos = UART_RXDMA_BUFSIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
	__enable_irq();

	if (xSemaphoreTake(g_rxMutex, (TickType_t)0)){

		if (rxBufPos != lastReadPos) {
			if (rxBufPos > lastReadPos) {
				uint32_t count = rxBufPos - lastReadPos;
				CO_fifo_write(&g_rxFifo, (const char*)&g_UARTRxDMABuf[lastReadPos], count, NULL);
			}else{
				/*Over run received*/
				uint32_t count = UART_RXDMA_BUFSIZE - lastReadPos;
				CO_fifo_write(&g_rxFifo, (const char*)&g_UARTRxDMABuf[lastReadPos], count, NULL);

				if (rxBufPos > 0){
					CO_fifo_write(&g_rxFifo, (const char*)&g_UARTRxDMABuf[0], rxBufPos, NULL);
				}
			}
			lastReadPos = rxBufPos;
		}

		if (lastReadPos == UART_RXDMA_BUFSIZE) {
			lastReadPos = 0;
		}

		// re-read msg
		len = internalPeekMsgFromFifo(&g_rxFifo, buf);
		xSemaphoreGive(g_rxMutex);
	}

	return len;
}


//
// UART callback functions
//

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART1) {

		// Move to next message
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		if (xSemaphoreTakeFromISR(g_txMutex, &xHigherPriorityTaskWoken)){
			size_t len = internalPeekMsgFromFifo(&g_txFifo, g_UARTTxDMABuf);
			if (len > 0) {
				HAL_UART_Transmit_DMA(&huart1, g_UARTTxDMABuf, len);
			}
			xSemaphoreGiveFromISR(g_txMutex, &xHigherPriorityTaskWoken);
		}

		if (xHigherPriorityTaskWoken == pdTRUE){
			portYIELD_FROM_ISR (xHigherPriorityTaskWoken);
		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		// Do nothing
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		uint32_t tmp1 = 0U, tmp2 = 0U;
		tmp1 = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
		tmp2 = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);

		if (tmp1 != RESET && tmp2 != RESET) {
			printf("Usart1 happens over run error.\n");

		    volatile uint8_t sysRxChar; // clear the regular interrupt
	        sysRxChar = (uint8_t) huart->Instance->DR; // not sure whether it is correct, but it does not work
	        (void) sysRxChar;

	        __HAL_UART_CLEAR_OREFLAG(huart);
	        huart->ErrorCode &= ~HAL_UART_ERROR_ORE;
		}
	}
}
