/*
 * CANStm32.c
 *
 *  Created on: 2020��5��21��
 *      Author: Light.Huang
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "can.h"

#include "CANopen.h"
#include "CO_driver_target.h"
#include "CO_OD.h"

static CAN_TxHeaderTypeDef TxHeader;

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */

  if (HAL_CAN_ActivateNotification(CANmodule->CANptr,
		  CAN_IT_RX_FIFO0_MSG_PENDING |
		  CAN_IT_RX_FIFO1_MSG_PENDING |
		  CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
    CANmodule->CANnormal = false;
  }

  if (HAL_CAN_Start(CANmodule->CANptr) != HAL_OK){
	CANmodule->CANnormal = false;
  }else{
    CANmodule->CANnormal = true;
  }
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANnormal = false;

    CANmodule->useCANrxFilters = false; /* Use standard id, none id filter */

    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;
    CANmodule->em = NULL;

    for(i=0U; i<rxSize; i++){
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }

    for(i=0U; i<txSize; i++){
        txArray[i].bufferFull = false;
    }

    /*STM32 CAN initialize*/
    CO_CANmodule_disable(CANmodule);
    HAL_CAN_MspDeInit(CANmodule->CANptr);

    /*ReInitialize msp gpio and nvic*/
    HAL_CAN_MspInit(CANmodule->CANptr);

    /* Time rate calc with http://www.bittiming.can-wiki.info/
     * The base MCU F405, CAN system clock: 42M
     * */
    uint32_t Prescaler;
    uint32_t seg1 = CAN_BS1_10TQ;
    uint32_t seg2 = CAN_BS2_1TQ;

    switch(CANbitRate){
    case 1000: {
    	Prescaler = 3;
    	seg1 = CAN_BS1_11TQ;
    	seg2 = CAN_BS2_2TQ;
    	break;
    }
    case 500:  {
    	Prescaler = 5;
    	seg1 = CAN_BS1_13TQ;
    	seg2 = CAN_BS2_2TQ;
    	break;
    }
    case 250:  {
    	Prescaler = 10;
    	seg1 = CAN_BS1_13TQ;
    	seg2 = CAN_BS2_2TQ;
    	break;   // default is 8
    }
    case 125:  {
    	Prescaler = 20;
    	seg1 = CAN_BS1_13TQ;
    	seg2 = CAN_BS2_2TQ;
    	break;
    }
    case 100:  {
    	Prescaler = 35;
    	seg1 = CAN_BS1_10TQ;
    	seg2 = CAN_BS2_1TQ;
    	break;
    }
    case 50:   {
    	Prescaler = 70;
    	seg1 = CAN_BS1_10TQ;
    	seg2 = CAN_BS2_1TQ;
    	break;
    }
    case 20:   {
    	Prescaler = 175;
    	seg1 = CAN_BS1_10TQ;
    	seg2 = CAN_BS2_1TQ;
    	break;
    }
    case 10:   {
    	Prescaler = 350;
    	seg1 = CAN_BS1_10TQ;
    	seg2 = CAN_BS2_1TQ;
    	break;
    }
    default: {// default setting
    	Prescaler = 7;
    	seg1 = CAN_BS1_10TQ;
    	seg2 = CAN_BS2_1TQ;
    }
    }

    /*Target baudrate 500K, now accept 250K compatible with FDCAN*/

    CAN_HandleTypeDef* canHandle = (CAN_HandleTypeDef*)CANmodule->CANptr;
    canHandle->Instance = CAN1;
    canHandle->Init.Prescaler = Prescaler;
    canHandle->Init.Mode = CAN_MODE_NORMAL;
    canHandle->Init.SyncJumpWidth = CAN_SJW_1TQ;
    canHandle->Init.TimeSeg1 = seg1;
    canHandle->Init.TimeSeg2 = seg2;
    canHandle->Init.TimeTriggeredMode = DISABLE;
    canHandle->Init.AutoBusOff = ENABLE;
    canHandle->Init.AutoWakeUp = ENABLE;
    canHandle->Init.AutoRetransmission = ENABLE;
    canHandle->Init.ReceiveFifoLocked = DISABLE;
    canHandle->Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(canHandle) != HAL_OK)
    {
      Error_Handler();
    }

    CAN_FilterTypeDef filter;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(canHandle, &filter) != HAL_OK) {
    	Error_Handler();
    }

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    /* turn off the module */
  CANmodule->CANnormal = false;
  HAL_CAN_DeactivateNotification(CANmodule->CANptr,
		  CAN_IT_RX_FIFO0_MSG_PENDING |
		  		  CAN_IT_RX_FIFO1_MSG_PENDING |
		  		  CAN_IT_TX_MAILBOX_EMPTY);
  HAL_CAN_Stop(CANmodule->CANptr);
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (CANrx_callback!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
        buffer->ident = ident << 2;
        if (rtr) buffer->ident |= 0x02;
        buffer->mask = ((mask & 0x07FF) << 2) | 0x02;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){

        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = (ident & 0x07FF) << 2;
        if (rtr) buffer->ident |= 0x02;

        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    /*
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
        }
        err = CO_ERROR_TX_OVERFLOW;
    }*/

    CO_LOCK_CAN_SEND();

    /* if CAN TX buffer is free, copy message to it */
    if(CANmodule->CANtxCount == 0 && (HAL_CAN_GetTxMailboxesFreeLevel(CANmodule->CANptr) > 0)){
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
        /* copy message and txRequest */

        TxHeader.StdId   = (buffer->ident >> 2) & 0x7FF;
        TxHeader.DLC     = buffer->DLC;
        TxHeader.ExtId   = 0;
        TxHeader.IDE     = CAN_ID_STD;
        TxHeader.RTR     = buffer->ident & 0x02;

        uint32_t mailBox;
        if (HAL_CAN_AddTxMessage(CANmodule->CANptr, &TxHeader, &buffer->data[0], &mailBox) != HAL_OK){
          err = HAL_CAN_GetError(CANmodule->CANptr);
        }else{
        	/* First CAN message (bootup) was sent successfully */
        	CANmodule->firstCANtxMessage = false;
        }

    }
    /* if no buffer is free, message will be sent by interrupt */
    else{
    	if (!buffer->bufferFull){
    		buffer->bufferFull = true;
    		CANmodule->CANtxCount++;
    	}
    }
    CO_UNLOCK_CAN_SEND();

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND();
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND();


    if(tpdoDeleted != 0U){
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
    }
}


void CO_CANpolling_Tx(CO_CANmodule_t *CANmodule)
{
	/*No pending tx msg*/
	if (HAL_CAN_GetTxMailboxesFreeLevel(CANmodule->CANptr) == 0)
		return;

	/* Clear flag from previous message */
	CANmodule->bufferInhibitFlag = false;

	/* Are there any new messages waiting to be send */
	if(CANmodule->CANtxCount > 0U) {
		uint16_t i;             /* index of transmitting message */

		/* first buffer */
		CO_CANtx_t *buffer = &CANmodule->txArray[0];

		/* search through whole array of pointers to transmit message buffers. */
		for(i = CANmodule->txSize; i > 0U; i--) {
			/* if message buffer is full, send it. */
			if(buffer->bufferFull) {

				/* Copy message to CAN buffer */
				CANmodule->bufferInhibitFlag = buffer->syncFlag;

				uint32_t mailBox;

		        TxHeader.StdId   = (buffer->ident >> 2) & 0x7FF;
		        TxHeader.DLC     = buffer->DLC;
		        TxHeader.ExtId   = 0;
		        TxHeader.IDE     = CAN_ID_STD;
		        TxHeader.RTR     = buffer->ident & 0x02;

		        if (HAL_CAN_AddTxMessage(CANmodule->CANptr, &TxHeader, &buffer->data[0], &mailBox) == HAL_OK){
		        	/* First CAN message (bootup) was sent successfully */
		        	CANmodule->firstCANtxMessage = false;

		        	buffer->bufferFull = false;
                	CANmodule->CANtxCount--;
                }

				break;                      /* exit for loop */
			} else {
				/*do nothing*/;
			}
			buffer++;
		}/* end of for loop */

		/* Clear counter if no more messages */
		if(i == 0U) {
			CANmodule->CANtxCount = 0U;
		}else {
			/*do nothing*/;
		}
	}
}
/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule)
{
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;

    /* get error counters from module. Id possible, function may use different way to
     * determine errors. */
    uint32_t err = ((CAN_HandleTypeDef*)(CANmodule->CANptr))->ErrorCode;

    if(CANmodule->errOld != err){
        CANmodule->errOld = err;

        if(err & HAL_CAN_ERROR_BOF){
        	/* bus off, 离线状态 */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else{
        	/* not bus off, 在线恢复 */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if(err & HAL_CAN_ERROR_EWG){
            	/* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);

            }else if(err & HAL_CAN_ERROR_EPV){
            	/* RX/TX bus passive, 被动错误 */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);

                if(!CANmodule->firstCANtxMessage){
                	CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);

                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if(err == HAL_CAN_ERROR_NONE){       /* no error */
                CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
            }
        }

        if((err & HAL_CAN_ERROR_RX_FOV0) || (err & HAL_CAN_ERROR_RX_FOV1)){
        	/* CAN RX bus overflow */
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
        }else{
        	CO_errorReset(em, CO_EM_CAN_RXB_OVERFLOW, err);
        }
    }
}


/******************************************************************************/
void CO_CANinterrupt(CO_CANmodule_t *CANmodule, uint32_t RxFifo)
{
    /* receive interrupt */
    if(1){
        CO_CANrxMsg_t rcvMsg;      /* pointer to received message in CAN module */
        uint16_t index;             /* index of received message */

        CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        HAL_CAN_GetRxMessage(CANmodule->CANptr, RxFifo, &rcvMsg.header, &rcvMsg.data[0]);

        rcvMsg.DLC   = rcvMsg.header.DLC;
        rcvMsg.ident = rcvMsg.header.StdId;

        /*Expand with CANopen rx/tx buffer setting*/
        rcvMsg.ident <<= 2;
        rcvMsg.ident |= rcvMsg.header.RTR;

        /*
        uint8_t cobId  = (rcvMsg.header.StdId & 0xFFFFFF80) >> 7;
        uint8_t nodeId = rcvMsg.header.StdId & 0x7F;

        printf("Rcv from bus: ident: %ld , cob: %d, node: %d, dlc: %ld, data[0]: 0x%02x\n",
        		rcvMsg.ident,
				cobId, nodeId,
				rcvMsg.header.DLC,
				rcvMsg.data[0]);
		*/

        if(CANmodule->useCANrxFilters){
            /* CAN module filters are used. Message with known 11-bit identifier has */
            /* been received */
            index = 0;  /* get index of the received message here. Or something similar */
            if(index < CANmodule->rxSize){
                buffer = &CANmodule->rxArray[index];
                /* verify also RTR */
                if(((rcvMsg.ident ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                }
            }
        }
        else{
            /* CAN module filters are not used, message with any standard 11-bit identifier */
            /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
            buffer = &CANmodule->rxArray[0];
            for(index = CANmodule->rxSize; index > 0U; index--){
                if(((rcvMsg.ident ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                    break;
                }
                buffer++;
            }
        }

        /* Call specific function, which will process the message */
        if(msgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)){
            buffer->CANrx_callback(buffer->object, (void*) &rcvMsg);
        }

        /* Clear interrupt flag */
    }


    /* transmit interrupt */
    else if(0){
        /* Clear interrupt flag */

        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        if(CANmodule->CANtxCount > 0U){
            uint16_t i;             /* index of transmitting message */

            /* first buffer */
            CO_CANtx_t *buffer = &CANmodule->txArray[0];
            /* search through whole array of pointers to transmit message buffers. */
            for(i = CANmodule->txSize; i > 0U; i--){
                /* if message buffer is full, send it. */
                if(buffer->bufferFull){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;

                    /* Copy message to CAN buffer */
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    /* canSend... */
                    break;                      /* exit for loop */
                }
                buffer++;
            }/* end of for loop */

            /* Clear counter if no more messages */
            if(i == 0U){
                CANmodule->CANtxCount = 0U;
            }
        }
    }
    else{
        /* some other interrupt reason */
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0){
		CO_CANinterrupt(CO->CANmodule[0], CAN_RX_FIFO0);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0){
		CO_CANinterrupt(CO->CANmodule[0], CAN_RX_FIFO1);
	}
}
