/*
 * home.c
 *
 *  Created on: 2020年10月8日
 *      Author: Administrator
 */


/*
 * 	607Ch: Home offset
	6098h: Homing method
	6099h: Homing speeds
	609Ah: Homing acceleration
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "c402.h"
#include "c402def.h"
#include "dsstate.h"
#include "nstepper.h"
#include "home.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "nmotion.h"
#include "mcdrv.h"

typedef void (*pHomeFinishedCallback)(int32_t);

typedef struct DS402_HOME_Data_ {
	DS402_HOME_status_t status;
	int32_t  offset;
	int32_t  speed;
	int32_t  acce;
	int32_t  reachWindow;
	int32_t  zeroPoint;

	pHomeFinishedCallback finishedCallback;
	int8_t   method;

}DS402_HOME_Data_t;

DS402_HOME_Data_t g_MotorHomeData;

extern TIM_HandleTypeDef htim13;

extern volatile int32_t g_IncSteps;
extern QueueHandle_t g_MotionMsgQueue;
extern Node_DriveProfile_t g_NodeDriveProfile;

void HOME_reset()
{
	g_MotorHomeData.status = DS402_HOME_None;
	g_MotorHomeData.reachWindow = 32;
}

void HOME_setProfile(int32_t offset, int8_t method, uint32_t speed, uint32_t acce)
{
	g_MotorHomeData.method = method;
	g_MotorHomeData.offset = offset;
	g_MotorHomeData.speed  = speed;
	g_MotorHomeData.acce   = acce;
}

bool HOME_Init(int32_t offset, int8_t method, uint32_t speed, uint32_t acce)
{
	bool_t ret = true;

	HOME_setProfile(offset, method, speed, acce);
	g_MotorHomeData.finishedCallback = NULL;

	HOME_reset();

	return ret;
}

void HOME_setFinishedCallback(void (*pfunct)(int32_t offset))
{
	g_MotorHomeData.finishedCallback = pfunct;
}

void HOME_finish(int32_t position)
{
	if (g_MotorHomeData.status != DS402_HOME_Finished) {
		g_MotorHomeData.status = DS402_HOME_Finished;
		if (g_MotorHomeData.finishedCallback){
			g_MotorHomeData.finishedCallback(position);
		}
	}

	MT_setReachCallback(NULL);
}

void HOME_active()
{
	MT_setReachCallback(HOME_finish);
	MT_setSpeed(g_MotorHomeData.speed);
	MT_setReachWindow(g_MotorHomeData.reachWindow);
}

/*Re-enter safe function
 * controlWord uint16_t : receive from host
 * return      uint16_t : execute result, and will be merged status word bit10, bit12 and bit13
 * */
uint16_t HOME_exec(uint16_t controlWord)
{
	uint16_t ret = 0;

	/*Get the operation mode*/
	DS402_OperMode_Action_t action = DS402_getModeActionFromControlWord(DS402_OperMode_Homing, controlWord);
	if (action == DS402_OperMode_HomingStart) {
		if (g_MotorHomeData.status == DS402_HOME_None) {

			printf("Motor start homeing.\n");

			/*Home begin*/
			g_MotorHomeData.status = DS402_HOME_SearchZeroPnt;

			/*Get home switch state*/
			GPIO_PinState state = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);

			MotionMsgItem_t item;

			if (state == g_NodeDriveProfile.homestate){
				/*Set the home offset point*/
				item.position = g_MotorHomeData.offset;

				/*Send Go home msg*/
				item.code = MMC_HomeGoHome;
			}else {
				/*Set the home speed and accelerate*/
				item.velocity = g_MotorHomeData.speed;
				item.acceleration = g_MotorHomeData.acce;

				/*Start motor, send home start msg*/
				item.code = MMC_HomeStart;
			}
			xQueueSend(g_MotionMsgQueue, &item, 0);
		}else {
			if (g_MotorHomeData.status == DS402_HOME_Error) {
				/*Status 10 reach bit set 1(stop), 13 error bit set 1 */
				ret = (1 << 10) | (1 << 13);
				return ret;
			}
		}

		if (g_MotorHomeData.status == DS402_HOME_Finished) {
			/*Status 10 reach bit set 1(stop), 12 attain bit set 1 */
			ret = (1 << 10) | (1 << 12);
			return ret;
		}
	}

	return ret;
}

void HOME_OnSwitchSignal(uint16_t pin)
{
	if (pin != GPIO_PIN_2) {
		return;
	}

	if (g_MotorHomeData.status != DS402_HOME_GoHome && g_MotionMsgQueue != NULL){

		/*Reset the callback*/
		MT_setReachCallback(HOME_finish);

		/*Send go home message*/
		MotionMsgItem_t item;
		item.code = MMC_HomeGoHome;
		item.position = g_MotorHomeData.offset;

		BaseType_t highTaskWoken = pdFALSE;
		if (pdPASS == xQueueSendFromISR(g_MotionMsgQueue, &item, &highTaskWoken)){
			g_MotorHomeData.status = DS402_HOME_GoHome;
		}

		if(highTaskWoken == pdTRUE) {
			portYIELD_FROM_ISR (highTaskWoken);
		}
	}
}
