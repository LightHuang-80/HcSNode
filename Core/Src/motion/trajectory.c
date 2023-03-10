/*
 * trajectory.c
 *
 *  Created on: 2020年11月25日
 *      Author: Administrator
 */

/* Trajectory position blocks*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "interpolation.h"
#include "nmotion.h"
#include "c402.h"
#include "log.h"
#include "mcdrv.h"

typedef struct TPBlock_ {
	int16_t mark;
	int16_t duration;

	int32_t serialNumber;
	float position;
	float velocity;
	float acceleration;
}TPBlock;

typedef struct TPBlocksChain_{
	TPBlock  curBlock;
	TPBlock  lastBlock;
	int32_t  tickcount;
	int8_t   pair;
}TPBlocksChain;

TPBlocksChain g_BlocksChain;

extern MT_MotionDrive_t g_MotionDriveHandle;
extern Node_DriveProfile_t g_NodeDriveProfile;

void TPB_Init()
{
	g_BlocksChain.pair = 0;
	g_BlocksChain.tickcount = 0;
	memset(&g_BlocksChain.curBlock, 0, sizeof(TPBlock));
	memset(&g_BlocksChain.lastBlock, 0, sizeof(TPBlock));
}

void TPB_blockspairReady()
{
	/*uint32_t tick = HAL_GetTick();

	LOG_Print(LOG_InfoLevel, "[%d]Trajectory ready: %d, %d, %d\n", tick,
			(int32_t)ceilf(g_BlocksChain.curBlock.position    *100000.0f),
			(int32_t)ceilf(g_BlocksChain.curBlock.velocity    *100000.0f),
			(int32_t)ceilf(g_BlocksChain.curBlock.acceleration*100000.0f)
	);*/

	/*Joint 3: home - upperlimit = -20773, homeoffset: 8357, upperlimit(appr): -12416*/
	/*rad ratio: (-12416-8357)/(1.57-0.01) = -13316.0256*/

	/*Joint 2: home - upperlimit = 12123, homeoffset: 8357, upperlimit(appr): 469*/
	/*rad ratio: (12123-469)/(1.57-0.01) = 8618.590*/

	float joint_lowerlimit_fromOrigin = g_NodeDriveProfile.lower_limit_range;
	float ratio = g_NodeDriveProfile.ratio;

	//float ratio = 8618.590;  //joint 1
	//float ratio = 14356.590; // joint 2
	//float ratio = 15320.590;

	/*setup the acceleration*/
	/*setup the final speed*/
	MotionMsgItem_t item;

	item.velocity = g_BlocksChain.curBlock.velocity * ratio;
	item.acceleration =  g_BlocksChain.curBlock.acceleration * ratio;

	item.position = ceilf(g_BlocksChain.curBlock.position * ratio) + joint_lowerlimit_fromOrigin;

	item.duration = g_BlocksChain.curBlock.serialNumber - g_BlocksChain.lastBlock.serialNumber;
	if (item.duration > 10) {
		item.duration = 1;
	}

	item.code = MMC_TargetSet;

	if (fabsf(item.velocity) <= 0.0001) {
		/*S-Curve interpolation*/
		item.type = 0;
	}else {
		/*Polyminic interpolation*/
		item.type = 1;
	}

	xQueueSend(g_MotionDriveHandle.posQueue, &item, 0);
}

void TPB_curblockReady()
{
	/*compare the last block msg*/
	if (g_BlocksChain.lastBlock.position != g_BlocksChain.curBlock.position) {
		/*hold point*/
		TPB_blockspairReady();
	}else {

	}

	memcpy(&g_BlocksChain.lastBlock, &g_BlocksChain.curBlock, sizeof(TPBlock));
	memset(&g_BlocksChain.curBlock, 0, sizeof(TPBlock));
}

void TPB_positionUpdate(float position)
{
	g_BlocksChain.curBlock.position = position;
	g_BlocksChain.curBlock.mark |= 0x01;

	if (g_BlocksChain.curBlock.mark == 0x0F) {
		TPB_curblockReady();
	}
}

void TPB_commandSerialNumberUpdate(int32_t serialNumber)
{
	g_BlocksChain.curBlock.serialNumber = serialNumber;
	g_BlocksChain.curBlock.mark |= 0x08;
	if (g_BlocksChain.curBlock.mark == 0x0F) {
		TPB_curblockReady();
	}
}

void TPB_velocityUpdate(float velocity)
{
	g_BlocksChain.curBlock.velocity = velocity;
	g_BlocksChain.curBlock.mark |= 0x02;

	if (g_BlocksChain.curBlock.mark == 0x0F) {
		TPB_curblockReady();
	}
}

void TPB_accelerationUpdate(float acceleration)
{
	g_BlocksChain.curBlock.acceleration = acceleration;
	g_BlocksChain.curBlock.mark |= 0x04;

	if (g_BlocksChain.curBlock.mark == 0x0F) {
		TPB_curblockReady();
	}
}
