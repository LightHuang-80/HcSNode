/*
 * mcdrv.h
 *
 *  Created on: 2020年10月5日
 *      Author: Administrator
 */

#ifndef SRC_CIA_MCDRV_H_
#define SRC_CIA_MCDRV_H_

#include "profile.h"
#include "CO_driver_target.h"

void MCD_Init();
void MCD_reset();
void MCD_setOn();
void MCD_OnProfileUpdate(Node_DriveProfile_t *profile);
void MCD_broadState(uint16_t timeDifference_us, bool_t syncWas);
void MCD_OnControlWordUpdate();
void MCD_OnTargetUpdate();
void MCD_OnVelocityUpdate();
void MCD_OnAccelerationUpdate();
void MCD_OnCommandSerialNumberUpdate();
void MCD_handleMsg(MotionMsgItem_t *item);

#endif /* SRC_CIA_MCDRV_H_ */
