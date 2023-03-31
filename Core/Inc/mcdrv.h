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
#include "c402.h"


#ifdef __cplusplus
extern "C" {
#endif

void MCD_Init();
void MCD_reset(MotionCtrlDef_t* pmc);
void MCD_setSwtichOnDisabled(MotionCtrlDef_t* pmc);

void MCD_changeMode(MotionCtrlDef_t* pmc);

void MCD_OnProfileUpdate(Node_DriveProfile_t *profile);
void MCD_OnControlWordUpdate(MotionCtrlDef_t* pmc);

void MCD_OnTargetUpdate();
void MCD_OnVelocityUpdate();
void MCD_OnAccelerationUpdate();
void MCD_OnCommandSerialNumberUpdate();

#ifdef __cplusplus
}
#endif
#endif /* SRC_CIA_MCDRV_H_ */
