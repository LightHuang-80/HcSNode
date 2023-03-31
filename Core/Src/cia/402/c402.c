/*
 * c402.c
 *
 *  Created on: 2020年8月3日
 *      Author: Administrator
 */

#include "main.h"
#include "c402def.h"
#include "dspp.h"

#include "301/CO_driver.h"
#include "301/CO_SDOserver.h"
#include "CO_OD.h"

#include "c402.h"

/* Motor driver:
 * a) Maintain the node status
 * b) report status to upper device
 *
 * 1. Receive control command
 * 2. Monitor node status
 * 3. Send-back node position
 * 4. Increase steps
 * 5. Setting velocity parameters, accelerate and decelerate
 * 6. plug and play
 * */


MotionCtrlDef_t g_MotionCtrl;

/* ============================
 * RPDO
 * ============================ */
CO_OD_entryRecord_t OD_record6040[1] = {
		{(void*)&g_MotionCtrl.controlWord, 0xFE, 2}
};

CO_OD_entryRecord_t OD_record6060[1] = {
		{(void*)&g_MotionCtrl.mode, 0xFE, 1}
};

CO_OD_entryRecord_t OD_record607A[1] = {
		{(void*)&g_MotionCtrl.targetPos, 0xFE, 4}
};

CO_OD_entryRecord_t OD_recordC101[1] = {
		{(void*)&g_MotionCtrl.commandSerialNumber, 0xFE, 4}
};

CO_OD_entryRecord_t OD_record6081[1] = {
		{(void*)&g_MotionCtrl.profileVel, 0xFE, 4}
};

CO_OD_entryRecord_t OD_record6083[1] = {
		{(void*)&g_MotionCtrl.acce, 0xFE, 4}
};

CO_OD_entryRecord_t OD_record6084[1] = {
		{(void*)&g_MotionCtrl.dece, 0xFE, 4}
};
/* ============================================ */

/* ============================
 * TPDO
 * ============================ */
CO_OD_entryRecord_t OD_record6041[1] = {
		{(void*)&g_MotionCtrl.statusWord, 0xFE, 2}
};

CO_OD_entryRecord_t OD_record6061[1] = {
		{(void*)&g_MotionCtrl.displayMode, 0xFE, 1}
};

CO_OD_entryRecord_t OD_record6063[1] = {
		{(void*)&g_MotionCtrl.currentPos, 0xFE, 4}
};

CO_OD_entryRecord_t OD_record606C[1] = {
		{(void*)&g_MotionCtrl.currentVel, 0xFE, 4}
};
