/*
 * c402.h
 *
 *  Created on: 2020年10月5日
 *      Author: Administrator
 */

#ifndef SRC_CIA_402_C402_H_
#define SRC_CIA_402_C402_H_

typedef struct MotorDriverData_ {
	uint16_t controlWord;   // 0x6040  control
	uint16_t statusWord;    // 0x6041  status

	int8_t   mode;			// 0x6060  profile mode
	int8_t   displayMode;   // 0x6061  display mode

	int32_t  currentPos;    // 0x6063  current pos
	uint32_t currentVel;    // 0x606C  current pos

	int32_t  commandSerialNumber;  // 0xC101 command serial number

	int32_t  targetPos;		// 0x607A  target pos
	uint32_t profileVel;    // 0x6081  profile velocity

	uint32_t acce;			// 0x6083  accelerate
	uint32_t dece;          // 0x6084  decelerate
}MotorDriverData_t;

/*Motor message code*/
#define MMC_HomeStart   0x0101
#define MMC_HomeGoHome  0x0102
#define MMC_HomeFinish  0x0103
#define MMC_HomeAtError 0x0104

#define MMC_TargetSet    0x0201
#define MMC_TargetStop   0x0202
#define MMC_TargetReach  0x0203
#define MMC_TargetAck    0x0204

#define MMC_StatusUpdate 0x0301

typedef struct MotionMsgItem_ {
	uint16_t code;
	uint16_t type;

	int32_t duration;
	int32_t position;
	float   velocity;
	float   acceleration;
}MotionMsgItem_t;

typedef struct StatusMsgItem_ {
	uint16_t code;
	uint16_t status;
}StatusMsgItem_t;

#endif /* SRC_CIA_402_C402_H_ */
