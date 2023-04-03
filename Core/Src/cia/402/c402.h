/*
 * c402.h
 *
 *  Created on: 2020年10月5日
 *      Author: Administrator
 */

#ifndef SRC_CIA_402_C402_H_
#define SRC_CIA_402_C402_H_

#include "301/CO_driver.h"
#include "301/CO_SDOserver.h"
#include "CO_OD.h"

struct sOD_Cia402_RAM {
	UNSIGNED32     FirstWord;

/*6093h*/ UNSIGNED32 position_factor[2];         //position_factor = numerator/divisor = Gear ratio * Increments/Rotation / Feed Constant
/*6094h*/ UNSIGNED32 velocity_encoder_factor[2]; //velocity_encoder_factor = numerator/divisor = gear_ratio * time_factor_v/feed_constant
/*6097h*/ UNSIGNED32 acceleration_factor[2];     //acceleration_factor = nummerator/divisor = gear_ratio * time_factor_a / feed_constant
/*607Eh*/ UNSIGNED8  polarity;                   // bit 6 velocity polarity, bit 7 position polarity
/*6073h*/ UNSIGNED16 max_current;

/*6065h*/ UNSIGNED32 following_error_window;
/*6066h*/ UNSIGNED16 following_error_time_out;
/*6067h*/ UNSIGNED32 position_window;
/*6068h*/ UNSIGNED16 position_window_time;
/*60F4h*/ INTEGER32  following_error_actual_value;

/*These limit the numerical range of the input value.
 * for relative position target set, the input value must be checked with range limit*/
/*607Bh*/ INTEGER32 position_range_limit[2];

/*Every new target position must be checked against these limits.
 * The limit positions are specifiedin position units (same as target position)
 * and are always relative to the machine home position.
<----
corrected min position limit = min position limit - home offset
corrected max position limit = max position limit - home offset
---->*/
/*607Dh*/ INTEGER32 software_position_limit[2];

/*maximum allowed speed in either direction during a profiled move*/
/*607Fh*/ UNSIGNED32 max_profile_velocity;

/*6080h*/ UNSIGNED32 max_motor_speed;

/*6081h*/ UNSIGNED32 profile_velocity;

/*6083h*/ UNSIGNED32 profile_acceleration;

/*6084h*/ UNSIGNED32 profile_deceleration;

/*6085h*/ UNSIGNED32 quick_stop_deceleration;

/*6086h*/ INTEGER16  motion_profile_type; //defautl 3 Jerk-limited ramp

/*60C5h*/ UNSIGNED32 max_acceleration;

/*60C6h*/ UNSIGNED32 max_deceleration;

/*Homing related parameters*/
/*607Ch*/ INTEGER32  home_offset;
/*6098h*/ INTEGER8 homing_method;
/*6099h*/ UNSIGNED32 homing_speeds[2];
/*609Ah*/ UNSIGNED32 homing_acceleration;

UNSIGNED32     LastWord;
};

extern struct sOD_Cia402_RAM CO_C402_Params;

typedef struct MotionCtrlDrv_ {
	uint16_t controlWord;   // 0x6040  control
	uint16_t statusWord;    // 0x6041  status

	int32_t  currentPos;    // 0x6063  current pos
	int32_t  currentVel;    // 0x606C  current pos

	int32_t  commandSerialNumber;  // 0xC101 command serial number

	int32_t  targetPos;		// 0x607A  target pos
	uint32_t profileVel;    // 0x6081  profile velocity
	uint32_t acce;			// 0x6083  accelerate
	uint32_t dece;          // 0x6084  decelerate

	int32_t  homeoffset;    // 0x607C  home offset to machine home position
	uint32_t homespeed;		// 0x6099
	uint32_t homeacce;      // 0x609A  homing acceleration

	uint32_t positionWindow; // 0x6067 reach window

	int8_t   mode;			// 0x6060  profile mode
	int8_t   displayMode;   // 0x6061  display mode

	int8_t   homemethod;	// 0x6098
}MotionCtrlDef_t;

/*Motor message code*/
#define MMC_HomeStart   0x0101
#define MMC_HomeGoHome  0x0102
#define MMC_HomeFinish  0x0103
#define MMC_HomeAtError 0x0104

#define MMC_TargetSet     0x0201
#define MMC_TargetStop    0x0202
#define MMC_TargetReach   0x0203
#define MMC_TargetAck     0x0204
#define MMC_MotionUpdated 0x0205
#define MMC_MotionHalt    0x0206

#define MMC_StatusUpdate 0x0301

typedef struct MotionMsgItem_ {
	uint16_t code;
	uint16_t type;
	int32_t position;
	float   velocity;
	float   acceleration;
	int16_t duration;
	uint8_t abs;
}MotionMsgItem_t;

typedef struct StatusMsgItem_ {
	uint16_t code;
	uint16_t status;
}StatusMsgItem_t;



#endif /* SRC_CIA_402_C402_H_ */
