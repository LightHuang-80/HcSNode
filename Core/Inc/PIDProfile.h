/*
 * PIDProfile.h
 *
 *  Created on: 2023年3月8日
 *      Author: Administrator
 */

#ifndef INC_PIDPROFILE_H_
#define INC_PIDPROFILE_H_

#include <string.h>
#include "rtwtypes.h"

typedef struct PIDS_{
	real32_T Kp;
	real32_T Ki;
	real32_T Kd;
	real32_T DerFilter;
}PID_p;

typedef struct {
  real32_T Integrator_DSTATE;            /* '<S33>/Integrator' */
  real32_T Filter_DSTATE;                /* '<S28>/Filter' */
} PIDDW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T TargetPos;                    /* '<Root>/TargetPos' */
  real32_T MaxVel;
  real32_T MaxAcce;
  real32_T MaxJerk;
} PIDExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T CmdPos;                       /* '<Root>/CmdPos' */
  real32_T CmdVel;
  real32_T CmdAcce;
} PIDExtY;

extern PIDExtY rtPIDY;

#ifdef __cplusplus
extern "C" {
#endif
  void PIDProfile_Initialize(real32_T Kp, real32_T Ki, real32_T Kd, real32_T dc);
  void PIDProfile_Start(real32_T targetPos, real32_T maxVel, real32_T maxAcce, real32_T maxJerk);
  void PIDProfile_Step(real32_T ts, real32_T curPos);

#ifdef __cplusplus
}
#endif

#endif /* INC_PIDPROFILE_H_ */
