/*
 * File: PosProfile.h
 *
 * Code generated for Simulink model 'PosProfile'.
 *
 * Model version                  : 1.59
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Mon Mar  6 13:15:28 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_PosProfile_h_
#define RTW_HEADER_PosProfile_h_
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifndef PosProfile_COMMON_INCLUDES_
#define PosProfile_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* PosProfile_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T PosIntegrator_DSTATE;       /* '<S1>/Pos Integrator' */
  real32_T VelIntegrator_DSTATE;       /* '<S1>/Vel Integrator' */
  real32_T AcceIntegrator_DSTATE;      /* '<S1>/Acce Integrator' */
  real32_T t;                          /* '<S1>/ppmode' */
  real32_T VelTrancErr;
  uint8_T PosIntegrator_IC_LOADING;    /* '<S1>/Pos Integrator' */
  uint8_T VelIntegrator_IC_LOADING;    /* '<S1>/Vel Integrator' */
  boolean_T t_not_empty;               /* '<S1>/ppmode' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T InitPos;                    /* '<Root>/InitPos' */
  real32_T TargetPos;                  /* '<Root>/TargetPos' */
  real32_T MaxVel;                     /* '<Root>/MaxVel' */
  real32_T InitVel;                    /* '<Root>/InitVel' */
  real32_T TargetVel;                  /* '<Root>/TargetVel' */
  real32_T MaxAcce;                    /* '<Root>/MaxAcce' */
  real32_T MaxJerk;                    /* '<Root>/MaxJerk' */
  real32_T Ts;                         /* '<Root>/Ts' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T CmdPos;                     /* '<Root>/CmdPos' */
  real32_T CmdVel;                     /* '<Root>/CmdVel' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */

/* Position Profile time phase*/
typedef struct {
	real32_T T;    /* Profile total time duration*/
	real32_T Tv;   /* 匀速时间 */
	real32_T Ta;   /* 加速时间 */
	real32_T Tj1;  /* 加速过程中的加加速时间 */
	real32_T Td;   /* 减速时间 */
	real32_T Tj2;  /* 减速过程中的减加速时间*/
}PosPTiming_T;

typedef struct {
	PosPTiming_T timing;	/* 阶段时间片*/
	uint32_T     tms;        /* 阶段时间毫秒*/
	real32_T     dist;		/* 阶段经过的距离*/
	real32_T	 dir;		/* 阶段方向*/
}PosPPhase_T;

extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Real-time Model object */
extern RT_MODEL *const rtM;

#ifdef __cplusplus
extern "C" {
#endif
/* Model entry point functions */
void PosProfile_Initialize(void);
void PosProfile_Start(real32_T distance,
		real32_T curVel,
		real32_T targetVel,
		real32_T maxVel,
		real32_T maxAcce,
		real32_T maxJerk,
		real32_T ts);
void PosProfile_StartHalt(real32_T curVel, real32_T maxJerk, real32_T ts);
void PosProfile_Step(real32_T curPos);
real32_T PosProfile_GetTargetPos();
boolean_T PosProfile_StepOver(uint32_T ticks);

#ifdef __cplusplus
}
#endif
/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('fm/Variant Subsystem/PosProfile')    - opens subsystem fm/Variant Subsystem/PosProfile
 * hilite_system('fm/Variant Subsystem/PosProfile/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'fm/Variant Subsystem'
 * '<S1>'   : 'fm/Variant Subsystem/PosProfile'
 * '<S2>'   : 'fm/Variant Subsystem/PosProfile/ppmode'
 */
#endif                                 /* RTW_HEADER_PosProfile_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
