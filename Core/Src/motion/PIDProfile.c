/*
 * PIDProfile.c
 *
 *  Created on: 2023年3月8日
 *      Author: Administrator
 */

#include <PIDProfile.h>

PID_p rtPID;

/* Block signals and states (default storage) */
PIDDW rtPIDDW;

/* External inputs (root inport signals with default storage) */
PIDExtU rtPIDU;

/* External outputs (root outports fed by signals with default storage) */
PIDExtY rtPIDY;

/* Model step function */
void PIDProfile_Step(real32_T ts, real32_T curPos)
{
  real32_T rtb_FilterCoefficient;
  real32_T rtb_perr;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/CurrentPos'
   *  Inport: '<Root>/TargetPos'
   */
  rtb_perr = rtPIDU.TargetPos - curPos;

  /* Gain: '<S36>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S28>/Filter'
   *  Gain: '<S27>/Derivative Gain'
   *  Sum: '<S28>/SumD'
   */
  rtb_FilterCoefficient = (rtPID.Kd * rtb_perr - rtPIDDW.Filter_DSTATE) * rtPID.DerFilter;

  /* Outport: '<Root>/CmdPos' incorporates:
   *  DiscreteIntegrator: '<S33>/Integrator'
   *  Gain: '<S38>/Proportional Gain'
   *  Sum: '<S42>/Sum'
   */
  rtPIDY.CmdPos = (rtPID.Kp * rtb_perr + rtPIDDW.Integrator_DSTATE) +
	rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
   *  Gain: '<S30>/Integral Gain'
   */
  rtPIDDW.Integrator_DSTATE += rtPID.Ki * rtb_perr * ts;

  /* Update for DiscreteIntegrator: '<S28>/Filter' */
  rtPIDDW.Filter_DSTATE += ts * rtb_FilterCoefficient;

  /* Calculate the cmd velocity*/
  real32_T cv = rtb_perr / ts;

  if (cv > rtPIDU.MaxVel)
	  cv = rtPIDU.MaxVel;

  if (cv < -rtPIDU.MaxVel)
	  cv = -rtPIDU.MaxVel;

  rtPIDY.CmdVel = cv;
}

void PIDProfile_Start(real32_T targetPos, real32_T maxVel, real32_T maxAcce, real32_T maxJerk)
{
	rtPIDU.TargetPos = targetPos;
	rtPIDU.MaxVel    = maxVel;
	rtPIDU.MaxAcce   = maxAcce;
	rtPIDU.MaxJerk   = maxJerk;

	/* Initialize the output value*/
	rtPIDY.CmdPos = 0.0f;
	rtPIDY.CmdVel = 0.0f;
	rtPIDY.CmdAcce = 0.0f;

}

void PIDProfile_Initialize(real32_T Kp, real32_T Ki, real32_T Kd, real32_T dc)
{
	rtPID.Kp = Kp;
	rtPID.Ki = Ki;
	rtPID.Kd = Kd;
	rtPID.DerFilter = dc;
}
