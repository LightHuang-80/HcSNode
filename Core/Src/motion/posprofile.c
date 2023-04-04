/*
 * File: PosProfile.c
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

#include <PosProfile.h>
#define NumBitsPerChar                 8U

/* Block signals and states (default storage) */
DW rtDW;
DW rtBrakeDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

PosPTiming_T  PosPTiming;

PosPPhase_T PosPPhase[2];

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real32_T rt_powf_snf(real32_T u0, real32_T u1);
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}


/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}


/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T tmp;
  real32_T tmp_0;
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else {
    tmp = fabsf(u0);
    tmp_0 = fabsf(u1);
    if (rtIsInfF(u1)) {
      if (tmp == 1.0F) {
        y = 1.0F;
      } else if (tmp > 1.0F) {
        if (u1 > 0.0F) {
          y = (rtInfF);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = (rtInfF);
      }
    } else if (tmp_0 == 0.0F) {
      y = 1.0F;
    } else if (tmp_0 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
      y = (rtNaNF);
    } else {
      y = powf(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void PosProfile_Step(real32_T curPos)
{
  real32_T jerk = 0.0f;

  /* DiscreteIntegrator: '<S1>/Pos Integrator' incorporates:
   *  Inport: '<Root>/InitPos'
   */
  if (rtDW.PosIntegrator_IC_LOADING != 0) {
    rtDW.PosIntegrator_DSTATE = rtU.InitPos;
  }

  /* Outport: '<Root>/CmdPos' incorporates:
   *  DiscreteIntegrator: '<S1>/Pos Integrator'
   */
  rtY.CmdPos = rtDW.PosIntegrator_DSTATE;

  /* DiscreteIntegrator: '<S1>/Vel Integrator' incorporates:
   *  Inport: '<Root>/InitVel'
   */
  if (rtDW.VelIntegrator_IC_LOADING != 0) {
    rtDW.VelIntegrator_DSTATE = rtU.InitVel;
  }

  /* Outport: '<Root>/CmdVel' incorporates:
   *  DiscreteIntegrator: '<S1>/Vel Integrator'
   */
  rtY.CmdVel = rtDW.VelIntegrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S1>/Pos Integrator' incorporates:
   *  DiscreteIntegrator: '<S1>/Vel Integrator'
   */
  rtDW.PosIntegrator_IC_LOADING = 0U;
  rtDW.PosIntegrator_DSTATE += rtU.Ts * rtDW.VelIntegrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S1>/Vel Integrator' incorporates:
   *  DiscreteIntegrator: '<S1>/Acce Integrator'
   */
  rtDW.VelIntegrator_IC_LOADING = 0U;
  rtDW.VelIntegrator_DSTATE += rtU.Ts * rtDW.AcceIntegrator_DSTATE;

  /* Correct velocity*/

  if (!rtDW.t_not_empty) {
      rtDW.t = rtU.Ts / 2.0F;
      rtDW.t_not_empty = true;
  } else {
      rtDW.t += rtU.Ts;
  }

  /* MATLAB Function: '<S1>/ppmode' incorporates:
   *  Inport: '<Root>/MaxJerk'
   */

  if (rtDW.t >= 0.0F && rtDW.t < PosPPhase[0].timing.Tj1){
	  /* Phase 0 - 1*/
	  jerk = -1.0f * PosPPhase[0].dir * rtU.MaxJerk;
  } else if (rtDW.t >= PosPPhase[0].timing.Tj1 && rtDW.t < PosPPhase[0].timing.Ta){
	  /* Phase 0 - 2*/
	  jerk = PosPPhase[0].dir * rtU.MaxJerk;
  } else if ((rtDW.t >= PosPPhase[0].timing.Ta) && (rtDW.t < PosPPhase[0].timing.T + PosPPhase[1].timing.Tj1)) {
	  /*Phase 1 - 1*/
	  jerk = PosPPhase[1].dir * rtU.MaxJerk;
  } else if ((rtDW.t >= PosPPhase[0].timing.T + PosPPhase[1].timing.Tj1) && (rtDW.t < (PosPPhase[0].timing.T + PosPPhase[1].timing.Ta - PosPPhase[1].timing.Tj1))) {
	  /*Phase 1 - 2*/
	  jerk = 0.0F;
  } else if ((rtDW.t >= (PosPPhase[0].timing.T + PosPPhase[1].timing.Ta - PosPPhase[1].timing.Tj1)) && (rtDW.t < PosPPhase[0].timing.T + PosPPhase[1].timing.Ta)) {
	  /*Phase 1 - 3*/
	  jerk = -1.0F * PosPPhase[1].dir * rtU.MaxJerk;
  } else if ((rtDW.t >= PosPPhase[0].timing.T + PosPPhase[1].timing.Ta) && (rtDW.t < PosPPhase[0].timing.T + PosPPhase[1].timing.Ta + PosPPhase[1].timing.Tv)) {
	  /*Phase 1 - 4*/
	  jerk = 0.0F;
  } else if ((rtDW.t >= (PosPPhase[0].timing.T + PosPPhase[1].timing.T - PosPPhase[1].timing.Td)) && (rtDW.t < (PosPPhase[0].timing.T + PosPPhase[1].timing.T - PosPPhase[1].timing.Td + PosPPhase[1].timing.Tj2))){
	  /*Phase 1 - 5*/
	  jerk = -1.0F * PosPPhase[1].dir * rtU.MaxJerk;
  } else if ((rtDW.t >= (PosPPhase[0].timing.T + PosPPhase[1].timing.T - PosPPhase[1].timing.Td + PosPPhase[1].timing.Tj2)) && (rtDW.t < (PosPPhase[0].timing.T + PosPPhase[1].timing.T - PosPPhase[1].timing.Tj2))) {
	  /*Phase 1 - 6*/
	  jerk = 0.0F;
  } else if (rtDW.t >= (PosPPhase[0].timing.T + PosPPhase[1].timing.T - PosPPhase[1].timing.Tj2) && rtDW.t < PosPPhase[0].timing.T + PosPPhase[1].timing.T) {
	  /*Phase 1 - 7*/
	  jerk = PosPPhase[1].dir * rtU.MaxJerk;
  } else {
	  /*Phase end*/
	  jerk = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S1>/Acce Integrator' */
  rtDW.AcceIntegrator_DSTATE += rtU.Ts * jerk;
}

void PosProfile_CalcPhase1Timing(real32_T distance, real32_T curVel, real32_T targetVel, real32_T maxVel, real32_T maxAcce, real32_T jerk)
{
  real32_T Ta_tmp;
  real32_T amax;
  real32_T delta;
  real32_T jerkp2;
  real32_T accep2;
  real32_T accep4;
  real32_T velsum;   // curVel + targetVel
  real32_T velminus;

  /* MATLAB Function: '<S1>/ppmode' incorporates:
   *  Inport: '<Root>/InitPos'
   *  Inport: '<Root>/InitVel'
   *  Inport: '<Root>/MaxAcce'
   *  Inport: '<Root>/MaxJerk'
   *  Inport: '<Root>/MaxVel'
   *  Inport: '<Root>/TargetPos'
   *  Inport: '<Root>/TargetVel'
   *  Inport: '<Root>/Ts'
   */
  amax = maxAcce;

  velminus = maxVel - curVel;
  Ta_tmp = maxAcce * maxAcce;

  if (velminus * jerk < Ta_tmp) {
	if (curVel > maxVel) {
	  PosPPhase[1].timing.Tj1 = 0.0f;
	  PosPPhase[1].timing.Ta = 0.0F;
	} else {
	  PosPPhase[1].timing.Tj1 = sqrtf(velminus / jerk);
	  PosPPhase[1].timing.Ta = 2.0F * PosPPhase[1].timing.Tj1;
	}
  } else {
	  PosPPhase[1].timing.Tj1 = maxAcce / jerk;
	  PosPPhase[1].timing.Ta = velminus / maxAcce + PosPPhase[1].timing.Tj1;
  }

  velminus = maxVel - targetVel;
  if (velminus * jerk < Ta_tmp) {
	  PosPPhase[1].timing.Tj2 = sqrtf(velminus / jerk);
	  PosPPhase[1].timing.Td = 2.0F * PosPPhase[1].timing.Tj2;
  } else {
	  PosPPhase[1].timing.Tj2 = maxAcce / jerk;
	  PosPPhase[1].timing.Td = velminus / maxAcce + PosPPhase[1].timing.Tj2;
  }

  /* T = Tv + Ta + Td */
  /* Make sure the targetPos great than initPos
   * assert(rtU.TargetPos > rtU.InitPos);*/

  PosPPhase[1].timing.Tv = (distance / maxVel - (curVel / maxVel + 1.0F) * (PosPPhase[1].timing.Ta / 2.0F))
	- (targetVel / maxVel + 1.0F) * (PosPPhase[1].timing.Td / 2.0F);

  if (PosPPhase[1].timing.Tv <= 0.0F) {

	  /*Cannot acceed the max velocity*/
	  PosPPhase[1].timing.Tv = 0.0F;

	  /* Recalc timing*/
	  jerkp2 = jerk * jerk;

	  real32_T curVelp2 = curVel * curVel;
	  real32_T targetVelp2 = targetVel * targetVel;

	  velsum = curVel + targetVel;

	  accep2 = maxAcce * maxAcce;
	  accep4 = accep2 * accep2;

	  delta = (distance * 4.0F - 2.0F * maxAcce / jerk * velsum) *
			  maxAcce + ((curVelp2 + targetVelp2) * 2.0F + accep4 / jerkp2);

	  PosPPhase[1].timing.Tj1 = maxAcce / jerk;
	  PosPPhase[1].timing.Td = sqrtf(delta);
	  Ta_tmp /= jerk;
	  PosPPhase[1].timing.Ta = ((Ta_tmp - 2.0F * curVel) + PosPPhase[1].timing.Td) / 2.0F / maxAcce;
	  PosPPhase[1].timing.Tj2 = maxAcce / jerk;
	  PosPPhase[1].timing.Td = ((Ta_tmp - 2.0F * targetVel) + PosPPhase[1].timing.Td) / 2.0F / maxAcce;

	  real32_T amaxp2 = amax * amax;
	  while ((PosPPhase[1].timing.Ta < 2.0F * PosPPhase[1].timing.Tj1) || (PosPPhase[1].timing.Td < 2.0F * PosPPhase[1].timing.Tj2)) {
		  amax -= maxAcce * 0.1F;
		  amaxp2 = amax * amax;
		  if (amax > 0.0F) {
			  delta = ((curVelp2 + targetVelp2) * 2.0F + amaxp2*amaxp2 / jerkp2) +
					   (distance * 4.0F - 2.0F * amax / jerk * velsum) * amax;
		  } else {
			  delta = ((curVelp2 + targetVelp2) * 2.0F + amaxp2*amaxp2 / jerkp2) -
					   (distance * 4.0F - 2.0F * amax / jerk * velsum) * amax;
		  }

		  real32_T sqrtdelta = sqrtf(delta);
		  PosPPhase[1].timing.Tj1 = amax / jerk;
		  PosPPhase[1].timing.Ta = ((amaxp2 / jerk - 2.0F * curVel) + sqrtdelta) / 2.0F / amax;
		  PosPPhase[1].timing.Tj2 = amax / jerk;
		  PosPPhase[1].timing.Td = ((amaxp2 / jerk - 2.0F * targetVel) + sqrtdelta) / 2.0F / amax;
	  }

	  if ((PosPPhase[1].timing.Ta < 0.0F) || (PosPPhase[1].timing.Td < 0.0F)) {

		  velminus = targetVel - curVel;

		  real32_T decelgap = 0.0;

		  if (curVel > targetVel) {
			  /*仅包含减速段*/
			  PosPPhase[1].timing.Ta = 0.0F;
			  PosPPhase[1].timing.Tj1 = 0.0F;
			  PosPPhase[1].timing.Td = distance * 2.0F / velsum;

			  decelgap = jerk*(jerk*distance*distance + velsum*velsum*velminus);
			  PosPPhase[1].timing.Tj2 = (distance * jerk - sqrtf(decelgap)) / jerk / velsum;
		  } else {
			  PosPPhase[1].timing.Td = 0.0F;
			  PosPPhase[1].timing.Tj2 = 0.0F;
			  PosPPhase[1].timing.Ta = distance * 2.0F / velsum;

			  decelgap = jerk*(jerk*distance*distance - velsum*velsum*velminus);
			  PosPPhase[1].timing.Tj1 = (distance * jerk - sqrtf(decelgap)) / jerk / velsum;
		  }
	  }
  }

  /* All time phase calculate end
   * T = Tv + Ta + Td
   * Ta >= 2*Tj1
   * Td >= 2*Tj2
   * */
  PosPPhase[1].timing.T = PosPPhase[1].timing.Tv + PosPPhase[1].timing.Ta + PosPPhase[1].timing.Td;
  PosPPhase[1].tms = (PosPPhase[1].timing.T * 1000.0F);
  PosPPhase[1].dist = distance;
}

void PosProfile_CalcPhase0Timing(real32_T curVel, real32_T jerk)
{
	/*assert curVel < 0*/
	PosPPhase[0].timing.Tj1 = sqrtf( curVel / jerk );
	PosPPhase[0].timing.Ta  = 2.0f * PosPPhase[0].timing.Tj1;
	PosPPhase[0].timing.T   = PosPPhase[0].timing.Ta;

	PosPPhase[0].dist       = 1.5f * curVel * PosPPhase[0].timing.Tj1 -
			                  0.5f * jerk * PosPPhase[0].timing.Tj1 * PosPPhase[0].timing.Tj1 * PosPPhase[0].timing.Tj1;
	PosPPhase[0].tms        = PosPPhase[0].timing.T * 1000;
}

void PosProfile_Prepare(real32_T distance, real32_T curVel, real32_T targetVel, real32_T maxVel, real32_T maxAcce, real32_T maxJerk)
{
	/* calc two profile phase
	 * set the direction of profile*/
	if ((curVel < 0 && targetVel > 0) || (curVel > 0 && targetVel < 0)){
		/*not support*/
		return;
	}

	/* -----------------------------------------------------------------*/
	/*           - distance                               + distance    */
	/* reverse target pos ------ origin pos ------- positive target pos */
	/*             dir -1                                 dir +1        */
	/*          velocity < 0                            velocity > 0    */

	int8_T hasTurn = 0;
	if (distance > 0 && curVel < 0){
		hasTurn = 1;
		PosPPhase[0].dir = -1.0f;
		PosPPhase[1].dir = 1.0f;
	}

	if (distance < 0 && curVel > 0){
		hasTurn = 1;
		PosPPhase[0].dir = 1.0f;
		PosPPhase[1].dir = -1.0f;
	}

	if (!hasTurn){
		real32_T velerr = velerr = (targetVel > curVel)? (curVel - targetVel):(targetVel - curVel);

		/*jmax * (jmax*abs(q1 - q0)^2 + (v1 + v0)^2*(v1 - v0));*/
		/*means the distance too short to plan caused of cur velocity*/
		real32_T cond = ( maxJerk * distance * distance + (curVel+targetVel) * (curVel+targetVel) * velerr);
		if (cond < 0){
			hasTurn = 1;
			if (curVel < 0){
				PosPPhase[0].dir = -1.0f;
				PosPPhase[1].dir = 1.0f;
			}else{
				PosPPhase[0].dir = 1.0f;
				PosPPhase[1].dir = -1.0f;
			}
		}
	}

	if (hasTurn){
		PosProfile_CalcPhase0Timing(abs(curVel), maxJerk);

		/*the next phase calc slice timing, and all parameters are positive*/
		distance = fabsf(distance) + PosPPhase[0].dist;
		PosProfile_CalcPhase1Timing(distance,
									0.0f,
									fabsf(targetVel),
									maxVel, maxAcce, maxJerk);
	}else{
		if (distance > 0)
			PosPPhase[1].dir = (curVel >= 0.0f) ? 1.0f : -1.0f;
		else
			PosPPhase[1].dir = (curVel <= 0.0f) ? -1.0f : 1.0f;

		PosProfile_CalcPhase1Timing(fabsf(distance),
									fabsf(curVel),
									fabsf(targetVel),
									maxVel, maxAcce, maxJerk);
	}
}

/* Model initialize function */
void PosProfile_Initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Pos Integrator' */
  rtDW.PosIntegrator_IC_LOADING = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Vel Integrator' */
  rtDW.VelIntegrator_IC_LOADING = 1U;

  /* Initialize the ts counter*/
  rtDW.t_not_empty = false;
}

/*
 * Build pos profile,
 * distance:  distance from current position, UNIT is one step
 * curVel:    current motor speed
 * targetVel: target motor speed
 * maxAcce:   max acceleration
 * maxJerk:   max jerk
 * ts:        profile per step time stamp
 * */
void PosProfile_Start(real32_T distance, real32_T curVel, real32_T targetVel, real32_T maxVel, real32_T maxAcce, real32_T maxJerk, real32_T ts)
{
	rtU.InitPos   = 0.0F;
	rtU.TargetPos = distance;

	rtU.InitVel   = curVel;
	rtU.TargetVel = targetVel;

	rtU.MaxVel  = maxVel;
	rtU.MaxAcce = maxAcce;
	rtU.MaxJerk = maxJerk;

	/*Time space per profile discrete step*/
	rtU.Ts = ts;

	/* InitializeConditions for DiscreteIntegrator: '<S1>/Pos Integrator' */
	rtDW.PosIntegrator_IC_LOADING = 1U;
    rtDW.PosIntegrator_DSTATE = rtU.InitPos;

	/* InitializeConditions for DiscreteIntegrator: '<S1>/Vel Integrator' */
	rtDW.VelIntegrator_IC_LOADING = 1U;
    rtDW.VelIntegrator_DSTATE = rtU.InitVel;

    /* InitializeConditions for acceleration*/
    rtDW.AcceIntegrator_DSTATE = 0.0F;

    /* Reset the ts counter*/
    rtDW.t_not_empty = false;

    /* Velocity tranc error*/
    rtDW.VelTrancErr = 0.0F;

    memset(&PosPPhase[0], 0 ,sizeof(PosPPhase_T));
    memset(&PosPPhase[1], 0 ,sizeof(PosPPhase_T));

    PosProfile_Prepare(distance, curVel, targetVel, maxVel, maxAcce, maxJerk);
}

void PosProfile_StartHalt(real32_T curVel, real32_T maxJerk, real32_T ts)
{
	rtU.InitPos   = 0.0F;
	rtU.TargetPos = 0.0F;

	rtU.InitVel   = curVel;
	rtU.TargetVel = 0.0F;

	rtU.MaxVel  = curVel;
	rtU.MaxAcce = maxJerk;
	rtU.MaxJerk = maxJerk;

	/*Time space per profile discrete step*/
	rtU.Ts = ts;

	/* InitializeConditions for DiscreteIntegrator: '<S1>/Pos Integrator' */
	rtDW.PosIntegrator_IC_LOADING = 1U;
    rtDW.PosIntegrator_DSTATE = rtU.InitPos;

	/* InitializeConditions for DiscreteIntegrator: '<S1>/Vel Integrator' */
	rtDW.VelIntegrator_IC_LOADING = 1U;
    rtDW.VelIntegrator_DSTATE = rtU.InitVel;

    /* InitializeConditions for acceleration*/
    rtDW.AcceIntegrator_DSTATE = 0.0F;

    /* Reset the ts counter*/
    rtDW.t_not_empty = false;

    /* Velocity tranc error*/
    rtDW.VelTrancErr = 0.0F;

    memset(&PosPPhase[0], 0 ,sizeof(PosPPhase_T));
    memset(&PosPPhase[1], 0 ,sizeof(PosPPhase_T));

    PosPPhase[0].dir = (curVel < 0.0F) ? -1.0F : 1.0F;
    PosProfile_CalcPhase0Timing(abs(curVel), maxJerk);

    rtU.TargetPos = PosPPhase[0].dir * PosPPhase[0].dist;
}

real32_T PosProfile_GetTargetPos()
{
	return PosPPhase[0].dist * PosPPhase[0].dir + PosPPhase[1].dist * PosPPhase[1].dir;
}

boolean_T PosProfile_StepOver(uint32_T ticks)
{
	if (ticks >= (PosPPhase[0].tms + PosPPhase[1].tms)){
		return true;
	}
	return false;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
