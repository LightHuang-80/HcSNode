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

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

PosPTiming_T PosPTiming;

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
  real32_T uv = truncf(rtDW.VelIntegrator_DSTATE);
  rtDW.VelTrancErr += rtDW.VelIntegrator_DSTATE - uv;

  /* Floating value tranc to UINT32*/
  if (rtDW.VelTrancErr >= 1.0f){
	  rtDW.VelIntegrator_DSTATE += 1.0F;
	  rtDW.VelTrancErr -= 1.0F;
  }

  if (!rtDW.t_not_empty) {
      rtDW.t = 0.0F;
      rtDW.t_not_empty = true;
  } else {
      rtDW.t += rtU.Ts;
  }

  /* MATLAB Function: '<S1>/ppmode' incorporates:
   *  Inport: '<Root>/MaxJerk'
   */
  if ((rtDW.t >= 0.0F) && (rtDW.t < PosPTiming.Tj1)) {
	  /*Phase 1*/
	  jerk = rtU.MaxJerk;
  } else if ((rtDW.t >= PosPTiming.Tj1) && (rtDW.t < (PosPTiming.Ta - PosPTiming.Tj1))) {
	  /*Phase 2*/
	  jerk = 0.0F;
  } else if ((rtDW.t >= (PosPTiming.Ta - PosPTiming.Tj1)) && (rtDW.t < PosPTiming.Ta)) {
	  /*Phase 3*/
	  jerk = -rtU.MaxJerk;
  } else if ((rtDW.t >= PosPTiming.Ta) && (rtDW.t < PosPTiming.Ta + PosPTiming.Tv)) {
	  /*Phase 4*/
	  jerk = 0.0F;
  } else if ((rtDW.t >= (PosPTiming.T - PosPTiming.Td)) && (rtDW.t < (PosPTiming.T - PosPTiming.Td + PosPTiming.Tj2))){
	  /*Phase 4*/
	  jerk = -rtU.MaxJerk;
  } else if ((rtDW.t >= (PosPTiming.T - PosPTiming.Td + PosPTiming.Tj2)) && (rtDW.t < (PosPTiming.T - PosPTiming.Tj2))) {
	  /*Phase 4*/
	  jerk = 0.0F;
  } else if (rtDW.t >= (PosPTiming.T - PosPTiming.Tj2) && rtDW.t < PosPTiming.T) {
	  /*Phase 4*/
	  jerk = rtU.MaxJerk;
  } else {
	  /*Phase 4*/
	  jerk = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S1>/Acce Integrator' */
  rtDW.AcceIntegrator_DSTATE += rtU.Ts * jerk;
}

void PosProfile_CalcTimePhase()
{
  real32_T Ta_tmp;
  real32_T Tv_tmp_tmp;
  real32_T amax;
  real32_T delta;
  real32_T delta_tmp;
  real32_T delta_tmp_0;
  real32_T delta_tmp_1;
  real32_T delta_tmp_tmp;

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
  amax = rtU.MaxAcce;

  real32_T deltaV0 = rtU.MaxVel - rtU.InitVel;
  Ta_tmp = rtU.MaxAcce * rtU.MaxAcce;

  if (deltaV0 * rtU.MaxJerk < Ta_tmp) {
	if (rtU.InitVel > rtU.MaxVel) {
	  PosPTiming.Tj1 = 0.0F;
	  PosPTiming.Ta = 0.0F;
	} else {
	  PosPTiming.Tj1 = sqrtf(deltaV0 / rtU.MaxJerk);
	  PosPTiming.Ta = 2.0F * PosPTiming.Tj1;
	}
  } else {
	  PosPTiming.Tj1 = rtU.MaxAcce / rtU.MaxJerk;
	  PosPTiming.Ta = deltaV0 / rtU.MaxAcce + PosPTiming.Tj1;
  }

  real32_T deltaV1 = rtU.MaxVel - rtU.TargetVel;
  if (deltaV1 * rtU.MaxJerk < Ta_tmp) {
	  PosPTiming.Tj2 = sqrtf(deltaV1 / rtU.MaxJerk);
	  PosPTiming.Td = 2.0F * PosPTiming.Tj1;
  } else {
	  PosPTiming.Tj2 = rtU.MaxAcce / rtU.MaxJerk;
	  PosPTiming.Td = deltaV1 / rtU.MaxAcce + PosPTiming.Tj2;
  }

  /* T = Tv + Ta + Td */
  /* Make sure the targetPos great than initPos
   * assert(rtU.TargetPos > rtU.InitPos);*/
  Tv_tmp_tmp = rtU.TargetPos - rtU.InitPos;

  PosPTiming.Tv = (Tv_tmp_tmp / rtU.MaxVel - (rtU.InitVel / rtU.MaxVel + 1.0F) * (PosPTiming.Ta / 2.0F))
	- (rtU.TargetVel / rtU.MaxVel + 1.0F) * (PosPTiming.Td / 2.0F);

  if (PosPTiming.Tv <= 0.0F) {
	  /*Cannot acceed the max velocity*/
	  PosPTiming.Tv = 0.0F;
	  delta_tmp = rtU.MaxJerk * rtU.MaxJerk;
	  delta_tmp_0 = (rtU.InitVel * rtU.InitVel + rtU.TargetVel * rtU.TargetVel) *
	  2.0F;
	  delta_tmp_tmp = rtU.InitVel + rtU.TargetVel;
	  delta_tmp_1 = Tv_tmp_tmp * 4.0F;
	  delta = (delta_tmp_1 - 2.0F * rtU.MaxAcce / rtU.MaxJerk * delta_tmp_tmp) *
			  rtU.MaxAcce + (delta_tmp_0 + rt_powf_snf(rtU.MaxAcce, 4.0F) / delta_tmp);
	  PosPTiming.Tj1 = rtU.MaxAcce / rtU.MaxJerk;
	  PosPTiming.Td = sqrtf(delta);
	  Ta_tmp /= rtU.MaxJerk;
	  PosPTiming.Ta = ((Ta_tmp - 2.0F * rtU.InitVel) + PosPTiming.Td) / 2.0F / rtU.MaxAcce;
	  PosPTiming.Tj2 = rtU.MaxAcce / rtU.MaxJerk;
	  PosPTiming.Td = ((Ta_tmp - 2.0F * rtU.TargetVel) + PosPTiming.Td) / 2.0F / rtU.MaxAcce;

	  while ((PosPTiming.Ta < 2.0F * PosPTiming.Tj1) || (PosPTiming.Td < 2.0F * PosPTiming.Tj2)) {
		  amax -= rtU.MaxAcce * 0.1F;
		  if (amax > 0.0F) {
			  delta = ((rtU.TargetPos - rtU.InitPos) * 4.0F - 2.0F * amax /
					  rtU.MaxJerk * (rtU.InitVel + rtU.TargetVel)) * amax +
						 ((rtU.InitVel * rtU.InitVel + rtU.TargetVel * rtU.TargetVel) * 2.0F +
								 rt_powf_snf(amax, 4.0F) / (rtU.MaxJerk * rtU.MaxJerk));
		  } else {
			  delta = (delta_tmp_0 + rt_powf_snf(amax, 4.0F) / delta_tmp) -
					  (delta_tmp_1 - 2.0F * amax / rtU.MaxJerk * delta_tmp_tmp) * amax;
	  }

	  PosPTiming.Tj1 = amax / rtU.MaxJerk;
	  PosPTiming.Td = sqrtf(delta);
	  PosPTiming.Ta = ((amax * amax / rtU.MaxJerk - 2.0F * rtU.InitVel) + PosPTiming.Td) / 2.0F / amax;
	  PosPTiming.Tj2 = amax / rtU.MaxJerk;
	  PosPTiming.Td = ((amax * amax / rtU.MaxJerk - 2.0F * rtU.TargetVel) + PosPTiming.Td) / 2.0F /
		amax;
	}

	real32_T deltaV = fabsf(rtU.TargetVel - rtU.InitVel);
	if ((PosPTiming.Ta < 0.0F) || (PosPTiming.Td < 0.0F)) {

	  if (rtU.InitVel > rtU.TargetVel) {
		  /*仅包含减速段*/
		  PosPTiming.Ta = 0.0F;
		  PosPTiming.Tj1 = 0.0F;
		  PosPTiming.Td = Tv_tmp_tmp * 2.0F / delta_tmp_tmp;
		  PosPTiming.Tj2 = (Tv_tmp_tmp * rtU.MaxJerk - sqrtf((Tv_tmp_tmp * Tv_tmp_tmp *
				 rtU.MaxJerk + delta_tmp_tmp * delta_tmp_tmp * deltaV) * rtU.MaxJerk)) / rtU.MaxJerk / delta_tmp_tmp;
	  } else {
		  PosPTiming.Td = 0.0F;
		  PosPTiming.Tj2 = 0.0F;
		  PosPTiming.Ta = Tv_tmp_tmp * 2.0F / delta_tmp_tmp;
		  PosPTiming.Tj1 = (Tv_tmp_tmp * rtU.MaxJerk - sqrtf((Tv_tmp_tmp * Tv_tmp_tmp *
				 rtU.MaxJerk - delta_tmp_tmp * delta_tmp_tmp * deltaV) * rtU.MaxJerk)) / rtU.MaxJerk / delta_tmp_tmp;
	  }
	}

	amax = rtU.MaxAcce;
	while (PosPTiming.Td < 2.0F * PosPTiming.Tj2) {
	  amax -= rtU.MaxAcce * 0.1F;
	  PosPTiming.Tj2 = amax / rtU.MaxJerk;
	  PosPTiming.Td = ((amax * amax / rtU.MaxJerk - 2.0F * rtU.TargetVel) + sqrtf(delta)) /
		2.0F / amax;
	}

	if (PosPTiming.Td < 0.0F) {
		/*仅包含加速段*/
		PosPTiming.Td = 0.0F;
		PosPTiming.Tj2 = 0.0F;
		PosPTiming.Ta = (rtU.TargetPos - rtU.InitPos) * 2.0F / (rtU.TargetVel + rtU.InitVel);
		PosPTiming.Tj1 = ((rtU.TargetPos - rtU.InitPos) * rtU.MaxJerk - (Tv_tmp_tmp *
			  Tv_tmp_tmp * rtU.MaxJerk - delta_tmp_tmp * delta_tmp_tmp * sqrtf(deltaV)) * rtU.MaxJerk) / rtU.MaxJerk /
		delta_tmp_tmp;
	}
  }

  /* All time phase calculate end
   * T = Tv + Ta + Td
   * Ta >= 2*Tj1
   * Td >= 2*Tj2
   * */
  PosPTiming.T = PosPTiming.Tv + PosPTiming.Ta + PosPTiming.Td;
  PosPTiming.ticks = PosPTiming.T * 1000;
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
void PosProfile_Start(uint32_T distance, int32_T curVel, int32_T targetVel, uint32_T maxVel, uint32_T maxAcce, uint32_T maxJerk, real32_T ts)
{
	rtU.InitPos = 0.0F;
	rtU.TargetPos = distance;

	rtU.InitVel = curVel;
	rtU.TargetVel = targetVel;

	rtU.MaxVel  = maxVel;
	rtU.MaxAcce = maxAcce;
	rtU.MaxJerk = maxJerk;

	/*Time space per profile discrete step*/
	rtU.Ts = ts;

	/* InitializeConditions for DiscreteIntegrator: '<S1>/Pos Integrator' */
	rtDW.PosIntegrator_IC_LOADING = 0U;
    rtDW.PosIntegrator_DSTATE = rtU.InitPos;

	/* InitializeConditions for DiscreteIntegrator: '<S1>/Vel Integrator' */
	rtDW.VelIntegrator_IC_LOADING = 0U;
    rtDW.VelIntegrator_DSTATE = rtU.InitVel;

    /* Reset the ts counter*/
    rtDW.t_not_empty = false;

    /* Velocity tranc error*/
    rtDW.VelTrancErr = 0.0F;

	PosProfile_CalcTimePhase();
}

boolean_T PosProfile_StepOver(uint32_T ticks)
{
	if (ticks >= PosPTiming.ticks){
		return true;
	}
	return false;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
