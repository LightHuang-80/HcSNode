/*
 * motor.c
 *
 *  Created on: 2023年3月24日
 *      Author: Administrator
 */

#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "motor.h"

#define STALL_VALUE             63 // [-64..63]
#define MICRO_STEPS             32

uint16_t g_WorkCurrent = 1600; // motor rms current

MOTOR_Model_t MOTOR_Model;
extern int32_t g_IncSteps;

void MOTOR_Init()
{
	MOTOR_Model.driver = NULL;
	MOTOR_Model.target   = g_IncSteps;
	MOTOR_Model.speed    = 0;
	MOTOR_Model.polarity = 1;
}

#if (USE_TMC5160 == 1)
void MOTOR_bindDriver(TMC5160Stepper* driver)
{
	MOTOR_Model.driver = driver;

	/* Driver En pin*/
	HAL_GPIO_WritePin(GPIOC, DRV_ENN_Pin, GPIO_PIN_RESET);

	  /* Start TMC external CLK, 12M*/
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	/* Simple delay*/
	osDelay(10);

	printf("Init Pos: %ld\n", g_IncSteps);

	driver->GSTAT(0);

	int32_t chopconf = driver->CHOPCONF();
	printf("TMC5160 chop conf: 0x%08x\n", (unsigned int)chopconf);

	/* Read vel actual*/
	int32_t va = driver->VACTUAL();
	printf("TMC5160 vactual: %ld\n", va);

	driver->defaults();
	driver->push();
    driver->begin();

    uint8_t conn = driver->test_connection();
    if (conn != 0){
	  printf("TMC5160 not connected, ret: %d\n", conn);
    }

    uint8_t version = driver->version();
    printf("TMC driver version: 0x%02x\n", version);

    driver->chm(0); // Standard mode (spreadCycle)

    /* CHOPPER_DEFAULT_24V, (4,5,0), tuned to: 4,5,2*/
    driver->toff(4); // 0: shutdown, 1: only with tbl>=2, DcStep: no less than 3
    driver->hend(5);
    driver->hstrt(0);

    driver->tbl(2); // blank_time(36)

    driver->pwm_freq(1);
    driver->TPOWERDOWN(10);
    //driver->pwm_autoscale(true);
    driver->TPWMTHRS(2000);

    driver->blank_time(24);

    /*Work current*/
    driver->rms_current(g_WorkCurrent); // mA
    driver->intpol(1);

    driver->microsteps(MICRO_STEPS);
    driver->TCOOLTHRS(400000); // 20bit max
    driver->THIGH(1200);
    driver->semin(5);
    driver->semax(2);
    driver->sedn(0b01);
    driver->sgt(STALL_VALUE);
    driver->irun(28);
    //driver->ihold(20);

    /* DcStep*/
    /* version 1.0, big noise
    driver->vhighfs(1);
    driver->vhighchm(1);
    driver->VDCMIN(800000);
    driver.dc_time(96);
    HAL_GPIO_WritePin(DCEN_GPIO_Port, DCEN_Pin, GPIO_PIN_SET);
   */

    bool enable = driver->isEnabled();
    if (enable){
	    printf("driver enable\n");
    }

    bool sdmode = driver->sd_mode();
    if (sdmode){
	    printf("driver use sdmode\n");
    }

    uint8_t state = driver->GSTAT();
    printf("driver state: %d\n", state);

    uint16_t ms = driver->microsteps();
    printf("motor micro steps: %d\n", ms);

	Stepper_Init(ms);
}
#else
void MOTOR_bindDriver(TMC2209Stepper* driver)
{
	MOTOR_Model.driver = driver;
	if (driver){
		Stepper_Init(driver->microsteps());
	}
}
#endif

void MOTOR_run()
{
	Stepper_Run();
}

void MOTOR_stop()
{
	MOTOR_Model.speed = 0;
	Stepper_Stop();
}

void MOTOR_loop(uint32_t ticks)
{
	/*Check the driver status*/
}

void MOTOR_setPolarity(int8_t polarity)
{
	MOTOR_Model.polarity = polarity;
}

int8_t MOTOR_getPolarity()
{
	return MOTOR_Model.polarity;
}

void MOTOR_setDir(int8_t dir)
{
	if (dir == MOTOR_DIR_FORWARD){
		Stepper_SetDir((MOTOR_Model.polarity == 1)?STEPPER_Dir_CW:STEPPER_Dir_CCW);
	}else if (dir == MOTOR_DIR_BACKWARD){
		Stepper_SetDir((MOTOR_Model.polarity == 1)?STEPPER_Dir_CCW:STEPPER_Dir_CW);
	}
}

int8_t MOTOR_getDir()
{
	int8_t dir = Stepper_GetDir();
	if ((MOTOR_Model.polarity == 1 && dir == STEPPER_Dir_CW) ||
	    (MOTOR_Model.polarity == -1 && dir == STEPPER_Dir_CCW))
		return MOTOR_DIR_FORWARD;

	return MOTOR_DIR_BACKWARD;
}

void MOTOR_setSpeed(int32_t speed)
{
	if (MOTOR_Model.polarity == 1){
		if (speed > 0)
			Stepper_SetDir(STEPPER_Dir_CW);
		else
			Stepper_SetDir(STEPPER_Dir_CCW);
	}else{
		if (speed > 0)
			Stepper_SetDir(STEPPER_Dir_CW);
		else
			Stepper_SetDir(STEPPER_Dir_CCW);
	}

	MOTOR_Model.speed = speed;
	Stepper_SetSpeed(abs(speed));
}

int32_t MOTOR_getSpeed()
{
	return MOTOR_Model.speed;
}

void MOTOR_setRelTarget(int32_t target)
{
	MOTOR_Model.target = g_IncSteps + MOTOR_Model.polarity * target;
}

void MOTOR_setAbsTarget(int32_t target)
{
	MOTOR_Model.target = MOTOR_Model.polarity * target;
}

int32_t MOTOR_getTarget()
{
	return MOTOR_Model.target;
}

int32_t MOTOR_getPosDiff()
{
	return /*MOTOR_Model.polarity * */ (MOTOR_Model.target - g_IncSteps);
}
