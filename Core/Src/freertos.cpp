/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
#include "can.h"
#include "as5048a.h"
#include "round.h"
#include "nstepper.h"
#include "TMCStepper.h"
#include "fsusart.h"
#include "message.h"
#include "nmotion.h"
#include "log.h"
#include "CANOpen.h"
#include "cannode.h"
#include "c402.h"
#include "cmd.h"
#include "PosProfile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern "C" void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
/* USER CODE END PD */

#define TARGET_JOINT 1

#if (TARGET_JOINT == 1)
  #define STALL_VALUE             63 // [-64..63]
  #define ENCODER_ReadAngle_Retry 15
  uint8_t  g_NodeId = 0x04;
  uint8_t  g_MicroSteps = 32;
  float    g_EncoderRatio = 32.0f;

  uint16_t g_WorkCurrent = 1600; // motor rms current
  GPIO_PinState g_EndStopTriggerState = GPIO_PIN_RESET; // Endstop trigger state
  uint32_t g_EndStopTriggerMode = GPIO_MODE_IT_FALLING;

  uint32_t g_MotionStepTimespace = 2;	// unit ms
#elif (TARGET_JOINT == 2)
  #define STALL_VALUE             63 // [-64..63]
  #define ENCODER_ReadAngle_Retry 15
  uint8_t  g_NodeId = 0x05;
  uint8_t  g_MicroSteps = 32;
  float    g_EncoderRatio = 32.0f;

  uint16_t g_WorkCurrent = 2200; // motor rms current
  GPIO_PinState g_EndStopTriggerState = GPIO_PIN_RESET; // Endstop trigger state
  uint32_t g_EndStopTriggerMode = GPIO_MODE_IT_RISING;
  uint32_t g_MotionStepTimespace = 4;

#elif (TARGET_JOINT == 3)
  #define STALL_VALUE             63 // [-64..63]
  #define ENCODER_ReadAngle_Retry 15
  uint8_t  g_NodeId = 0x0D;
  uint8_t  g_MicroSteps = 16;
  /* Joint3: 66.48(Encoder installed on input axis),
   * others same as microsteps, installed on motor */
  float    g_EncoderRatio = 16; //66.56f;

  uint16_t g_WorkCurrent = 1600; // motor rms current
  GPIO_PinState g_EndStopTriggerState = GPIO_PIN_RESET; // Endstop trigger state
  uint32_t g_EndStopTriggerMode = GPIO_MODE_IT_RISING;
  uint32_t g_MotionStepTimespace = 4;

#elif (TARGET_JOINT == 4)
  #define  STALL_VALUE     240 // [0..255]
  #define  DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
  #define  R_SENSE        0.11f // Match to your driver
  #define  ENCODER_ReadAngle_Retry 15

  uint8_t  g_NodeId = 0x34;
  uint16_t g_MicroSteps = 16;
  float    g_EncoderRatio = 16.0f;

  uint32_t      g_EndStopTriggerMode = GPIO_MODE_IT_FALLING;
  GPIO_PinState g_EndStopTriggerState = GPIO_PIN_RESET; // Endstop trigger state
  uint16_t      g_WorkCurrent = 1500;  // 1600ma
  uint32_t g_MotionStepTimespace = 4;
#endif

volatile int32_t  g_IncSteps;
volatile uint16_t g_AbsAngle;

QueueHandle_t g_MotionMsgQueue = NULL;
QueueHandle_t g_TargetPosQueue = NULL;
QueueHandle_t g_StatusMsgQueue = NULL;

xTimerHandle motorProcessTimer;

extern Node_DriveProfile_t g_NodeDriveProfile;
extern CAN_HandleTypeDef hcan1;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		  .name = "defaultTask",
		  .attr_bits = 0,
		  .cb_mem = NULL,
		  .cb_size = 0,
		  .stack_mem = NULL,
		  .stack_size = 128 * 4,
		  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
		  .name = "ledTask",
		  .attr_bits = 0,
		  .cb_mem = NULL,
		  .cb_size = 0,
		  .stack_mem = NULL,
		  .stack_size = 128 * 4,
		  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderTask */
osThreadId_t encoderTaskHandle;
const osThreadAttr_t encoderTask_attributes = {
		  .name = "encoderTask",
		  .attr_bits = 0,
		  .cb_mem = NULL,
		  .cb_size = 0,
		  .stack_mem = NULL,
		  .stack_size = 128 * 4,
		  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
		  .name = "motorTask",
		  .attr_bits = 0,
		  .cb_mem = NULL,
		  .cb_size = 0,
		  .stack_mem = NULL,
		  .stack_size = 1024 * 4,
		  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for canTask */
osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
		  .name = "canTask",
		  .attr_bits = 0,
		  .cb_mem = NULL,
		  .cb_size = 0,
		  .stack_mem = NULL,
		  .stack_size = 128 * 4,
		  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DefaultTaskMain(void *argument);
void LedTaskMain(void *argument);
void EncoderTaskMain(void *argument);
void MotorTaskMain(void *argument);
void CanCommTaskMain(void *argument);

void MotorProcessCB(TimerHandle_t xTimer);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* Initialize the USART*/
  FUART_Init();

  LOG_Init(LOG_InfoLevel, 1, LOG_USARTOut);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  motorProcessTimer = xTimerCreate("MotorProcessTimer",
		  	  	  	  	  	  	  pdMS_TO_TICKS( g_MotionStepTimespace ),
								  pdTRUE,
								  0,
								  MotorProcessCB);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  g_MotionMsgQueue = xQueueCreate(100, sizeof(MotionMsgItem_t));
  g_TargetPosQueue = xQueueCreate(100, sizeof(MotionMsgItem_t));
  g_StatusMsgQueue  = xQueueCreate(50,  sizeof(StatusMsgItem_t));

  // Motion 初始化
  MT_Init(g_MotionMsgQueue, g_TargetPosQueue, g_StatusMsgQueue);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(DefaultTaskMain, NULL, &defaultTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LedTaskMain, NULL, &ledTask_attributes);

  /* creation of encoderTask */
  encoderTaskHandle = osThreadNew(EncoderTaskMain, NULL, &encoderTask_attributes);

  /* creation of uartTask */
  motorTaskHandle = osThreadNew(MotorTaskMain, NULL, &motorTask_attributes);

  /* creation of canTask */
  canTaskHandle = osThreadNew(CanCommTaskMain, NULL, &canTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_DefaultTaskMain */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DefaultTaskMain */
void DefaultTaskMain(void *argument)
{
  /* USER CODE BEGIN DefaultTaskMain */
  /* Infinite loop */
  for(;;)
  {
	//MT_process_v3(g_IncSteps);
	osDelay(4);
  }
  /* USER CODE END DefaultTaskMain */
}

/* USER CODE BEGIN Header_LedTaskMain */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTaskMain */
void LedTaskMain(void *argument)
{
  /* USER CODE BEGIN LedTaskMain */
  /* Infinite loop */
  for(;;)
  {
    osDelay(16);
  }
  /* USER CODE END LedTaskMain */
}

/* USER CODE BEGIN Header_EncoderTaskMain */
/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderTaskMain */
void EncoderTaskMain(void *argument)
{
  /* USER CODE BEGIN EncoderTaskMain */
  /* Infinite loop */
  AS5048A_Init();

  uint16_t retry = 0;
  uint16_t result = HAL_OK;

  //AS5048A_ZeroPoint();

  do {
    result = AS5048A_ReadAngle((uint16_t*)&g_AbsAngle);
    if (result == HAL_OK)
      break;

    osDelay(1);
    retry ++;
  }while(retry < ENCODER_ReadAngle_Retry);

  if (result != HAL_OK){
	return;
  }

  printf("Initialized angle: %d\n", g_AbsAngle);

  RND_Init(g_AbsAngle);

  uint16_t angle = g_AbsAngle;

  /* Infinite loop */
  for(;;) {
	/*Feed the current angle*/
	RND_Input(g_AbsAngle);

	/*Update the increment steps*/
	// 40.24(64teeths)/9.68(16teeths) * 16(g_MicroSteps) = 66.51
	g_IncSteps = RND_GetIncSteps(g_AbsAngle, g_EncoderRatio); //

	/*loop read, and update global absolute angle*/
	uint16_t status = AS5048A_ReadAngle((uint16_t*)&angle);
	if (status == HAL_OK && angle != 0){
		g_AbsAngle = angle;
	}

	//LOG_Print(LOG_InfoLevel, "curinc: %d, angle: %d\n", g_IncSteps, g_AbsAngle);
	osDelay(2);
  }
  /* USER CODE END EncoderTaskMain */
}

void MotorProcessCB(TimerHandle_t xTimer)
{
	MT_process_v3(g_MotionStepTimespace);
}
/* USER CODE BEGIN Header_UITaskMain */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UITaskMain */
#if (USE_TMC5160 == 1)
void MotorTaskMain(void *argument)
{
  FUART_beginReceive(&huart1);

  /* USER CODE BEGIN UITaskMain */
  Stepper_Init(g_MicroSteps);

  /* Start TMC external CLK, 12M*/
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  /* Driver En pin*/
  HAL_GPIO_WritePin(GPIOC, DRV_ENN_Pin, GPIO_PIN_RESET);

  /* Simple delay*/
  osDelay(10);

  TMC5160Stepper driver(0, 0.075, 0, 0, 0, -1);

  driver.GSTAT(0);

  int32_t chopconf = driver.CHOPCONF();
  printf("TMC5160 chop conf: 0x%08x\n", (unsigned int)chopconf);

  /* Read vel actual*/
  int32_t va = driver.VACTUAL();
  printf("TMC5160 vactual: %ld\n", va);

  driver.defaults();
  driver.push();
  driver.begin();

  uint8_t conn = driver.test_connection();
  if (conn != 0){
    printf("TMC5160 not connected, ret: %d\n", conn);
  }

  uint8_t version = driver.version();
  printf("TMC driver version: 0x%02x\n", version);

  driver.chm(0); // Standard mode (spreadCycle)

  /* CHOPPER_DEFAULT_24V, (4,5,0), tuned to: 4,5,2*/
  driver.toff(4); // 0: shutdown, 1: only with tbl>=2, DcStep: no less than 3
  driver.hend(6);
  driver.hstrt(2);

  driver.tbl(2); // blank_time(36)

  driver.pwm_freq(1);
  driver.TPOWERDOWN(10);
  //driver.pwm_autoscale(true);
  driver.TPWMTHRS(2000);

  driver.blank_time(24);

  /*Work current*/
  driver.rms_current(g_WorkCurrent); // mA
  driver.intpol(1);

  driver.microsteps(g_MicroSteps);
  driver.TCOOLTHRS(400000); // 20bit max
  driver.THIGH(1200);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  /* DcStep*/
  /* version 1.0, big noise
  driver.vhighfs(1);
  driver.vhighchm(1);
  driver.VDCMIN(800000);
  driver.dc_time(96);
  HAL_GPIO_WritePin(DCEN_GPIO_Port, DCEN_Pin, GPIO_PIN_SET);
*/

  bool enable = driver.isEnabled();
  if (enable){
	  printf("driver enable\n");
  }

  bool sdmode = driver.sd_mode();
  if (sdmode){
	  printf("driver use sdmode\n");
  }

  uint8_t state = driver.GSTAT();
  printf("driver state: %d\n", state);

  uint16_t ms = driver.microsteps();
  printf("motor micro steps: %d\n", ms);

  /* Register TMC driver*/
  JNT_registerDriver(&driver);

  MotionMsgItem_t item;
  item.position = 24800;
  xQueueSend(g_MotionMsgQueue, &item, 0);

  printf("Init Pos: %ld\n", g_IncSteps);

  xTimerStart(motorProcessTimer, 0);

  /* Infinite loop */
  for(;;)
  {
	//JNT_processMsg();
	  //uint32_t ticks = 5;
	  //MT_process_v3(ticks);
	  osDelay(4);
  }
  /* USER CODE END UITaskMain */
}
#elif (USE_TMC2209 == 1)
void MotorTaskMain(void *argument)
{
  /* USER CODE BEGIN UartTaskMain */
  /*TMC220X driver*/

  SoftwareSerial sserial(GPIOB, GPIO_PIN_7, GPIOB, GPIO_PIN_6);
  sserial.begin(115200);

  TMC2209Stepper driver(&sserial, R_SENSE, DRIVER_ADDRESS);
#if (USE_TMC2208 == 1)
  TMC2208Stepper driver(&sserial, R_SENSE);
#endif

  driver.begin();

  uint8_t conn = driver.test_connection();
  unsigned int otp = driver.OTP_READ();
  printf("TMC driver test connection: %d, otp: 0x%08x\n", conn, otp);

  driver.toff(4);          // default 4
  driver.blank_time(24);
  driver.rms_current(g_WorkCurrent); // mA
  driver.microsteps(g_MicroSteps);

  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);

  driver.ihold(28);
  driver.TPWMTHRS(8000);

  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);

  /*Initialize finished*/
  sserial.end();

  JNT_registerDriver(&driver);

  /*Debug output mode*/
  /* Infinite loop */

  for(;;) {
	//JNT_processMsg();
	osDelay(10);
  }
  /* USER CODE END UartTaskMain */
}
#endif
/* USER CODE BEGIN Header_CanCommTaskMain */
/**
* @brief Function implementing the canTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanCommTaskMain */
void CanCommTaskMain(void *argument)
{
  /* USER CODE BEGIN CanCommTaskMain */
  uint8_t ret;

  if (NODE_new() == CO_ERROR_NO){
    ret = NODE_Init((void*)&hcan1, g_NodeId);
  }else{
    printf("Create CAN node failed.\n");
	return;
  }

  if (ret != CO_ERROR_NO){
    printf("Node initialize failed. [%d]\n", ret);
	return;
  }

  bool_t   syncWas;
  uint32_t timerNext;

  uint32_t timeInterval = CAN_LOOP_DUARTION * TMR_TASK_INTERVAL;

  /* Infinite loop */
  for(;;) {
	if(CO->CANmodule[0]->CANnormal){

	  CO_process(CO, timeInterval, &timerNext);

	  syncWas = CO_process_SYNC(CO, timeInterval, &timerNext);

	  NODE_process(timeInterval, syncWas);
    }
	osDelay(CAN_LOOP_DUARTION);
  }
  /* USER CODE END CanCommTaskMain */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
