#include "mode.h"

#include "usart.h"
#include "tim.h"

#include "FreeRTOS.h" 
#include "event_groups.h"
#include "cmsis_os2.h"


#define PARAMATER 15u
#define PARAMATERNUMBER 30u
#define MOTOR_BRAKE_TIME 500u  /* 500ms  */

#define MOTOR_340_STEP    40950u  /* Angle 360 degree, 360 * 40950 / 340 */
#define MOTOR_360_STEP    43359u  /* Angle 360 degree, 360 * 40950 / 340 */
#define MOTOR_BOUNDARY_VALUE_STEP     100u

typedef enum
{
  MODE_1 = 0x00,
  MODE_2,
  MODE_3,
  MODE_4,
  MODE_5,
  MODE_6,
  MODE_7,
  MODE_8,
  MODE_MAX_NUM,
  MODE_INVALID = 0xFF
}MODE_COMMAND_E;

typedef enum
{
  MODE_CONTROL_MODE_NONE,
  MODE_CONTROL_MODE_LIN_COMMAND,
  MODE_CONTROL_MODE_LIN_POS,
  MODE_CONTROL_MODE_KEY_COMMAND,
  MODE_CONTROL_MODE_KEY_RUN,
  MODE_CONTROL_MODE_MAX_NUM
}MODE_CONTROL_MODE_E;

typedef enum
{
  MODE_MOTOR_RUN_STEP_IDLE,
  MODE_MOTOR_RUN_STEP_FORWARD,
  MODE_MOTOR_RUN_STEP_REVERSE,
  MODE_MOTOR_RUN_STEP_BRAKE,
  MODE_MOTOR_RUN_STEP_STOP
}MODE_MOTOR_RUN_STEP_E;

typedef struct
{
  uint16_t modeMotorOutHigh;
  uint16_t modeMotorOutLow;
}MODE_MOTOR_OUT_S;


typedef enum
{
  MODE_MODE_CHANGE_STEP_IDLE,
  MODE_MODE_CHANGE_STEP_TO_FIRST_POSITION,
  MODE_MODE_CHANGE_STEP_ARIVE_FIRST_POSITION,
  MODE_MODE_CHANGE_STEP_TO_SECOND_POSITION,
  MODE_MODE_CHANGE_STEP_ARIVE_SECOND_POSITION
}MODE_MODE_CHANGE_STEP_S;

typedef enum
{
  MODE_MOTOR_STATUS_IDLE,
  MODE_MOTOR_STATUS_START,
  MODE_MOTOR_STATUS_RUNNING,
  MODE_MOTOR_STATUS_ERROR,
  MODE_MOTOR_STATUS_STOP
}MODE_MOTOR_STATUS_S;

typedef struct
{
  uint16_t firstValvePosition;
  uint16_t secondValvePosition;
}MODE_POSITION_S;

extern EventGroupHandle_t ModeEventHandle;

osStatus_t osStatus;

extern osMessageQueueId_t SENT_CurrentPositionHandle;
extern osMessageQueueId_t LIN_MasterTargetPositionHandle;
extern osMessageQueueId_t LIN_MasterModeCommandHandle;
extern osMessageQueueId_t MODE_MotorOutHandle;

extern uint16_t positionValue;
extern PWM_MOTOR_OUT_S pwmMotorOut;
extern LIN_MASTER_MESSAGE_U linMasterMessage;
extern LIN_YTSENT_MESSAGE_U linYtSentMessage;

static volatile MODE_COMMAND_E modeCommand;
//MODE_COMMAND_E modeCommand;
static MODE_COMMAND_E currentMode;

static uint8_t runCycle;
static uint32_t modeCurrentPos;
static uint32_t modeTargetPos;

  static MODE_CONTROL_MODE_E controlMode;

static MODE_MOTOR_OUT_S modeMotorOut;
static MODE_MOTOR_RUN_STEP_E modeMotorRunStep;

static MODE_MOTOR_STATUS_S modeMotorStatus;

static MODE_POSITION_S modePosition[MODE_MAX_NUM] = {\
  {12044, 2409},  /* First step is 100 Second step is 20 */
  {36132, 3613},  /* First step is 300 Second step is 30 */
  {24088, 6022},  /* First step is 200 Second step is 50 */
  {13249, 9635},  /* First step is 110 Second step is 80 */
  {38541, 30110},  /* First step is 320 Second step is 250 */
  {38541, 6022},  /* First step is 320 Second step is 50 */
  {32519, 21679},  /* First step is 270 Second step is 180 */
  {24088, 9635}};  /* First step is 200 Second step is 80 */


static void MODE_MotorAction(void);


void MODE_Init(void)
{
  modeCommand = MODE_INVALID;
  controlMode = MODE_CONTROL_MODE_NONE;
  runCycle = 0;
  modeCurrentPos = 0;
  modeTargetPos = 0;
  modeMotorOut.modeMotorOutHigh = 0;
  modeMotorOut.modeMotorOutLow = 0;
}


static void MODE_ModeChange(void)
{
  static MODE_MODE_CHANGE_STEP_S modeRunStep = MODE_MODE_CHANGE_STEP_IDLE;
  static uint16_t firstPos;
  static uint16_t secondtPos;

  switch(modeRunStep)
  {
    case MODE_MODE_CHANGE_STEP_IDLE:
    {
      if(modeCommand < MODE_MAX_NUM)
      {
        firstPos = modePosition[modeCommand].firstValvePosition;
        secondtPos = modePosition[modeCommand].secondValvePosition;
        
        modeRunStep = MODE_MODE_CHANGE_STEP_TO_FIRST_POSITION;
        runCycle = 0;
      } 
      break;
    }
    case MODE_MODE_CHANGE_STEP_TO_FIRST_POSITION:
    {
      modeTargetPos = firstPos + MOTOR_360_STEP;
      modeMotorStatus = MODE_MOTOR_STATUS_START;
      modeRunStep = MODE_MODE_CHANGE_STEP_ARIVE_FIRST_POSITION;
      break;
    }
    case MODE_MODE_CHANGE_STEP_ARIVE_FIRST_POSITION:
    {
      if(MODE_MOTOR_STATUS_IDLE == modeMotorStatus)
      {
        modeRunStep = MODE_MODE_CHANGE_STEP_TO_SECOND_POSITION;
        runCycle = 0;
      }
      break;
    }
    case MODE_MODE_CHANGE_STEP_TO_SECOND_POSITION:
    {
      modeTargetPos = secondtPos;
      modeMotorStatus = MODE_MOTOR_STATUS_START;
      modeRunStep = MODE_MODE_CHANGE_STEP_ARIVE_SECOND_POSITION;
      break;
    }
    case MODE_MODE_CHANGE_STEP_ARIVE_SECOND_POSITION:
    {
      if(MODE_MOTOR_STATUS_IDLE == modeMotorStatus)
      {
        modeRunStep = MODE_MODE_CHANGE_STEP_IDLE;
        runCycle = 0;
      }
      break;
    }
    
  }
}

static void MODE_MotorAction(void)
{
  void *msg_ptr;
  uint8_t *msg_prio;
  static uint16_t counter;
  uint8_t i;
  static volatile uint16_t brakeTimer;
  
  static uint16_t positionValueDelta;
  static uint16_t positionValueDeltaList[PARAMATERNUMBER];
  static uint16_t positionValueDeltaAverage;
  static uint16_t modeCurrentPosOld;

  switch(modeMotorRunStep)
  {
    case MODE_MOTOR_RUN_STEP_IDLE:
    {
       HAL_GPIO_WritePin(UP_GPIO_Port, UP_Pin, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(DOWN_GPIO_Port, DOWN_Pin, GPIO_PIN_RESET);
       modeMotorStatus = MODE_MOTOR_STATUS_IDLE;
       brakeTimer = 0;
       
      if((modeCurrentPos + 500) < modeTargetPos)
      {
        pwmMotorOut.pwmMotorOutHigh = 100;
        pwmMotorOut.pwmMotorOutLow = 0;
        xEventGroupSetBits(ModeEventHandle, 0x01);
      
        modeMotorRunStep = MODE_MOTOR_RUN_STEP_FORWARD;
        modeMotorStatus = MODE_MOTOR_STATUS_RUNNING;
      }
      else if(modeCurrentPos > (modeTargetPos + 500))
      {
        pwmMotorOut.pwmMotorOutHigh = 0;
        pwmMotorOut.pwmMotorOutLow = 100;
        xEventGroupSetBits(ModeEventHandle, 0x01);

        modeMotorRunStep = MODE_MOTOR_RUN_STEP_REVERSE;
        modeMotorStatus = MODE_MOTOR_STATUS_RUNNING;
      }
      break;
    }
    case MODE_MOTOR_RUN_STEP_FORWARD:
    {
      /* Get delta value between two tick 1ms */
      if(modeCurrentPos > modeCurrentPosOld)
      {
        positionValueDelta = (uint16_t)(modeCurrentPos - modeCurrentPosOld);
      }
      else
      {
        positionValueDelta = (uint16_t)(modeCurrentPosOld - modeCurrentPos);
      }
      /* Push delta value to buffer, length PARAMATERNUMBER */
      for(i = 0; i < (PARAMATERNUMBER - 1); i++)
      {
        positionValueDeltaList[PARAMATERNUMBER - 1 - i] = positionValueDeltaList[PARAMATERNUMBER -2 - i];
      }
      positionValueDeltaList[0] = positionValueDelta;
      /* Calculate average value, speed = delta S/ delta t */
      positionValueDeltaAverage = 0;
      for(i = 0; i < PARAMATERNUMBER; i++)
      {
        positionValueDeltaAverage = positionValueDeltaAverage + positionValueDeltaList[i];
      }
      positionValueDeltaAverage = positionValueDeltaAverage/PARAMATERNUMBER;
      
      if((modeCurrentPos + (PARAMATER * positionValueDeltaAverage)) > modeTargetPos)
      {
        modeMotorRunStep = MODE_MOTOR_RUN_STEP_BRAKE;
        
        pwmMotorOut.pwmMotorOutHigh = 100;
        pwmMotorOut.pwmMotorOutLow = 100;
        xEventGroupSetBits(ModeEventHandle, 0x01);
        HAL_GPIO_WritePin(UP_GPIO_Port, UP_Pin, GPIO_PIN_SET);
      }
      
      break;
    }
    case MODE_MOTOR_RUN_STEP_REVERSE:
    {
      /* Get delta value between two tick 1ms */
      if(modeCurrentPos > modeCurrentPosOld)
      {
        positionValueDelta = (uint16_t)(modeCurrentPos - modeCurrentPosOld);
      }
      else
      {
        positionValueDelta = (uint16_t)(modeCurrentPosOld - modeCurrentPos);
      }
      /* Push delta value to buffer, length PARAMATERNUMBER */
      for(i = 0; i < (PARAMATERNUMBER - 1); i++)
      {
        positionValueDeltaList[(PARAMATERNUMBER - 1) - i] = positionValueDeltaList[(PARAMATERNUMBER - 2) - i];
      }
      positionValueDeltaList[0] = positionValueDelta;

      /* Calculate average value, speed = delta S/ delta t */
      positionValueDeltaAverage = 0;
      for(i = 0; i < PARAMATERNUMBER; i++)
      {
        positionValueDeltaAverage = positionValueDeltaAverage + positionValueDeltaList[i];
      }
      positionValueDeltaAverage = positionValueDeltaAverage/PARAMATERNUMBER;
      
      if(modeCurrentPos < (modeTargetPos + (PARAMATER * positionValueDeltaAverage)))
      {
        modeMotorRunStep = MODE_MOTOR_RUN_STEP_BRAKE;
        brakeTimer = 0;
        
        pwmMotorOut.pwmMotorOutHigh = 100;
        pwmMotorOut.pwmMotorOutLow = 100;
        xEventGroupSetBits(ModeEventHandle, 0x01);

        HAL_GPIO_WritePin(DOWN_GPIO_Port, DOWN_Pin, GPIO_PIN_SET);
      }
      break;
    }
    case MODE_MOTOR_RUN_STEP_BRAKE:
    {
      if(MOTOR_BRAKE_TIME < brakeTimer)
      {
        modeMotorRunStep = MODE_MOTOR_RUN_STEP_STOP;
      }
      else
      {
        brakeTimer++;
      }
      
      break;
    }
    case MODE_MOTOR_RUN_STEP_STOP:
    {
      pwmMotorOut.pwmMotorOutHigh = 0;
      pwmMotorOut.pwmMotorOutLow = 0;
      xEventGroupSetBits(ModeEventHandle, 0x01);
      
      modeMotorRunStep = MODE_MOTOR_RUN_STEP_IDLE;
      modeMotorStatus = MODE_MOTOR_STATUS_STOP;
      controlMode = MODE_CONTROL_MODE_NONE;
      break;
    }
  }

  modeCurrentPosOld = modeCurrentPos;
}


void MODE_ControlModeHandel(void)
{
  static uint8_t masterModeCommandOld;
  static uint16_t masterTargetPosOld;
  

  if(MODE_CONTROL_MODE_NONE == controlMode)
  {
    if(masterModeCommandOld != linMasterMessage.message.masterModeCommand)
    {
      modeCommand = linMasterMessage.message.masterModeCommand;
      controlMode = MODE_CONTROL_MODE_LIN_COMMAND;
    }
    else if(masterTargetPosOld != linMasterMessage.message.masterTargetPos)
    {
      modeCommand = MODE_INVALID;
      modeTargetPos = linMasterMessage.message.masterTargetPos * 10;
      controlMode = MODE_CONTROL_MODE_LIN_POS;
    }
    else
    {
      modeCommand = MODE_INVALID;
    }
  }
  
  masterModeCommandOld = linMasterMessage.message.masterModeCommand;
  masterTargetPosOld = linMasterMessage.message.masterTargetPos;
}

void MODE_Task(void *argument)
{

  static uint16_t positionValueOld;
  
  
  MODE_Init();

  for(;;)
  {
/*
    osStatus = osMessageQueueGet(LIN_MasterTargetPositionHandle, (void *)&modeTargetPos, msg_prio, 0);
    osStatus = osMessageQueueGet(LIN_MasterModeCommandHandle, (void *)&modeCommand, msg_prio, 0);
    osStatus = osMessageQueueGet(SENT_CurrentPositionHandle, (void *)&modeCurrentPos, msg_prio, 0);
*/


    //osMessageQueuePut(MODE_MotorOutHandle, (void *)&modeMotorOut, 0, 0);
    //osThreadYield();
    if((positionValueOld >= (MOTOR_340_STEP - MOTOR_BOUNDARY_VALUE_STEP)) && (positionValue <= MOTOR_BOUNDARY_VALUE_STEP))
    {
      runCycle = runCycle + 1;
    }
    if((positionValue >= (MOTOR_340_STEP - MOTOR_BOUNDARY_VALUE_STEP)) && (positionValueOld <= MOTOR_BOUNDARY_VALUE_STEP))
    {
      if(runCycle > 0)
      {
        runCycle = runCycle - 1;
      }
    }
    modeCurrentPos = runCycle * MOTOR_360_STEP + positionValue;
    //modeTargetPos = linMasterMessage.message.masterTargetPos;
    MODE_ControlModeHandel();
    MODE_ModeChange();
    MODE_MotorAction();

    positionValueOld = positionValue;
    
    osDelay(1);
  }
}
