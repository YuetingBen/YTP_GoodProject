#include "mode.h"

#include "usart.h"
#include "tim.h"

#include "FreeRTOS.h" 
#include "event_groups.h"
#include "cmsis_os2.h"


#define PARAMATER 15u
#define PARAMATERNUMBER 30u
#define MOTOR_BRAKE_TIME 500u  /* 500ms  */


#define MOTOR_SMALLEST_STEP   50u  /* Angle 0.41 degree, 10 * 340 / 40950*/
#define MOTOR_340_STEP    40950u  /* Angle 360 degree, 360 * 40950 / 340 */
#define MOTOR_360_STEP    43359u  /* Angle 360 degree, 360 * 40950 / 340 */
#define MOTOR_ONE_CYCLE_SMALLEST_STEP    43359u  /* Angle 360 degree, 360 * 40950 / 340 */
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
  MODE_MODE_POSITION_STEP_IDLE,
  MODE_MODE_POSITION_STEP_TO_TARGET_POSITION,
  MODE_MODE_POSITION_ARIVE_TARGET_POSITION
}MODE_MODE_POSITION_STEP_S;

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


extern osEventFlagsId_t MotorOutEventHandle;
extern osMessageQueueId_t SENT_CurrentPositionQueueHandle;
extern osMessageQueueId_t LIN_MasterTargetPositionQueueHandle;
extern osMessageQueueId_t LIN_MasterModeCommandQueueHandle;
extern osMessageQueueId_t MODE_MotorOutQueueHandle;

extern uint16_t positionValue;


static volatile MODE_COMMAND_E modeCommand;
//MODE_COMMAND_E modeCommand;
static MODE_COMMAND_E currentMode;

static uint8_t runCycle;
static uint32_t modeCurrentPos;
static uint32_t modeTargetPos;

static uint16_t masterTargetPos;

static MODE_CONTROL_MODE_E controlMode;

static MODE_MOTOR_OUT_S modeMotorOut;
static MODE_MOTOR_OUT_S modeMotorOutOld;
static MODE_MOTOR_OUT_S *modeMotorOutOldPtr;
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

static void MODE_ModePositionChange(void)
{
  static MODE_MODE_CHANGE_STEP_S posRunStep = MODE_MODE_POSITION_STEP_IDLE;

  switch(posRunStep)
  {
    case MODE_MODE_POSITION_STEP_IDLE:
    {
      posRunStep = MODE_MODE_POSITION_STEP_TO_TARGET_POSITION;
      break;
    }
    case MODE_MODE_POSITION_STEP_TO_TARGET_POSITION:
    {
      posRunStep = MODE_MODE_POSITION_ARIVE_TARGET_POSITION;
      modeTargetPos = masterTargetPos * 10;
      break;
    }
    case MODE_MODE_POSITION_ARIVE_TARGET_POSITION:
    {
      if(MODE_MOTOR_STATUS_IDLE == modeMotorStatus)
      {
        posRunStep = MODE_MODE_POSITION_STEP_IDLE;
        controlMode = MODE_CONTROL_MODE_NONE;
      }
      break;
    }
  }
}


static void MODE_ModeCommandChange(void)
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
      if(MOTOR_ONE_CYCLE_SMALLEST_STEP > (modeTargetPos - modeCurrentPos))
      {
        modeTargetPos = modeTargetPos + MOTOR_360_STEP;
      }
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
        controlMode = MODE_CONTROL_MODE_NONE;
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
  static uint32_t motorOutMessageQueue[2];

  switch(modeMotorRunStep)
  {
    case MODE_MOTOR_RUN_STEP_IDLE:
    {
       HAL_GPIO_WritePin(UP_GPIO_Port, UP_Pin, GPIO_PIN_RESET);
       HAL_GPIO_WritePin(DOWN_GPIO_Port, DOWN_Pin, GPIO_PIN_RESET);
       modeMotorStatus = MODE_MOTOR_STATUS_IDLE;
       brakeTimer = 0;
       
      if((modeCurrentPos + MOTOR_SMALLEST_STEP) < modeTargetPos)
      {
        modeMotorOut.modeMotorOutHigh = 0;
        modeMotorOut.modeMotorOutLow= 100;
      
        modeMotorRunStep = MODE_MOTOR_RUN_STEP_FORWARD;
        modeMotorStatus = MODE_MOTOR_STATUS_RUNNING;
      }
      else if(modeCurrentPos > (modeTargetPos + MOTOR_SMALLEST_STEP))
      {
        modeMotorOut.modeMotorOutHigh = 100;
        modeMotorOut.modeMotorOutLow= 0;

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

        modeMotorOut.modeMotorOutHigh = 100;
        modeMotorOut.modeMotorOutLow= 100;
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

        modeMotorOut.modeMotorOutHigh = 100;
        modeMotorOut.modeMotorOutLow= 100;
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
      modeMotorOut.modeMotorOutHigh = 0;
      modeMotorOut.modeMotorOutLow= 0;
      
      modeMotorRunStep = MODE_MOTOR_RUN_STEP_IDLE;
      modeMotorStatus = MODE_MOTOR_STATUS_STOP;
      break;
    }
  }
  /* Trigger MotorOutEventHandle */  
  if((modeMotorOutOld.modeMotorOutHigh  != modeMotorOut.modeMotorOutHigh) 
    || (modeMotorOutOld.modeMotorOutLow != modeMotorOut.modeMotorOutLow))
  {
    motorOutMessageQueue[0] = (uint32_t)modeMotorOut.modeMotorOutHigh;
    motorOutMessageQueue[1] = (uint32_t)modeMotorOut.modeMotorOutLow;
    osMessageQueuePut(MODE_MotorOutQueueHandle, (uint32_t*)&modeMotorOut, 0, 0);
    xEventGroupSetBits(MotorOutEventHandle, 0x01);
  }

  modeCurrentPosOld = modeCurrentPos;
  //modeMotorOutOldPtr = &modeMotorOut;
  modeMotorOutOld.modeMotorOutHigh = modeMotorOut.modeMotorOutHigh;
  modeMotorOutOld.modeMotorOutLow = modeMotorOut.modeMotorOutLow;
}


void MODE_ControlModeHandel(void)
{
  static uint8_t masterModeCommand;

  static uint8_t masterModeCommandOld;
  static uint16_t masterTargetPosOld;

  uint8_t *msg_prio;

  osMessageQueueGet(LIN_MasterModeCommandQueueHandle, (void *)&masterModeCommand, msg_prio, 0);
  osMessageQueueGet(LIN_MasterTargetPositionQueueHandle, (void *)&masterTargetPos, msg_prio, 0);

  if(MODE_CONTROL_MODE_NONE == controlMode)
  {
    if(masterModeCommandOld != masterModeCommand)
    {
      modeCommand = (MODE_COMMAND_E)masterModeCommand;
      controlMode = MODE_CONTROL_MODE_LIN_COMMAND;
    }
    else if(masterTargetPosOld != masterTargetPos)
    {
      modeCommand = MODE_INVALID;
      controlMode = MODE_CONTROL_MODE_LIN_POS;
    }
    else
    {
      modeCommand = MODE_INVALID;
    }
  }

  switch(controlMode)
  {
    case MODE_CONTROL_MODE_NONE:
    {
      break;
    }
    case MODE_CONTROL_MODE_LIN_COMMAND:
    {
      MODE_ModeCommandChange();
      break;
    }
    case MODE_CONTROL_MODE_LIN_POS:
    {
      MODE_ModePositionChange(); 
      break;
    }
    case MODE_CONTROL_MODE_KEY_COMMAND:
    {
      break;
    }
    case MODE_CONTROL_MODE_KEY_RUN:
    {
      break;
    }
    default:
    {
      break;
    }
  }

  if(MODE_CONTROL_MODE_NONE != controlMode)
  {
    MODE_MotorAction();
  }
  
  masterModeCommandOld = masterModeCommand;
  masterTargetPosOld = masterTargetPos;
}

void MODE_Task(void *argument)
{
  static uint16_t positionValueOld;
  uint16_t positionValueTemp;

  osStatus_t osStatus;

  uint8_t *msg_prio;
  
  MODE_Init();

  for(;;)
  {
    //osMessageQueueGet(SENT_CurrentPositionQueueHandle, (uint16_t *)&positionValueTemp, msg_prio, 0);
    positionValueTemp = positionValue;
    if((positionValueOld >= (MOTOR_340_STEP - MOTOR_BOUNDARY_VALUE_STEP)) && (positionValueTemp <= MOTOR_BOUNDARY_VALUE_STEP))
    {
      runCycle = runCycle + 1;
    }
    if((positionValueTemp >= (MOTOR_340_STEP - MOTOR_BOUNDARY_VALUE_STEP)) && (positionValueOld <= MOTOR_BOUNDARY_VALUE_STEP))
    {
      if(runCycle > 0)
      {
        runCycle = runCycle - 1;
      }
    }
    modeCurrentPos = runCycle * MOTOR_360_STEP + positionValueTemp;
    
    MODE_ControlModeHandel();

    positionValueOld = positionValueTemp;
    
    osDelay(1);
  }
}
