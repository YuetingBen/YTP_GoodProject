#include "mode.h"

#include "usart.h"
#include "tim.h"

#include "FreeRTOS.h" 
#include "event_groups.h"
#include "cmsis_os2.h"


#define PARAMATER 15
#define PARAMATERNUMBER 30

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
  MODE_MOTOR_IDLE,
  MODE_MOTOR_FORWARD,
  MODE_MOTOR_REVERSE,
  MODE_MOTOR_BREAK,
  MODE_MOTOR_STOP
}MODE_MOTOR_STATUS_E;

typedef struct
{
  uint16_t modeMotorOutHigh;
  uint16_t modeMotorOutLow;
}MODE_MOTOR_OUT_S;

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

static MODE_COMMAND_E modeCommand;
static uint16_t modeCurrentPos;
static uint16_t modeTargetPos;

uint16_t positionValueDelta;
uint16_t positionValueDeltaList[PARAMATERNUMBER];
uint16_t positionValueDeltaAverage;


static MODE_MOTOR_OUT_S modeMotorOut;
static MODE_MOTOR_STATUS_E modeMotorStatus;
static MODE_MOTOR_STATUS_E modeMotorStatusOld;
void MODE_Init(void)
{
  modeCurrentPos = 0;
  modeTargetPos = 0;
  modeMotorOut.modeMotorOutHigh = 0;
  modeMotorOut.modeMotorOutLow = 0;
}

void MODE_Task(void *argument)
{
  //static osStatus_t osStatus;
  void *msg_ptr;
  uint8_t *msg_prio;
  static uint16_t positionValueOld;


  static uint16_t counter;

  uint8_t i;

  
  MODE_Init();
  //
  for(;;)
  {
/*
    osStatus = osMessageQueueGet(LIN_MasterTargetPositionHandle, (void *)&modeTargetPos, msg_prio, 0);
    osStatus = osMessageQueueGet(LIN_MasterModeCommandHandle, (void *)&modeCommand, msg_prio, 0);
    osStatus = osMessageQueueGet(SENT_CurrentPositionHandle, (void *)&modeCurrentPos, msg_prio, 0);
*/
    switch(modeMotorStatus)
    {
      case MODE_MOTOR_IDLE:
      {
         HAL_GPIO_WritePin(UP_GPIO_Port, UP_Pin, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(DOWN_GPIO_Port, DOWN_Pin, GPIO_PIN_RESET);
         
        if((positionValue + 500) < linMasterMessage.message.masterTargetPos)
        {
          modeMotorStatus = MODE_MOTOR_FORWARD;
          pwmMotorOut.pwmMotorOutHigh = 100;
          pwmMotorOut.pwmMotorOutLow = 0;
        }
        else if(positionValue > (linMasterMessage.message.masterTargetPos + 500))
        {
          pwmMotorOut.pwmMotorOutHigh = 0;
          pwmMotorOut.pwmMotorOutLow = 100;

          modeMotorStatus = MODE_MOTOR_REVERSE;
        }
        break;
      }
      case MODE_MOTOR_FORWARD:
      {
        /* Get delta value between two tick 1ms */
        if(positionValue > positionValueOld)
        {
          positionValueDelta = (uint16_t)(positionValue - positionValueOld);
        }
        else
        {
          positionValueDelta = (uint16_t)(positionValueOld - positionValue);
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
        
        if((positionValue + (PARAMATER * positionValueDeltaAverage)) > linMasterMessage.message.masterTargetPos)
        {
          modeMotorStatus = MODE_MOTOR_BREAK;
          pwmMotorOut.pwmMotorOutHigh = 100;
          pwmMotorOut.pwmMotorOutLow = 100;

         HAL_GPIO_WritePin(UP_GPIO_Port, UP_Pin, GPIO_PIN_SET);
        }
        
        break;
      }
      case MODE_MOTOR_REVERSE:
      {
        /* Get delta value between two tick 1ms */
        if(positionValue > positionValueOld)
        {
          positionValueDelta = (uint16_t)(positionValue - positionValueOld);
        }
        else
        {
          positionValueDelta = (uint16_t)(positionValueOld - positionValue);
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
        
        if(positionValue < (linMasterMessage.message.masterTargetPos + (PARAMATER * positionValueDeltaAverage)))
        {
          modeMotorStatus = MODE_MOTOR_BREAK;
          pwmMotorOut.pwmMotorOutHigh = 100;
          pwmMotorOut.pwmMotorOutLow = 100;

          HAL_GPIO_WritePin(DOWN_GPIO_Port, DOWN_Pin, GPIO_PIN_SET);
        }
        break;
      }
      case MODE_MOTOR_BREAK:
      {
        pwmMotorOut.pwmMotorOutHigh = 100;
        pwmMotorOut.pwmMotorOutLow = 100;
        
        modeMotorStatus = MODE_MOTOR_STOP;
        break;
      }
      case MODE_MOTOR_STOP:
      {
        pwmMotorOut.pwmMotorOutHigh = 100;
        pwmMotorOut.pwmMotorOutLow = 100;
        
        modeMotorStatus = MODE_MOTOR_IDLE;
        break;
      }
    }
#if 0
    positionValueDelta = positionValue - positionValueOld;
    if(positionValue + 500 < linMasterMessage.message.masterTargetPos)
    {
      pwmMotorOut.pwmMotorOutHigh = 50;
      pwmMotorOut.pwmMotorOutLow = 0;
    }
    else if(linMasterMessage.message.masterTargetPos + 500 < positionValue)
    {
      pwmMotorOut.pwmMotorOutHigh = 0;
      pwmMotorOut.pwmMotorOutLow = 70;
    }
    else
    {
      pwmMotorOut.pwmMotorOutHigh = 0;
      pwmMotorOut.pwmMotorOutLow = 0;
    }
 #endif
    //osMessageQueuePut(MODE_MotorOutHandle, (void *)&modeMotorOut, 0, 0);
    //osThreadYield();
    if(modeMotorStatusOld != modeMotorStatus)
    {
      xEventGroupSetBits(ModeEventHandle, 0x01);
    }
    positionValueOld = positionValue;
    modeMotorStatusOld = modeMotorStatus;
    osDelay(1);
  }
}
