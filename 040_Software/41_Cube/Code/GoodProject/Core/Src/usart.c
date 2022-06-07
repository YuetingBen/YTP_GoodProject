/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "FreeRTOS.h" 
#include "event_groups.h"
#include "cmsis_os2.h"

#include "i2c.h"
#include "mode.h"

#define SOFTWARE_VERSION_NUMBER       0x15
#define LIN_BREAK_VALUE      0x00u
#define LIN_SYNC_VALUE       0x55u

#define LIN_RX_MASTER_ID          0x04u
#define LIN_TX_YTSENT_ID          0x03u

#define LIN_RX_MASTER_MCV_ID      0x34u
#define LIN_TX_YT_STATUS_ID       0x35u

#define LIN_RX_MASTER_CALIBRARION_ID          0x21u
#define LIN_TX_YT_CALIBRARION_ID              0x22u

#define LIN_EVENT_SENT         1 << 0
#define LIN_EVENT_RECEIVE    1 << 1

#define LIN_DATA_BUFFER_BREAK_INDEX      0
#define LIN_DATA_BUFFER_SYNC_INDEX       1
#define LIN_DATA_BUFFER_PID_INDEX        2
#define LIN_DATA_BUFFER_DATA_INDEX       3
#define LIN_DATA_BUFFER_MAX_INDEX       12

/* ID field bits definition */
#define LIN_ID_ID0   ((uint8_t) 0x01u)
#define LIN_ID_ID1   ((uint8_t) 0x02u)
#define LIN_ID_ID2   ((uint8_t) 0x04u)
#define LIN_ID_ID3   ((uint8_t) 0x08u)
#define LIN_ID_ID4   ((uint8_t) 0x10u)
#define LIN_ID_ID5   ((uint8_t) 0x20u)
#define LIN_ID_P0    ((uint8_t) 0x40u)
#define LIN_ID_P1    ((uint8_t) 0x80u)

#define LIN_ID2PID(id)  ((uint8_t)( ( LIN_ID_P0 *  \
                                  ((((id) & (LIN_ID_ID0)) / LIN_ID_ID0) ^  \
                                  (((id) & (LIN_ID_ID1)) / LIN_ID_ID1) ^  \
                                  (((id) & (LIN_ID_ID2)) / LIN_ID_ID2) ^  \
                                  (((id) & (LIN_ID_ID4)) / LIN_ID_ID4)) ) +  \
                                ( LIN_ID_P1 *  \
                                  ((((id) & (LIN_ID_ID1)) / LIN_ID_ID1) ^  \
                                  (((id) & (LIN_ID_ID3)) / LIN_ID_ID3) ^  \
                                  (((id) & (LIN_ID_ID4)) / LIN_ID_ID4) ^  \
                                  (((id) & (LIN_ID_ID5)) / LIN_ID_ID5) ^ 1u)) + \
                                ((id) & (0x3Fu))))


typedef enum
{
  LIN_STATE_IDLE,
  LIN_STATE_RECV_SYNC,
  LIN_STATE_RECV_ID,
  LIN_STATE_RECV_DATA,
  LIN_STATE_SEND_DATA,
  LIN_STATE_IGNORE_DATA,
  LIN_STATE_MAXNUM
}LIN_STATE_E;


typedef struct
{
  uint8_t dataBufferArray[12];
  uint8_t dataBufferIndex;
  
  uint8_t id;
  uint8_t pid;
  uint8_t length;
  uint8_t dataArray[8];
  uint8_t checksum;
}LIN_DATA_S;


#pragma pack (1)
typedef struct
{
  uint8_t masterModeCommand;
  uint16_t masterTargetPos;
  uint8_t masterTargetEnable;
  uint8_t masterInitRequest;
}LIN_MASTER_MESSAGE_S;

typedef union
{
  LIN_MASTER_MESSAGE_S message;
  uint8_t dataArray[8];
}LIN_MASTER_MESSAGE_U;


typedef struct
{
  uint8_t masterModeCommand : 4;
  uint8_t reverse : 4;
  uint8_t masterInitRequest;
}LIN_MASTERMCV_MESSAGE_S;

typedef union
{
  LIN_MASTERMCV_MESSAGE_S message;
  uint8_t dataArray[8];
}LIN_MASTERMCV_MESSAGE_U;

typedef struct
{
  uint16_t currentPos;
  uint16_t angle;
}LIN_YTSENT_MESSAGE_S;

typedef union
{
  LIN_YTSENT_MESSAGE_S message;
  uint8_t dataArray[8];
}LIN_YTSENT_MESSAGE_U;


typedef struct
{
  uint8_t workingMode;
  uint8_t currentStatus : 4;
  uint8_t error : 1;
  uint8_t reverse1 :3;
  uint8_t reverse2[5];
  uint8_t version;
}LIN_YTSTATUS_MESSAGE_S;

typedef union
{
  LIN_YTSTATUS_MESSAGE_S message;
  uint8_t dataArray[8];
}LIN_YTSTATUS_MESSAGE_U;

typedef struct
{
  uint8_t mode;
  uint16_t firstPos;
  uint16_t secondPos;
  uint8_t valveType;
}LIN_YTCAL_MESSAGE_S;

typedef union
{
  LIN_YTCAL_MESSAGE_S message;
  uint8_t dataArray[8];
}LIN_YTCAL_MESSAGE_U;

typedef struct
{
  uint8_t mode;
  uint16_t firstPos;
  uint16_t secondPos;
  uint8_t enable;
}LIN_MASTERCAL_MESSAGE_S;

typedef union
{
  LIN_MASTERCAL_MESSAGE_S message;
  uint8_t dataArray[8];
}LIN_MASTERCAL_MESSAGE_U;
#pragma pack ()

static uint8_t linMasterReceivedFlag = RESET;
static uint8_t linMasterMcvReceivedFlag = RESET;

static LIN_DATA_S linTransferData;
static LIN_STATE_E linState = LIN_STATE_IDLE;
static LIN_MASTER_MESSAGE_U linMasterMessage;
static LIN_YTSENT_MESSAGE_U linYtSentMessage;
static LIN_MASTERMCV_MESSAGE_U linMasterMcvMessage;
static LIN_YTSTATUS_MESSAGE_U linYtStatusMessage;
static LIN_YTCAL_MESSAGE_U linYtCalMessage;
static LIN_MASTERCAL_MESSAGE_U linMasterCalMessage;

static EventGroupHandle_t uartEventHandle = NULL;

extern uint16_t positionValue;
extern osMessageQueueId_t SENT_CurrentPositionQueueHandle;
extern osMessageQueueId_t LIN_MasterTargetPositionQueueHandle;
extern osMessageQueueId_t LIN_MasterModeCommandQueueHandle;


static uint8_t LIN_ComputeChecksum(LIN_DATA_S *linMessage);
static void LIN_MessagesRxHandel(void);
static void LIN_MessagesTxHandel(void);
static void LIN_MessagesSentHandel(void);
static void LIN_RX_MASTER_MessagesUpdate(void);
static void LIN_RX_MASTERMCV_MessagesUpdate(void);
static void LIN_RX_MASTERCAL_MessagesUpdate(void);
void HAL_UART_RxManage(void);
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  static  uint8_t data;
  if(USART1 == huart->Instance)
  {
     HAL_UART_Receive_IT(&huart1, &data, 1);
     linTransferData.dataBufferArray[linTransferData.dataBufferIndex] = data;
     linTransferData.dataBufferIndex++;

     HAL_UART_RxManage();
   }
}

static uint8_t LIN_ComputeChecksum(LIN_DATA_S *linMessage)
{
  /* The LIN checksum is the inverted sum-with-carry of all data bytes */
  uint16_t tmpValue;
  uint8_t i;

  if ((0x3C == linMessage->id) ||
      (0x3D ==  linMessage->id))
  {
    tmpValue = 0U;  /* Classic checksum without ID */
  }
  else 
  {
    tmpValue = linMessage->pid; /* Enhanced checksum without ID */
  }

  
  /* Sum data bytes */
  for(i = 0; i < linMessage->length; i++)
  {
    tmpValue += linMessage->dataArray[i];
    
    /* Add carries */
    tmpValue += (tmpValue >> 8U);
    tmpValue &= 0xFFU;
  }
  
  /* Return inverted sum (byte) */
  return (~((uint8_t)tmpValue));
}

void HAL_UART_RxManage(void)
{  
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static BaseType_t xResult;
  
  uint8_t i;
  uint8_t tmpValue;
  
  switch(linState)
  {
    case LIN_STATE_IDLE:
    {
      linTransferData.dataBufferIndex = LIN_DATA_BUFFER_BREAK_INDEX;
      if(LIN_BREAK_VALUE == linTransferData.dataBufferArray[LIN_DATA_BUFFER_BREAK_INDEX])
      {
        linState = LIN_STATE_RECV_SYNC;
        linTransferData.dataBufferIndex = LIN_DATA_BUFFER_SYNC_INDEX;
      }
      break;
    }
    case LIN_STATE_RECV_SYNC:
    {
      if(LIN_SYNC_VALUE == linTransferData.dataBufferArray[LIN_DATA_BUFFER_SYNC_INDEX])
      {
        linState = LIN_STATE_RECV_ID;
      }
      else
      {
        linState = LIN_STATE_IDLE;
        linTransferData.dataBufferIndex = 0;
      }
      break;
    }
    case LIN_STATE_RECV_ID:
    {
      if(LIN_ID2PID(LIN_TX_YTSENT_ID) == linTransferData.dataBufferArray[LIN_DATA_BUFFER_PID_INDEX])
      {
        linState = LIN_STATE_SEND_DATA;
        linTransferData.length = 0x08u;
        linTransferData.id = LIN_TX_YTSENT_ID;
        linTransferData.pid = LIN_ID2PID(linTransferData.id);
        xEventGroupSetBitsFromISR(uartEventHandle, LIN_EVENT_SENT, &xHigherPriorityTaskWoken);
      }
      else if(LIN_ID2PID(LIN_TX_YT_STATUS_ID) == linTransferData.dataBufferArray[LIN_DATA_BUFFER_PID_INDEX])
      {
        linState = LIN_STATE_SEND_DATA;
        linTransferData.length = 0x08u;
        linTransferData.id = LIN_TX_YT_STATUS_ID;
        linTransferData.pid = LIN_ID2PID(linTransferData.id);
        xEventGroupSetBitsFromISR(uartEventHandle, LIN_EVENT_SENT, &xHigherPriorityTaskWoken);
      }
      else if(LIN_ID2PID(LIN_TX_YT_CALIBRARION_ID) == linTransferData.dataBufferArray[LIN_DATA_BUFFER_PID_INDEX])
      {
        linState = LIN_STATE_SEND_DATA;
        linTransferData.length = 0x08u;
        linTransferData.id = LIN_TX_YT_CALIBRARION_ID;
        linTransferData.pid = LIN_ID2PID(linTransferData.id);
        xEventGroupSetBitsFromISR(uartEventHandle, LIN_EVENT_SENT, &xHigherPriorityTaskWoken);
      }
      else if(LIN_ID2PID(LIN_RX_MASTER_ID) == linTransferData.dataBufferArray[LIN_DATA_BUFFER_PID_INDEX])
      {
        linState = LIN_STATE_RECV_DATA;
        linTransferData.id = LIN_RX_MASTER_ID;
        linTransferData.pid = LIN_ID2PID(linTransferData.id);
        linTransferData.length = 0x08u;
      }
      else if(LIN_ID2PID(LIN_RX_MASTER_MCV_ID) == linTransferData.dataBufferArray[LIN_DATA_BUFFER_PID_INDEX])
      {
        linState = LIN_STATE_RECV_DATA;
        linTransferData.id = LIN_RX_MASTER_MCV_ID;
        linTransferData.pid = LIN_ID2PID(linTransferData.id);
        linTransferData.length = 0x08u;
      }
      else if(LIN_ID2PID(LIN_RX_MASTER_CALIBRARION_ID) == linTransferData.dataBufferArray[LIN_DATA_BUFFER_PID_INDEX])
      {
        linState = LIN_STATE_RECV_DATA;
        linTransferData.id = LIN_RX_MASTER_CALIBRARION_ID;
        linTransferData.pid = LIN_ID2PID(linTransferData.id);
        linTransferData.length = 0x08u;
      }
      else
      {
        linState = LIN_STATE_IDLE;
      }
      break;
    }
    case LIN_STATE_RECV_DATA:
    {
      if((LIN_DATA_BUFFER_DATA_INDEX + linTransferData.length + 1) <= linTransferData.dataBufferIndex)
      {
        for(i = 0; i < linTransferData.length; i++)
        {
          linTransferData.dataArray[i] = linTransferData.dataBufferArray[i + LIN_DATA_BUFFER_DATA_INDEX];
        }
        xEventGroupSetBitsFromISR(uartEventHandle, LIN_EVENT_RECEIVE, &xHigherPriorityTaskWoken);
        linState = LIN_STATE_IDLE;
        linTransferData.dataBufferIndex = 0;
      }
      break;
    }
    case LIN_STATE_IGNORE_DATA:
    {
      if(LIN_DATA_BUFFER_MAX_INDEX <= linTransferData.dataBufferIndex)
      {
        linState = LIN_STATE_IDLE;
        linTransferData.dataBufferIndex = 0;
      }
      break;
    }
    default:
    {
      linState = LIN_STATE_IDLE;
      break;
    }
  }
}


static void LIN_MessagesRxHandel(void)
{
  LIN_RX_MASTER_MessagesUpdate();
  LIN_RX_MASTERMCV_MessagesUpdate();
  LIN_RX_MASTERCAL_MessagesUpdate();
}

static void LIN_RX_MASTER_MessagesUpdate(void)
{
  uint8_t i;
  uint16_t linMasterTargetPositionValue;
  uint8_t linMasterTargetEnableValue;
  uint8_t linMasterModeCommandValue;

  if(LIN_RX_MASTER_ID == linTransferData.id)
  {
    for(i = 0; i < linTransferData.length; i++)
    {
      linMasterMessage.dataArray[i] = linTransferData.dataArray[i];
    }
    linMasterModeCommandValue = linMasterMessage.message.masterModeCommand;
    linMasterTargetPositionValue = linMasterMessage.message.masterTargetPos;
    linMasterTargetEnableValue = linMasterMessage.message.masterTargetEnable;

    if(SET == linMasterTargetEnableValue)
    {
      linMasterTargetPositionValue = linMasterMessage.message.masterTargetPos;
    }
    else
    {
      linMasterTargetPositionValue = 0xFFFF;
    }

    osMessageQueuePut(LIN_MasterModeCommandQueueHandle, (void *)&linMasterModeCommandValue, 0, 0);
    osMessageQueuePut(LIN_MasterTargetPositionQueueHandle, (void *)&linMasterTargetPositionValue, 0, 0);

    if(SET == linMasterMessage.message.masterInitRequest)
    {
        __set_FAULTMASK(1);           
        HAL_NVIC_SystemReset();
    }

    linMasterReceivedFlag = SET;
  }

}

static void LIN_RX_MASTERMCV_MessagesUpdate(void)
{
  uint8_t i;
  static uint8_t linMasterModeCommandValue;

  if(LIN_RX_MASTER_MCV_ID == linTransferData.id)
  {
    for(i = 0; i < linTransferData.length; i++)
    {
      linMasterMcvMessage.dataArray[i] = linTransferData.dataArray[i];
    }
    linMasterModeCommandValue = linMasterMcvMessage.message.masterModeCommand;

    osMessageQueuePut(LIN_MasterModeCommandQueueHandle, (void *)&linMasterModeCommandValue, 0, 0);

    if(SET == linMasterMcvMessage.message.masterInitRequest)
    {
        __set_FAULTMASK(1);           
        HAL_NVIC_SystemReset();
    }

    linMasterMcvReceivedFlag = SET;
  }
}

uint8_t LIN_RX_MASTERMCV_ReceiveFlagGet(void)
{
  uint8_t retVal;
  retVal = linMasterMcvReceivedFlag;
  linMasterMcvReceivedFlag = RESET;
  return((uint8_t)retVal);
}

uint8_t LIN_RX_MASTER_ReceiveFlagGet(void)
{
  uint8_t retVal;
  retVal = linMasterReceivedFlag;
  linMasterReceivedFlag = RESET;
  return((uint8_t)retVal);
}

void LIN_RX_MASTERCAL_MessagesUpdate(void)
{
  uint8_t i;
  uint8_t posBuffer[4];
  
  if(LIN_RX_MASTER_CALIBRARION_ID == linTransferData.id)
  {
    for(i = 0; i < linTransferData.length; i++)
    {
      linMasterCalMessage.dataArray[i] = linTransferData.dataArray[i];
    }
    
    if(SET == linMasterCalMessage.message.enable)
    {
      posBuffer[0] = (uint8_t)linMasterCalMessage.message.firstPos;
      posBuffer[1] = (uint8_t)(linMasterCalMessage.message.firstPos >> 8);
      posBuffer[2] = (uint8_t)linMasterCalMessage.message.secondPos;
      posBuffer[3] = (uint8_t)(linMasterCalMessage.message.secondPos >> 8);
      EEPROM_WriteRequest(linMasterCalMessage.message.mode, posBuffer);
    }
    /* Config valve type */
    else if(0xAA == linMasterCalMessage.message.enable)
    {
      EEPROM_WriteRequest(EE_VALVE_TYPE, &linMasterCalMessage.dataArray[0]);
    }
    else
    {
      /* Do Nothing */
    }
  }
}

static void LIN_MessagesTxHandel(void)
{
  uint8_t i;
  uint8_t tmpValue;
  
  if(LIN_TX_YTSENT_ID == linTransferData.id)
  {
    for(i = 0; i < linTransferData.length; i++)
    {
      linTransferData.dataArray[i] = linYtSentMessage.dataArray[i];
    }
    linTransferData.checksum = LIN_ComputeChecksum(&linTransferData);
    while(HAL_UART_Transmit_IT(&huart1, linTransferData.dataArray, linTransferData.length) != HAL_OK);
    while(HAL_UART_Transmit_IT(&huart1, &linTransferData.checksum , 1) != HAL_OK);
    UART_Start_Receive_IT(&huart1, &tmpValue, 1);
  }
  else if(LIN_TX_YT_CALIBRARION_ID == linTransferData.id)
  {
    for(i = 0; i < linTransferData.length; i++)
    {
      linTransferData.dataArray[i] = linYtCalMessage.dataArray[i];
    }
    linTransferData.checksum = LIN_ComputeChecksum(&linTransferData);
    while(HAL_UART_Transmit_IT(&huart1, linTransferData.dataArray, linTransferData.length) != HAL_OK);
    while(HAL_UART_Transmit_IT(&huart1, &linTransferData.checksum , 1) != HAL_OK);
    UART_Start_Receive_IT(&huart1, &tmpValue, 1);
  }
  else
  {
    /* Do nothing */
  }
}

static void LIN_TX_YTSENT_MessagesUpdate(void)
{
  static uint16_t linCurrentPositionValue;
  static uint16_t linYtSentCurrentPositionValue;

  static uint32_t linAngleValue;
  
  osStatus_t osStatus;
  void *msg_ptr;
  uint8_t *msg_prio;

  osMessageQueueGet(SENT_CurrentPositionQueueHandle, (void *)&linYtSentCurrentPositionValue, msg_prio, 0); 
  linAngleValue = linYtSentCurrentPositionValue * 3400 / 40950;
  linYtSentMessage.dataArray[0] = (uint8_t)(linYtSentCurrentPositionValue);
  linYtSentMessage.dataArray[1] = (uint8_t)(linYtSentCurrentPositionValue >> 8);

  linYtSentMessage.dataArray[2] = (uint8_t)(linAngleValue);
  linYtSentMessage.dataArray[3] = (uint8_t)(linAngleValue >> 8);
}

static void LIN_TX_YTSTATUS_MessagesUpdate(void)
{
  linYtStatusMessage.message.workingMode = MODE_GetMode();
  linYtStatusMessage.message.currentStatus = MODE_GetModeStatus();
  linYtStatusMessage.message.error = MODE_GetModeErrorStatus();
  linYtStatusMessage.message.version = SOFTWARE_VERSION_NUMBER;
}


static void LIN_TX_YTCAL_MessagesUpdate(void)
{
  static EE_ITEM_E modeName;
  uint8_t posBuffer[4];
  uint8_t valveType;
  
  if(modeName < EE_MODE8_POSITION)
  {
    modeName++;
  }
  else
  {
    modeName = 0;
  }

  EEPROM_ReadRequest(modeName, posBuffer);
  EEPROM_ReadRequest(EE_VALVE_TYPE, &valveType);

  linYtCalMessage.message.mode = (uint8_t)modeName;
  linYtCalMessage.message.firstPos = (uint16_t)(posBuffer[0] + posBuffer[1] * 256);
  linYtCalMessage.message.secondPos = (uint16_t)(posBuffer[2] + posBuffer[3] * 256);
  linYtCalMessage.message.valveType = (uint8_t)(valveType);
  
}

void HAL_UART_Task(void * argument)
{
  uint8_t i;
  uint8_t tmpValue;
  EventBits_t  eventTrigger;

  HAL_GPIO_WritePin(LIN_SLEEP_GPIO_Port, LIN_SLEEP_Pin, GPIO_PIN_SET);

  UART_Start_Receive_IT(&huart1, &tmpValue, 1);

  uartEventHandle = xEventGroupCreate();

  xEventGroupSetBits(uartEventHandle, LIN_EVENT_RECEIVE);
  
  /* Infinite loop */
  for(;;)
  {
    eventTrigger = xEventGroupWaitBits(uartEventHandle,
                                (LIN_EVENT_SENT |LIN_EVENT_RECEIVE) ,
                                pdTRUE,
                                pdFALSE,
                                1);

    if(LIN_EVENT_SENT == (eventTrigger & LIN_EVENT_SENT))
    {
      //LIN_MessagesTxHandel();
#if 1
      if(LIN_TX_YTSENT_ID == linTransferData.id)
      {
        LIN_TX_YTSENT_MessagesUpdate();
        for(i = 0; i < linTransferData.length; i++)
        {
          linTransferData.dataArray[i] = linYtSentMessage.dataArray[i];
        }
        linTransferData.checksum = LIN_ComputeChecksum(&linTransferData);
        while(HAL_UART_Transmit_IT(&huart1, linTransferData.dataArray, linTransferData.length) != HAL_OK);
        while(HAL_UART_Transmit_IT(&huart1, &linTransferData.checksum , 1) != HAL_OK);
        UART_Start_Receive_IT(&huart1, &tmpValue, 1);
      }
      else if(LIN_TX_YT_STATUS_ID == linTransferData.id)
      {
        LIN_TX_YTSTATUS_MessagesUpdate();
        for(i = 0; i < linTransferData.length; i++)
        {
          linTransferData.dataArray[i] = linYtStatusMessage.dataArray[i];
        }
        linTransferData.checksum = LIN_ComputeChecksum(&linTransferData);
        while(HAL_UART_Transmit_IT(&huart1, linTransferData.dataArray, linTransferData.length) != HAL_OK);
        while(HAL_UART_Transmit_IT(&huart1, &linTransferData.checksum , 1) != HAL_OK);
        UART_Start_Receive_IT(&huart1, &tmpValue, 1);
      }
      else if(LIN_TX_YT_CALIBRARION_ID == linTransferData.id)
      {
        LIN_TX_YTCAL_MessagesUpdate();
        for(i = 0; i < linTransferData.length; i++)
        {
          linTransferData.dataArray[i] = linYtCalMessage.dataArray[i];
        }
        linTransferData.checksum = LIN_ComputeChecksum(&linTransferData);
        while(HAL_UART_Transmit_IT(&huart1, linTransferData.dataArray, linTransferData.length) != HAL_OK);
        while(HAL_UART_Transmit_IT(&huart1, &linTransferData.checksum , 1) != HAL_OK);
        UART_Start_Receive_IT(&huart1, &tmpValue, 1);
      }
      else
      {
        /* Do nothing */
      }
#endif
    }
    else if(LIN_EVENT_RECEIVE == (eventTrigger & LIN_EVENT_RECEIVE))
    {
      LIN_MessagesRxHandel();
    }
    else
    {
      /* Do nothing */
    }
  }
}

/* USER CODE END 1 */
