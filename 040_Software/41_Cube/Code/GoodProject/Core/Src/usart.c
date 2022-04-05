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

#define LIN_SYNC_VALUE      0x55u
#define LIN_RX_ID      0x04u
#define LIN_TX_ID      0x03u

#define LIN_DATA_BUFFER_BREAK_INDEX      0
#define LIN_DATA_BUFFER_SYNC_INDEX       1
#define LIN_DATA_BUFFER_PID_INDEX        2
#define LIN_DATA_BUFFER_DATA_INDEX       3


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
  LIN_STATE_MAXNUM
}LIN_STATE_E;


typedef struct
{
  uint8_t dataBufferArray[12];
  uint8_t dataBufferIndex;
  uint8_t id;
  uint8_t pid;
  uint8_t length;
}LIN_DATA_S;

static LIN_DATA_S linTransferData;
LIN_STATE_E linState = LIN_STATE_IDLE;



#define VA_UPDATE_EVENT    0x01

osEventFlagsId_t uartEventHandle;
const osEventFlagsAttr_t uartEvent_attributes = {
  .name = "uartEvent"
};

uint8_t testFlag;
static uint8_t LinReceiveData[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};


static uint8_t LIN_ComputeLen(uint8_t linId);
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
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
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




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  testFlag++;
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}



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


void HAL_UART_TransmitHeader()
{
  
}

static uint8_t LIN_ComputeLen(uint8_t linId)
{
  uint8_t retLen;
  /* The LIN 2.x message length coding is the following :
  **
  ** ID5  ID4  Len
  **   0   0    2
  **   0   1    2
  **   1   0    4
  **   1   1    8
  */
  if(0x02 == (linId & 0x30))
  {
    retLen = 4;
  }
  else if(0x03 == (linId & 0x30))
  {
    retLen = 8;
  }
  else
  {
    retLen = 2;
  }
  return retLen;
}

void HAL_UART_RxManage(void)
{  
  BaseType_t xHigherPriorityTaskWoken;
  switch(linState)
  {
    case LIN_STATE_IDLE:
    {
      linState = LIN_STATE_RECV_SYNC;
      break;
    }
    case LIN_STATE_RECV_SYNC:
    {
      if(LIN_SYNC_VALUE == linTransferData.dataBufferArray[LIN_DATA_BUFFER_SYNC_INDEX])
      {
        linState = LIN_STATE_RECV_ID;
      }
      break;
    }
    case LIN_STATE_RECV_ID:
    {
      if(LIN_ID2PID(LIN_TX_ID) == linTransferData.dataBufferArray[LIN_DATA_BUFFER_PID_INDEX])
      {
        linState = LIN_STATE_SEND_DATA;
        linTransferData.id = LIN_TX_ID;
        xEventGroupSetBitsFromISR(uartEventHandle, VA_UPDATE_EVENT, &xHigherPriorityTaskWoken);
      }
      else if(LIN_ID2PID(LIN_RX_ID) == linTransferData.dataBufferArray[2])
      {
        linTransferData.id = LIN_RX_ID;
        linState = LIN_STATE_RECV_DATA;
      }
      else
      {
        /* Do nothing */
      }
      linTransferData.length = 0x08u;
      linTransferData.pid = LIN_ID2PID(linTransferData.id);
      break;
    }
    case LIN_STATE_RECV_DATA:
    {
      if((LIN_DATA_BUFFER_DATA_INDEX + linTransferData.length + 1) <= linTransferData.dataBufferIndex)
      {
        linState = LIN_STATE_IDLE;
        linTransferData.dataBufferIndex = 0;
      }
      break;
    }
  }
}

void HAL_UART_Task(void * argument)
{
  static uint8_t linSentdataArray[8] = {0x12, 0x34, 0x56, 0x78, 0x90, 0x12, 0x34, 0x56};
  static uint8_t linSentdataCheckSum = 0xB7;
  
  uint8_t lin_data1;
  uint8_t LIN_RPID;
  uint8_t TstLin1_MASTER_SendBuff[9];

  uint8_t LinData[8] = {0x00, 0x00, 0xBD, 0x00, 0x00, 0x00, 0x00, 0x00};


  lin_data1 = 0x55;
  LIN_RPID = 0x42;

  uint8_t r_event;


  HAL_GPIO_WritePin(LIN_SLEEP_GPIO_Port, LIN_SLEEP_Pin, GPIO_PIN_SET);

  UART_Start_Receive_IT(&huart1, LinReceiveData, 1);

  uartEventHandle = osEventFlagsNew(&uartEvent_attributes);
  
  /* Infinite loop */
  for(;;)
  {
    r_event = xEventGroupWaitBits(uartEventHandle,
                                VA_UPDATE_EVENT,
                                pdTRUE,
                                pdTRUE,
                                osWaitForever);
    
    
    {
      while(HAL_UART_Transmit_IT(&huart1, linSentdataArray, linTransferData.length) != HAL_OK);
      while(HAL_UART_Transmit_IT(&huart1, &linSentdataCheckSum , 1) != HAL_OK);
      
      linState = LIN_STATE_IDLE;
      linTransferData.dataBufferIndex = 0;
    } 
    osDelay(100);
  }
}

/* USER CODE END 1 */
