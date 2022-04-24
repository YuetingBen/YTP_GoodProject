/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "mode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId uartTaskHandle;
osThreadId canTaskHandle;
osThreadId timTaskHandle;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN */
osThreadId_t CANHandle;
const osThreadAttr_t CAN_attributes = {
  .name = "CAN",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART */
osThreadId_t UARTHandle;
const osThreadAttr_t UART_attributes = {
  .name = "UART",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TIM */
osThreadId_t TIMHandle;
const osThreadAttr_t TIM_attributes = {
  .name = "TIM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MODE */
osThreadId_t MODEHandle;
const osThreadAttr_t MODE_attributes = {
  .name = "MODE",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SENT_CurrentPositionQueue */
osMessageQueueId_t SENT_CurrentPositionQueueHandle;
uint8_t SENT_CurrentPositionBuffer[ 1 * sizeof( uint16_t ) ];
osStaticMessageQDef_t SENT_CurrentPositionControlBlock;
const osMessageQueueAttr_t SENT_CurrentPositionQueue_attributes = {
  .name = "SENT_CurrentPositionQueue",
  .cb_mem = &SENT_CurrentPositionControlBlock,
  .cb_size = sizeof(SENT_CurrentPositionControlBlock),
  .mq_mem = &SENT_CurrentPositionBuffer,
  .mq_size = sizeof(SENT_CurrentPositionBuffer)
};
/* Definitions for LIN_MasterTargetPositionQueue */
osMessageQueueId_t LIN_MasterTargetPositionQueueHandle;
uint8_t LIN_MasterTargetPositionBuffer[ 1 * sizeof( uint16_t ) ];
osStaticMessageQDef_t LIN_MasterTargetPositionControlBlock;
const osMessageQueueAttr_t LIN_MasterTargetPositionQueue_attributes = {
  .name = "LIN_MasterTargetPositionQueue",
  .cb_mem = &LIN_MasterTargetPositionControlBlock,
  .cb_size = sizeof(LIN_MasterTargetPositionControlBlock),
  .mq_mem = &LIN_MasterTargetPositionBuffer,
  .mq_size = sizeof(LIN_MasterTargetPositionBuffer)
};
/* Definitions for LIN_MasterModeCommandQueue */
osMessageQueueId_t LIN_MasterModeCommandQueueHandle;
uint8_t LIN_MasterModeCommandBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t LIN_MasterModeCommandControlBlock;
const osMessageQueueAttr_t LIN_MasterModeCommandQueue_attributes = {
  .name = "LIN_MasterModeCommandQueue",
  .cb_mem = &LIN_MasterModeCommandControlBlock,
  .cb_size = sizeof(LIN_MasterModeCommandControlBlock),
  .mq_mem = &LIN_MasterModeCommandBuffer,
  .mq_size = sizeof(LIN_MasterModeCommandBuffer)
};
/* Definitions for MODE_MotorOutQueue */
osMessageQueueId_t MODE_MotorOutQueueHandle;
uint8_t MODE_MotorOutBuffer[ 1 * sizeof( uint32_t ) ];
osStaticMessageQDef_t MODE_MotorOutControlBlock;
const osMessageQueueAttr_t MODE_MotorOutQueue_attributes = {
  .name = "MODE_MotorOutQueue",
  .cb_mem = &MODE_MotorOutControlBlock,
  .cb_size = sizeof(MODE_MotorOutControlBlock),
  .mq_mem = &MODE_MotorOutBuffer,
  .mq_size = sizeof(MODE_MotorOutBuffer)
};
/* Definitions for MotorOutEvent */
osEventFlagsId_t MotorOutEventHandle;
const osEventFlagsAttr_t MotorOutEvent_attributes = {
  .name = "MotorOutEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void HAL_CAN_Task(void *argument);
extern void HAL_UART_Task(void *argument);
extern void HAL_TIM_Task(void *argument);
extern void MODE_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of SENT_CurrentPositionQueue */
  SENT_CurrentPositionQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &SENT_CurrentPositionQueue_attributes);

  /* creation of LIN_MasterTargetPositionQueue */
  LIN_MasterTargetPositionQueueHandle = osMessageQueueNew (1, sizeof(uint16_t), &LIN_MasterTargetPositionQueue_attributes);

  /* creation of LIN_MasterModeCommandQueue */
  LIN_MasterModeCommandQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &LIN_MasterModeCommandQueue_attributes);

  /* creation of MODE_MotorOutQueue */
  MODE_MotorOutQueueHandle = osMessageQueueNew (1, sizeof(uint32_t), &MODE_MotorOutQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN */
  CANHandle = osThreadNew(HAL_CAN_Task, NULL, &CAN_attributes);

  /* creation of UART */
  UARTHandle = osThreadNew(HAL_UART_Task, NULL, &UART_attributes);

  /* creation of TIM */
  TIMHandle = osThreadNew(HAL_TIM_Task, NULL, &TIM_attributes);

  /* creation of MODE */
  MODEHandle = osThreadNew(MODE_Task, NULL, &MODE_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of MotorOutEvent */
  MotorOutEventHandle = osEventFlagsNew(&MotorOutEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(UP_GPIO_Port, UP_Pin, GPIO_PIN_SET);
    //HAL_GPIO_WritePin(DOWN_GPIO_Port, DOWN_Pin, GPIO_PIN_SET);
    osDelay(500);

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(UP_GPIO_Port, UP_Pin, GPIO_PIN_RESET);
    //HAL_GPIO_WritePin(DOWN_GPIO_Port, DOWN_Pin, GPIO_PIN_RESET);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

