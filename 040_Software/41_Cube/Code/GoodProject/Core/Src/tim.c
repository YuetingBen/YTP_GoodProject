/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h" 
#include "event_groups.h"
#include "cmsis_os2.h"

/*
typedef struct
{
  uint16_t pwmMotorOutHigh;
  uint16_t pwmMotorOutLow;
}PWM_MOTOR_OUT_S;
*/
PWM_MOTOR_OUT_S pwmMotorOut;

extern osMessageQueueId_t SENT_CurrentPositionHandle;
extern osMessageQueueId_t MODE_MotorOutHandle;
EventGroupHandle_t ModeEventHandle;
static uint8_t dataList[30];

uint16_t positionValue;
uint16_t timerValue;
static uint8_t index;
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_icHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PC6     ------> TIM3_CH1
    PC7     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_TIM3_ENABLE();

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t crc4_cal(uint8_t *data, uint8_t len)
{
  uint8_t CRC4_Table[16]= {0,13,7,10,14,3,9,4,1,12,6,11,15,2,8,5};
  uint8_t result = 0x03;
  uint8_t tableNo = 0;
  uint8_t i;
  for(i = 0; i < len; i++)    
  {
    tableNo = result ^ data[i];
    result = CRC4_Table[tableNo];
  }
  return result;
}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  static uint32_t counter;
  static uint32_t counterOld;
  static uint32_t counterDelta;
  
  /* Prevent unused argument(s) compilation warning */
  if(htim->Instance==TIM1)
  {
    counter = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
    if(counter > counterOld)
    {
      counterDelta = counter - counterOld;
    }
    else
    {
      counterDelta = counter + 65536 - counterOld;
    }
    if((counterDelta > 158) && (counterDelta < 178))
    {
      index = 0;
    }
    dataList[index] = (uint8_t)((counterDelta - 1)/3 + 1 - 12);

    if(index >= 8)
    {
      dataList[29] = crc4_cal(&dataList[2], 6);
      if(dataList[29] == dataList[8] )
      {
        positionValue = dataList[2] * 256 + dataList[3] * 16 + dataList[4];
        positionValue = positionValue * 10;
        timerValue = timerValue + 1;
        //positionValue = 0x123;
        osMessageQueuePut(SENT_CurrentPositionHandle, (void *)&positionValue, 0, 0);
        osThreadYield();
      }
    }

    index++;
  }


  counterOld = counter;
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
}




void HAL_TIM_Task(void * argument)
{
  osStatus_t osStatus;
  void *msg_ptr;
  uint8_t *msg_prio;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

    ModeEventHandle = xEventGroupCreate();
  /* Infinite loop */
  for(;;)
  {
#if 0
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 300);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 400);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 800);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    osDelay(5000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 400);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    osDelay(1000);


    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 300);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 400);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 600);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 800);
    osDelay(5000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 400);
    osDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    osDelay(1000);
#endif
    //osStatus = osMessageQueueGet(MODE_MotorOutHandle, (void *)&pwmMotorOut, msg_prio, 0);
    xEventGroupWaitBits(ModeEventHandle,
                                (0x01) ,
                                pdTRUE,
                                pdTRUE,
                                osWaitForever);
                                
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmMotorOut.pwmMotorOutHigh);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmMotorOut.pwmMotorOutLow);


    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);
    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    //osDelay(2000);
  }
}


/* USER CODE END 1 */
