/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#pragma pack (1)
typedef struct
{
  uint8_t masterModeCommand;
  uint16_t masterTargetPos;
}LIN_MASTER_MESSAGE_S;
#pragma pack ()


typedef union
{
  LIN_MASTER_MESSAGE_S message;
  uint8_t dataArray[8];
}LIN_MASTER_MESSAGE_U;

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
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void HAL_UART_Task(void * argument);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

