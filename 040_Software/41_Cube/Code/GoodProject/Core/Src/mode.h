/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mode.h
  * @brief   This file contains all the function prototypes for
  *          the mode.c file
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
#ifndef __MODE_H__
#define __MODE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MODE_Init(void);

/* USER CODE BEGIN Prototypes */
void MODE_Task(void *argument);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MODE_H__ */


