/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN Private defines */
typedef enum
{
  EE_MODE0_POSITION,
  EE_MODE1_POSITION,
  EE_MODE2_POSITION,
  EE_MODE3_POSITION,
  EE_MODE4_POSITION,
  EE_MODE5_POSITION,
  EE_MODE6_POSITION,
  EE_MODE7_POSITION,
  EE_MODE8_POSITION,
  EE_A,
  EE_B,
  EE_C,
  EE_MODE_MAX_NUM_ITEM
}EE_ITEM_E;
/* USER CODE END Private defines */

void MX_I2C2_Init(void);

/* USER CODE BEGIN Prototypes */
extern void EEPROM_Task(void *argument);
extern void EEPROM_WriteRequest(EE_ITEM_E name,  uint8_t *pData);
extern void EEPROM_ReadRequest(EE_ITEM_E name,  uint8_t *pData);
extern void EEPROM_GetPosition(EE_ITEM_E name,  uint16_t *firstPos, uint16_t *secondPos);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

