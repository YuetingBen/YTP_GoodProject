/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */


#define EEPROM_BASE_ADDRESS 0x0000u
#define EEPROM_CHECKSUM_ADDRESS 0x00F0u

typedef struct
{
  uint16_t firstPosition;
  uint16_t secondPosition;
}EE_POSITION_S;



#pragma pack (1)
typedef struct
{
  EE_POSITION_S eePosition[8];
  uint16_t testDataU16;
  uint8_t testDataU8;
  uint8_t testDataU8Arrat[5];
}EE_DATA_TYPE_S;
#pragma pack ()

typedef enum
{
  EE_IDEL,
  EE_READ_REQUEST,
  EE_WRITE_REQUEST,
}EE_STATUS_E;

typedef struct
{
  EE_ITEM_E eeName;
  EE_STATUS_E eeStatus;
  uint8_t *dataBufferPtr;
  uint8_t *defaultDataBufferPtr;
  uint16_t dataLen;
  uint16_t offsetAdd;
}EE_HANDEL_TYPE_S;


static EE_DATA_TYPE_S eeDefaultData = {\
  {\
  {0x0541, 0x0431},
  {0x0632, 0x0512},
  {0x0763, 0x0433},
  {0x0894, 0x0634},
  {0x0955, 0x0335},
  {0x0A76, 0x0436},
  {0x0B37, 0x0937},
  {0x0C08, 0x0838}},
  0xabcd,
  0x86,
  {0x1a, 0x1b, 0x1c, 0x1d, 0x1e}
  };

static EE_DATA_TYPE_S eeMirrorData = {0};


static EE_HANDEL_TYPE_S eeData[EE_MODE_MAX_NUM_ITEM] = {\
  /* EE_MODE0_POSITION */
  {EE_MODE0_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE0_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE0_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE0_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE1_POSITION */
  {EE_MODE1_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE1_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE1_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE1_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE2_POSITION */
  {EE_MODE2_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE2_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE2_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE2_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE3_POSITION */
  {EE_MODE3_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE3_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE3_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE3_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE4_POSITION */
  {EE_MODE4_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE4_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE4_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE4_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE5_POSITION */
  {EE_MODE5_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE5_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE5_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE5_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE6_POSITION */
  {EE_MODE6_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE6_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE6_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE6_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE7_POSITION */
  {EE_MODE7_POSITION, EE_IDEL, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE7_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE7_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE7_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_A */
  {EE_A, EE_IDEL, 
  (uint8_t *)&eeMirrorData.testDataU16, 
  (uint8_t *)&eeDefaultData.testDataU16, 
  sizeof(uint16_t), 
  (uint16_t)((uint8_t *)&eeDefaultData.testDataU16 - (uint8_t *)&eeDefaultData)},

  /* EE_B */
  {EE_B, EE_IDEL, 
  (uint8_t *)&eeMirrorData.testDataU8, 
  (uint8_t *)&eeDefaultData.testDataU8, 
  sizeof(uint8_t), 
  (uint16_t)((uint8_t *)&eeDefaultData.testDataU8 - (uint8_t *)&eeDefaultData)},


  /* EE_C */
  {EE_C, EE_IDEL, 
  (uint8_t *)&eeMirrorData.testDataU8Arrat, 
  (uint8_t *)&eeDefaultData.testDataU8Arrat, 
  5, 
  (uint16_t)((uint8_t *)&eeDefaultData.testDataU8Arrat - (uint8_t *)&eeDefaultData)},
  };

void EEPROM_Task(void *argument);
static void EEPROM_DataInitRead(void);
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void EEPROM_WriteRequest(EE_ITEM_E name,  uint8_t *pData)
{
  EE_HANDEL_TYPE_S *itemPtr = &eeData[name];
  uint8_t writeFlag = RESET;
  uint8_t i;
  for(i = 0; i < itemPtr->dataLen; i++)
  {
    if(pData[i] != itemPtr->dataBufferPtr[i])
    {
      writeFlag = SET;
    }
  }
  if(RESET != writeFlag)
  {
    itemPtr->eeStatus = EE_WRITE_REQUEST;
    for(i = 0; i < itemPtr->dataLen; i++)
    {
      itemPtr->dataBufferPtr[i] = pData[i];
    }
  }
}


void EEPROM_ReadRequest(EE_ITEM_E name,  uint8_t *pData)
{
  EE_HANDEL_TYPE_S *itemPtr = &eeData[name];
  uint8_t i;

  for(i = 0; i < itemPtr->dataLen; i++)
  {
    pData[i] = itemPtr->dataBufferPtr[i];
  }
}


static void EEPROM_Read(uint16_t MemAddress,  uint8_t *pData, uint16_t Size)
{
  while(HAL_OK != HAL_I2C_Mem_Read(&hi2c2, 0xA1, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 0xFF));
  osDelay(10);
}

static void EEPROM_Write(uint16_t MemAddress,  uint8_t *pData, uint16_t Size)
{
  while(HAL_OK != HAL_I2C_Mem_Write(&hi2c2, 0xA0, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, 0xFFF));
  osDelay(10);
}

static uint8_t EEPROM_CheckSum(uint8_t *pData, uint16_t size)
{
  uint16_t checkSumRet;
  uint8_t i;

  checkSumRet = 0;
  for(i = 0; i < size; i++)
  {
    checkSumRet = (uint8_t)((uint8_t)checkSumRet + (uint8_t)pData[i]);
  }

  checkSumRet = (uint8_t)(0xFF -(0xFF & checkSumRet));
  return((uint8_t)checkSumRet);
}

static void EEPROM_DataInitRead(void)
{
  static uint8_t checkSumReadArray[EE_MODE_MAX_NUM_ITEM] = {0};
  static EE_HANDEL_TYPE_S *itemPtr = eeData;
  uint8_t i;
  uint8_t j;
  uint8_t checkSum;
  uint8_t dataReadBuffer[10];
  /* Read out checksum */
  EEPROM_Read(EEPROM_CHECKSUM_ADDRESS, (uint8_t *)&checkSumReadArray, sizeof(EE_DATA_TYPE_S));
  /* Calculate checkSum based on data buffer */
  for(i = 0; i < EE_MODE_MAX_NUM_ITEM; i++)
  {
    EEPROM_Read(EEPROM_BASE_ADDRESS + itemPtr->offsetAdd, (uint8_t *)dataReadBuffer,  itemPtr->dataLen);

    checkSum = EEPROM_CheckSum(dataReadBuffer, itemPtr->dataLen);
    /* If the checksum correct, save the EEPROM data to dataBufferPtr(EEPROM Mirror buffer) */
    if(checkSum == checkSumReadArray[i])
    {
      for(j = 0; j < itemPtr->dataLen; j++)
      {
        itemPtr->dataBufferPtr[j] = dataReadBuffer[j];
      }
    }
    else
    {
      for(j = 0; j < itemPtr->dataLen; j++)
      {
        itemPtr->dataBufferPtr[j] = itemPtr->defaultDataBufferPtr[j];
        itemPtr->eeStatus = EE_WRITE_REQUEST;
      }
    }
    itemPtr++;
  }
}

static void EEPROM_Handel(void)
{
  static EE_HANDEL_TYPE_S *itemPtr = eeData;
  static uint8_t index;
  uint8_t j;
  uint8_t checkSum;

  if(EE_MODE_MAX_NUM_ITEM >index)
  {
    if(EE_WRITE_REQUEST == itemPtr->eeStatus)
    {
      checkSum = EEPROM_CheckSum(itemPtr->dataBufferPtr, itemPtr->dataLen);

      EEPROM_Write(itemPtr->offsetAdd, itemPtr->dataBufferPtr, itemPtr->dataLen);
      itemPtr->eeStatus = EE_IDEL;

      EEPROM_Write((EEPROM_CHECKSUM_ADDRESS + index), (uint8_t *)&checkSum, 1);
    }
    index++;
    itemPtr++;
  }
  else
  {
    index = 0;
    itemPtr = eeData;
  }
}


void EEPROM_Task(void *argument)
{
  static uint8_t dataTemp[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  EEPROM_DataInitRead();
  for(;;)
  {

    EEPROM_Handel();
    osDelay(100);
  }
}
/* USER CODE END 1 */
