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
  EE_A,
  EE_B,
  EE_C,
  EE_MODE_MAX_NUM_ITEM
}EE_ITEM_E;

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
  {0x1001, 0x1111},
  {0x2002, 0x2222},
  {0x3003, 0x3333},
  {0x4004, 0x4444},
  {0x5005, 0x5555},
  {0x6006, 0x6666},
  {0x7007, 0x7777},
  {0x8008, 0x8888}},
  0x4567,
  0x88,
  {0xaa, 0xbb, 0xcc, 0xdd, 0xee}
  };

static EE_DATA_TYPE_S eeMirrorData = {0};


static EE_HANDEL_TYPE_S eeData[EE_MODE_MAX_NUM_ITEM] = {\
  /* EE_MODE0_POSITION */
  {EE_MODE0_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE0_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE0_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE0_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE1_POSITION */
  {EE_MODE1_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE1_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE1_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE1_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE2_POSITION */
  {EE_MODE2_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE2_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE2_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE2_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE3_POSITION */
  {EE_MODE3_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE3_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE3_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE3_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE4_POSITION */
  {EE_MODE4_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE4_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE4_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE4_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE5_POSITION */
  {EE_MODE5_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE5_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE5_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE5_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE6_POSITION */
  {EE_MODE6_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE6_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE6_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE6_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_MODE7_POSITION */
  {EE_MODE7_POSITION, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.eePosition[EE_MODE7_POSITION], 
  (uint8_t *)&eeDefaultData.eePosition[EE_MODE7_POSITION], 
  sizeof(EE_POSITION_S), 
  (uint16_t)((uint8_t *)&eeDefaultData.eePosition[EE_MODE7_POSITION] - (uint8_t *)&eeDefaultData)},

  /* EE_A */
  {EE_A, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.testDataU16, 
  (uint8_t *)&eeDefaultData.testDataU16, 
  sizeof(uint16_t), 
  (uint16_t)((uint8_t *)&eeDefaultData.testDataU16 - (uint8_t *)&eeDefaultData)},

  /* EE_B */
  {EE_B, EE_WRITE_REQUEST, 
  (uint8_t *)&eeMirrorData.testDataU8, 
  (uint8_t *)&eeDefaultData.testDataU8, 
  sizeof(uint8_t), 
  (uint16_t)((uint8_t *)&eeDefaultData.testDataU8 - (uint8_t *)&eeDefaultData)},


  /* EE_C */
  {EE_C, EE_WRITE_REQUEST, 
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
  return((uint8_t)checkSumRet);
}

static void EEPROM_DataInitRead(void)
{
  static EE_DATA_TYPE_S eeReaddata;
  static uint8_t checkSumReadArray[EE_MODE_MAX_NUM_ITEM] = {0};
  static uint8_t eeWritedataList[8] = {0};
  uint8_t i;
  uint8_t j;
  static uint8_t checkSumValue;
  static EE_HANDEL_TYPE_S *itemPtrTemp = eeData;

  static uint8_t dataPtrAffay[10];
  
  EEPROM_Read(EEPROM_CHECKSUM_ADDRESS, (uint8_t *)&checkSumReadArray, sizeof(EE_DATA_TYPE_S));

  /* CheckSum check*/
  for(i = 0; i < EE_MODE_MAX_NUM_ITEM; i++)
  {
    EEPROM_Read(EEPROM_BASE_ADDRESS + itemPtrTemp->offsetAdd, (uint8_t *)dataPtrAffay,  itemPtrTemp->dataLen);

    checkSumValue = EEPROM_CheckSum(dataPtrAffay, itemPtrTemp->dataLen);
    /*
    for(j = 0; j < itemPtr->dataLen; j++)
    {
      checkSum = checkSum + dataPtrAffay[j];
    }
    */
    if(checkSumValue == checkSumReadArray[i])
    {
      for(j = 0; j < itemPtrTemp->dataLen; j++)
      {
        itemPtrTemp->dataBufferPtr[j] = dataPtrAffay[j];
      }
    }
    itemPtrTemp++;
  }
}

static void EEPROM_Handel(void)
{
  uint8_t i;
  uint8_t j;
  static EE_HANDEL_TYPE_S *itemPtr;
  uint8_t checkSum;

  itemPtr = eeData;
  for(i = 0; i < EE_MODE_MAX_NUM_ITEM; i++)
  {
    if(EE_WRITE_REQUEST == itemPtr->eeStatus)
    {
      checkSum = 0;
      checkSum = EEPROM_CheckSum(itemPtr->defaultDataBufferPtr, itemPtr->dataLen);

      EEPROM_Write(itemPtr->offsetAdd, itemPtr->defaultDataBufferPtr, itemPtr->dataLen);
      itemPtr->eeStatus = EE_IDEL;

      EEPROM_Write((EEPROM_CHECKSUM_ADDRESS + i), (uint8_t *)&checkSum, 1);
    }
    itemPtr++;
  }
}


void EEPROM_Task(void *argument)
{
/*
  static uint8_t checkSumReadArray[EE_MODE_MAX_NUM_ITEM] = {0};
  osDelay(10);
  HAL_I2C_Mem_Read(&hi2c2, 0xA0, EEPROM_CHECKSUM_ADDRESS, I2C_MEMADD_SIZE_8BIT, (uint8_t *)checkSumReadArray, EE_MODE_MAX_NUM_ITEM, 0xFFFF);
  osDelay(10);
*/
  static uint8_t dataTemp[10] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
  EEPROM_DataInitRead();
  for(;;)
  {
    //EEPROM_Write(0, (uint8_t *)dataTemp, 8);
    EEPROM_Handel();
    osDelay(1000);
  }
}
/* USER CODE END 1 */
