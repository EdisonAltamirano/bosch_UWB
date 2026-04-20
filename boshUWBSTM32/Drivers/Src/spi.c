/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
#include <string.h>
/* USER CODE END 0 */

SPI_HandleTypeDef hspi6;

/* SPI6 init function */
void MX_SPI6_Init(void)
{

  /* USER CODE BEGIN SPI6_Init 0 */

  /* USER CODE END SPI6_Init 0 */

  /* USER CODE BEGIN SPI6_Init 1 */

  /* USER CODE END SPI6_Init 1 */
  hspi6.Instance = SPI6;
  hspi6.Init.Mode = SPI_MODE_MASTER;
  hspi6.Init.Direction = SPI_DIRECTION_2LINES;
  hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi6.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi6.Init.NSS = SPI_NSS_SOFT;
  hspi6.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi6.Init.CRCPolynomial = 0x0;
  hspi6.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_04DATA;
  hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI6_Init 2 */

  /* USER CODE END SPI6_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(spiHandle->Instance==SPI6)
  {
  /* USER CODE BEGIN SPI6_MspInit 0 */

  /* USER CODE END SPI6_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* SPI6 clock enable */
    __HAL_RCC_SPI6_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI6 GPIO Configuration
    PA5     ------> SPI6_SCK
    PA6     ------> SPI6_MISO
    PB5     ------> SPI6_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI6_MspInit 1 */

  /* USER CODE END SPI6_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI6)
  {
  /* USER CODE BEGIN SPI6_MspDeInit 0 */

  /* USER CODE END SPI6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI6_CLK_DISABLE();

    /**SPI6 GPIO Configuration
    PA5     ------> SPI6_SCK
    PA6     ------> SPI6_MISO
    PB5     ------> SPI6_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5);

  /* USER CODE BEGIN SPI6_MspDeInit 1 */

  /* USER CODE END SPI6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint32_t nucleo_spi_master_transfer_blocking(const uint8_t * send_buffer, uint8_t * receive_buffer, uint16_t transfer_byte_count, uint32_t timeout)
{
	if(((send_buffer==NULL) && (receive_buffer==NULL)) || (transfer_byte_count==0)) return 0x01;
	if((send_buffer!=NULL) && (receive_buffer==NULL))
	{
		return HAL_SPI_Transmit(&hspi6, send_buffer, transfer_byte_count, timeout);
	}

	if((send_buffer==NULL) && (receive_buffer!=NULL))
	{

		return HAL_SPI_Receive(&hspi6, receive_buffer, transfer_byte_count, timeout);
	}

	if((send_buffer!=NULL) && (receive_buffer!=NULL))
	{
		return HAL_SPI_TransmitReceive(&hspi6, send_buffer, receive_buffer, transfer_byte_count, timeout);
	}

}

uint32_t nucleo_spi_write(const uint8_t * send_buffer, uint16_t transfer_byte_count, uint32_t timeout)
{

	uint32_t ret_val;
//	uint16_t crc;
//    crc = Radar_UtilsCalcCRC(gphscaHostCommSpiBuffer.tx, 254);
//
//    gphscaHostCommSpiBuffer.tx[254]=crc& 0xFF;
//    gphscaHostCommSpiBuffer.tx[255]=crc>>8 & 0xFF;


    //ToDo: Add SPI Comm
    HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_RESET);    /* Clear Output on port (CS LOW) */
//    while(SPI_GetStateRDYLine()==1u);
    /* Start the blocking transfer */
    ret_val = nucleo_spi_master_transfer_blocking(send_buffer, NULL, transfer_byte_count, timeout);
    HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_SET);         /* Clear Output on port (CS HIGH) */
//    while(SPI_GetStateRDYLine()==0u);

    return ret_val;
}


uint32_t nucleo_spi_read(uint8_t * receive_buffer, uint16_t transfer_byte_count, uint32_t timeout)
{
	uint32_t ret_val;
//    while(SPI_GetStateINTLine()==1u);

	memset(receive_buffer,0u,transfer_byte_count);
	HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_RESET);        /* Clear Output on port (CS LOW) */

//	while(SPI_GetStateRDYLine()==1u);
	ret_val = nucleo_spi_master_transfer_blocking(NULL, receive_buffer, transfer_byte_count, timeout);

	HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_SET);          /* Clear Output on port (CS HIGH) */
//	while(SPI_GetStateRDYLine()==0u);

	return ret_val;

}

uint32_t nucleo_spi_read_both(const uint8_t * send_buffer, uint8_t * receive_buffer, uint16_t transfer_byte_count, uint32_t timeout)
{
	uint32_t ret_val;
//    while(SPI_GetStateINTLine()==1u);

	memset(receive_buffer,0xFFu,transfer_byte_count);
	HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_RESET);        /* Clear Output on port (CS LOW) */

//	while(SPI_GetStateRDYLine()==1u);
	ret_val = nucleo_spi_master_transfer_blocking(send_buffer, receive_buffer, transfer_byte_count, timeout);

	HAL_GPIO_WritePin(SPI6_CS_GPIO_Port, SPI6_CS_Pin, GPIO_PIN_SET);          /* Clear Output on port (CS HIGH) */
//	while(SPI_GetStateRDYLine()==0u);

	return ret_val;

}
/* USER CODE END 1 */

