/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dma.c
  * @brief   This file provides code for the configuration
  *          of all the requested memory to memory DMA transfers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"

/* USER CODE BEGIN 0 */
#include "stm32f4xx_hal.h"

// DMA error handler stub for improved error management
void DMA_ErrorHandler(void) {
	 // TODO: set a fault flag, count errors, maybe trigger I2C/UART recoveries
	  (void)hdma;
}

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{

	 /* DMA controller clock enable */
	  __HAL_RCC_DMA2_CLK_ENABLE();

	  /* ---- USART1_RX uses DMA2 Stream2 / Channel 4 ---- */
	  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);   // >= MAX_SYSCALL_PRI (FreeRTOS safe)
	  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	  /* ---- USART1_TX (placeholder) uses DMA2 Stream7 / Channel 4 ---- */
	  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0);   // enabled now, you can use later
	  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/* USER CODE BEGIN 2 */

// Additional DMA functions and improvements can be added here

/* USER CODE END 2 */

