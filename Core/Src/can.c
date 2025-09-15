/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

// CAN error handler stub for improved error management
void CAN_ErrorHandler(void) {
  // TODO: Implement error handling (logging, recovery, etc.)
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
    // Accept-all filter -> FIFO0
    CAN_FilterTypeDef f = {0};
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0; f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) { Error_Handler(); }

    // Start CAN and enable RX/error IRQs
    if (HAL_CAN_Start(&hcan1) != HAL_OK) { Error_Handler(); }
    if (HAL_CAN_ActivateNotification(&hcan1,
          CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF |
          CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE) != HAL_OK) { Error_Handler(); }

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */
    /* USER CODE BEGIN CAN1_MspInit 1 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);   // FreeRTOS-safe (>= MAX_SYSCALL_PRI)
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 1);   // status/error (optional but useful)
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    /* USER CODE END CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 1);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */
  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE END CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

// Minimal TX helper: standard 11-bit ID, 8 bytes
HAL_StatusTypeDef CAN1_SendStd(uint16_t id, const uint8_t data[8]) {
  CAN_TxHeaderTypeDef tx = {0}; uint32_t mb;
  tx.StdId=id; tx.IDE=CAN_ID_STD; tx.RTR=CAN_RTR_DATA; tx.DLC=8;
  return HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)data, &mb);
}
// Called when RX FIFO0 has a pending frame
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx; uint8_t d[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, d) == HAL_OK) {
    // TODO: push to a FreeRTOS queue if/when you start receiving from ESP32
    // For now, drop or set a flag so you can see activity in a debugger.
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t err = HAL_CAN_GetError(hcan);
  // TODO: increment counters / log. AutoBusOff is enabled, so HAL will recover.
  (void)err;
}


/* USER CODE END 1 */
