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

/* ===========================
   INCLUSIÓN DE CABECERAS
   =========================== */
// Incluye el archivo de cabecera propio para la configuración de CAN
#include "can.h"

/* USER CODE BEGIN 0 */

/* Added: include FreeRTOS headers so TickType_t, pdMS_TO_TICKS, xTaskGetTickCount
   and vTaskDelay are declared. This fixes the unknown type / implicit function errors. */
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END 0 */

/* ===========================
   FUNCIONES AUXILIARES DE USUARIO
   =========================== */
/* USER CODE BEGIN 0 */

/**
 * @brief Manejador de errores para CAN.
 * 
 * Esta función es un "stub" (plantilla vacía) para que el usuario implemente
 * manejo de errores personalizado en caso de fallos en la comunicación CAN.
 * Por ejemplo, se puede agregar registro de errores, recuperación, etc.
 */
void CAN_ErrorHandler(void) {
  // TODO: Implementar manejo de errores (registro, recuperación, etc.)
}

/* USER CODE END 0 */

/* ===========================
   VARIABLES GLOBALES
   =========================== */
/**
 * @brief Estructura de manejo para CAN1.
 * 
 * CAN_HandleTypeDef es una estructura definida por HAL que contiene toda la
 * información necesaria para operar el periférico CAN (dirección base, configuración,
 * estado, buffers, etc.).
 */
CAN_HandleTypeDef hcan1;

/* ===========================
   FUNCIÓN DE INICIALIZACIÓN DE CAN1
   =========================== */
/**
 * @brief Inicializa el periférico CAN1 con los parámetros deseados.
 * 
 * Esta función configura la velocidad de comunicación, modo de operación,
 * sincronización, y activa características como recuperación automática y
 * retransmisión. También configura el filtro de aceptación y activa las
 * interrupciones necesarias.
 */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  // Sección para inicialización personalizada antes de la configuración estándar
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  // Sección para inicialización personalizada antes de la configuración estándar
  /* USER CODE END CAN1_Init 1 */

  // Asigna la instancia de hardware (CAN1)
  hcan1.Instance = CAN1;
  // Configura el preescalador para la velocidad del bus CAN
  hcan1.Init.Prescaler = 5;
  // Configura el modo de operación (normal)
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  // Configura el ancho de salto de sincronización (2 time quanta)
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  // Configura el segmento de tiempo 1 (13 time quanta)
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  // Configura el segmento de tiempo 2 (2 time quanta)
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  // Desactiva el modo de tiempo disparado
  hcan1.Init.TimeTriggeredMode = DISABLE;
  // Activa la recuperación automática en caso de bus-off
  hcan1.Init.AutoBusOff = ENABLE;
  // Activa el despertar automático
  hcan1.Init.AutoWakeUp = ENABLE;
  // Activa la retransmisión automática de mensajes
  hcan1.Init.AutoRetransmission = ENABLE;
  // Desactiva el bloqueo de FIFO de recepción
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  // Activa la prioridad de FIFO de transmisión
  hcan1.Init.TransmitFifoPriority = ENABLE;
  // Inicializa el periférico CAN con los parámetros anteriores
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler(); // Si falla, llama a la función de error
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // ensure filter configured and notifications enabled (already in file); must be called after HAL_CAN_Init
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = 0; f.FilterIdLow = 0;
  f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterActivation = ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) { Error_Handler(); }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) { Error_Handler(); }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK) { Error_Handler(); }
  /* USER CODE END CAN1_Init 2 */

}

/* ===========================
   FUNCIÓN DE INICIALIZACIÓN DE MSP (Microcontroller Support Package)
   =========================== */
/**
 * @brief Inicializa los recursos de bajo nivel para CAN1.
 * 
 * Esta función configura los pines GPIO para TX/RX, habilita el reloj del periférico,
 * y habilita las interrupciones necesarias para CAN1.
 * 
 * @param canHandle Puntero a la estructura de manejo CAN.
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */
  // Sección para inicialización personalizada antes de la configuración estándar
  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**
     * Configura los pines GPIO para CAN1:
     * PA11 -> CAN1_RX (recepción)
     * PA12 -> CAN1_TX (transmisión)
     */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;           // Modo alternativo push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;               // Sin resistencias de pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;// Velocidad máxima
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;        // Función alternativa CAN1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */
    // Configura la prioridad y habilita las interrupciones para CAN1
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);   // Prioridad compatible con FreeRTOS
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 1);   // Interrupción de estado/error
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE END CAN1_MspInit 1 */
  }
}

/* ===========================
   FUNCIÓN DE DESINICIALIZACIÓN DE MSP
   =========================== */
/**
 * @brief Libera los recursos de bajo nivel para CAN1.
 * 
 * Esta función deshabilita el reloj del periférico, libera los pines GPIO,
 * y deshabilita las interrupciones de CAN1.
 * 
 * @param canHandle Puntero a la estructura de manejo CAN.
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */
  // Sección para desinicialización personalizada antes de la configuración estándar
  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**
     * Libera los pines GPIO usados por CAN1:
     * PA11 -> CAN1_RX
     * PA12 -> CAN1_TX
     */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */
    // Deshabilita las interrupciones de CAN1
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* ===========================
   FUNCIONES DE USUARIO ADICIONALES
   =========================== */
/* USER CODE BEGIN 1 */

/* Improved transmit helper: retries for a short bounded time if mailboxes busy.
   Returns HAL_OK on success, HAL_ERROR on failure after timeout. */
HAL_StatusTypeDef CAN1_SendStd(uint16_t id, const uint8_t data[8]) {
  CAN_TxHeaderTypeDef tx = {0};
  uint32_t mailbox;
  tx.StdId = id;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;

  const TickType_t timeout = pdMS_TO_TICKS(5); // try for up to 5 ms
  TickType_t start = xTaskGetTickCount();
  while (HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)data, &mailbox) != HAL_OK) {
    if ((xTaskGetTickCount() - start) > timeout) {
      return HAL_ERROR;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return HAL_OK;
}

/* Provide a simple can_start() wrapper called from main to ensure CAN is started
   and notifications are active. This resolves the implicit declaration warning
   and gives a single entry to (re)start CAN at runtime. */
void can_start(void)
{
  CAN_FilterTypeDef f = {0};
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterIdHigh = 0; f.FilterIdLow = 0;
  f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) { Error_Handler(); }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) { Error_Handler(); }
  if (HAL_CAN_ActivateNotification(&hcan1,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK) { Error_Handler(); }
}

/* USER CODE END 1 */
