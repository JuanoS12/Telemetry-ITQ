/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
// Incluye el archivo de cabecera propio para la configuración de USART
#include "usart.h"
// Incluye definiciones específicas de DMA para STM32F4
#include "stm32f4xx_hal_dma.h" // Para DMA_NORMAL, DMA_PRIORITY_HIGH

/* ===========================
   FUNCIONES AUXILIARES DE USUARIO
   =========================== */
/* USER CODE BEGIN 0 */
#include "cmsis_os.h"
#include "stream_buffer.h"
#include "FreeRTOS.h"
#include "task.h"

// GPS RX buffer for DMA (single circular buffer used by HAL_UARTEx_ReceiveToIdle_DMA)
static uint8_t gps_rx_buf[512];
// Overruns counter for diagnostics (incremented in ISR when SB_GPS is full)
static volatile uint32_t gps_stream_overruns_local = 0;

static void GPS_RearmDma(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_rx_buf, sizeof(gps_rx_buf)) != HAL_OK) {
    Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
 * @brief Manejador de errores para USART.
 * 
 * Esta función es un "stub" (plantilla vacía) para que el usuario implemente
 * manejo de errores personalizado en caso de fallos en la comunicación USART.
 * Por ejemplo, se puede agregar registro de errores, recuperación, etc.
 */
void USART_ErrorHandler(void) {
  // TODO: Implementar manejo de errores (registro, recuperación, etc.)
}

/* USER CODE END 0 */

/* ===========================
   VARIABLES GLOBALES
   =========================== */
/**
 * @brief Estructura de manejo para USART1.
 * 
 * UART_HandleTypeDef es una estructura definida por HAL que contiene toda la
 * información necesaria para operar el periférico UART (dirección base, configuración,
 * estado, buffers, etc.).
 */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/**
 * @brief Estructura de manejo para DMA de recepción USART1.
 * 
 * DMA_HandleTypeDef es una estructura definida por HAL que gestiona la transferencia
 * de datos entre periféricos y memoria usando el controlador DMA.
 */
DMA_HandleTypeDef hdma_usart1_rx;

/* ===========================
   FUNCIÓN DE INICIALIZACIÓN DE USART1
   =========================== */
/**
 * @brief Inicializa el periférico USART1 con los parámetros deseados.
 * 
 * Esta función configura la velocidad de transmisión (baudrate), formato de datos,
 * modo de operación, control de flujo y sobre-muestreo. Si la inicialización falla,
 * llama a Error_Handler().
 */
void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  // Sección para inicialización personalizada antes de la configuración estándar
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  // Sección para inicialización personalizada antes de la configuración estándar
  /* USER CODE END USART1_Init 1 */

  // Asigna la instancia de hardware (USART1)
  huart1.Instance = USART1;
  // Configura la velocidad de transmisión (baudrate)
  huart1.Init.BaudRate = 9600;
  // Configura el tamaño de palabra (8 bits)
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  // Configura el número de bits de parada (1)
  huart1.Init.StopBits = UART_STOPBITS_1;
  // Configura la paridad (ninguna)
  huart1.Init.Parity = UART_PARITY_NONE;
  // Configura el modo de operación (transmisión y recepción)
  huart1.Init.Mode = UART_MODE_TX_RX;
  // Desactiva el control de flujo por hardware
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  // Configura el sobre-muestreo (16x)
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  // Inicializa el periférico UART con los parámetros anteriores
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler(); // Si falla, llama a la función de error
  }
  /* USER CODE BEGIN USART1_Init 2 */
  // Sección para inicialización personalizada después de la configuración estándar
  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief Inicializa el periférico USART2 para telemetría hacia PC.
 */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ===========================
   FUNCIÓN DE INICIALIZACIÓN DE MSP (Microcontroller Support Package)
   =========================== */
/**
 * @brief Inicializa los recursos de bajo nivel para USART1.
 * 
 * Esta función configura los pines GPIO para TX/RX, habilita el reloj del periférico,
 * configura el DMA para recepción y habilita la interrupción de USART1.
 * 
 * @param uartHandle Puntero a la estructura de manejo UART.
 */
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  // Estructura para configurar los pines GPIO
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */
    // Sección para inicialización personalizada antes de la configuración estándar
    /* USER CODE END USART1_MspInit 0 */

    // Habilita el reloj para USART1
    __HAL_RCC_USART1_CLK_ENABLE();

    // Habilita el reloj para el puerto GPIOA (donde están los pines TX/RX)
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**
     * Configura los pines GPIO para USART1:
     * PA9  -> USART1_TX (transmisión)
     * PA10 -> USART1_RX (recepción)
     */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;           // Modo alternativo push-pull
    GPIO_InitStruct.Pull = GPIO_NOPULL;               // Sin resistencias de pull-up/down
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;// Velocidad máxima
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;      // Función alternativa USART1
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;           // Selecciona el stream de DMA
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;      // Canal de DMA para USART1_RX
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY; // Dirección: periférico -> memoria
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE; // No incrementa dirección periférico
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;     // Incrementa dirección de memoria
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; // Alineación por byte
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    // Alineación por byte
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;            // Modo normal (no circular)
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH; // Prioridad alta para GPS RX
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE; // Sin FIFO
    // Inicializa el DMA con la configuración anterior
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler(); // Si falla, llama a la función de error
    }

    // Asocia el DMA de recepción al manejador UART
    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    // Configura la prioridad de la interrupción USART1 (nivel 5)
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    // Habilita la interrupción USART1 en el NVIC
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USER CODE BEGIN USART1_MspInit 1 */
    // Sección para inicialización personalizada después de la configuración estándar
    /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */
    /* USER CODE END USART2_MspInit 0 */

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**
     * USART2 se expone en PA2 (TX) y PA3 (RX) hacia el PC.
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN USART2_MspInit 1 */
    /* No se usan interrupciones/DMA para USART2 por ahora. */
    /* USER CODE END USART2_MspInit 1 */
  }
}

/* ===========================
   FUNCIÓN DE DESINICIALIZACIÓN DE MSP
   =========================== */
/**
 * @brief Libera los recursos de bajo nivel para USART1.
 * 
 * Esta función deshabilita el reloj del periférico, libera los pines GPIO,
 * desinicializa el DMA y deshabilita la interrupción de USART1.
 * 
 * @param uartHandle Puntero a la estructura de manejo UART.
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */
    // Sección para desinicialización personalizada antes de la configuración estándar
    /* USER CODE END USART1_MspDeInit 0 */

    // Deshabilita el reloj para USART1
    __HAL_RCC_USART1_CLK_DISABLE();

    /**
     * Libera los pines GPIO usados por USART1:
     * PA9  -> USART1_TX
     * PA10 -> USART1_RX
     */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    // Desinicializa el DMA de recepción
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    // Deshabilita la interrupción USART1 en el NVIC
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    /* USER CODE BEGIN USART1_MspDeInit 1 */
    // Sección para desinicialización personalizada después de la configuración estándar
    /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
    /* USER CODE BEGIN USART2_MspDeInit 0 */
    /* USER CODE END USART2_MspDeInit 0 */

    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USER CODE BEGIN USART2_MspDeInit 1 */
    /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* ===========================
   FUNCIONES DE USUARIO ADICIONALES
   =========================== */
/* USER CODE BEGIN 1 */

/**
 * @brief Start GPS reception using HAL UART "ReceiveToIdle" + DMA.
 *
 * Preconditions / notes:
 * - A StreamBuffer SB_GPS must exist before calling this function.
 *   MX_FREERTOS_Init (freertos.c) creates SB_GPS; call GPS_Start() after that creation.
 * - Verify the DMA stream/channel mapping that CubeMX generated for USART1 RX and match
 *   hdma_usart1_rx.Instance / channel here. If your CubeMX uses a different stream, adjust.
 */
extern StreamBufferHandle_t SB_GPS; // defined/created in freertos.c
void GPS_Start(void)
{
  // Ensure SB_GPS exists
  configASSERT(SB_GPS != NULL);

  // Start HAL "Receive to Idle" using DMA. This fills gps_rx_buf and triggers HAL_UARTEx_RxEventCallback
  GPS_RearmDma();
}

/**
 * @brief UART RxEvent callback invoked by HAL when DMA detect "Idle" (end of frame).
 *        This implementation sends the received bytes into the SB_GPS stream buffer from ISR.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Send bytes to stream buffer from ISR. Returns number actually written.
    size_t sent = xStreamBufferSendFromISR(SB_GPS, gps_rx_buf, (size_t)Size, &xHigherPriorityTaskWoken);
    if (sent != Size) {
      // Not all bytes fit: increment overrun counter for diagnostics
      gps_stream_overruns_local += (Size - sent);
    }
    // Re-arm reception for next packet
    GPS_RearmDma();
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

/**
 * @brief UART error callback: attempt to re-arm the RX DMA to continue receiving.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    // Try to recover reception: re-arm DMA receive-to-idle
    GPS_RearmDma();
  }
}

// Aquí puedes agregar funciones adicionales para USART (envío, recepción, etc.)

/* USER CODE END 1 */
