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

/* ===========================
   INCLUSIÓN DE CABECERAS
   =========================== */
// Incluye el archivo de cabecera propio para la configuración de DMA
#include "dma.h"

/* ===========================
   FUNCIONES AUXILIARES DE USUARIO
   =========================== */
/* USER CODE BEGIN 0 */
#include "stm32f4xx_hal.h" // Incluye definiciones HAL para STM32F4

/**
 * @brief Manejador de errores para DMA.
 * 
 * Esta función es un "stub" (plantilla vacía) para que el usuario implemente
 * manejo de errores personalizado en caso de fallos en transferencias DMA.
 * Por ejemplo, se puede agregar registro de errores, recuperación, etc.
 */
void DMA_ErrorHandler(void) {
  // TODO: set a fault flag, count errors, maybe trigger I2C/UART recoveries
}
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */
// Aquí puedes agregar variables globales o funciones relacionadas con DMA
/* USER CODE END 1 */

/**
  * @brief Habilita el reloj del controlador DMA y configura las interrupciones.
  * 
  * Esta función inicializa el hardware DMA del STM32, habilitando el reloj
  * y configurando las prioridades y habilitación de las interrupciones para
  * los streams usados por USART1 (RX y TX).
  * 
  * El DMA permite transferencias de datos entre periféricos y memoria sin
  * intervención directa del CPU, lo que mejora el rendimiento y reduce la carga.
  */
void MX_DMA_Init(void)
{

     /* DMA controller clock enable */
      __HAL_RCC_DMA2_CLK_ENABLE(); // Habilita el reloj para el controlador DMA2

      /* ---- USART1_RX usa DMA2 Stream2 / Channel 4 ---- */
      // Configura la prioridad de la interrupción para el stream de recepción
      HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);   // >= MAX_SYSCALL_PRI (FreeRTOS safe)
      HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);           // Habilita la interrupción

      /* ---- USART1_TX (placeholder) usa DMA2 Stream7 / Channel 4 ---- */
      // Configura la prioridad de la interrupción para el stream de transmisión
      HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0);   // enabled now, you can use later
      HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);           // Habilita la interrupción

      // Nota: Si usas otros periféricos con DMA (ADC, I2C, etc.), debes habilitar sus streams aquí
}

/* USER CODE BEGIN 2 */
/**
 * Aquí puedes agregar funciones adicionales para mejorar el manejo de DMA,
 * como callbacks personalizados, inicialización de otros streams, o rutinas
 * para recuperación en caso de error.
 */
/* USER CODE END 2 */

