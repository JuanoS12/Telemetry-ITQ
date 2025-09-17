/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
// Incluye el archivo de cabecera propio para la configuración de GPIO
#include "gpio.h"

/* ===========================
   FUNCIONES AUXILIARES DE USUARIO
   =========================== */
/* USER CODE BEGIN 0 */

/**
 * @brief Manejador de errores para GPIO.
 * 
 * Esta función es un "stub" (plantilla vacía) para que el usuario implemente
 * manejo de errores personalizado en caso de fallos en la configuración o uso
 * de los pines GPIO. Por ejemplo, se puede agregar registro de errores, recuperación, etc.
 */
void GPIO_ErrorHandler(void) {
  // TODO: Implementar manejo de errores (registro, recuperación, etc.)
}

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
// Aquí puedes agregar variables globales o funciones relacionadas con GPIO
/* USER CODE END 1 */

/**
  * @brief Inicializa y configura los pines GPIO usados en el proyecto.
  * 
  * Esta función activa el reloj de los puertos GPIO necesarios y configura
  * cada pin según su función: entrada, salida, analógico, interrupción, etc.
  * 
  * El tipo GPIO_InitTypeDef es una estructura definida por HAL que permite
  * especificar el número de pin, modo de operación, resistencias de pull-up/down,
  * velocidad, y función alternativa si aplica.
  */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0}; // Estructura para configuración de pines

  /* ===========================
     HABILITACIÓN DE RELOJES GPIO
     =========================== */
  // Antes de usar cualquier pin, se debe habilitar el reloj del puerto correspondiente
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Habilita el reloj para el puerto GPIOA
  __HAL_RCC_GPIOB_CLK_ENABLE(); // Habilita el reloj para el puerto GPIOB

  /* ===========================
     CONFIGURACIÓN DE PINES DE ENTRADA
     =========================== */
  // Configura los pines PA6 y PA7 como entradas digitales sin resistencias de pull-up/down
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;         // Selecciona los pines PA6 y PA7
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;              // Modo de entrada digital
  GPIO_InitStruct.Pull = GPIO_NOPULL;                  // Sin resistencias internas
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);              // Aplica la configuración al puerto A

  // Configura los pines PB0 y PB1 como entradas digitales sin resistencias de pull-up/down
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;         // Selecciona los pines PB0 y PB1
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;              // Modo de entrada digital
  GPIO_InitStruct.Pull = GPIO_NOPULL;                  // Sin resistencias internas
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);              // Aplica la configuración al puerto B

  /* ===========================
     NOTA SOBRE OTRAS CONFIGURACIONES
     =========================== */
  // Si necesitas configurar pines como salidas, analógicos, con interrupciones, etc.,
  // debes agregar bloques similares aquí usando GPIO_InitStruct y HAL_GPIO_Init.
  // Ejemplo para salida:
  // GPIO_InitStruct.Pin = GPIO_PIN_X;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOX, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
/**
 * Aquí puedes agregar funciones adicionales para mejorar el manejo de GPIO,
 * como rutinas para leer o escribir pines, inicializar LEDs, botones, o
 * configurar interrupciones externas (EXTI).
 */
/* USER CODE END 2 */
