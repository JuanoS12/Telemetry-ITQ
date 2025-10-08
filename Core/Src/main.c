/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
// Archivos de cabecera generados por CubeMX y HAL
#include "main.h"         // Definiciones globales del proyecto (pines, periféricos, prototipos)
#include "cmsis_os.h"     // API de FreeRTOS (sistema operativo en tiempo real)
#include "can.h"          // Inicialización y manejo de CAN
#include "dma.h"          // Inicialización y manejo de DMA
#include "i2c.h"          // Inicialización y manejo de I2C
#include "usart.h"        // Inicialización y manejo de UART
#include "gpio.h"         // Inicialización y manejo de GPIO

/* ===========================
   INCLUSIONES ADICIONALES
   =========================== */
/* USER CODE BEGIN Includes */
#include <stdio.h>        // Funciones estándar de entrada/salida (printf)
#include <assert.h>       // Macros para comprobaciones (ASSERT)
#include "stream_buffer.h"// FreeRTOS: buffers de flujo para comunicación entre tareas
/* USER CODE END Includes */

/* ===========================
   TIPOS DE DATOS PRIVADOS
   =========================== */
/* USER CODE BEGIN PTD */
// Aquí puedes definir tus propios tipos de datos (struct, enum, etc.)
/* USER CODE END PTD */

/* ===========================
   DEFINICIONES DE MACROS
   =========================== */
/* USER CODE BEGIN PD */
// Aquí puedes definir macros para constantes, configuraciones, etc.
/* USER CODE END PD */

/* ===========================
   MACROS PRIVADAS
   =========================== */
/* USER CODE BEGIN PM */
// Macro para imprimir mensajes de log por UART
#define LOG(msg)    printf("[LOG] %s\n", msg)
// Macro para verificar expresiones y detener el programa si son falsas
#define ASSERT(expr)    assert(expr)
/* USER CODE END PM */

/* ===========================
   VARIABLES PRIVADAS
   =========================== */
/* USER CODE BEGIN PV */
// Aquí puedes definir variables globales privadas para este archivo

extern StreamBufferHandle_t SB_GPS; // Stream buffer definido en freertos.c
/* USER CODE END PV */

/* ===========================
   PROTOTIPOS DE FUNCIONES
   =========================== */
// Prototipo para configurar el reloj del sistema
void SystemClock_Config(void);
// Prototipo para inicializar objetos de FreeRTOS (tareas, colas, etc.)
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
// Aquí puedes declarar prototipos de funciones propias

extern void GPS_Start(void);
/* USER CODE END PFP */

/* ===========================
   CÓDIGO DE USUARIO
   =========================== */
/* USER CODE BEGIN 0 */
// removed static GPS_Start implementation here (moved to usart.c)
/* USER CODE END 0 */

/**
  * @brief  Punto de entrada principal de la aplicación.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // Aquí puedes inicializar variables antes de configurar el hardware
  /* USER CODE END 1 */

  /* ===========================
     INICIALIZACIÓN DEL MCU
     =========================== */
  // Inicializa la HAL (Hardware Abstraction Layer) y el Systick
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Aquí puedes agregar inicialización personalizada antes del reloj
  /* USER CODE END Init */

  // Configura el reloj del sistema (frecuencia de CPU y periféricos)
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Inicialización personalizada después del reloj
  /* USER CODE END SysInit */

  /* ===========================
     INICIALIZACIÓN DE PERIFÉRICOS
     =========================== */
  // Inicializa los periféricos configurados en CubeMX
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */
  // Aqui puedes agregar inicializacion de perifericos o variables propias
  can_start();                // starts CAN and filters (function in can.c)
  // initialize IMU low-level here if you want, but prefer the RTOS task to init MPU
  // mpu6050_init(); // commented: MPU init moved/handled by freertos sensor task using mpu6050.c

  /* ADC DMA continuo -- commented until ADC channels are validated
     HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_NUM_CH);
     (TODO: re-enable after ADC channel mapping verified) */

  /* GPS UART RX handled by usart.c GPS_Start() AFTER SB_GPS is created in MX_FREERTOS_Init */
  // HAL_UART_Receive_IT(&huart2, (uint8_t*)&gps_rx_byte, 1); // removed

  /* Colas and StreamBuffer creation moved to freertos.c (MX_FREERTOS_Init) */
  // q_raw  = xQueueCreate(8, sizeof(TelemetryRaw_t)); // kept in freertos.c / refactor
  // q_proc = xQueueCreate(8, sizeof(TelemetryProc_t));

  /* Tareas created in MX_FREERTOS_Init() */
  MX_FREERTOS_Init();

  /* Start scheduler */
  vTaskStartScheduler();

  while (1) { }
}

/**
  * @brief Configura el reloj del sistema.
  * @retval None
  *
  * Esta función ajusta la fuente de reloj principal, la frecuencia de CPU,
  * y los divisores de los buses AHB y APB. Es generada por CubeMX.
  */
void SystemClock_Config(void)
{
  // Estructuras para configuración de osciladores y relojes
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configura el voltaje de salida del regulador interno
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Inicializa los osciladores RCC según los parámetros especificados
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Usa el oscilador interno HSI
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // Activa HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Activa PLL para multiplicar frecuencia
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // Fuente PLL: HSI
  RCC_OscInitStruct.PLL.PLLM = 8;                            // Predivisor PLL
  RCC_OscInitStruct.PLL.PLLN = 160;                          // Multiplicador PLL
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                // Divisor final PLL
  RCC_OscInitStruct.PLL.PLLQ = 4;                            // Divisor para USB/SDIO/RNG
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(); // Si falla, llama a la función de error
  }

  /** Inicializa los relojes de CPU, AHB y APB
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Fuente principal: PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // Sin división para AHB
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;          // APB1 a 1/4 de HCLK
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;          // APB2 a 1/2 de HCLK

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler(); // Si falla, llama a la función de error
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Función ejecutada en caso de error.
  * @retval None
  *
  * Esta función se llama cuando ocurre un error crítico en la inicialización
  * o ejecución. Deshabilita las interrupciones y entra en un bucle infinito.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq(); // Deshabilita todas las interrupciones
  while (1)
  {
    // Bucle infinito: puedes agregar parpadeo de LED para indicar error
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reporta el nombre del archivo y la línea donde ocurrió un error de assert_param.
  * @param  file: puntero al nombre del archivo fuente
  * @param  line: número de línea donde ocurrió el error
  * @retval None
  *
  * Esta función se llama cuando una comprobación de parámetros falla.
  * Imprime el archivo y la línea por UART.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
