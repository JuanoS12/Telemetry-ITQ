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
/* USER CODE END PFP */

/* ===========================
   CÓDIGO DE USUARIO
   =========================== */
/* USER CODE BEGIN 0 */
// Aquí puedes agregar funciones auxiliares, variables, etc.
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
  MX_GPIO_Init();         // Inicializa los pines GPIO
  MX_DMA_Init();          // Inicializa el controlador DMA
  MX_USART1_UART_Init();  // Inicializa UART1 (usado para GPS)
  MX_CAN1_Init();         // Inicializa el bus CAN1
  MX_I2C1_Init();         // Inicializa el bus I2C1

  /* USER CODE BEGIN 2 */
  // Aquí puedes agregar inicialización de periféricos o variables propias

  // Declaración de variables externas y estáticas para GPS
  extern UART_HandleTypeDef huart1;           // Manejador de UART1 (definido en usart.c)
  extern StreamBufferHandle_t SB_GPS;         // Buffer de flujo para GPS (definido en freertos.c)
  static uint8_t gps_rx[512];                 // Buffer para recepción de datos GPS por UART

  // Función local para iniciar la recepción de datos GPS por DMA
  static void GPS_Start(void){
    // Inicia la recepción por DMA hasta que llegue un byte de "idle" (fin de trama)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_rx, sizeof gps_rx);
    // Deshabilita la interrupción de mitad de transferencia (HT) para evitar callbacks innecesarios
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  }
  // Llama a la función para iniciar la recepción de GPS
  GPS_Start();
  /* USER CODE END 2 */

  /* ===========================
     INICIALIZACIÓN DEL SCHEDULER (FreeRTOS)
     =========================== */
  // Inicializa el kernel de FreeRTOS (crea estructuras internas)
  osKernelInitialize();

  // Inicializa los objetos de FreeRTOS (tareas, colas, semáforos, etc.)
  MX_FREERTOS_Init();

  // Inicia el scheduler de FreeRTOS (las tareas comienzan a ejecutarse)
  osKernelStart();

  // El control nunca debería regresar aquí, ya que el scheduler toma el control

  /* ===========================
     BUCLE INFINITO DE SEGURIDAD
     =========================== */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Si el scheduler falla, el programa queda aquí
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Puedes agregar código de emergencia aquí (parpadeo de LED, etc.)
    /* USER CODE END 3 */
  }
  /* USER CODE END 3 */
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

/**
  * @brief Callback de recepción UART por DMA con evento "idle".
  * @param huart: puntero al manejador de UART
  * @param Size: cantidad de bytes recibidos
  *
  * Esta función se llama automáticamente cuando se recibe una trama por UART1
  * usando DMA y se detecta un evento "idle" (fin de transmisión).
  * El buffer recibido se envía a un StreamBuffer de FreeRTOS para procesarlo
  * en una tarea. Luego se reinicia la recepción por DMA.
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // Verifica que el evento proviene de UART1
  if (huart->Instance == USART1){
    BaseType_t w = pdFALSE; // Variable para indicar si se debe hacer un cambio de contexto
    extern StreamBufferHandle_t SB_GPS; // Buffer de flujo para GPS
    extern uint8_t gps_rx[];            // Buffer de recepción GPS

    // Envía los datos recibidos al StreamBuffer (para que otra tarea los procese)
    xStreamBufferSendFromISR(SB_GPS, gps_rx, Size, &w);

    // Reinicia la recepción por DMA para la siguiente trama
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_rx, sizeof gps_rx);

    // Deshabilita la interrupción de mitad de transferencia (HT)
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    // Si una tarea de mayor prioridad fue desbloqueada, solicita cambio de contexto
    portYIELD_FROM_ISR(w);
  }
}

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
