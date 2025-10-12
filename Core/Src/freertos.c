/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications made by Alonso 
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
// Cabeceras de FreeRTOS y HAL necesarias para tareas, colas y buffers
#include "FreeRTOS.h"      // Núcleo de FreeRTOS
#include "task.h"          // API de tareas
#include "main.h"          // Definiciones globales del proyecto
#include "cmsis_os.h"      // API de CMSIS-RTOS (wrapper de FreeRTOS)

/* ===========================
   INCLUSIONES ADICIONALES
   =========================== */
/* USER CODE BEGIN Includes */
#include "queue.h"         // API de colas FreeRTOS
#include "stream_buffer.h" // API de buffers de flujo FreeRTOS
#include "usart.h"         // Handlers UART (huart2 para telemetría)
#include "mpu6050.h"       // Prototipos del driver de IMU
#include <string.h>        // Funciones estándar de manejo de cadenas
#include <stdio.h>         // snprintf para tramas UART
#include <math.h>          // Funciones matemáticas
#include <stdlib.h>        // Funciones estándar (atoi, atof)
/* USER CODE END Includes */

/* ===========================
   TIPOS DE DATOS PRIVADOS
   =========================== */

// Estructura para un mensaje CAN (frame estándar)
typedef struct { uint16_t id; uint8_t d[8]; } CANMsg;

// --- GPS parser y helpers ---
// Estructura para almacenar un "fix" GPS (información de posición y calidad)
typedef struct { 
  uint8_t fix, sats;         // fix: 1 si hay posición válida, sats: número de satélites
  float hdop, speed_ms, course_deg; // hdop: precisión, velocidad en m/s, rumbo en grados
  double lat_deg, lon_deg;   // latitud y longitud en grados decimales
  uint32_t tms;              // timestamp en ms
} GPSFix;

// Convierte coordenadas NMEA (ddmm.mmmm) a grados decimales
static double dm2deg(double dm){ 
  double d=floor(dm/100.0), m=dm-d*100.0; 
  return d+m/60.0; 
}

// Parsea una línea NMEA y llena la estructura GPSFix
static void GPS_ParseLine(const char* line, GPSFix* g){
  if (!strncmp(line,"$GPRMC",6)||!strncmp(line,"$GNRMC",6)){
    char *f[16]={0};
    char tmp[128];
    size_t len=0U;
    while (len < sizeof(tmp)-1U && line[len] != '\0') { len++; }
    memcpy(tmp, line, len);
    tmp[len] = '\0';
    char *p=tmp; int i=0;
    while(i<16 && (f[i]=strtok(i?NULL:p,","))) {i++; p=NULL;}
    if (f[2] && f[2][0]=='A'){
      g->lat_deg=dm2deg(atof(f[3])); if (f[4] && f[4][0]=='S') g->lat_deg=-g->lat_deg;
      g->lon_deg=dm2deg(atof(f[5])); if (f[6] && f[6][0]=='W') g->lon_deg=-g->lon_deg;
      g->speed_ms=(float)(atof(f[7])*0.514444f);
      g->course_deg=(float)atof(f[8]);
      g->fix=1;
      g->tms=HAL_GetTick();
    }
  } else if (!strncmp(line,"$GPGGA",6)||!strncmp(line,"$GNGGA",6)){
    char *f[16]={0};
    char tmp[128];
    size_t len=0U;
    while (len < sizeof(tmp)-1U && line[len] != '\0') { len++; }
    memcpy(tmp, line, len);
    tmp[len] = '\0';
    char *p=tmp; int i=0;
    while(i<16 && (f[i]=strtok(i?NULL:p,","))) {i++; p=NULL;}
    if (f[6]){ int q=atoi(f[6]); g->fix = (q>0)? (g->fix?g->fix:1):0; }
    if (f[7]) g->sats=(uint8_t)atoi(f[7]);
    if (f[8]) g->hdop=(float)atof(f[8]);
  }
}

static BaseType_t CAN_SendMessage(QueueHandle_t queue, const CANMsg *msg, TickType_t timeoutTicks, uint32_t *dropCounter)
{
  BaseType_t result = xQueueSend(queue, msg, timeoutTicks);
  if (result != pdPASS && dropCounter != NULL) {
    (*dropCounter)++;
  }
  return result;
}

/* ===========================
   DEFINICIONES DE COLAS Y BUFFERS
   =========================== */
/* USER CODE BEGIN PD */
// Colas y buffers globales para comunicación entre tareas
QueueHandle_t Q_CAN_TX;
StreamBufferHandle_t SB_GPS;    // buffer used by usart.c
StreamBufferHandle_t SB_UART_TX; // telemetría ASCII hacia PC
static const TickType_t CAN_TX_TIMEOUT = pdMS_TO_TICKS(5); // Espera maxima antes de descartar
static uint32_t can_tx_drop_imu = 0;  // Contador de descartes de IMU
static uint32_t can_tx_drop_gps = 0;  // Contador de descartes de GPS
static volatile uint32_t uart_drop_bytes = 0;  // Bytes que no cupieron en buffer UART


// Estructura para muestras de IMU (acelerómetro, giroscopio, temperatura)
typedef struct { 
  float ax,ay,az,gx,gy,gz,tempC; // ax,ay,az: aceleraciones; gx,gy,gz: giros; tempC: temperatura
  uint32_t tms;                  // timestamp en ms
} IMUSample;
/* USER CODE END PD */

/* ===========================
   DEFINICIÓN DE TAREAS Y ATRIBUTOS
   =========================== */
// Cada tarea tiene un identificador y atributos (nombre, stack, prioridad)
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t CanTxTaskHandle;
const osThreadAttr_t CanTxTask_attributes = {
  .name = "CanTxTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
osThreadId_t UartTxTaskHandle;
const osThreadAttr_t UartTxTask_attributes = {
  .name = "UartTxTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* ===========================
   PROTOTIPOS DE FUNCIONES DE TAREA
   =========================== */
void StartSensorTask(void *argument); // Tarea de adquisición de sensores (IMU)
void StartGPSTask(void *argument);    // Tarea de procesamiento de GPS
void StartCanTxTask(void *argument);  // Tarea de transmisión CAN
void StartTask04(void *argument);     // Tarea de LED (indicador)
void StartUartTxTask(void *argument); // Tarea de transmisión UART hacia PC

/* ===========================
   INICIALIZACIÓN DE FREERTOS
   =========================== */
/**
  * @brief  Inicializa FreeRTOS: crea colas, buffers y tareas.
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_QUEUES */
  Q_CAN_TX   = xQueueCreate(32, sizeof(CANMsg));
  SB_GPS     = xStreamBufferCreate(512, 32);
  SB_UART_TX = xStreamBufferCreate(512, 32);
  configASSERT(Q_CAN_TX != NULL);
  configASSERT(SB_GPS != NULL);
  configASSERT(SB_UART_TX != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create tasks */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);
  GPSTaskHandle    = osThreadNew(StartGPSTask, NULL, &GPSTask_attributes);
  CanTxTaskHandle  = osThreadNew(StartCanTxTask, NULL, &CanTxTask_attributes);
  LedTaskHandle    = osThreadNew(StartTask04, NULL, &LedTask_attributes);
  UartTxTaskHandle = osThreadNew(StartUartTxTask, NULL, &UartTxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // After creating SB_GPS we can start the GPS DMA safely
  extern void GPS_Start(void); // implemented in usart.c
  GPS_Start();
  /* USER CODE END RTOS_THREADS */
}

/* ===========================
   TAREA DE SENSORES (IMU)
   =========================== */
/**
  * @brief  Tarea que lee la IMU (MPU6050) y envía los datos por CAN.
  * @param  argument: No usado
  * @retval None
  *
  * Flujo:
  * 1. Inicializa la IMU.
  * 2. Cada 10 ms (100 Hz) lee los datos de acelerómetro, giroscopio y temperatura.
  * 3. Empaqueta los datos en dos mensajes CAN y los envía a la cola Q_CAN_TX.
  */
static void UartPublish(const char *line, size_t len)
{
  if (SB_UART_TX == NULL || line == NULL || len == 0U) {
    return;
  }
  size_t sent = xStreamBufferSend(SB_UART_TX, line, len, 0);
  if (sent < len) {
    uart_drop_bytes += (uint32_t)(len - sent);
  }
}

void StartSensorTask(void *argument)
{
  (void)argument;
  if (MPU6050_Init_200Hz() != HAL_OK) {
    Error_Handler();
  } // Inicializa la IMU a 200 Hz
  TickType_t t0 = xTaskGetTickCount();
  uint16_t ctr=0; // Contador de muestras (útil para debug)
  for(;;){
    IMUSample s;
    if (MPU6050_Read(&s)==HAL_OK){
      // Convierte los datos físicos a enteros para CAN (ejemplo: ax*1000 para mm/s^2)
      int16_t ax=lrintf(s.ax*1000), ay=lrintf(s.ay*1000), az=lrintf(s.az*1000);
      int16_t gx=lrintf(s.gx*100),  gy=lrintf(s.gy*100),  gz=lrintf(s.gz*100), tC=lrintf(s.tempC*100);
      CANMsg m;
      // Primer mensaje: aceleraciones + contador
      m.id=0x100; memcpy(&m.d[0],&ax,2); memcpy(&m.d[2],&ay,2); memcpy(&m.d[4],&az,2); memcpy(&m.d[6],&ctr,2);
      CAN_SendMessage(Q_CAN_TX, &m, CAN_TX_TIMEOUT, &can_tx_drop_imu);
      // Segundo mensaje: giros + temperatura
      m.id=0x101; memcpy(&m.d[0],&gx,2); memcpy(&m.d[2],&gy,2); memcpy(&m.d[4],&gz,2); memcpy(&m.d[6],&tC,2);
      CAN_SendMessage(Q_CAN_TX, &m, CAN_TX_TIMEOUT, &can_tx_drop_imu);
      // Telemetría por UART hacia PC en formato CSV con escalas enteras
      char line[128];
      int len = snprintf(line, sizeof(line),
                         "IMU,%lu,%d,%d,%d,%d,%d,%d,%d\r\n",
                         (unsigned long)s.tms,
                         ax, ay, az,
                         gx, gy, gz,
                         tC);
      if (len > 0) {
        UartPublish(line, (size_t)len);
      }
      ctr++;
    }
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(10)); // Espera hasta el siguiente periodo (100 Hz)
  }
}

/* ===========================
   TAREA DE GPS
   =========================== */
/**
* @brief Tarea que procesa datos crudos de GPS y los envía por CAN.
* @param argument: No usado
* @retval None
*
* Flujo:
* 1. Lee datos crudos del buffer SB_GPS (llenado por DMA UART).
* 2. Reconstruye líneas NMEA y las parsea.
* 3. Si hay posición válida (fix), empaqueta lat/lon y velocidad/rumbo/hdop en mensajes CAN.
*/
void StartGPSTask(void *argument)
{
  (void)argument;
  GPSFix g={0}; char line[128]; size_t ll=0;
  uint8_t chunk[128];
  for(;;){
    // Lee hasta 128 bytes del buffer de flujo (espera hasta que haya datos)
    size_t n = xStreamBufferReceive(SB_GPS, chunk, sizeof(chunk), portMAX_DELAY);
    // Reconstruye líneas NMEA (terminadas en '\n')
    for (size_t i=0;i<n;i++){
      char c=(char)chunk[i]; if (c=='\r') continue; // Ignora retorno de carro
      if (c=='\n'){ 
        line[ll]='\0'; // Fin de línea
        if (ll>6){ 
          GPS_ParseLine(line,&g); // Parsea la línea
          if (g.fix){
            // Si hay posición válida, empaqueta y envía por CAN
            int32_t lat=llrint(g.lat_deg*1e7), lon=llrint(g.lon_deg*1e7);
            CANMsg m;
            m.id=0x120; memcpy(&m.d[0],&lat,4); memcpy(&m.d[4],&lon,4);
            CAN_SendMessage(Q_CAN_TX, &m, CAN_TX_TIMEOUT, &can_tx_drop_gps);
            int16_t v=lrintf(g.speed_ms*100), cr=lrintf(g.course_deg*100), hd=lrintf(g.hdop*100);
            m.id=0x121; memcpy(&m.d[0],&v,2); memcpy(&m.d[2],&cr,2); memcpy(&m.d[4],&hd,2); m.d[6]=g.fix; m.d[7]=g.sats;
            CAN_SendMessage(Q_CAN_TX, &m, CAN_TX_TIMEOUT, &can_tx_drop_gps);
            char out[128];
            int len = snprintf(out, sizeof(out),
                               "GPS,%lu,%ld,%ld,%d,%d,%d,%u,%u\r\n",
                               (unsigned long)g.tms,
                               (long)lat, (long)lon,
                               v, cr, hd,
                               g.fix, g.sats);
            if (len > 0) {
              UartPublish(out, (size_t)len);
            }
          }
        }
        ll=0; // Reinicia el buffer de línea
      } else if (ll<sizeof(line)-1) line[ll++]=c; else ll=0; // Acumula caracteres
    }
  }
}

/* ===========================
   TAREA DE TRANSMISIÓN CAN
   =========================== */
/**
* @brief Tarea que toma mensajes de la cola Q_CAN_TX y los transmite por CAN.
* @param argument: No usado
* @retval None
*
* Flujo:
* 1. Espera mensajes en la cola Q_CAN_TX.
* 2. Cuando hay uno, lo transmite usando CAN1_SendStd.
* 3. Si la transmisión falla, reintenta tras un pequeño delay.
*/
extern HAL_StatusTypeDef CAN1_SendStd(uint16_t id, const uint8_t data[8]);
void StartCanTxTask(void *argument)
{
  (void)argument;
  CANMsg m;
  for(;;){
    // Espera hasta recibir un mensaje en la cola
    if (xQueueReceive(Q_CAN_TX, &m, portMAX_DELAY) == pdPASS){
      // Intenta transmitir por CAN, reintenta si falla
      while (CAN1_SendStd(m.id, m.d) != HAL_OK){
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }
}

/* ===========================
   TAREA DE LED (INDICADOR)
   =========================== */
/**
* @brief Tarea que puede usarse para parpadear un LED o indicar estado.
* @param argument: No usado
* @retval None
*
* Actualmente solo espera, pero puedes agregar control de LED aquí.
*/
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Bucle infinito */
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1)); // Espera 1 ms (placeholder)
  }
  /* USER CODE END StartTask04 */
}

void StartUartTxTask(void *argument)
{
  (void)argument;
  uint8_t chunk[128];
  for(;;) {
    size_t n = xStreamBufferReceive(SB_UART_TX, chunk, sizeof(chunk), portMAX_DELAY);
    if (n > 0U) {
      (void)HAL_UART_Transmit(&huart2, chunk, (uint16_t)n, HAL_MAX_DELAY);
    }
  }
}

/* ===========================
   CÓDIGO DE USUARIO ADICIONAL
   =========================== */
/* USER CODE BEGIN Application */
// Aquí puedes agregar funciones auxiliares, rutinas de inicialización, etc.
/* USER CODE END Application */

