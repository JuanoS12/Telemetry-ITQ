/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "stream_buffer.h"
#include <string.h>

#include <math.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

// CAN frame type for all CAN messages
typedef struct { uint16_t id; uint8_t d[8]; } CANMsg;

// --- GPS parser and helpers ---
typedef struct { uint8_t fix,sats; float hdop,speed_ms,course_deg; double lat_deg,lon_deg; uint32_t tms; } GPSFix;
static double dm2deg(double dm){ double d=floor(dm/100.0), m=dm-d*100.0; return d+m/60.0; }
static void GPS_ParseLine(const char* line, GPSFix* g){
  if (!strncmp(line,"$GPRMC",6)||!strncmp(line,"$GNRMC",6)){
    char *f[16]={0}; char tmp[128]; strncpy(tmp,line,sizeof(tmp));
    char *p=tmp; int i=0; while(i<16 && (f[i]=strtok(i?NULL:p,","))) {i++; p=NULL;}
    if (f[2] && f[2][0]=='A'){ g->lat_deg=dm2deg(atof(f[3])); if (f[4][0]=='S') g->lat_deg=-g->lat_deg;
      g->lon_deg=dm2deg(atof(f[5])); if (f[6][0]=='W') g->lon_deg=-g->lon_deg;
      g->speed_ms=(float)(atof(f[7])*0.514444f); g->course_deg=(float)atof(f[8]); g->fix=1; g->tms=HAL_GetTick(); }
  } else if (!strncmp(line,"$GPGGA",6)||!strncmp(line,"$GNGGA",6)){
    char *f[16]={0}; char tmp[128]; strncpy(tmp,line,sizeof(tmp));
    char *p=tmp; int i=0; while(i<16 && (f[i]=strtok(i?NULL:p,","))) {i++; p=NULL;}
    if (f[6]){ int q=atoi(f[6]); g->fix = (q>0)? (g->fix?g->fix:1):0; }
    if (f[7]) g->sats=(uint8_t)atoi(f[7]); if (f[8]) g->hdop=(float)atof(f[8]);
  }
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Queues / buffers (visible to other files via extern if needed)
QueueHandle_t Q_CAN_TX;   // items: struct {uint16_t id; uint8_t d[8];}
StreamBufferHandle_t SB_GPS; // raw GPS bytes from USART1 IDLE DMA

// IMU sample struct
typedef struct { float ax,ay,az,gx,gy,gz,tempC; uint32_t tms; } IMUSample;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for GPSTask */
osThreadId_t GPSTaskHandle;
const osThreadAttr_t GPSTask_attributes = {
  .name = "GPSTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanTxTask */
osThreadId_t CanTxTaskHandle;
const osThreadAttr_t CanTxTask_attributes = {
  .name = "CanTxTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSensorTask(void *argument);
void StartGPSTask(void *argument);
void StartCanTxTask(void *argument);
void StartTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */



  /* USER CODE BEGIN RTOS_QUEUES */
  // Create CAN TX queue for CAN frames
  Q_CAN_TX = xQueueCreate(32, sizeof(CANMsg));
  SB_GPS   = xStreamBufferCreate(512, 32);
  configASSERT(Q_CAN_TX != NULL);
  configASSERT(SB_GPS   != NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of GPSTask */
  GPSTaskHandle = osThreadNew(StartGPSTask, NULL, &GPSTask_attributes);

  /* creation of CanTxTask */
  CanTxTaskHandle = osThreadNew(StartCanTxTask, NULL, &CanTxTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartTask04, NULL, &LedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the SensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
extern HAL_StatusTypeDef MPU6050_Init_200Hz(void);
extern HAL_StatusTypeDef MPU6050_Read(IMUSample *s);
void StartSensorTask(void *argument)
{
  (void)argument;
  MPU6050_Init_200Hz();
  TickType_t t0 = xTaskGetTickCount();
  uint16_t ctr=0;
  for(;;){
    IMUSample s;
    if (MPU6050_Read(&s)==HAL_OK){
      int16_t ax=lrintf(s.ax*1000), ay=lrintf(s.ay*1000), az=lrintf(s.az*1000);
      int16_t gx=lrintf(s.gx*100),  gy=lrintf(s.gy*100),  gz=lrintf(s.gz*100), tC=lrintf(s.tempC*100);
      CANMsg m;
      m.id=0x100; memcpy(&m.d[0],&ax,2); memcpy(&m.d[2],&ay,2); memcpy(&m.d[4],&az,2); memcpy(&m.d[6],&ctr,2);
      xQueueSend(Q_CAN_TX,&m,0);
      m.id=0x101; memcpy(&m.d[0],&gx,2); memcpy(&m.d[2],&gy,2); memcpy(&m.d[4],&gz,2); memcpy(&m.d[6],&tC,2);
      xQueueSend(Q_CAN_TX,&m,0);
      ctr++;
    }
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(10)); // 100 Hz
  }
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the GPSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument)
{
  (void)argument;
  GPSFix g={0}; char line[128]; size_t ll=0;
  uint8_t chunk[128];
  for(;;){
    GPSFix g={0}; char line[192]; size_t ll=0;
    for (size_t i=0;i<n;i++){
      char c=(char)chunk[i]; if (c=='\r') continue;
      if (c=='\n'){ line[ll]='\0';
        if (ll>6){ GPS_ParseLine(line,&g);
          if (g.fix){
            int32_t lat=llrint(g.lat_deg*1e7), lon=llrint(g.lon_deg*1e7);
            CANMsg m;
            m.id=0x120; memcpy(&m.d[0],&lat,4); memcpy(&m.d[4],&lon,4); xQueueSend(Q_CAN_TX,&m,0);
            int16_t v=lrintf(g.speed_ms*100), cr=lrintf(g.course_deg*100), hd=lrintf(g.hdop*100);
            m.id=0x121; memcpy(&m.d[0],&v,2); memcpy(&m.d[2],&cr,2); memcpy(&m.d[4],&hd,2); m.d[6]=g.fix; m.d[7]=g.sats;
            xQueueSend(Q_CAN_TX,&m,0);
                         xQueueSend(Q_CAN_TX,&m,pdMS_TO_TICKS(5));
        } ll=0;
      } else if (ll<sizeof(line)-1) line[ll++]=c; else ll=0;
                         xQueueSend(Q_CAN_TX,&m,pdMS_TO_TICKS(5));
  }
}

/* USER CODE BEGIN Header_StartCanTxTask */
/**
* @brief Function implementing the CanTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanTxTask */
extern HAL_StatusTypeDef CAN1_SendStd(uint16_t id, const uint8_t data[8]);
void StartCanTxTask(void *argument)
{
  (void)argument;
  CANMsg m;
  for(;;){
  if (xQueueReceive(Q_CAN_TX, &m, portMAX_DELAY) == pdPASS){
      while (CAN1_SendStd(m.id, m.d) != HAL_OK){
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
  vTaskDelay(pdMS_TO_TICKS(1));
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

