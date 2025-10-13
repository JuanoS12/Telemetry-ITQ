#ifndef MPU6050_H_INCLUDED
#define MPU6050_H_INCLUDED

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float tempC;
  uint32_t tms;
} IMUSample;

HAL_StatusTypeDef MPU6050_Init_200Hz(void);
HAL_StatusTypeDef MPU6050_Read(IMUSample *s);

#endif /* MPU6050_H_INCLUDED */
