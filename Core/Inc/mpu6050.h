#ifndef MPU6050_H_INCLUDED
#define MPU6050_H_INCLUDED
#include <stdint.h>
typedef struct { float ax,ay,az,gx,gy,gz,tempC; uint32_t tms; } IMUSample;
HAL_StatusTypeDef MPU6050_Init_200Hz(void);
HAL_StatusTypeDef MPU6050_Read(IMUSample *s);


#endif