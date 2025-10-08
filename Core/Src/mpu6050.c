#include "mpu6050.h"
#include "i2c.h"   // for hi2c1
#include "stm32f4xx_hal.h"
#include <math.h>

#define MPU6050_ADDR         (0x68<<1)
#define MPU6050_REG_PWR_MGMT1 0x6B
#define MPU6050_REG_ACCEL_XOUT 0x3B
#define MPU6050_SCALE_ACC    16384.0f
#define MPU6050_SCALE_GYRO   131.0f

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef MPU6050_Init_200Hz(void)
{
  uint8_t cmd[2];
  // Wake device (clear sleep)
  cmd[0] = MPU6050_REG_PWR_MGMT1;
  cmd[1] = 0x00;
  if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cmd, 2, 100) != HAL_OK) return HAL_ERROR;
  // Additional configuration (sample rate / DLPF) can be added here if needed
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read(IMUSample *s)
{
  uint8_t reg = MPU6050_REG_ACCEL_XOUT;
  uint8_t buf[14];
  if (HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100) != HAL_OK) return HAL_ERROR;
  if (HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100) != HAL_OK) return HAL_ERROR;

  int16_t ax = (buf[0]<<8) | buf[1];
  int16_t ay = (buf[2]<<8) | buf[3];
  int16_t az = (buf[4]<<8) | buf[5];
  int16_t temp = (buf[6]<<8) | buf[7];
  int16_t gx = (buf[8]<<8) | buf[9];
  int16_t gy = (buf[10]<<8) | buf[11];
  int16_t gz = (buf[12]<<8) | buf[13];

  s->ax = ((float)ax) / MPU6050_SCALE_ACC * 9.80665f;
  s->ay = ((float)ay) / MPU6050_SCALE_ACC * 9.80665f;
  s->az = ((float)az) / MPU6050_SCALE_ACC * 9.80665f;
  s->gx = ((float)gx) / MPU6050_SCALE_GYRO * (M_PI/180.0f);
  s->gy = ((float)gy) / MPU6050_SCALE_GYRO * (M_PI/180.0f);
  s->gz = ((float)gz) / MPU6050_SCALE_GYRO * (M_PI/180.0f);
  s->tempC = ((float)temp) / 340.0f + 36.53f;
  s->tms = HAL_GetTick();
  return HAL_OK;
}