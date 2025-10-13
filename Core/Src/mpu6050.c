#include "mpu6050.h"
#include "i2c.h"   // for hi2c1
#include "stm32f4xx_hal.h"
#include <math.h>

#define MPU6050_ADDR         (0x68<<1)
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG     0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_PWR_MGMT1  0x6B
#define MPU6050_REG_WHO_AM_I   0x75
#define MPU6050_REG_ACCEL_XOUT 0x3B
#define MPU6050_WHO_AM_I_VALUE 0x68
#define MPU6050_SCALE_ACC    16384.0f
#define MPU6050_SCALE_GYRO   131.0f

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef MPU6050_Init_200Hz(void)
{
  HAL_StatusTypeDef status;

  /* Give the sensor time after power-up as recommended in the datasheet */
  HAL_Delay(100);

  /* Verify the device identity before continuing configuration */
  uint8_t reg = MPU6050_REG_WHO_AM_I;
  uint8_t who = 0U;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100);
  if (status != HAL_OK) {
    return status;
  }
  status = HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, &who, 1, 100);
  if (status != HAL_OK || who != MPU6050_WHO_AM_I_VALUE) {
    return HAL_ERROR;
  }

  uint8_t cmd[2];

  /* Wake device (clear sleep) and select PLL as clock source */
  cmd[0] = MPU6050_REG_PWR_MGMT1;
  cmd[1] = 0x01;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cmd, 2, 100);
  if (status != HAL_OK) {
    return status;
  }

  /* Sample rate: 1 kHz / (1 + SMPLRT_DIV) = 200 Hz -> divider = 4 */
  cmd[0] = MPU6050_REG_SMPLRT_DIV;
  cmd[1] = 4;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cmd, 2, 100);
  if (status != HAL_OK) {
    return status;
  }

  /* Configure digital low-pass filter for 44 Hz bandwidth (CONFIG = 3) */
  cmd[0] = MPU6050_REG_CONFIG;
  cmd[1] = 0x03;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cmd, 2, 100);
  if (status != HAL_OK) {
    return status;
  }

  /* Set gyro full scale to ±500 dps (GYRO_CONFIG = 0x08) */
  cmd[0] = MPU6050_REG_GYRO_CONFIG;
  cmd[1] = 0x08;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cmd, 2, 100);
  if (status != HAL_OK) {
    return status;
  }

  /* Set accel full scale to ±4 g (ACCEL_CONFIG = 0x08) */
  cmd[0] = MPU6050_REG_ACCEL_CONFIG;
  cmd[1] = 0x08;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, cmd, 2, 100);
  if (status != HAL_OK) {
    return status;
  }

  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read(IMUSample *s)
{
  if (s == NULL) {
    return HAL_ERROR;
  }
  uint8_t reg = MPU6050_REG_ACCEL_XOUT;
  uint8_t buf[14];
  HAL_StatusTypeDef status;
  status = HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, &reg, 1, 100);
  if (status != HAL_OK) {
    return status;
  }
  status = HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR, buf, 14, 100);
  if (status != HAL_OK) {
    return status;
  }

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