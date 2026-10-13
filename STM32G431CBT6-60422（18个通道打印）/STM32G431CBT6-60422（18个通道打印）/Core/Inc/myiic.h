#ifndef _MYIIC_H
#define _MYIIC_H

#include "main.h"

#define MPU6050_ADDR  0xD0  // MPU6050 I2C地址 (0x68 << 1)

#define SMPLRT_DIV    0x19
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B
#define TEMP_OUT_H    0x41
#define GYRO_XOUT_H   0x43
#define PWR_MGMT_1    0x6B
#define WHO_AM_I      0x75
#define M_PI          3.1415926

// 模拟I2C引脚定义
#define I2C_SCL_PIN GPIO_PIN_8
#define I2C_SCL_PORT GPIOA
#define I2C_SDA_PIN GPIO_PIN_9
#define I2C_SDA_PORT GPIOA

void MPU6050_Init(void);
void MPU6050_ReadRawData(int16_t *accel, int16_t *gyro);
//void CalculateAngles(int16_t *accel, float *pitch, float *roll);
void CalculateAngles(int16_t *accel, int16_t *gyro, float *pitch, float *roll, float *yaw);

void MPU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num);
void MPU6050ReadAcc(short *accData);
void MPU6050ReadGyro(short *gyroData);
void MPU6050ReadTemp(short *tempData);
void MPU6050_ReturnTemp(float *Temperature);

#endif

