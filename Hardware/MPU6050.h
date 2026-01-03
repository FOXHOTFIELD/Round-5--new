#include <stdint.h>

#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"
#include "MPU6050_Reg.h"
#include "softi2c.h"
#include "OLED.h"
#include "Delay.h"
#include "MadgwickAHRS.h"
#include "comm.h"

#include <stdlib.h>

#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"

/* 1=运行上电自检与零偏校准；0=使用预置零偏值跳过自检 */
#ifndef ENABLE_MPU6050_SELFTEST
#define ENABLE_MPU6050_SELFTEST 0
#endif

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/* 返回 1 表示自检/校准完成，姿态数据可安全输出 */
uint8_t MPU6050_IsCalibrated(void);

void MPU6050_GetPose(void);

extern int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
extern int16_t ax_off, ay_off, az_off;
extern int16_t gx_off, gy_off, gz_off;
extern volatile uint8_t g_selfcorrect_finished;

#endif
