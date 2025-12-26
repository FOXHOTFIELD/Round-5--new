#include <stdint.h>

#ifndef __MPU6050_H
#define __MPU6050_H

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU6050_SelfCorrect(void);
void MPU6050_StartPoseTask(void);

extern int16_t AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
extern int16_t ax_off, ay_off, az_off;
extern int16_t gx_off, gy_off, gz_off;
extern uint8_t g_selfcorrect_finished;

#endif
