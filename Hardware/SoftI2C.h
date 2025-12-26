/* 软件I2C (Bit-bang) 简单实现，SCL=PB10, SDA=PB11 */
#ifndef __SOFTI2C_H
#define __SOFTI2C_H

#include "stm32f10x.h"

void Soft_I2C_Init(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
void Soft_I2C_SendByte(uint8_t byte);
uint8_t Soft_I2C_ReadByte(uint8_t ack);
uint8_t Soft_I2C_WaitAck(void);
void Soft_I2C_Ack(void);
void Soft_I2C_NAck(void);

#endif
