// 编码器(旋转式)读取接口 - 使用 PA0/PA1 两路
// TIM2 编码器模式硬件解码，内部提供 32 位累计计数（16 位 CNT + 软件高位）

#pragma once

#ifndef __ENCODER_H
#define __ENCODER_H


#include <stdint.h>

// 初始化编码器引脚：PA0/PA1 设为上拉输入
void Encoder_Init(void);

// 轮询接口（TIM 方案下可不使用，保留以兼容旧代码）
void Encoder_Update(void);

// 读取当前累计计数值（顺时针为正，逆时针为负）
int32_t Encoder_GetCount(void);

// 设置/重置计数值（例如清零）
void Encoder_SetCount(int32_t count);

// 采样辅助：返回当前累计计数并清零，便于按固定周期统计转速
// 典型用法：在周期任务中调用该函数得到 Δcnt；结合周期时长与每圈脉冲数计算转速
int32_t Encoder_GetAndReset(void);

#endif
