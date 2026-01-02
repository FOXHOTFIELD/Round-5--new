// 编码器(旋转式)读取接口 - 支持两路：
// ENC1：PA0/PA1 -> TIM2；ENC2：PB6/PB7 -> TIM4
// TIMx 编码器模式硬件解码，内部提供 32 位累计计数（16 位 CNT + 软件高位）

#pragma once

#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>

#include "comm.h"
#include "filter.h"
/*
 * 集成说明（与 stm32f10x_it.c 的关系）：
 * - 不需要修改 stm32f10x_it.h。
 * - 请在 stm32f10x_it.c 中转发 TIM2/TIM4 的更新中断：
 *     void TIM2_IRQHandler(void) { Encoder_TIM2_IRQHandler(); }
 *     void TIM4_IRQHandler(void) { Encoder_TIM4_IRQHandler(); }
 *   或者在该文件中直接定义 TIM2_IRQHandler/TIM4_IRQHandler（不推荐，集中 ISR 更清晰）。
 */

// ---------------- 编码器 ID 定义 ----------------
typedef enum
{
	ENCODER_A = 0,   // ENC1：PA0/PA1 -> TIM2
	ENCODER_B  = 1,   // ENC2：PB6/PB7 -> TIM4
	ENCODER_COUNT
} EncoderId_t;


// 初始化指定编码器（GPIO 上拉输入，TIM 编码器模式）
void Encoder_Init(EncoderId_t id);

// 轮询接口（TIM 方案下可不使用，保留以兼容旧代码）
void Encoder_Update(EncoderId_t id);

// 读取当前累计计数值（顺时针为正，逆时针为负）
int32_t Encoder_GetCount(EncoderId_t id);

// 设置/重置计数值（例如清零）
void Encoder_SetCount(EncoderId_t id, int32_t count);

// 采样辅助：返回当前累计计数并清零，便于按固定周期统计转速
// 典型用法：在周期任务中调用该函数得到 Δcnt；结合周期时长与每圈脉冲数计算转速
int32_t Encoder_GetAndReset(EncoderId_t id);

// 读取当前累计计数并映射为百分比 -100.00 .. 100.00（保留两位小数）
float Encoder_GetCountPercent(EncoderId_t id);

// 返回当前累计计数映射的百分比并清零计数（便于周期统计）
float Encoder_GetAndResetPercent(EncoderId_t id);

// TIM 更新中断转发入口（供 stm32f10x_it.c 调用）
void Encoder_TIM2_IRQHandler(void);
void Encoder_TIM4_IRQHandler(void);

#endif
