// 编码器(旋转式)读取接口 - 使用 PA0/PA1 两路
// TIM2 编码器模式硬件解码，内部提供 32 位累计计数（16 位 CNT + 软件高位）

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
 * - 请在 stm32f10x_it.c 中提供 TIM2 的中断服务：
 *     void TIM2_IRQHandler(void) { Encoder_TIM2_IRQHandler(); }
 *   或者在该文件中直接定义 TIM2_IRQHandler（不推荐，统一集中 ISR 更清晰）。
 */

// ---------------- 引脚与外设定义（可按需调整） ----------------
#define ENC_GPIO            GPIOA               // 编码器所在端口
#define ENC_GPIO_RCC        RCC_APB2Periph_GPIOA// 端口时钟
#define ENC_CH_A            GPIO_Pin_0          // A 通道 -> PA0
#define ENC_CH_B            GPIO_Pin_1          // B 通道 -> PA1
// 定时器选择
#define ENC_TIM             TIM2
#define ENC_TIM_RCC         RCC_APB1Periph_TIM2


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

// 读取当前累计计数并映射为百分比 -100.00 .. 100.00（保留两位小数）
float Encoder_GetCountPercent(void);

// 返回当前累计计数映射的百分比并清零计数（便于周期统计）
float Encoder_GetAndResetPercent(void);

#endif
