#pragma once

#ifndef __MOTORDRIVE_H_
#define __MOTORDRIVE_H_

#include <stdint.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

/* 引脚宏定义 */
/* PWM A: TIM1_CH1 -> PA8 */
#define PWMA_GPIO           GPIOA
#define PWMA_PIN            GPIO_Pin_8
#define PWMA_TIM            TIM1

/* PWM B: TIM1_CH4 -> PA11 */
#define PWMB_GPIO           GPIOA
#define PWMB_PIN            GPIO_Pin_11
#define PWMB_TIM            TIM1

/* 控制引脚（TB6612 等驱动） */
#define STBY_GPIO           GPIOB
#define STBY_PIN            GPIO_Pin_5

#define AIN1_GPIO           GPIOB
#define AIN1_PIN            GPIO_Pin_13
#define AIN2_GPIO           GPIOB
#define AIN2_PIN            GPIO_Pin_12

#define BIN1_GPIO           GPIOB
#define BIN1_PIN            GPIO_Pin_15
#define BIN2_GPIO           GPIOB
#define BIN2_PIN            GPIO_Pin_14

/* PWM 参数：降低分辨率以提升频率到约 25kHz */
/* 72MHz / ((PSC+1)*(ARR+1)) = 25kHz -> PSC=0, ARR=2880-1=2879 */
#define PWM_ARR             (2880 - 1)
#define PWM_PSC_TIM1        (0)

enum Motor_Mode_t
{
	Motor_Mode_break = 0,       /* 快速制动（双高） */
	Motor_Mode_stop,            /* 停止（双低） */
	Motor_Mode_frd_rotation,    /* 正转 */
	Motor_Mode_rvs_rotation     /* 反转 */
};

/* 电机状态结构体：Speed 为理论最大速度的百分比（支持小数，-100.00..100.00） */
typedef struct Motor_Status_t
{
	enum Motor_Mode_t Mode;   /* 当前模式 */
	float           SpeedInPercent;  /* 速度百分比（可带小数，-100.00..100.00；负数为反转） */
} Motor_Status_t;

typedef enum Motor_STBY_t
{
    Disable = 0,
    Enable  = 1,
}Motor_STBY_t;

typedef struct MotorId_t
{
	uint8_t index;                    /* 电机编号（1/2） */
	GPIO_TypeDef *in1_gpio;
	uint16_t in1_pin;
	GPIO_TypeDef *in2_gpio;
	uint16_t in2_pin;
	volatile uint16_t *ccr;           /* PWM 比较寄存器地址 */
} MotorId_t;

void MD_Init(void);
void Motor1_SetCompare(uint16_t Compare);
void Motor2_SetCompare(uint16_t Compare);
void Motor_SetMode(int numMotor, enum Motor_Mode_t Mode);
/* 扩大输入范围到 ±10000，与 ARR 一致 */
void Motor_SetPWM(uint8_t index, int16_t PWM);
/* 统一映射：按速度百分比设置（支持 xx.xx%；-100..100），自动映射到 CCR */
void Motor_SetSpeedPercent(uint8_t index, float percent);
void Motor_SetStandbyState(FunctionalState NewState);

/* 通过 MotorId 进行控制的接口 */
const MotorId_t *Motor_GetId(uint8_t index);
void Motor_SetModeById(const MotorId_t *motor, enum Motor_Mode_t Mode);
void Motor_SetPWMById(const MotorId_t *motor, int16_t PWM);
void Motor_SetSpeedPercentById(const MotorId_t *motor, float percent);


#endif /* __MOTORDRIVE_H_ */
