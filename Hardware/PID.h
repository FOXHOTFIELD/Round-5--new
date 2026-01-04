#pragma once

#ifndef __PID_H
#define __PID_H

#include "MotorDrive.h"

#define PID_UPDATE_MS (1)

typedef struct PID_Config_t
{
	float Kp;
	float Ki;
	float Kd;
	
	float ErrorIntMax;
	float ErrorIntMin;
	
	float OutMax;
	float OutMin;
	
	float OutOffset;

	float FfGain;        /* 前馈增益：直接按目标叠加到输出 */
	float ReverseDecay;   /* 误差符号翻转时衰减历史项/输出，范围 0..1 */
}PID_Config_t;

///* 前向声明：与电机驱动中的状态结构保持一致 */
//typedef struct Motor_Status_t Motor_Status_t;

typedef struct PID_Parameter_t
{
    float Target;
	Motor_Status_t *motor_status;
	float Error;

	/* 增量式 PID 需要保存历史误差与上一周期输出 */
	float prev_error_1;   /* e(k-1) */
	float prev_error_2;   /* e(k-2) */
	float last_output;    /* u(k-1) */
} PID_Parameter_t;

/* 接口声明：增量式 PID 控制（速度） */
void PID_Init(void);
void PID_Reset(PID_Parameter_t *status);
void PID_AttachMotor(PID_Parameter_t *status, Motor_Status_t *motor);
void PID_SetTarget(PID_Parameter_t *status, float target);
/*
 * 增量式 PID 计算一步控制量
 * cfg   : PID 参数配置
 * status: PID 运行状态（含历史项）
 * feedback: 当前速度反馈值
 * 返回值: 新的控制输出（已限幅并叠加偏置）
 */
/* 使用已绑定的 motor_status 作为反馈来源，调用时无需传入 feedback */
float PID_ComputeIncremental(const PID_Config_t *cfg, PID_Parameter_t *status);
/* 可指定外部反馈（姿态环直接用陀螺仪姿态值） */
float PID_ComputeIncrementalWithFeedback(const PID_Config_t *cfg, PID_Parameter_t *status, float feedback);

/* PID 任务循环计数，用于 DWT 统计实际帧率 */
extern volatile uint32_t g_pid_loop_counter;

typedef enum {
	PID_LOOP_SPEED = 0,
	PID_LOOP_ATTITUDE = 1,   /* 姿态（俯仰）环：输出期望速度，作为速度环目标 */
	/* 预留其它环，如 PID_LOOP_POSITION = 2, ... */
} PID_Loop_enum;

typedef enum {
	PID_MOTOR_1 = 0,
	PID_MOTOR_2 = 1,
} PID_Motor_enum;

typedef enum {
    PID_CONFIG_KP = 0,
    PID_CONFIG_KI = 1,
    PID_CONFIG_KD = 2,
    PID_CONFIG_OUT_OFFSET = 3,
    PID_CONFIG_OUT_MAX = 4,
    PID_CONFIG_OUT_MIN = 5,
	PID_CONFIG_FF_GAIN = 6,
	PID_CONFIG_REVERSE_DECAY = 7,
} PID_Config_enum;

/* 通用编辑接口：指定环与参数进行修改（线程安全） */
/* 返回 0 成功，负值表示错误（-1: 未知环， -2: 未知参数） */
/*
 * @param param选择对应config内的参数
 */
int PID_EditConfig(PID_Loop_enum loop, PID_Config_enum param, float value);

/* 可选：读取参数（线程安全） */
int PID_GetConfigParam(PID_Loop_enum loop, PID_Config_enum param, float *out_value);

int PID_GetParamTarget(PID_Loop_enum loop, PID_Motor_enum motor, float *out_value);

int PID_EditTarget(PID_Loop_enum loop, PID_Motor_enum motor, float value);

#endif
