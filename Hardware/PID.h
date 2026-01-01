#pragma once

#ifndef __PID_H
#define __PID_H

#include "MotorDrive.h"

#define PID_UPDATE_MS (10)

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

typedef enum {
    PID_LOOP_SPEED = 0,
    /* 预留其它环，如 PID_LOOP_POSITION = 1, ... */
} PID_Loop_enum;

typedef enum {
    PID_PARAM_KP = 0,
    PID_PARAM_KI = 1,
    PID_PARAM_KD = 2,
    PID_PARAM_OUT_OFFSET = 3,
    PID_PARAM_OUT_MAX = 4,
    PID_PARAM_OUT_MIN = 5,
} PID_Param_enum;

/* 通用编辑接口：指定环与参数进行修改（线程安全） */
/* 返回 0 成功，负值表示错误（-1: 未知环， -2: 未知参数） */
int PID_EditConfig(PID_Loop_enum loop, PID_Param_enum param, float value);

/* 可选：读取参数（线程安全） */
int PID_GetConfigParam(PID_Loop_enum loop, PID_Param_enum param, float *out_value);


#endif
