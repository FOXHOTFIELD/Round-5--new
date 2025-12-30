#include "MotorDrive.h"

#pragma once

#ifndef __PID_H
#define __PID_H



typedef struct PID_Config
{
	float Kp;
	float Ki;
	float Kd;
	
	float ErrorIntMax;
	float ErrorIntMin;
	
	float OutMax;
	float OutMin;
	
	float OutOffset;
}PID_Config;

///* 前向声明：与电机驱动中的状态结构保持一致 */
//typedef struct Motor_Status_t Motor_Status_t;

typedef struct PID_Status
{
    float Target;
	Motor_Status_t *motor_status;
	float Error;

	/* 增量式 PID 需要保存历史误差与上一周期输出 */
	float prev_error_1;   /* e(k-1) */
	float prev_error_2;   /* e(k-2) */
	float last_output;    /* u(k-1) */
} PID_Status;

/* 接口声明：增量式 PID 控制（速度） */
void PID_Reset(PID_Status *status);
void PID_AttachMotor(PID_Status *status, Motor_Status_t *motor);
void PID_SetTarget(PID_Status *status, float target);
/*
 * 增量式 PID 计算一步控制量
 * cfg   : PID 参数配置
 * status: PID 运行状态（含历史项）
 * feedback: 当前速度反馈值
 * 返回值: 新的控制输出（已限幅并叠加偏置）
 */
/* 使用已绑定的 motor_status 作为反馈来源，调用时无需传入 feedback */
float PID_ComputeIncremental(const PID_Config *cfg, PID_Status *status);



#endif
