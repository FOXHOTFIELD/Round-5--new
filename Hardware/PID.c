#include "PID.h"
#include "comm.h"
#include "Encoder.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

/*
 * 增量式 PID 控制（速度）实现
 * 公式：
 *   Δu(k) = Kp*[e(k) - e(k-1)] + Ki*e(k) + Kd*[e(k) - 2e(k-1) + e(k-2)]
 *   u(k)  = u(k-1) + Δu(k)
 * 说明：
 * - 本实现将 Ki 的离散化（含采样周期）折算到参数中，直接乘以 e(k)
 * - 输出限幅在无偏置的原始控制量上进行；最终返回值叠加 OutOffset 用于克服静摩擦等偏置
 * - 增量式 PID 不显式保存积分量；若需限制积分“过冲”，可在使用时对误差 e 进行限幅
 *
 * 使用建议：
 * 1) 采样周期：保持固定（如 1ms/5ms），Ki/Kd 的取值需包含采样周期离散化的影响。
 * 2) 量纲统一：Target 与 feedback 建议同为“速度百分比”（-100..100），便于直接驱动电机。
 * 3) 限幅策略：OutMin/OutMax 用于限制原始输出 raw_u（不含偏置）；OutOffset 在最终输出叠加。
 * 4) 参数整定：
 *    - Kp 决定响应速度与稳态误差；过大会震荡。
 *    - Ki 消除稳态误差；过大会积累导致过冲，可减小或对 e 限幅。
 *    - Kd 抑制快速变化与噪声；编码器反馈噪声较大时可适当减小。
 * 5) 映射输出：返回值可直接用于 `Motor_SetSpeedPercent(index, out)`，或换算为 PWM/CCR。
 */

static void vPIDTask(void *pv);

static PID_Config_t PID_Config_Speed = {
    .ErrorIntMax = 100, .ErrorIntMin = -100,
    .Kp = 0.8f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .OutMax = 100, .OutMin = -100,
    .OutOffset = 0,
};

static PID_Parameter_t PID_Para_Speed;
extern Motor_Status_t motor_status[2];

void PID_Init(void)
{
    PID_Reset(&PID_Para_Speed);
    PID_AttachMotor(&PID_Para_Speed, motor_status);
    PID_SetTarget(&PID_Para_Speed, 20.0f);
    
    xTaskCreate(vPIDTask, "PIDTask", 160, NULL, tskIDLE_PRIORITY+1, NULL);
}

static inline float clamp_float(float x, float minv, float maxv)
{
	/*
	 * 简单限幅：保证值在 [minv, maxv] 区间内。
	 * 用于限制原始控制量 raw_u，避免输出超过允许范围。
	 */
	if (x > maxv) return maxv;
	if (x < minv) return minv;
	return x;
}

void PID_Reset(PID_Parameter_t *para)
{
	/*
	 * 重置运行状态：清零误差与历史项，输出归零。
	 * 在开始控制或更换目标值跨度较大时，可调用以避免历史残留影响。
	 */
	if (para == NULL)
        para = &PID_Para_Speed;

	para->Error = 0.0f;
	para->prev_error_1 = 0.0f;
	para->prev_error_2 = 0.0f;
	para->last_output = 0.0f; /* 原始控制量（未叠加偏置） */
}

void PID_AttachMotor(PID_Parameter_t *para, Motor_Status_t *motor)
{
	/*
	 * 绑定电机状态指针（可选）。
	 * 若仅在外部将 PID 输出传入电机接口，此指针可不使用。
	 */
	if (!para) return;
	para->motor_status = motor;
}

void PID_SetTarget(PID_Parameter_t *status, float target)
{
	/* 设置速度目标（建议单位：百分比 -100..100） */
	if (!status) return;
	status->Target = target;
    
    
}

float PID_ComputeIncremental(const PID_Config_t *cfg, PID_Parameter_t *status)
{
	if (!cfg || !status) return 0.0f;

	/* 性能优化：缓存频繁访问的字段到局部变量，减少结构体读写次数 */
	const float Kp = cfg->Kp;
	const float Ki = cfg->Ki;
	const float Kd = cfg->Kd;
	const float OutMin = cfg->OutMin;
	const float OutMax = cfg->OutMax;
	const float OutOffset = cfg->OutOffset;

	float prev1 = status->prev_error_1;
	float prev2 = status->prev_error_2;
	float last_out = status->last_output;

	/* 读取反馈并计算误差（尽量只读一次 motor_status 指针） */
	float fb = 0.0f;
	Motor_Status_t *ms = status->motor_status;
	if (ms) fb = ms->SpeedInPercent;

	float e = status->Target - fb;
	status->Error = e;

	/* P 项（增量形式） */
	float p_term = Kp * (e - prev1);

	/* 对积分项使用误差限幅以限制积分累积（避免风up），改为使用配置中的 ErrorIntMin/Max */
	float e_i = e;
	if (e_i > cfg->ErrorIntMax) e_i = cfg->ErrorIntMax;
	else if (e_i < cfg->ErrorIntMin) e_i = cfg->ErrorIntMin;
	float i_term = Ki * e_i;

	/* D 项二阶差分 */
	float d_term = Kd * (e - 2.0f * prev1 + prev2);

	float delta_u = p_term + i_term + d_term;

	/* 计算并限幅原始输出，尽量使用局部变量以减少写内存次数 */
	float raw_u = last_out + delta_u;
	if (raw_u > OutMax) raw_u = OutMax;
	else if (raw_u < OutMin) raw_u = OutMin;

	/* 把历史项一次性写回状态结构体（减少多次写操作） */
	status->prev_error_2 = prev1;
	status->prev_error_1 = e;
	status->last_output = raw_u;

	/* 叠加偏置并返回（偏置通常很小，直接加在最后） */
	return raw_u + OutOffset;
}

/* 新增实现：按环/参数修改配置（保持 static 配置体不暴露） */
static inline PID_Config_t * PID_GetConfigByLoop(int loop)
{
    switch (loop) {
        case PID_LOOP_SPEED: return &PID_Config_Speed;
        /* case PID_LOOP_POSITION: return &PID_Config_Position; // 以后扩展 */
        default: return NULL;
    }
}

int PID_EditConfig(PID_Loop_enum loop, PID_Param_enum param, float value)
{
    PID_Config_t *cfg = PID_GetConfigByLoop((int)loop);
    if (!cfg) return -1;

    taskENTER_CRITICAL();
    switch (param) {
        case PID_PARAM_KP: cfg->Kp = value; break;
        case PID_PARAM_KI: cfg->Ki = value; break;
        case PID_PARAM_KD: cfg->Kd = value; break;
        case PID_PARAM_OUT_OFFSET: cfg->OutOffset = value; break;
        case PID_PARAM_OUT_MAX: cfg->OutMax = value; break;
        case PID_PARAM_OUT_MIN: cfg->OutMin = value; break;
        default:
            taskEXIT_CRITICAL();
            return -2;
    }
    taskEXIT_CRITICAL();
    return 0;
}

int PID_GetConfigParam(PID_Loop_enum loop, PID_Param_enum param, float *out_value)
{
    if (!out_value) return -3;
    PID_Config_t *cfg = PID_GetConfigByLoop((int)loop);
    if (!cfg) return -1;

    taskENTER_CRITICAL();
    switch (param) {
        case PID_PARAM_KP: *out_value = cfg->Kp; break;
        case PID_PARAM_KI: *out_value = cfg->Ki; break;
        case PID_PARAM_KD: *out_value = cfg->Kd; break;
        case PID_PARAM_OUT_OFFSET: *out_value = cfg->OutOffset; break;
        case PID_PARAM_OUT_MAX: *out_value = cfg->OutMax; break;
        case PID_PARAM_OUT_MIN: *out_value = cfg->OutMin; break;
        default:
            taskEXIT_CRITICAL();
            return -2;
    }
    taskEXIT_CRITICAL();
    return 0;
}

static void vPIDTask(void *pv)
{
    (void)pv;

    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(PID_UPDATE_MS);

    for (;;)
    {

        float pct = Encoder_GetAndResetPercent();
        /* 写入共享 encoder 值，vCommTask 会合并发送 */
        Comm_encoder = pct;

        PID_Para_Speed.motor_status->SpeedInPercent = Comm_encoder;

        float out = PID_ComputeIncremental(&PID_Config_Speed, &PID_Para_Speed);
        
        Comm_out = out;

        Motor_SetSpeedPercent(1, out);

        vTaskDelayUntil(&last, period);
    }
}
