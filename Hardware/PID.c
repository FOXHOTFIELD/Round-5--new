#include "PID.h"
#include "comm.h"
#include "Encoder.h"
#include "MPU6050.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

static inline float fabsf_local(float x) { return (x >= 0.0f) ? x : -x; }

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

#define PID_SPEED_MOTOR_COUNT 2

static PID_Config_t PID_Config_Speed = {
    .ErrorIntMax = 10000, .ErrorIntMin = -10000,
    .Kp = 1.2f,   /* 再提比例，加快跟随 */
    .Ki = 0.005f,  /* 1ms 采样下保持原先积分时间常数 */
    .Kd = 1.6f,   /* 1ms 采样下保持原先微分时间常数 */
    .OutMax = 100, .OutMin = -100,
    .OutOffset = 0,
    .FfGain = 0.65f,  /* 增强目标前馈，减少纯 PID 耗时 */
    .ReverseDecay = 0.6f,
};

/* 姿态环（俯仰角 -> 期望速度），输出直接作为速度环 Target 使用 */
static PID_Config_t PID_Config_Attitude = {
    .ErrorIntMax = 15, .ErrorIntMin = -15,
    .Kp = 3.0f,   /* 提高比例，加快姿态纠偏 */
    .Ki = 0.02f,  /* 1ms 采样下保持原积分时间常数 */
    .Kd = 0.0f,   /* 提高微分，抑制快速倾倒时的尖峰 */
    .OutMax = 100, .OutMin = -100, /* 允许更快的姿态输出幅度 */
    .OutOffset = 0,
    .FfGain = 0.0f,
    .ReverseDecay = 0.6f, /* 误差翻转时快速衰减历史项/输出，减弱“打摆” */
};

static PID_Parameter_t PID_Param_Speed[PID_SPEED_MOTOR_COUNT];
static PID_Parameter_t PID_Param_Attitude;
extern Motor_Status_t motor_status[2];

volatile uint32_t g_pid_loop_counter = 0; /* PID 任务循环计数供 DWT 统计 */

void PID_Init(void)
{
    for (int i = 0; i < PID_SPEED_MOTOR_COUNT; ++i)
    {
        PID_Reset(&PID_Param_Speed[i]);
        PID_AttachMotor(&PID_Param_Speed[i], motor_status + i);
    }

    /* 姿态环：默认目标为 0° 直立，反馈由 MPU6050 写入 Comm_pitch */
    PID_Reset(&PID_Param_Attitude);
    PID_SetTarget(&PID_Param_Attitude, 0.0f);
    
    xTaskCreate(vPIDTask, "PIDTask", 160, NULL, tskIDLE_PRIORITY+2, NULL);
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
    {
        for (int i = 0; i < PID_SPEED_MOTOR_COUNT; ++i)
        {
            PID_Reset(&PID_Param_Speed[i]);
        }
        PID_Reset(&PID_Param_Attitude);
        return;
    }

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

static float PID_ComputeIncremental_Internal(const PID_Config_t *cfg, PID_Parameter_t *status, float feedback, uint8_t use_feedback_override)
{
    if (!cfg || !status) return 0.0f;

    /* 性能优化：缓存频繁访问的字段到局部变量，减少结构体读写次数 */
    const float Kp = cfg->Kp;
    const float Ki = cfg->Ki;
    const float Kd = cfg->Kd;
    const float OutMin = cfg->OutMin;
    const float OutMax = cfg->OutMax;
    const float OutOffset = cfg->OutOffset;
    const float FfGain = cfg->FfGain;
    const float RevDecay = cfg->ReverseDecay;

    float prev1 = status->prev_error_1;
    float prev2 = status->prev_error_2;
    float last_out = status->last_output;

    /* 读取反馈：默认用 motor_status 速度；姿态环走外部反馈 */
    float fb = 0.0f;
    if (use_feedback_override) {
        fb = feedback;
    } else {
        Motor_Status_t *ms = status->motor_status;
        if (ms) fb = ms->SpeedInPercent;
    }

    float e = status->Target - fb;
    status->Error = e;

    /* 速度环专用：目标很小时快速衰减历史输出，避免“目标到 0 但输出还在大幅度” */
    if (!use_feedback_override) {
        const float tgt_small = 5.0f;  /* 速度目标小于 5% 视为需要快速收敛 */
        const float err_small = 10.0f; /* 误差也不大时才清，避免干扰大幅纠偏 */
        if (fabsf_local(status->Target) < tgt_small && fabsf_local(e) < err_small) {
            prev1 *= 0.35f;
            prev2 *= 0.35f;
            last_out *= 0.35f;
        }
    }

    /* 姿态环近零角度不再设置死区衰减，保持线性响应 */

    /* 误差反转时衰减历史项，降低积累带来的拖尾；小误差区不衰减以避免“掉头下坠” */
    float mag = 0.5f * (fabsf_local(e) + fabsf_local(prev1));
    if ((e * prev1) < 0.0f && mag > 1.0f && RevDecay > 0.0f && RevDecay < 1.0f) {
        prev1 *= RevDecay;
        prev2 *= RevDecay;
        last_out *= RevDecay;
    }

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

    /* 前馈按目标比例叠加，PID 只对剩余部分工作 */
    float ff = FfGain * status->Target;

    /* 计算并限幅原始输出（含前馈） */
    float composed = last_out + delta_u + ff;
    composed = clamp_float(composed, OutMin, OutMax);

    /* 把历史项一次性写回状态结构体（减少多次写操作） */
    status->prev_error_2 = prev1;
    status->prev_error_1 = e;
    status->last_output = composed - ff; /* 保存 PID 部分，避免前馈重复积分 */

    /* 叠加偏置并返回（偏置通常很小，直接加在最后） */
    return composed + OutOffset;
}

float PID_ComputeIncremental(const PID_Config_t *cfg, PID_Parameter_t *status)
{
    return PID_ComputeIncremental_Internal(cfg, status, 0.0f, 0);
}

float PID_ComputeIncrementalWithFeedback(const PID_Config_t *cfg, PID_Parameter_t *status, float feedback)
{
    return PID_ComputeIncremental_Internal(cfg, status, feedback, 1);
}

/* 新增实现：按环/参数修改配置（保持 static 配置体不暴露） */
static inline PID_Config_t * PID_GetConfigByLoop(int loop)
{
    switch (loop) {
        case PID_LOOP_SPEED: return &PID_Config_Speed;
        case PID_LOOP_ATTITUDE: return &PID_Config_Attitude;
        /* case PID_LOOP_POSITION: return &PID_Config_Position; // 以后扩展 */
        default: return NULL;
    }
}
static inline PID_Parameter_t * PID_GetParameterByLoop(PID_Loop_enum loop, PID_Motor_enum motor)
{
    if (loop == PID_LOOP_SPEED) {
        if (motor == PID_MOTOR_1) return &PID_Param_Speed[0];
        if (motor == PID_MOTOR_2) return &PID_Param_Speed[1];
    }
    if (loop == PID_LOOP_ATTITUDE) {
        (void)motor; /* 姿态环与具体电机无关，忽略电机索引 */
        return &PID_Param_Attitude;
    }
    return NULL;
}

int PID_EditConfig(PID_Loop_enum loop, PID_Config_enum param, float value)
{
    PID_Config_t *cfg = PID_GetConfigByLoop((int)loop);
    if (!cfg) return -1;

    taskENTER_CRITICAL();
    switch (param) {
        case PID_CONFIG_KP: cfg->Kp = value; break;
        case PID_CONFIG_KI: cfg->Ki = value; break;
        case PID_CONFIG_KD: cfg->Kd = value; break;
        case PID_CONFIG_OUT_OFFSET: cfg->OutOffset = value; break;
        case PID_CONFIG_OUT_MAX: cfg->OutMax = value; break;
        case PID_CONFIG_OUT_MIN: cfg->OutMin = value; break;
        case PID_CONFIG_FF_GAIN: cfg->FfGain = value; break;
        case PID_CONFIG_REVERSE_DECAY: cfg->ReverseDecay = value; break;
        default:
            taskEXIT_CRITICAL();
            return -2;
    }
    taskEXIT_CRITICAL();
    return 0;
}


int PID_GetParamTarget(PID_Loop_enum loop, PID_Motor_enum motor, float *out_value)
{
    if (!out_value) return -3;
    PID_Parameter_t *param = PID_GetParameterByLoop(loop, motor);
    if (!param) return -1;

    taskENTER_CRITICAL();
        *out_value = param->Target;
    taskEXIT_CRITICAL();

    return 0;
}

int PID_GetConfigParam(PID_Loop_enum loop, PID_Config_enum param, float *out_value)
{
    if (!out_value) return -3;
    PID_Config_t *cfg = PID_GetConfigByLoop((int)loop);
    if (!cfg) return -1;

    taskENTER_CRITICAL();
    switch (param) {
        case PID_CONFIG_KP: *out_value = cfg->Kp; break;
        case PID_CONFIG_KI: *out_value = cfg->Ki; break;
        case PID_CONFIG_KD: *out_value = cfg->Kd; break;
        case PID_CONFIG_OUT_OFFSET: *out_value = cfg->OutOffset; break;
        case PID_CONFIG_OUT_MAX: *out_value = cfg->OutMax; break;
        case PID_CONFIG_OUT_MIN: *out_value = cfg->OutMin; break;
        case PID_CONFIG_FF_GAIN: *out_value = cfg->FfGain; break;
        case PID_CONFIG_REVERSE_DECAY: *out_value = cfg->ReverseDecay; break;
        default:
            taskEXIT_CRITICAL();
            return -2;
    }
    taskEXIT_CRITICAL();
    return 0;
}

int PID_EditTarget(PID_Loop_enum loop, PID_Motor_enum motor, float value)
{
    PID_Parameter_t *param = PID_GetParameterByLoop(loop, motor);
    if (!param) return -1;

    taskENTER_CRITICAL();
        PID_SetTarget(param, value);
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
        /* 姿态环内环：读取俯仰角，直接输出到电机 */
        float pitch_deg = 0.0f;
        taskENTER_CRITICAL();
            pitch_deg = Comm_pitch;
        taskEXIT_CRITICAL();

        /* 直接用姿态环（pitch -> 电机输出）作为内环 */
        float out = PID_ComputeIncrementalWithFeedback(&PID_Config_Attitude, &PID_Param_Attitude, pitch_deg);
        Comm_pose_tag = pitch_deg;

        /* 写入共享 encoder 值，vCommTask 会合并发送 */
        float pct = Encoder_GetAndResetPercent(ENCODER_A);
        Comm_encoder_a = pct;
        PID_Param_Speed[0].motor_status->SpeedInPercent = pct;

        pct = Encoder_GetAndResetPercent(ENCODER_B);
        Comm_encoder_b = pct;
        PID_Param_Speed[1].motor_status->SpeedInPercent = pct;


        Comm_out_a = out;
        Comm_out_b = out;
        Motor_SetSpeedPercent(1, out);
        Motor_SetSpeedPercent(2, out);

        g_pid_loop_counter++;

        vTaskDelayUntil(&last, period);
    }
}
