#include "PID.h"
#include "MotorDrive.h"  /* 访问 motor_status->SpeedInPercent 需要完整类型定义 */

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

void PID_Reset(PID_Status *status)
{
	/*
	 * 重置运行状态：清零误差与历史项，输出归零。
	 * 在开始控制或更换目标值跨度较大时，可调用以避免历史残留影响。
	 */
	if (!status) return;
	status->Error = 0.0f;
	status->prev_error_1 = 0.0f;
	status->prev_error_2 = 0.0f;
	status->last_output = 0.0f; /* 原始控制量（未叠加偏置） */
}

void PID_AttachMotor(PID_Status *status, Motor_Status_t *motor)
{
	/*
	 * 绑定电机状态指针（可选）。
	 * 若仅在外部将 PID 输出传入电机接口，此指针可不使用。
	 */
	if (!status) return;
	status->motor_status = motor;
}

void PID_SetTarget(PID_Status *status, float target)
{
	/* 设置速度目标（建议单位：百分比 -100..100） */
	if (!status) return;
	status->Target = target;
}

float PID_ComputeIncremental(const PID_Config *cfg, PID_Status *status)
{
	if (!cfg || !status) return 0.0f;

	/* 当前误差：目标 - 反馈速度
	 * 现在假定已正确绑定电机（status->motor_status 不为空），直接使用 motor_status->SpeedInPercent 作为反馈。
	 * 若未绑定，使用 0 作为退化反馈（调用者应保证已绑定）。
	 */
	float fb = 0.0f;
	if (status->motor_status) {
		fb = status->motor_status->SpeedInPercent;
	}
	float e = status->Target - fb;
	status->Error = e;
	/* 增量式 PID 三项 */
	float p_term = cfg->Kp * (e - status->prev_error_1);
	/*
	 * Ki 离散化说明：
	 * - 常见离散形式为 Ki*T*e(k)，其中 T 为采样周期；若已将 T 合入 Ki，则可直接 Ki*e(k)。
	 * - 若担心积分“过冲”，可在外层对 e 进行限幅（例如 clamp(e, ErrorIntMin, ErrorIntMax)）。
	 */
	float i_term = cfg->Ki * e; /* Ki 已包含采样周期的折算 */

	/* D 项离散近似：e(k) - 2e(k-1) + e(k-2) 对应二阶差分，等价于对速度变化的抑制 */
	float d_term = cfg->Kd * (e - 2.0f * status->prev_error_1 + status->prev_error_2);

	float delta_u = p_term + i_term + d_term;

	/* 原始输出（未加偏置），并进行限幅 */
	float raw_u = status->last_output + delta_u;
	raw_u = clamp_float(raw_u, cfg->OutMin, cfg->OutMax);

	/* 更新历史项 */
	status->prev_error_2 = status->prev_error_1;
	status->prev_error_1 = e;
	status->last_output = raw_u;

	/* 返回最终输出：叠加偏置后用于驱动（例如映射到 PWM/百分比） */
	float final_u = raw_u + cfg->OutOffset;
	return final_u;
}

