#include "filter.h"
#include <math.h>

/* 本地定义 PI 常量，避免依赖 M_PI 宏 */
static const float PI = 3.14159265358979323846f;

/* 使用双线性变换设计二阶巴特沃斯低通滤波器。
 * 使用预变形（pre-warping）和标准系数计算方法。
 */
void butterworth2_init(FilterBiquad *f, float cutoff_hz, float fs)
{
	if (!f || cutoff_hz <= 0.0f || fs <= 0.0f) return;

	float K = tanf(PI * cutoff_hz / fs);
	float K2 = K * K;
	const float sqrt2 = 1.4142135623730951f;

	float a0 = 1.0f + sqrt2 * K + K2;

	f->b0 = K2 / a0;
	f->b1 = 2.0f * f->b0;
	f->b2 = f->b0;

	f->a1 = 2.0f * (K2 - 1.0f) / a0;
	f->a2 = (1.0f - sqrt2 * K + K2) / a0;

	/* 清零 DF1 与 DF2T 的状态 */
	f->x1 = f->x2 = 0.0f;
	f->y1 = f->y2 = 0.0f;
	f->d1 = f->d2 = 0.0f;

	/* 为 Cortex-M3（无 FPU）准备 Q31 系数以便使用整数量化核心加速。
	 * 将浮点系数缩放到 Q31 范围。
	 */
	const double Q31_SCALE = 2147483648.0; /* 2^31 */
	f->q_b0 = (int32_t) (f->b0 * Q31_SCALE + (f->b0 >= 0 ? 0.5 : -0.5));
	f->q_b1 = (int32_t) (f->b1 * Q31_SCALE + (f->b1 >= 0 ? 0.5 : -0.5));
	f->q_b2 = (int32_t) (f->b2 * Q31_SCALE + (f->b2 >= 0 ? 0.5 : -0.5));
	f->q_a1 = (int32_t) (f->a1 * Q31_SCALE + (f->a1 >= 0 ? 0.5 : -0.5));
	f->q_a2 = (int32_t) (f->a2 * Q31_SCALE + (f->a2 >= 0 ? 0.5 : -0.5));

	f->q_d1 = f->q_d2 = 0;

	/* 默认在 STM32F1 上启用 Q31 路径（可由用户关闭）。 */
	f->use_q31 = 1;
}

/* 优化处理：直接式 II 转置（DF2T）。
 * DF2T 减少内存读写次数并具有较好的数值稳定性。
 */
static inline float butterworth2_process_df2t(FilterBiquad *f, float in)
{
	/* 将系数加载到寄存器 */
	const float b0 = f->b0;
	const float b1 = f->b1;
	const float b2 = f->b2;
	const float a1 = f->a1;
	const float a2 = f->a2;

	float y = b0 * in + f->d1;

	float d1 = b1 * in - a1 * y + f->d2;
	float d2 = b2 * in - a2 * y;

	f->d1 = d1;
	f->d2 = d2;

	return y;
}

/* 向后兼容的接口：内部优先使用 DF2T 以提高性能。 */
float butterworth2_process(FilterBiquad *f, float in)
{
	if (!f) return in;

	if (f->use_q31) {
		/* 将浮点输入转换为 Q31（带取整） */
		const double Q31_SCALE = 2147483648.0;
		int32_t xq = (int32_t)(in * Q31_SCALE);

		/* Q31 DF2T 内核，使用 64 位累加器（编译器会生成 SMULL 指令）
		 * y = (b0 * x) >> 31 + q_d1
		 * d1_new = (b1 * x)>>31 - (a1 * y)>>31 + q_d2
		 * d2_new = (b2 * x)>>31 - (a2 * y)>>31
		 */
		int32_t yq;
		{
			int64_t acc = (int64_t)f->q_b0 * (int64_t)xq; /* Q62 */
			int32_t part = (int32_t)(acc >> 31); /* Q31 */
			/* 与先前状态相加（Q31） */
			int64_t sum = (int64_t)part + (int64_t)f->q_d1;
			/* 饱和保护到 int32 范围 */
			if (sum > INT32_MAX) sum = INT32_MAX;
			if (sum < INT32_MIN) sum = INT32_MIN;
			yq = (int32_t)sum;

			int64_t t1 = (int64_t)f->q_b1 * (int64_t)xq;
			int64_t t2 = (int64_t)f->q_a1 * (int64_t)yq;
			int32_t d1_new = (int32_t)((t1 >> 31) - (t2 >> 31) + (int64_t)f->q_d2);

			int64_t t3 = (int64_t)f->q_b2 * (int64_t)xq;
			int64_t t4 = (int64_t)f->q_a2 * (int64_t)yq;
			int32_t d2_new = (int32_t)((t3 >> 31) - (t4 >> 31));

			f->q_d1 = d1_new;
			f->q_d2 = d2_new;
		}

		/* 将 Q31 输出转换回浮点 */
		float out = (float)((double)yq / 2147483648.0);
		return out;
	}

	return butterworth2_process_df2t(f, in);
}

void filter_reset(FilterBiquad *f)
{
	if (!f) return;
	f->x1 = f->x2 = 0.0f;
	f->y1 = f->y2 = 0.0f;
	f->d1 = f->d2 = 0.0f;
	f->q_d1 = f->q_d2 = 0;
}

/* 示例用法：
 * 在采样循环中：
 * static FilterBiquad lp;
 * butterworth2_init(&lp, 50.0f, 1000.0f); // 截止 50 Hz，采样 1 kHz
 * float filtered = butterworth2_process(&lp, raw_sample);
 */
