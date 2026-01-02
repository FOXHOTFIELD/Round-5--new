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

	/* 仅使用浮点实现：无定点/Q31 系数或状态 */
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

/* (已移除 Q31 内联汇编实现 - 使用纯浮点路径) */

/* 向后兼容的接口：内部优先使用 DF2T 以提高性能。 */
float butterworth2_process(FilterBiquad *f, float in)
{
	if (!f) return in;
	return butterworth2_process_df2t(f, in);
}

void filter_reset(FilterBiquad *f)
{
	if (!f) return;
	f->x1 = f->x2 = 0.0f;
	f->y1 = f->y2 = 0.0f;
	f->d1 = f->d2 = 0.0f;
	/* 纯浮点实现，无 Q31 状态 */
}

/* 示例用法：
 * 在采样循环中：
 * static FilterBiquad lp;
 * butterworth2_init(&lp, 50.0f, 1000.0f); // 截止 50 Hz，采样 1 kHz
 * float filtered = butterworth2_process(&lp, raw_sample);
 */
