//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Madgwick 的 IMU 与 AHRS 姿态估计算法（C 实现）。
// 原始开源地址（英文）：http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// 本文件在保留原始实现的基础上，补充了完整中文注释，包括：
// 1) 使用方法、输入输出、单位与采样频率要求；
// 2) 算法核心思想（基于梯度下降的最小化测量误差）、
//    数学模型（四元数、方向余弦、加速度/磁场参考方向、误差函数与其梯度）；
// 3) 实现细节（快速反平方根、数值稳定性、归一化等）。
//
// 重要说明：
// - 本算法输出的是设备姿态四元数 q = [q0, q1, q2, q3]，代表从参考坐标系到传感器坐标系的旋转。
// - 若需要欧拉角（yaw/pitch/roll），可在应用层用四元数转换。
// - 采样频率 sampleFreq 必须与实际传感器数据更新频率一致。
// - 陀螺仪输入必须为弧度/秒（rad/s）。如为度/秒需先转换。
//
// 日期			作者			备注
// 29/09/2011	SOH Madgwick	初始发布
// 02/10/2011	SOH Madgwick	优化以降低 CPU 负载
// 19/02/2012	SOH Madgwick	磁力计测量进行归一化
// 04/12/2025	中文注释版		补充全面中文注释与数学解释
//
// 数学原理概述（简要）：
// - 姿态用单位四元数 q 表示旋转。陀螺仪给出角速度 ω（rad/s），四元数的时间导数为：
//     dq/dt = 0.5 * q ⊗ [0, ω]
//   这里 ⊗ 为四元数乘法，[0, ω] 是以角速度为虚部的纯四元数。
// - 加速度计与磁力计提供了对重力向量 g 与地磁向量 b 在传感器坐标系的观测。
//   由当前四元数 q 可预测（期望）这些向量的方向，将观测与期望的差异形成误差函数。
// - 通过对误差函数的梯度（对 q 的导数）进行下降，得到修正项 s（本实现中的 s0..s3），
//   再以比例增益 beta 将其作为反馈项加入到 dq/dt 中，抑制陀螺仪积分漂移并逼近真实姿态。
// - 每步将更新后的 q 归一化为单位四元数，以保证数值稳定。
//
// 误差函数与梯度（直观解释）：
// - 设 q 对应的旋转矩阵 R(q)，则参考重力方向在传感器系的期望值为 R(q) * g_ref，
//   其中 g_ref = [0, 0, 1]（单位方向，代表向下）。同理磁场参考方向 b_ref 由 R(q) 与地磁在地球系的分量得到。
// - 观测值（归一化后的加速度 a、磁场 m）与期望值之间的差异构成误差；
//   将该误差对 q 的导数即为梯度方向 s，沿着 -s 更新可减小误差。
// - 代码中的 s0..s3 即对应梯度方向的四元数各分量，推导细节可参考原论文与开源注释实现。
//
// 数值技巧：
// - invSqrt 使用著名的快速反平方根近似以加速归一化。
// - 所有使用到的向量在参与误差计算前都会归一化以避免 NaN 与尺度影响。
// - beta 为增益，越大越依赖加速度/磁力计（收敛快、抗陀螺漂移），越小越平滑但可能积累漂移。
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>
#include <stdint.h>

// 前置声明：fast 三角近似中会调用 invSqrt
float invSqrt(float x);

// 三角函数近似模式（可运行时切换）：
// 0=使用标准库；1=查表法（LUT）；2=多项式近似
#ifndef TRIG_APPROX_DEFAULT
#define TRIG_APPROX_DEFAULT 1
#endif
static volatile int trig_mode = TRIG_APPROX_DEFAULT;
void set_trig_approx_mode(int mode) { if (mode>=TRIG_STD && mode<=TRIG_POLY) trig_mode = mode; }

// 可调查表精度（编译期参数）：
#ifndef ASIN_LUT_SIZE
#define ASIN_LUT_SIZE 257   // [-1,1] 区间查表，线性插值

#endif

#ifndef ATAN_LUT_SIZE
#define ATAN_LUT_SIZE 257   // [0,1]  区间查表，线性插值

#endif

// 查表实现
static float asin_lut[ASIN_LUT_SIZE];
static float atan_lut[ATAN_LUT_SIZE];
static int trig_lut_ready = 0;
static void init_trig_lut(void) {
	if (trig_lut_ready) return;
	for (int i = 0; i < ASIN_LUT_SIZE; ++i) {
		float x = -1.0f + (2.0f * i) / (ASIN_LUT_SIZE - 1);
		asin_lut[i] = asinf(x);
	}
	for (int i = 0; i < ATAN_LUT_SIZE; ++i) {
		float t = (float)i / (ATAN_LUT_SIZE - 1);
		atan_lut[i] = atanf(t);
	}
	trig_lut_ready = 1;
}
static inline float lut_asinf(float x) {
	if (!trig_lut_ready) init_trig_lut();
	if (x <= -1.0f) return -M_PI / 2.0f;
	if (x >=  1.0f) return  M_PI / 2.0f;
	float pos = (x + 1.0f) * 0.5f * (ASIN_LUT_SIZE - 1);
	int i = (int)pos;
	float frac = pos - (float)i;
	if (i >= ASIN_LUT_SIZE - 1) { i = ASIN_LUT_SIZE - 2; frac = 1.0f; }
	float a0 = asin_lut[i];
	float a1 = asin_lut[i + 1];
	return a0 + (a1 - a0) * frac;
}
static inline float lut_atan_unit(float t) {
	if (!trig_lut_ready) init_trig_lut();
	if (t <= 0.0f) return 0.0f;
	if (t >= 1.0f) return atan_lut[ATAN_LUT_SIZE - 1];
	float pos = t * (ATAN_LUT_SIZE - 1);
	int i = (int)pos;
	float frac = pos - (float)i;
	if (i >= ATAN_LUT_SIZE - 1) { i = ATAN_LUT_SIZE - 2; frac = 1.0f; }
	float a0 = atan_lut[i];
	float a1 = atan_lut[i + 1];
	return a0 + (a1 - a0) * frac;
}

// 多项式近似实现
static inline float poly_atan01(float t) {
	// 经验近似：atan(t) ≈ t / (1 + 0.28 t^2), t ∈ [0,1]
	float tt = t * t;
	return t / (1.0f + 0.28f * tt);
}
static inline float poly_atan2f(float y, float x) {
	if (x == 0.0f && y == 0.0f) return 0.0f;
	float ax = x >= 0.0f ? x : -x;
	float ay = y >= 0.0f ? y : -y;
	float angle;
	if (ax > ay) {
		float t = (ax == 0.0f) ? 0.0f : (ay / ax);
		angle = poly_atan01(t);
		if (x < 0.0f) angle = (y >= 0.0f) ? (M_PI - angle) : (angle - M_PI);
		else          angle = (y >= 0.0f) ? angle : -angle;
	} else {
		float t = (ay == 0.0f) ? 1.0f : (ax / ay);
		angle = (M_PI * 0.5f) - poly_atan01(t);
		if (y < 0.0f) angle = (x >= 0.0f) ? -angle : (angle - M_PI);
		else          angle = (x >= 0.0f) ? angle : (M_PI - angle);
	}
	return angle;
}
static inline float poly_asinf(float x) {
	// 利用恒等式 asin(x) = atan2(x, sqrt(1-x^2))，sqrt 用 invSqrt 近似
	if (x <= -1.0f) return -M_PI / 2.0f;
	if (x >=  1.0f) return  M_PI / 2.0f;
	float s = 1.0f - x * x;
	if (s <= 0.0f) return (x >= 0.0f) ? (M_PI / 2.0f) : (-M_PI / 2.0f);
	float inv = invSqrt(s);
	float root = s * inv; // ≈ sqrt(s)
	return poly_atan2f(x, root);
}

// 统一包装：根据当前模式选择实现
static inline float fast_asinf(float x) {
	switch (trig_mode) {
		case TRIG_LUT:  return lut_asinf(x);
		case TRIG_POLY: return poly_asinf(x);
		default:        return asinf(x);
	}
}
static inline float fast_atan2f(float y, float x) {
	switch (trig_mode) {
		case TRIG_LUT:  {
			if (!trig_lut_ready) init_trig_lut();
			if (x == 0.0f && y == 0.0f) return 0.0f;
			float ax = x >= 0.0f ? x : -x;
			float ay = y >= 0.0f ? y : -y;
			float angle;
			if (ax > ay) {
				float t = (ax == 0.0f) ? 0.0f : (ay / ax);
				angle = lut_atan_unit(t);
				if (x < 0.0f) angle = (y >= 0.0f) ? (M_PI - angle) : (angle - M_PI);
				else          angle = (y >= 0.0f) ? angle : -angle;
			} else {
				float t = (ay == 0.0f) ? 1.0f : (ax / ay);
				angle = (M_PI * 0.5f) - lut_atan_unit(t);
				if (y < 0.0f) angle = (x >= 0.0f) ? -angle : (angle - M_PI);
				else          angle = (x >= 0.0f) ? angle : (M_PI - angle);
			}
			return angle;
		}
		case TRIG_POLY: return poly_atan2f(y, x);
		default:        return atan2f(y, x);
	}
}

//---------------------------------------------------------------------------------------------------
// Definitions（参数定义）
// sampleFreq：算法内部将陀螺积分按 1/sampleFreq 的时间步进行累计，因此必须与数据到达频率一致。
// betaDef：默认反馈增益（2*Kp）。可在运行时通过修改全局变量 beta 进行调参。

#define sampleFreq	100.0f		// 采样频率（Hz）。务必与实际采样一致，否则积分会产生误差。
#define betaDef		0.3f		// 默认算法增益（2*Kp）。越大越相信加速度/磁力计，收敛更快但动态下可能抖。
// 预计算采样周期倒数，避免在每次更新中重复除法
static const float invSampleFreq = 1.0f / sampleFreq;

//---------------------------------------------------------------------------------------------------
// Variable definitions（全局状态变量）

volatile float beta = betaDef;							// 算法增益（2*Kp）。可根据噪声/动态调整，如 0.05~0.3。
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// 初始姿态四元数（单位四元数），默认无旋转。

float yaw = 0, pitch = 0, roll = 0;

//---------------------------------------------------------------------------------------------------
// Function declarations（内部函数声明）


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update（含磁力计的完整融合）
// 输入：
// - gx, gy, gz：陀螺仪角速度（rad/s）。
// - ax, ay, az：加速度计测量（任意单位，内部归一化）。
// - mx, my, mz：磁力计测量（任意单位，内部归一化）。
// 当磁力计数据不可用（全为 0）时，自动退化为仅 IMU 更新。

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	const float dt = invSampleFreq;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	// 若磁力计无效（全为 0），避免在归一化时出现 NaN，改用仅 IMU 更新。
	float magNorm2 = mx * mx + my * my + mz * mz;
	if(magNorm2 <= 1e-12f) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	// 陀螺积分形成的四元数导数（来源：dq/dt = 0.5 * q ⊗ [0, ω] 展开后的分量表达式）
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	// 仅当加速度计非零时才计算反馈项，避免归一化产生 NaN。
	float accNorm2 = ax * ax + ay * ay + az * az;
	if(accNorm2 > 1e-12f) {

		// Normalise accelerometer measurement
		// 加速度归一化（仅方向用于误差函数）
		recipNorm = invSqrt(accNorm2);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		// 磁力计归一化（仅方向用于误差函数）
		recipNorm = invSqrt(magNorm2);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		// 预先计算的中间变量以减少重复算术开销。
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		// 地磁参考方向计算：在当前姿态下，地磁在传感器系的期望分量（hx, hy, 以及垂直分量 bz）。
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		// 使用 invSqrt 近似替代 sqrtf，提高速度：sqrt(s) ≈ s * invSqrt(s)
		{ float s = hx * hx + hy * hy; float inv = invSqrt(s); _2bx = s * inv; }
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		// 梯度下降校正：构造误差函数对 q 的梯度（s0..s3），作为反馈方向。
		// 这些表达式为对期望与观测的差异进行推导的结果，详见原作者开源实现与论文。
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		// 应用反馈：以增益 beta 沿着梯度方向修正陀螺积分，使姿态向测量一致的方向逼近。
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	// 以采样周期（1/sampleFreq）积分四元数导数，得到新的姿态。
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	// 归一化为单位四元数，避免数值漂移导致长度偏离 1。
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update（仅用陀螺仪+加速度计，无磁力计航向参考）
// 在没有磁力计时，算法依赖加速度作为重力方向参考，无法确定绝对航向（yaw）。

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	const float dt = invSampleFreq;

	// Rate of change of quaternion from gyroscope
	// 陀螺积分的四元数导数（推导同上）
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	// 仅在加速度非零时进行反馈计算。
	float accNorm2 = ax * ax + ay * ay + az * az;
	if(accNorm2 > 1e-12f) {

		// Normalise accelerometer measurement
		// 加速度归一化（仅方向）
		recipNorm = invSqrt(accNorm2);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		// 预计算中间变量。
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		// 梯度下降校正项（无磁力计，因此仅基于重力方向误差）。
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		// 应用反馈：抑制陀螺积分漂移，使姿态更贴近重力方向。
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	// 按采样周期积分四元数导数。
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	// 姿态归一化。
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root（快速反平方根）
// 参考：https://en.wikipedia.org/wiki/Fast_inverse_square_root
// 用途：将向量归一化时需要 1/sqrt(v·v)，该近似可显著提高速度。
// 注意：该算法为近似值，配合一次牛顿迭代（下面的 1.5f - ...）可提高精度。

float invSqrt(float x) {
	float halfx = 0.5f * x;
	union { float f; uint32_t i; } conv;
	conv.f = x;
	conv.i = 0x5f3759dfu - (conv.i >> 1);
	float y = conv.f;
	// One Newton-Raphson iteration to improve accuracy
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
// 输入：单位四元数 q0..q3
// 输出：yaw/pitch/roll（弧度）
void quat_to_euler_zyx(float q0, float q1, float q2, float q3,
					   float* yaw, float* pitch, float* roll) {
	float siny_cosp = 2.0f * (q0*q3 + q1*q2);
	float cosy_cosp = 1.0f - 2.0f * (q2*q2 + q3*q3);
	*yaw = fast_atan2f(siny_cosp, cosy_cosp);

    float sinp = 2.0f * (q0*q2 - q3*q1);
	if (sinp >= 1.0f)      *pitch =  M_PI / 2.0f;   // clamp
	else if (sinp <= -1.0f)*pitch = -M_PI / 2.0f;
	else                   *pitch = fast_asinf(sinp);

    float sinr_cosp = 2.0f * (q0*q1 + q2*q3);
    float cosr_cosp = 1.0f - 2.0f * (q1*q1 + q2*q2);
	*roll = fast_atan2f(sinr_cosp, cosr_cosp);
}
