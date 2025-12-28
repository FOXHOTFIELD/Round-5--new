/* filter.h
 * 二阶巴特沃斯双二阶节低通滤波器接口
 */
#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

typedef struct {
    float b0, b1, b2; /* 前馈系数 */
    float a1, a2;     /* 反馈系数（假定 a0 = 1） */
    /* 状态：为兼容性保留两种形式。
     * DF1 使用 x1,x2,y1,y2；优化后的 DF2T 使用 d1,d2。
     */
    float x1, x2;     /* 上一输入（用于 DF1 兼容） */
    float y1, y2;     /* 上一输出（用于 DF1 兼容） */
    float d1, d2;     /* DF2 转置形式的状态（推荐） */
    /* 纯浮点实现：未包含任何 Q31 / 定点相关字段 */
} FilterBiquad;

/* 初始化一个二阶巴特沃斯低通双二阶节。
 * cutoff_hz：截止频率（Hz）
 * fs：采样率（Hz）
 */
void butterworth2_init(FilterBiquad *f, float cutoff_hz, float fs);

/* 对单个采样进行滤波，返回滤波后的值 */
float butterworth2_process(FilterBiquad *f, float in);

/* 将滤波器状态清零 */
void filter_reset(FilterBiquad *f);

#endif /* __FILTER_H */
