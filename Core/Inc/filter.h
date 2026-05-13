#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 滑动平均滤波器结构 */
typedef struct {
    float* buffer;       /* 数据缓冲区 */
    uint16_t window_size;/* 窗口大小 */
    uint16_t index;      /* 当前索引 */
    float sum;           /* 累加和 */
    uint8_t is_full;     /* 缓冲区是否已满 */
} MovingAvgFilter_t;

/* 一阶低通滤波器结构 */
typedef struct {
    float alpha;         /* 滤波系数 (0 < alpha < 1) */
    float prev_output;   /* 上一次输出 */
    uint8_t initialized; /* 初始化标志 */
} LowPassFilter_t;

/* 陷波滤波器结构 (二阶IIR) */
typedef struct {
    float b0, b1, b2;   /* 分子系数 */
    float a1, a2;       /* 分母系数 */
    float x1, x2;       /* 输入历史 */
    float y1, y2;       /* 输出历史 */
    uint8_t initialized; /* 初始化标志 */
} NotchFilter_t;

/* 卡尔曼滤波器结构 (1维) */
typedef struct {
    float x;             /* 状态估计 */
    float P;             /* 协方差估计 */
    float Q;             /* 过程噪声协方差 */
    float R;             /* 测量噪声协方差 */
    float K;             /* 卡尔曼增益 */
    uint8_t initialized; /* 初始化标志 */
} KalmanFilter_t;

/* 互补滤波器结构 - 用于IMU姿态融合 */
typedef struct {
    float angle;         /* 融合后的角度 */
    float gyro_bias;     /* 陀螺仪零偏 */
    float alpha;         /* 加速度计权重 (0-1) */
    float dt;            /* 采样时间间隔 */
    uint8_t initialized; /* 初始化标志 */
} ComplementaryFilter_t;

/* Alpha-Beta滤波器结构 - 用于IMU数据跟踪 */
typedef struct {
    float x;             /* 位置/角度估计 */
    float x_dot;           /* 速度/角速度估计 */
    float alpha;         /* Alpha系数 */
    float beta;          /* Beta系数 */
    float dt;            /* 采样时间间隔 */
    uint8_t initialized; /* 初始化标志 */
} AlphaBetaFilter_t;

/* 滑动平均滤波器函数 */
void MovingAvg_Init(MovingAvgFilter_t* filter, float* buffer, uint16_t window_size);
float MovingAvg_Update(MovingAvgFilter_t* filter, float input);
void MovingAvg_Reset(MovingAvgFilter_t* filter);

/* 一阶低通滤波器函数 */
void LowPass_Init(LowPassFilter_t* filter, float alpha);
void LowPass_SetAlpha(LowPassFilter_t* filter, float alpha);
float LowPass_Update(LowPassFilter_t* filter, float input);
void LowPass_Reset(LowPassFilter_t* filter);

/* 陷波滤波器函数 */
void Notch_Init(NotchFilter_t* filter, float sample_freq, float notch_freq, float bandwidth);
float Notch_Update(NotchFilter_t* filter, float input);
void Notch_Reset(NotchFilter_t* filter);

/* 卡尔曼滤波器函数 */
void Kalman_Init(KalmanFilter_t* filter, float Q, float R, float initial_x, float initial_P);
float Kalman_Update(KalmanFilter_t* filter, float measurement);
void Kalman_Reset(KalmanFilter_t* filter);

/* 互补滤波器函数 */
void Complementary_Init(ComplementaryFilter_t* filter, float alpha, float dt);
float Complementary_Update(ComplementaryFilter_t* filter, float gyro_rate, float acc_angle);
void Complementary_Reset(ComplementaryFilter_t* filter);

/* Alpha-Beta滤波器函数 */
void AlphaBeta_Init(AlphaBetaFilter_t* filter, float alpha, float beta, float dt);
void AlphaBeta_Update(AlphaBetaFilter_t* filter, float measurement);
void AlphaBeta_Reset(AlphaBetaFilter_t* filter);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H__ */
