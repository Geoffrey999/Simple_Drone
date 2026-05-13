
#include "filter.h"
#include <string.h>

/* ========================================================
   1. Moving Average Filter
   ======================================================== */

void MovingAvg_Init(MovingAvgFilter_t* filter, float* buffer, uint16_t window_size)
{
    if (filter == NULL || buffer == NULL || window_size == 0) {
        return;
    }
    filter->buffer = buffer;
    filter->window_size = window_size;
    filter->index = 0;
    filter->sum = 0.0f;
    filter->is_full = 0;
    memset(buffer, 0, window_size * sizeof(float));
}

float MovingAvg_Update(MovingAvgFilter_t* filter, float input)
{
    if (filter == NULL || filter->buffer == NULL) {
        return input;
    }
    
    filter->sum -= filter->buffer[filter->index];
    filter->sum += input;
    filter->buffer[filter->index] = input;
    
    filter->index++;
    if (filter->index >= filter->window_size) {
        filter->index = 0;
        filter->is_full = 1;
    }
    
    if (filter->is_full) {
        return filter->sum / (float)filter->window_size;
    } else {
        return filter->sum / (float)filter->index;
    }
}

void MovingAvg_Reset(MovingAvgFilter_t* filter)
{
    if (filter == NULL || filter->buffer == NULL) {
        return;
    }
    filter->index = 0;
    filter->sum = 0.0f;
    filter->is_full = 0;
    memset(filter->buffer, 0, filter->window_size * sizeof(float));
}

/* ========================================================
   2. First-Order Low Pass Filter
   ======================================================== */

void LowPass_Init(LowPassFilter_t* filter, float alpha)
{
    if (filter == NULL) {
        return;
    }
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    filter->alpha = alpha;
    filter->prev_output = 0.0f;
    filter->initialized = 0;
}

void LowPass_SetAlpha(LowPassFilter_t* filter, float alpha)
{
    if (filter == NULL) {
        return;
    }
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    filter->alpha = alpha;
}

float LowPass_Update(LowPassFilter_t* filter, float input)
{
    if (filter == NULL) {
        return input;
    }
    
    if (!filter->initialized) {
        filter->prev_output = input;
        filter->initialized = 1;
        return input;
    }
    
    float output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = output;
    return output;
}

void LowPass_Reset(LowPassFilter_t* filter)
{
    if (filter == NULL) {
        return;
    }
    filter->prev_output = 0.0f;
    filter->initialized = 0;
}

/* ========================================================
   3. Notch Filter
   ======================================================== */

void Notch_Init(NotchFilter_t* filter, float sample_freq, float notch_freq, float bandwidth)
{
    if (filter == NULL) {
        return;
    }
    
    float omega0 = 2.0f * M_PI * notch_freq / sample_freq;
    float alpha = sinf(omega0) * sinhf(logf(2.0f) / 2.0f * bandwidth * omega0 / sinf(omega0));
    
    float cos_omega0 = cosf(omega0);
    
    filter->b0 = 1.0f;
    filter->b1 = -2.0f * cos_omega0;
    filter->b2 = 1.0f;
    
    float a0 = 1.0f + alpha;
    filter->a1 = -2.0f * cos_omega0;
    filter->a2 = 1.0f - alpha;
    
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;
    
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->initialized = 0;
}

float Notch_Update(NotchFilter_t* filter, float input)
{
    if (filter == NULL) {
        return input;
    }
    
    if (!filter->initialized) {
        filter->x1 = input;
        filter->x2 = input;
        filter->y1 = input;
        filter->y2 = input;
        filter->initialized = 1;
        return input;
    }
    
    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                   - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}

void Notch_Reset(NotchFilter_t* filter)
{
    if (filter == NULL) {
        return;
    }
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->initialized = 0;
}

/* ========================================================
   4. Kalman Filter
   ======================================================== */

void Kalman_Init(KalmanFilter_t* filter, float Q, float R, float initial_x, float initial_P)
{
    if (filter == NULL) {
        return;
    }
    filter->Q = Q;
    filter->R = R;
    filter->x = initial_x;
    filter->P = initial_P;
    filter->K = 0.0f;
    filter->initialized = 1;
}

float Kalman_Update(KalmanFilter_t* filter, float measurement)
{
    if (filter == NULL || !filter->initialized) {
        return measurement;
    }
    
    filter->P = filter->P + filter->Q;
    filter->K = filter->P / (filter->P + filter->R);
    filter->x = filter->x + filter->K * (measurement - filter->x);
    filter->P = (1.0f - filter->K) * filter->P;
    
    return filter->x;
}

void Kalman_Reset(KalmanFilter_t* filter)
{
    if (filter == NULL) {
        return;
    }
    filter->x = 0.0f;
    filter->P = 1.0f;
    filter->K = 0.0f;
    filter->initialized = 0;
}

/* ========================================================
   5. Complementary Filter
   ======================================================== */

void Complementary_Init(ComplementaryFilter_t* filter, float alpha, float dt)
{
    if (filter == NULL) {
        return;
    }
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    filter->alpha = alpha;
    filter->dt = dt;
    filter->angle = 0.0f;
    filter->gyro_bias = 0.0f;
    filter->initialized = 0;
}

float Complementary_Update(ComplementaryFilter_t* filter, float gyro_rate, float acc_angle)
{
    if (filter == NULL) {
        return acc_angle;
    }
    
    if (!filter->initialized) {
        filter->angle = acc_angle;
        filter->initialized = 1;
        return acc_angle;
    }
    
    float gyro_corrected = gyro_rate - filter->gyro_bias;
    float angle_gyro = filter->angle + gyro_corrected * filter->dt;
    
    filter->angle = (1.0f - filter->alpha) * angle_gyro + filter->alpha * acc_angle;
    
    return filter->angle;
}

void Complementary_Reset(ComplementaryFilter_t* filter)
{
    if (filter == NULL) {
        return;
    }
    filter->angle = 0.0f;
    filter->gyro_bias = 0.0f;
    filter->initialized = 0;
}

/* ========================================================
   6. Alpha-Beta Filter
   ======================================================== */

void AlphaBeta_Init(AlphaBetaFilter_t* filter, float alpha, float beta, float dt)
{
    if (filter == NULL) {
        return;
    }
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    if (beta < 0.0f) beta = 0.0f;
    if (beta > 2.0f) beta = 2.0f;
    filter->alpha = alpha;
    filter->beta = beta;
    filter->dt = dt;
    filter->x = 0.0f;
    filter->x_dot = 0.0f;
    filter->initialized = 0;
}

void AlphaBeta_Update(AlphaBetaFilter_t* filter, float measurement)
{
    if (filter == NULL) {
        return;
    }
    
    if (!filter->initialized) {
        filter->x = measurement;
        filter->x_dot = 0.0f;
        filter->initialized = 1;
        return;
    }
    
    float x_pred = filter->x + filter->x_dot * filter->dt;
    float residual = measurement - x_pred;
    
    filter->x = x_pred + filter->alpha * residual;
    filter->x_dot = filter->x_dot + filter->beta * residual / filter->dt;
}

void AlphaBeta_Reset(AlphaBetaFilter_t* filter)
{
    if (filter == NULL) {
        return;
    }
    filter->x = 0.0f;
    filter->x_dot = 0.0f;
    filter->initialized = 0;
}
