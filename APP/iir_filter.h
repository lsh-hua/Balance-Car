#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * ============================================================
 * 二阶 Butterworth 低通 IIR
 *
 * 采样率：500 Hz
 * 截止频率：15 Hz
 *
 * 差分方程：
 *
 * y[n] = b0*x[n]
 *      + b1*x[n-1]
 *      + b2*x[n-2]
 *      - a1*y[n-1]
 *      - a2*y[n-2]
 *
 * 注意：
 * 这里的 a1、a2 已经按照标准形式保存。
 * ============================================================
 */

typedef struct
{
    float x1;
    float x2;

    float y1;
    float y2;
} IIR2_Filter_t;


/* 初始化一个二阶 IIR 滤波器 */
void IIR2_Init(IIR2_Filter_t *filter, float initial_value);


/* 输入一个采样点，返回一个滤波后的采样点 */
float IIR2_Update(IIR2_Filter_t *filter, float input);


#ifdef __cplusplus
}
#endif

#endif
