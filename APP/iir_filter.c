#include "iir_filter.h"

/*
 * ============================================================
 * 二阶 Butterworth 低通 IIR
 *
 * Fs = 500 Hz
 * Fc = 15 Hz
 *
 * Butterworth:
 * Q = 1 / sqrt(2)
 *
 * 系数由双线性变换得到。
 * ============================================================
 */

static const float B0 = 0.0078202080f;
static const float B1 = 0.0156404161f;
static const float B2 = 0.0078202080f;

static const float A1 = -1.7347257688f;
static const float A2 = 0.7660066009f;


/*
 * 初始化滤波器
 *
 * initial_value：
 * 使用当前传感器值作为初始输出，
 * 避免刚启动时从 0 慢慢爬到真实值。
 */
void IIR2_Init(IIR2_Filter_t *filter, float initial_value)
{
    if (filter == 0)
        return;

    filter->x1 = initial_value;
    filter->x2 = initial_value;

    filter->y1 = initial_value;
    filter->y2 = initial_value;
}


/*
 * 二阶 IIR 更新
 *
 * y[n] =
 *      b0*x[n]
 *    + b1*x[n-1]
 *    + b2*x[n-2]
 *    - a1*y[n-1]
 *    - a2*y[n-2]
 */
float IIR2_Update(IIR2_Filter_t *filter, float input)
{
    if (filter == 0)
        return input;

    float output;

    output =
          B0 * input
        + B1 * filter->x1
        + B2 * filter->x2
        - A1 * filter->y1
        - A2 * filter->y2;

    /*
     * 更新历史数据
     */
    filter->x2 = filter->x1;
    filter->x1 = input;

    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}
