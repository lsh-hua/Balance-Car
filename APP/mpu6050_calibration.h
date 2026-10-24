#ifndef MPU6050_CALIBRATION_H
#define MPU6050_CALIBRATION_H

#include <stdint.h>

/* 芯片原始 LSB 减固定偏置后乘轴增益；物理量换算前保留浮点精度。 */
typedef struct
{
    uint32_t version;
    float bias_lsb[3];
    float gain[3];
} MPU6050_AccelCalibration_t;

extern const MPU6050_AccelCalibration_t g_mpu6050_accel_calibration;

#endif
