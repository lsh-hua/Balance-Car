#ifndef MPU6050_STARTUP_H
#define MPU6050_STARTUP_H

#include <stdint.h>

/* 阶段：0未开始，1唤醒，2配置读回，3静止校准，4启动配置与校准完成。
 * 阶段4不保证后续采样持续健康；运行故障仍由主程序独立锁存。
 * 错误计数跨本次启动的各次尝试累计，最近失败信息不被成功事务覆盖。
 */
typedef struct
{
    uint8_t attempts;
    uint8_t stage;
    uint32_t write_errors;
    uint8_t last_write_status;
    uint8_t last_write_register;
    uint32_t last_write_error;
    uint32_t verify_errors; /* 仅统计读成功但值不符；读取失败见IMU读取错误计数。 */
    uint8_t last_verify_register;
    uint8_t last_verify_value;
} MPU6050_StartupDiagnostics_t;

extern volatile MPU6050_StartupDiagnostics_t g_mpu6050_startup_diag;

#endif
