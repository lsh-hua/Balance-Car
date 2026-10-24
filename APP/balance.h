#ifndef BALANCE_H
#define BALANCE_H

#include "main.h"
#include "pid.h"
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "online_tune.h"

void Control_Init(void);
void Control_Loop(void);
void Motor_Update(void);
void Control_Timing_Diagnostic_Tick(void);
/* 固定参数快照；历史在线变体在控制tick中应用，跨上下文读取须保护。 */
void Control_GetTuning(OnlineTuneParams_t *out);
/* 只读代码中的毫度基础参考；控制/门禁/回执共用，不提供在线改基准接口。 */
int32_t Control_GetBasePitchMdeg(void);
/* 只读有效参考，供main与控制内部使用同一相对角基准。 */
float Control_GetPitchReferenceDeg(void);
void Control_ApplyTuning(const OnlineTuneParams_t *parameters);
void Control_ResetTuningState(void);

extern volatile uint32_t g_speed_loop_seq;
extern volatile int motor_left_pwm;
extern volatile int motor_right_pwm;
extern volatile int16_t g_target_angle_x100;
extern volatile int16_t g_current_speed_x100;
extern volatile int16_t g_balance_output;

#endif
