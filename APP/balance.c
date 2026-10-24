#include "balance.h"

extern float yaw,pitch,roll;

PID_t balance_pid;
PID_t speed_pid;
PID_t turn_pid;

volatile int motor_left_pwm;
volatile int motor_right_pwm;
volatile uint32_t g_speed_loop_seq = 0;
volatile int16_t g_target_angle_x100 = 0;
volatile int16_t g_current_speed_x100 = 0;
volatile int16_t g_balance_output = 0;

float current_speed;
float pitch_offset = 0.0f;



#define ENCODER_DIRECTION -1
#define CONTROL_TICK_DT     0.002f
#define SPEED_LOOP_PERIOD   0.010f
#define SPEED_LOOP_DIVIDER  5u
#define SPEED_TARGET_ANGLE_LIMIT 2.0f
#define SPEED_LPF_ALPHA  0.05f
/* 用户确认保留代码中的基础角0与默认trim0；实际平衡参考仍待实验验证。 */
#define PITCH_LEVEL_OFFSET_DEG    0.0f
#define PITCH_TRIM_DEG            0.0f

/* 默认恢复零速外环；离线单环回归可通过编译选项显式覆盖。 */
#ifndef ANGLE_ONLY_TEST_ENABLE
#define ANGLE_ONLY_TEST_ENABLE 0
#endif
/* 电机命令上限独立于单环/串级模式，不限制未饱和增益或轮速。 */
#define BALANCE_OUTPUT_LIMIT  3500.0f

static OnlineTuneParams_t control_tuning;
static float target_angle = 0.0f;
static uint8_t speed_loop_counter = 0;

void Control_GetTuning(OnlineTuneParams_t *out)
{
    if (out) *out = control_tuning;
}

int32_t Control_GetBasePitchMdeg(void)
{
    return (int32_t)(PITCH_LEVEL_OFFSET_DEG * 1000.0f);
}

float Control_GetPitchReferenceDeg(void)
{
    return pitch_offset;
}

void Control_ResetTuningState(void)
{
    /* SET与重新ARM共用完整复位，不能携带旧目标、分频或滤波历史。 */
    balance_pid = (PID_t){0};
    speed_pid = (PID_t){0};
    PID_Init(&balance_pid, control_tuning.kp_milli / 1000.0f, 0.0f,
             control_tuning.kd_milli / 1000.0f);
    balance_pid.integral_max = 0.0f;
    balance_pid.output_max = (float)control_tuning.output_limit;
    PID_Init(&speed_pid, 0.00250f, 0.0f, 0.0f);
    speed_pid.output_max = SPEED_TARGET_ANGLE_LIMIT;
    target_angle = 0.0f;
    speed_loop_counter = 0;
    current_speed = 0.0f;
    g_balance_output = 0;
    g_target_angle_x100 = 0;
    g_current_speed_x100 = 0;
    /* g_speed_loop_seq是本次启动的调度累计序号，SET/ARM不能使其倒退。 */
    motor_left_pwm = motor_right_pwm = 0;
}

void Control_ApplyTuning(const OnlineTuneParams_t *parameters)
{
    if (!parameters || parameters->kp_milli < 0 || parameters->kp_milli > 600000 ||
        parameters->kd_milli < 0 || parameters->kd_milli > 5000 ||
        parameters->trim_mdeg < ONLINE_TUNE_TRIM_MIN_MDEG ||
        parameters->trim_mdeg > ONLINE_TUNE_TRIM_MAX_MDEG ||
        parameters->output_limit < ONLINE_TUNE_OUTPUT_LIMIT_MIN ||
        parameters->output_limit > ONLINE_TUNE_OUTPUT_LIMIT_MAX) return;

    control_tuning = *parameters;
    /* 只改变控制参考，不写传感器姿态或固定加速度标定。 */
    pitch_offset = Control_GetBasePitchMdeg() / 1000.0f + control_tuning.trim_mdeg / 1000.0f;
    Control_ResetTuningState();
}

static int16_t Speed_ToTelemetry(float speed)
{
    float scaled = speed * 100.0f;

    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    return (int16_t)scaled;
}

void Control_Init(void)
{
    g_speed_loop_seq = 0;
    g_target_angle_x100 = 0;
    g_current_speed_x100 = 0;
    g_balance_output = 0;
    /* 使用源码固定P/D、trim和容量；默认固件不通过串口修改参数。 */
    const OnlineTuneParams_t defaults = {300000, 1800,
        (int32_t)(PITCH_TRIM_DEG * 1000.0f), (int32_t)BALANCE_OUTPUT_LIMIT};
    Control_ApplyTuning(&defaults);

//    PID_Init(&turn_pid, 2.0f, 0.0f, 0.05f);
//    turn_pid.output_max = 2000.0f;

}

void Control_Loop(void)
{
    /*
     * 实测确认：
     *   左电机 PWM -> TIM4 编码器
     *   右电机 PWM -> TIM3 编码器
     * 因此这里不能按定时器编号直接命名左右轮。
     */
    int encoder_left  = Read_Encoder_HAL(&htim4);
    int encoder_right = Read_Encoder_HAL(&htim3);
    Get_Velocity_Form_Encoder(encoder_left, encoder_right);

    float pitch_corrected = pitch - pitch_offset;

    float current_speed_raw = ENCODER_DIRECTION * (Velocity_Left + Velocity_Right) / 2.0f;

    current_speed = SPEED_LPF_ALPHA * current_speed_raw + (1.0f - SPEED_LPF_ALPHA) * current_speed;
    
    /* 外环：速度环每 5 个控制 tick 更新一次，输出目标俯仰角 */
#if !ANGLE_ONLY_TEST_ENABLE
    const float target_speed = 0.0f;
    if (++speed_loop_counter >= SPEED_LOOP_DIVIDER)
    {
        speed_loop_counter = 0;
        g_speed_loop_seq++;
        PID_Calculate_Dt(&speed_pid, target_speed, current_speed, SPEED_LOOP_PERIOD);
        target_angle = speed_pid.output;
    }
#else
    /* 角度环单独测试：冻结速度环输出，目标角度保持 0° */
    target_angle = 0.0f;
#endif

    /* 内环：直立环（角度环），输出直接控制电机 */
    PID_Calculate_Dt(&balance_pid, target_angle, pitch_corrected, CONTROL_TICK_DT);
    g_target_angle_x100 = (int16_t)(target_angle * 100.0f);
    g_current_speed_x100 = Speed_ToTelemetry(current_speed);
    g_balance_output = (int16_t)balance_pid.output;
    float speed_output = -balance_pid.output;

    float turn_diff = 0.0f;

    float left  = speed_output + turn_diff;
    float right = speed_output - turn_diff;

    if (left  >  7199.0f) left  =  7199.0f;
    if (left  < -7199.0f) left  = -7199.0f;
    if (right >  7199.0f) right =  7199.0f;
    if (right < -7199.0f) right = -7199.0f;

    if (pitch_corrected > 45.0f || pitch_corrected < -45.0f)
    {
        left  = 0;
        right = 0;
    }

    motor_left_pwm  = (int)left;
    motor_right_pwm = (int)right;
}

void Control_Timing_Diagnostic_Tick(void)
{
    static uint8_t diagnostic_counter = 0;

    if (++diagnostic_counter >= SPEED_LOOP_DIVIDER)
    {
        diagnostic_counter = 0;
        g_speed_loop_seq++;
    }
}

void Motor_Update(void)
{
    Set_Pwm(motor_left_pwm, motor_right_pwm);
}
