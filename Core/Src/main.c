/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "mpu6050.h"
#include "pid.h"
#include "motor.h"
#include "balance.h"
#include "mpu6050_calibration.h"
#include "mpu6050_startup.h"
#include "online_tune.h"
#include <float.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * 闭环控制安全开关：
 * 0 = 只采集姿态并发送串口，不驱动电机
 * 1 = 执行 PID 并更新电机 PWM
 *
 * 默认取消在线调参；配置、校准和初始采样完成后自动执行闭环。
 * IMU故障或相对倾角严格超出正负45度后锁存停机，复位才可恢复。
 * 下载/复位前仍须架空并固定，上电校准后车轮可能立即转动。
 * 显式编译在线测试变体时，仍须ARM及持续心跳，不能把两种模式混用。
 */
#define BALANCE_CONTROL_ENABLE 1

/*
 * Phase 2 开环阶跃测试：
 * 0 = 电机保持停止，只发送扩展遥测帧
 * 1 = 执行单电机 PWM 阶跃序列
 *
 * 测试前必须确认车轮离地。PHASE2_TEST_MOTOR=0 测左电机，=1 测右电机。
 */
#define PHASE2_MOTOR_STEP_TEST 0
#define PHASE2_TEST_MOTOR      1
#define PHASE2_STEP_PWM        1000

/* 两种模式都会写电机，禁止同时启用，避免阶跃覆盖闭环输出。 */
#if BALANCE_CONTROL_ENABLE && PHASE2_MOTOR_STEP_TEST
#error "闭环控制与电机阶跃测试不能同时启用"
#endif

/* 时序诊断：打开后把控制 tick 和速度环序号加入遥测帧 */
#define TIMING_DIAGNOSTIC_ENABLE 1

/* IMU 诊断：主循环输出寄存器快照、校准和读取状态；验证后改回 0。 */
#define UART_BOOT_DIAGNOSTIC_ENABLE 1

#if ONLINE_TUNING_ENABLE && (!BALANCE_CONTROL_ENABLE || PHASE2_MOTOR_STEP_TEST || !TIMING_DIAGNOSTIC_ENABLE)
#error "在线调参需要闭环总开关1、阶跃0、61字节诊断开启"
#endif

#if TIMING_DIAGNOSTIC_ENABLE
#define TELEMETRY_TX_DIVIDER 5u
#else
#define TELEMETRY_TX_DIVIDER 3u
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if ONLINE_TUNING_ENABLE
static uint8_t g_tuning_rx_byte;
static volatile uint8_t g_tuning_rx_restart_pending;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct __attribute__((packed)) {
    uint8_t  header[2];     // 0xAA 0x55，帧同步头
    uint32_t timestamp_ms;  // 从启动开始的毫秒数
    int16_t  accel[3];      // 芯片原码，X/Y/Z，单位LSB
    int16_t  gyro[3]; 	// X, Y, Z

		int16_t  accel_filtered[3]; // 固定标定后低通，g乘16384编码
		int16_t  gyro_filtered[3];

		int16_t  pitch_x100;       // Pitch × 100
		int16_t  roll_x100;        // Roll × 100

		int16_t  pwm_left;
		int16_t  pwm_right;
		int32_t  encoder_left;
		int32_t  encoder_right;

#if TIMING_DIAGNOSTIC_ENABLE
		uint32_t control_tick_seq;
		uint32_t speed_loop_seq;
		int16_t  target_angle_x100;
		int16_t  current_speed_x100;
		int16_t  balance_output;
#endif
	
		uint8_t  checksum;      // 简单校验和
} DataFrame_t;

static volatile uint32_t g_tick_ms = 0;       // 中断计数器
static volatile uint8_t  g_frame_ready = 0;   // 数据就绪标志
static DataFrame_t       g_frame;             // 当前帧（ISR写，主循环读）
static volatile uint32_t g_control_tick_seq = 0;
/* 故障锁存后只保留轻量时序与遥测；排障并复位后才重新启动采样。 */
static volatile uint8_t g_imu_fault_latched = 1u;
#if !ONLINE_TUNING_ENABLE
/* 自动运行不等于倾倒后自动重启；复位是唯一解除途径。 */
static volatile uint8_t g_control_fault_latched = 0u;
#endif
static uint32_t g_uart_tx_errors = 0;
static uint8_t g_uart_last_error_status = HAL_OK;
static uint16_t g_uart_last_error_remaining = 0;

/* 主循环取走一份完整遥测帧；复制期间禁止中断，避免读到半帧数据 */
static uint8_t DataFrame_Take(DataFrame_t *out)
{
    uint8_t ready;

    __disable_irq();
    ready = g_frame_ready;
    if (ready)
    {
        *out = g_frame;
        g_frame_ready = 0;
    }
    __enable_irq();

    return ready;
}

/* Phase 2：累计编码器计数，供 PC 端计算轮速 */
static int32_t g_encoder_left_total = 0;
static int32_t g_encoder_right_total = 0;

static void Phase2_GetStepPwm(uint32_t timestamp_ms,
                              int16_t *pwm_left,
                              int16_t *pwm_right)
{
    *pwm_left = 0;
    *pwm_right = 0;

#if PHASE2_MOTOR_STEP_TEST
    int16_t pwm = 0;

    if (timestamp_ms >= 2000 && timestamp_ms < 5000)
        pwm = PHASE2_STEP_PWM;
    else if (timestamp_ms >= 7000 && timestamp_ms < 10000)
        pwm = -PHASE2_STEP_PWM;

#if PHASE2_TEST_MOTOR == 0
    *pwm_left = pwm;
#else
    *pwm_right = pwm;
#endif
#endif
}

/* 计算校验和：除header和checksum本身外所有字节累加取低8位 */
static uint8_t CalcChecksum(const DataFrame_t *f)
{
    uint8_t sum = 0;

    uint8_t *p = (uint8_t *)&f->timestamp_ms;

		for (int i = 0;
				 i < sizeof(f->timestamp_ms)
					 + sizeof(f->accel)
					 + sizeof(f->gyro)
					 + sizeof(f->accel_filtered)
					 + sizeof(f->gyro_filtered)
					 + sizeof(f->pitch_x100)
					 + sizeof(f->roll_x100)
					 + sizeof(f->pwm_left)
					 + sizeof(f->pwm_right)
					 + sizeof(f->encoder_left)
					 + sizeof(f->encoder_right)
#if TIMING_DIAGNOSTIC_ENABLE
					 + sizeof(f->control_tick_seq)
					 + sizeof(f->speed_loop_seq)
					 + sizeof(f->target_angle_x100)
					 + sizeof(f->current_speed_x100)
					 + sizeof(f->balance_output)
#endif
					 ;
				 i++)
    {
        sum += p[i];
    }

    return sum;
}

/* 比较同时拒绝NaN和正负无穷，避免依赖ARMCC的isfinite实现。 */
static uint8_t Control_FloatIsFinite(float value)
{
    return (uint8_t)(value >= -FLT_MAX && value <= FLT_MAX);
}

/* 遥测先验证再限幅，非法角填0并通过IMU故障状态标记。 */
static int16_t DataFrame_AngleX100(float value)
{
    if (!Control_FloatIsFinite(value)) return 0;
    if (value >= 327.67f) return 32767;
    if (value <= -327.68f) return -32768;
    return (int16_t)(value * 100.0f);
}


/* 保存发送失败现场，避免下次发送覆盖 HAL 的剩余字节数。 */
static void Uart_TransmitChecked(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, data, size, timeout_ms);

    if (status != HAL_OK)
    {
        g_uart_tx_errors++;
        g_uart_last_error_status = (uint8_t)status;
        g_uart_last_error_remaining = huart3.TxXferCount;
    }
}

#if ONLINE_TUNING_ENABLE
/* 恢复接收只在主循环执行；中断内不等待、不解析命令、不发送回执。 */
static void Tuning_ServiceReception(void)
{
    uint32_t primask;
    if (!g_tuning_rx_restart_pending) return;
    if (HAL_UART_AbortReceive(&huart3) != HAL_OK) return;
    primask = __get_PRIMASK();
    __disable_irq();
    g_tuning_rx_restart_pending = 0u;
    if (HAL_UART_Receive_IT(&huart3, &g_tuning_rx_byte, 1u) != HAL_OK)
    {
        g_tuning_rx_restart_pending = 1u;
        OnlineTune_RxError();
    }
    __set_PRIMASK(primask);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;
    if (g_tuning_rx_restart_pending) return;
    OnlineTune_RxByte(g_tuning_rx_byte);
    if (HAL_UART_Receive_IT(huart, &g_tuning_rx_byte, 1u) != HAL_OK)
    {
        OnlineTune_RxError();
        g_tuning_rx_restart_pending = 1u;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;
    OnlineTune_RxError();
    g_tuning_rx_restart_pending = 1u;
}
#endif

#if UART_BOOT_DIAGNOSTIC_ENABLE
/* 元数据使用定点整数，避免依赖浮点 printf；转换仅在主循环执行。 */
static long Calibration_FixedPoint(float value, double units)
{
    double scaled = (double)value * units;
    return (long)(scaled + (scaled >= 0.0 ? 0.5 : -0.5));
}

static void Uart_SendAccelCalibration(void)
{
    const MPU6050_AccelCalibration_t *cal = &g_mpu6050_accel_calibration;
    char message[180];
    int length = snprintf(message, sizeof(message),
        "[ACAL] v=%lu bx=%ld by=%ld bz=%ld sx=%ld sy=%ld sz=%ld\r\n",
        (unsigned long)cal->version,
        Calibration_FixedPoint(cal->bias_lsb[0], 1000.0),
        Calibration_FixedPoint(cal->bias_lsb[1], 1000.0),
        Calibration_FixedPoint(cal->bias_lsb[2], 1000.0),
        Calibration_FixedPoint(cal->gain[0], 100000000.0),
        Calibration_FixedPoint(cal->gain[1], 100000000.0),
        Calibration_FixedPoint(cal->gain[2], 100000000.0));

    if (length > 0 && length < (int)sizeof(message))
        Uart_TransmitChecked((const uint8_t *)message, (uint16_t)length, 30);
}

static void Uart_SendImuStartup(void)
{
    MPU6050_StartupDiagnostics_t state = g_mpu6050_startup_diag;
    char message[200];
    int length = snprintf(message, sizeof(message),
        "[ISTART] v=1 tries=%u stage=%u werr=%lu wstatus=%u wreg=%02X wbus=%08lX verr=%lu vreg=%02X vval=%02X\r\n",
        (unsigned)state.attempts, (unsigned)state.stage,
        (unsigned long)state.write_errors, (unsigned)state.last_write_status,
        (unsigned)state.last_write_register, (unsigned long)state.last_write_error,
        (unsigned long)state.verify_errors, (unsigned)state.last_verify_register,
        (unsigned)state.last_verify_value);

    if (length > 0 && length < (int)sizeof(message))
        Uart_TransmitChecked((const uint8_t *)message, (uint16_t)length, 30);
}

static void Uart_SendImuDiagnostics(void)
{
    char message[240];
    int length = snprintf(message, sizeof(message),
        "[IMU] id=%02X acc=%02X gyr=%02X pwr=%02X cfg=%u cal=%u n=%u valid=%u status=%u err=%lu zero=%lu bus=%08lX fault=%u txerr=%lu txstatus=%u txleft=%u stop=%u\r\n",
        (unsigned)g_mpu6050_diag.who_am_i, (unsigned)g_mpu6050_diag.accel_config,
        (unsigned)g_mpu6050_diag.gyro_config, (unsigned)g_mpu6050_diag.pwr_mgmt_1,
        (unsigned)g_mpu6050_diag.config_valid, (unsigned)g_mpu6050_diag.calibration_valid,
        (unsigned)g_mpu6050_diag.calibration_samples, (unsigned)g_mpu6050_diag.sample_valid,
        (unsigned)g_mpu6050_diag.last_read_status, (unsigned long)g_mpu6050_diag.read_errors,
        (unsigned long)g_mpu6050_diag.zero_samples, (unsigned long)g_mpu6050_diag.last_i2c_error,
        (unsigned)g_imu_fault_latched, (unsigned long)g_uart_tx_errors,
        (unsigned)g_uart_last_error_status, (unsigned)g_uart_last_error_remaining,
#if ONLINE_TUNING_ENABLE
        0u);
#else
        (unsigned)g_control_fault_latched);
#endif

    if (length > 0 && length < (int)sizeof(message))
        Uart_TransmitChecked((const uint8_t *)message, (uint16_t)length, 30);
}
#endif

/* TIM2 1kHz中断服务函数 —— 方案A：每2次中断采集一次，有效500Hz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) return;
    
		static uint8_t skip_counter = 0;
		static uint8_t tx_counter = 0;
    
    /* 每2次中断执行一次采集+发送 */
    if (++skip_counter >= 2)
    {
        skip_counter = 0;
        g_control_tick_seq++;
        
				int16_t ax = 0, ay = 0, az = 0;
				int16_t gx = 0, gy = 0, gz = 0;

				int16_t ax_f = 0, ay_f = 0, az_f = 0;
				int16_t gx_f = 0, gy_f = 0, gz_f = 0;
				int16_t pwm_left, pwm_right;
#if PHASE2_MOTOR_STEP_TEST || !BALANCE_CONTROL_ENABLE
				int16_t encoder_left_delta, encoder_right_delta;
#endif

				uint8_t imu_sample_valid = 0u;
        /* 首次读取失败后锁存，避免长 I2C 超时反复占用中断、饿死串口发送。 */
        if (!g_imu_fault_latched)
        {
            imu_sample_valid = MPU6050_ReadRawFiltered(
						&ax, &ay, &az,
						&gx, &gy, &gz,
						&ax_f, &ay_f, &az_f,
						&gx_f, &gy_f, &gz_f
				);
            if (!imu_sample_valid) g_imu_fault_latched = 1u;
        }
        if (g_imu_fault_latched) g_mpu6050_diag.sample_valid = 0u;
        if (imu_sample_valid) GET_Angle();

        /* 非有限值必须在PID、PWM及遥测整数编码前拒收；相减溢出也不例外。 */
        float pitch_reference = Control_GetPitchReferenceDeg();
        float relative_pitch = pitch - pitch_reference;
        if (!Control_FloatIsFinite(pitch) || !Control_FloatIsFinite(roll)
            || !Control_FloatIsFinite(pitch_reference) || !Control_FloatIsFinite(relative_pitch))
        {
            g_imu_fault_latched = 1u;
            g_mpu6050_diag.sample_valid = 0u;
            imu_sample_valid = 0u;
        }
#if !ONLINE_TUNING_ENABLE
        if (imu_sample_valid && (relative_pitch > 45.0f || relative_pitch < -45.0f))
            g_control_fault_latched = 1u;
#endif

#if ONLINE_TUNING_ENABLE
        OnlineTune_ControlTick(HAL_GetTick(), g_control_tick_seq,
            (uint8_t)(imu_sample_valid && !g_imu_fault_latched
                && g_mpu6050_diag.config_valid && g_mpu6050_diag.calibration_valid), pitch);
#endif

#if BALANCE_CONTROL_ENABLE
        /* 姿态已经在本采样周期更新，Control_Loop 不再重复调用 GET_Angle */
        if (imu_sample_valid && g_mpu6050_diag.config_valid && g_mpu6050_diag.calibration_valid
#if !ONLINE_TUNING_ENABLE
            && !g_control_fault_latched
#endif
            )
            Control_Loop();
        else
            motor_left_pwm = motor_right_pwm = 0;
#if ONLINE_TUNING_ENABLE
        /* 许可检查与GPIO/CCR提交必须原子，避免RX错误在二者之间抢占。
         * 只屏蔽这段有限寄存器操作，采样、计算、解析与发送都在外部。 */
        uint32_t motor_primask = __get_PRIMASK();
        __disable_irq();
        if (!OnlineTune_IsArmed()) motor_left_pwm = motor_right_pwm = 0;
        Motor_Update();
        __set_PRIMASK(motor_primask);
#else
        Motor_Update();
#endif
#elif TIMING_DIAGNOSTIC_ENABLE
        /* 闭环关闭时只运行时序计数，不计算或输出电机控制量 */
        Control_Timing_Diagnostic_Tick();
#endif
			
        /* 时间戳每帧增加2（因为实际采样间隔是2ms） */
        g_tick_ms += 2;

        /* Phase 2：输出开环 PWM，并累计编码器计数 */
#if PHASE2_MOTOR_STEP_TEST
        Phase2_GetStepPwm(g_tick_ms, &pwm_left, &pwm_right);
        Set_Pwm(pwm_left, pwm_right);

        /* 与电机物理侧保持一致：左电机对应 TIM4，右电机对应 TIM3 */
        encoder_left_delta = (int16_t)Read_Encoder_HAL(&htim4);
        encoder_right_delta = (int16_t)Read_Encoder_HAL(&htim3);
        g_encoder_left_total += encoder_left_delta;
        g_encoder_right_total += encoder_right_delta;
#else
        pwm_left = 0;
        pwm_right = 0;
#if !BALANCE_CONTROL_ENABLE
        /* 闭环关闭时仍读取编码器，保证零 PWM 遥测可以验证编码器链路 */
        encoder_left_delta = (int16_t)Read_Encoder_HAL(&htim4);
        encoder_right_delta = (int16_t)Read_Encoder_HAL(&htim3);
        g_encoder_left_total += encoder_left_delta;
        g_encoder_right_total += encoder_right_delta;
        Set_Pwm(0, 0);
#else
        /* 闭环开启时由 Control_Loop 读取编码器，避免同一 tick 重复清零 */
#endif
#endif

#if BALANCE_CONTROL_ENABLE
        /* 记录保护后的电机命令；驱动层仍会将绝对值小于100的命令归零。 */
        pwm_left = (int16_t)motor_left_pwm;
        pwm_right = (int16_t)motor_right_pwm;
#endif

        /* 读取失败或启动校验未完成时，禁止向驱动层施加任何测试/闭环输出。 */
        if (!imu_sample_valid || !g_mpu6050_diag.config_valid || !g_mpu6050_diag.calibration_valid
#if !ONLINE_TUNING_ENABLE
            || g_control_fault_latched
#endif
            )
        {
            pwm_left = pwm_right = 0;
            motor_left_pwm = motor_right_pwm = 0;
            Set_Pwm(0, 0);
        }
        
        /* 打包二进制帧 */
        g_frame.header[0] = 0xAA;
        g_frame.header[1] = 0x55;
        g_frame.timestamp_ms = g_tick_ms;
        g_frame.accel[0] = ax;
        g_frame.accel[1] = ay;
        g_frame.accel[2] = az;
        g_frame.gyro[0]  = gx;
        g_frame.gyro[1]  = gy;
        g_frame.gyro[2]  = gz;
				
				g_frame.accel_filtered[0] = ax_f;
				g_frame.accel_filtered[1] = ay_f;
				g_frame.accel_filtered[2] = az_f;

				g_frame.gyro_filtered[0] = gx_f;
				g_frame.gyro_filtered[1] = gy_f;
				g_frame.gyro_filtered[2] = gz_f;
				
				g_frame.pitch_x100 = DataFrame_AngleX100(pitch);
				g_frame.roll_x100  = DataFrame_AngleX100(roll);

				g_frame.pwm_left = pwm_left;
				g_frame.pwm_right = pwm_right;
				g_frame.encoder_left = g_encoder_left_total;
				g_frame.encoder_right = g_encoder_right_total;

#if TIMING_DIAGNOSTIC_ENABLE
				g_frame.control_tick_seq = g_control_tick_seq;
				g_frame.speed_loop_seq = g_speed_loop_seq;
				g_frame.target_angle_x100 = g_target_angle_x100;
				g_frame.current_speed_x100 = g_current_speed_x100;
				g_frame.balance_output = g_balance_output;
#endif
				
        g_frame.checksum = CalcChecksum(&g_frame);
        
					/* 默认 47 字节；打开当前诊断字段后为 61 字节 */
					if (++tx_counter >= TELEMETRY_TX_DIVIDER)
					{
							tx_counter = 0;
							g_frame_ready = 1;
					}
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* 临时诊断：先初始化 UART，隔离后续外设初始化是否卡死 */
  MX_USART3_UART_Init();
#if UART_BOOT_DIAGNOSTIC_ENABLE
  {
    static const uint8_t uart_ready_message[] = "UART_READY\r\n";
    Uart_TransmitChecked(uart_ready_message,
                      sizeof(uart_ready_message) - 1u,
                      100);
  }
#endif
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* HAL 阻塞 I2C 的超时依赖 SysTick；允许其抢占 TIM2，避免异常时超时停滞。 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  MPU6050_Init();
	MPU6050_Filter_Init();
  Control_Init();
#if ONLINE_TUNING_ENABLE
  OnlineTune_Init();
  g_tuning_rx_restart_pending = 1u;
  Tuning_ServiceReception();
#endif
  /* 配置、校准和初始样本均成功才进入高速采样；启动失败不在 ISR 中重试。 */
  g_imu_fault_latched = !(g_mpu6050_diag.config_valid
      && g_mpu6050_diag.calibration_valid && g_mpu6050_diag.sample_valid);
  if (g_imu_fault_latched) g_mpu6050_diag.sample_valid = 0u;
  HAL_TIM_Base_Start_IT(&htim2);
	
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ONLINE_TUNING_ENABLE
    {
        char reply[240];
        size_t reply_length;
        Tuning_ServiceReception();
        OnlineTune_Process();
        reply_length = OnlineTune_TakeReply(reply, sizeof(reply));
        if (reply_length)
            Uart_TransmitChecked((const uint8_t *)reply, (uint16_t)reply_length, 40u);
    }
#endif
#if UART_BOOT_DIAGNOSTIC_ENABLE
    {
        static uint32_t last_boot_ms = 0;
        uint32_t now_ms = HAL_GetTick();

        if ((uint32_t)(now_ms - last_boot_ms) >= 500u)
        {
            last_boot_ms = now_ms;
            Uart_SendImuDiagnostics();
            Uart_SendImuStartup();
            Uart_SendAccelCalibration();
        }
    }
#endif
//		Set_Pwm(1400,1400);
		 DataFrame_t frame_to_send;
		 if (DataFrame_Take(&frame_to_send))
    {
        /* 61 字节诊断帧在 115200 baud 下约需 5.3 ms，留出足够超时余量 */
        Uart_TransmitChecked((const uint8_t *)&frame_to_send, sizeof(frame_to_send), 20);
    }
		
    /* 主循环空闲，控制逻辑在 TIM2 中断中执行 */
  }
  /* USER CODE END 3 */
}

/*
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
