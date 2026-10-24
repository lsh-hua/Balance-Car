#include "motor.h"
#include "pid.h"
#include "encoder.h"

#define MOTOR_DEAD_ZONE  100  // 小于此绝对命令时置零，阈值应以实测为准

void Motor_Init()
{
	/* 电机相关初始化由 CubeMX 在 MX_TIM1_Init() 和 MX_GPIO_Init() 中完成 */
}

int myabs(int a)
{		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}

static int apply_deadzone(int pwm)
{
    /* 只过滤死区内的小命令，不补偿摩擦，也不能据此保证不溜车。 */
    if (pwm >= MOTOR_DEAD_ZONE)  return pwm;
    if (pwm <= -MOTOR_DEAD_ZONE) return pwm;
    return 0;
}

void Set_Pwm(int motor_left, int motor_right)
{
    // 死区过滤：绝对值小于100置零，其余命令保持原值
    motor_left  = apply_deadzone(motor_left);
    motor_right = apply_deadzone(motor_right);

    // ========== 左轮控制 (BIN1/PB13 BIN2/PB12 + TIM1_CH4/PA11/PWMB) ==========
    if(motor_left > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // BIN1 = 1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // BIN2 = 0
    }
    else if(motor_left < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // BIN1 = 0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // BIN2 = 1
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // 停止：BIN1 = 0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // 停止：BIN2 = 0
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, myabs(motor_left)); // PA11 = PWMB

    // ========== 右轮控制 (AIN1/PB14 AIN2/PB15 + TIM1_CH1/PA8/PWMA) ==========
    if(motor_right > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // AIN1 = 1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // AIN2 = 0
    }
    else if(motor_right < 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // AIN1 = 0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // AIN2 = 1
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // 停止：AIN1 = 0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // 停止：AIN2 = 0
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, myabs(motor_right)); // PA8 = PWMA
}
