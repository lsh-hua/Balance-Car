#include "main.h"
#include "encoder.h"

float Velocity_Left,Velocity_Right;


int Read_Encoder_HAL(TIM_HandleTypeDef *htim)
{
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(htim);  // 读当前值
    __HAL_TIM_SET_COUNTER(htim, 0);                       // 清零
    return cnt;
}


void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{         
  float Rotation_Speed_L,Rotation_Speed_R;                                                
  Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
  Velocity_Left = Rotation_Speed_L*PI*Diameter_67;                
  Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
  Velocity_Right = Rotation_Speed_R*PI*Diameter_67;                
}