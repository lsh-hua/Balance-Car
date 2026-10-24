#ifndef ENCODER_H
#define ENCODER_H

#include "tim.h"

#define PI 3.14159265
#define Control_Frequency  500.0
#define Diameter_67  67.0
#define EncoderMultiples   4.0
#define Encoder_precision  13.0
#define Reduction_Ratio  30.0

extern float Velocity_Left,Velocity_Right;



void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
int Read_Encoder_HAL(TIM_HandleTypeDef *htim);


#endif
