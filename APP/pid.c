#include "pid.h"

extern float dt;

void PID_Init(PID_t *PID,float Kp,float Ki,float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->Ref = 0;
	PID->last_err = 0;
	PID->last_measure = 0;
	PID->derivative_initialized = 0;
	PID->integral = 0;
	PID->integral_max = 0;
	PID->output_max = 0;
}

void PID_Calculate_Dt(PID_t *PID,float Ref,float Measure,float sample_dt)
{
	float dt_safe = (sample_dt > 0.0f) ? sample_dt : 0.005f;
	
	PID->Measure = Measure;
	PID->Ref = Ref;
	PID->err = PID->Ref - PID->Measure;
	
	// 梯形积分：只在 Ki != 0 时累积，避免无意义积分饱和
	if (PID->Ki != 0.0f)
	{
		PID->integral += (PID->err + PID->last_err) * dt_safe * 0.5f;
		
		// 积分限幅
		if(PID->integral_max > 0)
		{
			if(PID->integral > PID->integral_max) PID->integral = PID->integral_max;
			if(PID->integral < -PID->integral_max) PID->integral = -PID->integral_max;
		}
	}
	
	float Pout = PID->Kp * PID->err;
	float Iout = PID->Ki * PID->integral;
	/* 微分作用于测量值：设定值跳变时不产生微分冲击 */
	float Dout = 0.0f;
	if (PID->derivative_initialized)
		Dout = -PID->Kd * (PID->Measure - PID->last_measure) / dt_safe;
	
	PID->output = Pout + Iout + Dout;
	
	// 输出限幅 + 积分抗饱和：只在未饱和时保留积分增量
	if(PID->output_max > 0)
	{
		if(PID->output > PID->output_max)
		{
			PID->output = PID->output_max;
			if (PID->Ki != 0.0f)
				PID->integral -= (PID->err + PID->last_err) * dt_safe * 0.5f;
		}
		else if(PID->output < -PID->output_max)
		{
			PID->output = -PID->output_max;
			if (PID->Ki != 0.0f)
				PID->integral -= (PID->err + PID->last_err) * dt_safe * 0.5f;
		}
	}
	
	PID->last_err = PID->err;
	PID->last_measure = PID->Measure;
	PID->derivative_initialized = 1;
}

void PID_Calculate(PID_t *PID,float Ref,float Measure)
{
	PID_Calculate_Dt(PID, Ref, Measure, dt);
}
