#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct{
	float Ref;
	float Measure;	
	float Kp,Ki,Kd;
	float err, last_err;
	float last_measure;
	uint8_t derivative_initialized;
	float integral;
	float integral_max, output_max;
	float output;
}PID_t;

void PID_Init(PID_t *PID,float Kp,float Ki,float Kd);
void PID_Calculate(PID_t *PID,float Ref,float Measure);
void PID_Calculate_Dt(PID_t *PID,float Ref,float Measure,float sample_dt);

#endif
