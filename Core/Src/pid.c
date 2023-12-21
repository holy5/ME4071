/*
 * pid.c
 *
 *  Created on: Dec 7, 2023
 *      Author: ADMIN
 */

#include "pid.h"
#include <stdlib.h>

float pid_calculate(PIDParams_st *p)
{
	float error=(p->setpoint - p->current_value);
	float now = HAL_GetTick(); // to s
	float Ts = now - p->prev_time;
	p->error_sum += error*Ts;
	float out;
	float Kp = p->Kp;
	float Kd = p->Kd;
	float Ki = p->Ki;

	if(p->enable_anti_windup==1){
		if(p->anti_windup_error < abs(error)){
			out=Kp*(error)+Kd*(error-p->prev_error)/Ts;
		}
		else{
			out=(Kp*(error)) +( Ki*(p->error_sum)) + (Kd*(error-p->prev_error)/Ts);
		}
	}else{
		out=Kp*(error) + Ki*(p->error_sum) + Kd*(error-p->prev_error)/Ts;
	}

	if (out > p->outmax){
		out = p->outmax;
	} else if(out < p->outmin){
		out = p->outmin;
	}

	p->prev_error = error;
	p->prev_time = now;
	return out;
}
