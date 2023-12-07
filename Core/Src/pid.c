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
	uint32_t now = HAL_GetTick()/1000; // to s
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
		p->is_sat = 1;
		out = p->outmax;
	} else if(out < p->outmin){
		p->is_sat = 1;
		out = p->outmin;
	}else{
		p->is_sat = 0;
	}

	p->prev_error = error;
	p->prev_time = now;
	return out;
}
