/*
 * pid.h
 *
 *  Created on: Dec 7, 2023
 *      Author: ADMIN
 */
#include "main.h"

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	int enable_anti_windup;
	float anti_windup_error;
	float outmin;
	float outmax;
	int anti_windup;
	int is_sat;
	int is_same_sign;
	float current_value;
	float setpoint;
	float prev_time;
	float prev_error;
	int32_t error_sum;
}PIDParams_st;


float pid_calculate(PIDParams_st* p);

#endif /* PID_H_ */
