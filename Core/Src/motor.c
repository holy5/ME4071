
#include "motor.h"
#include <stdlib.h>

//Motor
static void motor_setDutyCycle(motorParams_st* mp, uint32_t duty_cycle){
	float pw_resolution = (((float)(*mp->htim).Init.Period + 1.0f) / 100.0f);
	uint16_t pw_desired = pw_resolution * duty_cycle;
	__HAL_TIM_SET_COMPARE(mp->htim, mp->pwm_channel, pw_desired);
}

void motor_init(motorParams_st* mp){
	//HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET); //Turn on driver
	HAL_TIM_PWM_Start(mp->htim, mp->pwm_channel);
	motor_setDutyCycle(mp,0);
}

void motor_stopMotor(motorParams_st* mp){
	HAL_GPIO_WritePin(mp->in_port, mp->in1_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mp->in_port, mp->in2_pin, GPIO_PIN_RESET);
}

void motor_standByMode(void){
	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
}

void motor_setMotorPWM(motorParams_st* mp, int16_t duty_cycle_percentage){
// 100 means full speed forward; -100 means full speed backward
//	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
	if(duty_cycle_percentage>0){
		HAL_GPIO_WritePin(mp->in_port, mp->in1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mp->in_port, mp->in2_pin, GPIO_PIN_SET);
		motor_setDutyCycle(mp, duty_cycle_percentage);
	}else if (duty_cycle_percentage<0){
		HAL_GPIO_WritePin(mp->in_port, mp->in1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mp->in_port, mp->in2_pin, GPIO_PIN_SET);
		motor_setDutyCycle(mp,-duty_cycle_percentage);
	}
}
void motor_readEncoder(motorParams_st* mp){
	mp->ucount = __HAL_TIM_GET_COUNTER(mp->enc_htim);
	mp->scount = (int16_t)mp->ucount;
	mp->position = (int16_t)(mp->scount/4);
}

void motor_calculateRPM(motorParams_st* mp, int32_t current_time){


	mp->rpm = 60/(mp->pulse_per_rev*1e-5*abs(current_time - mp->prev_time));
	mp->prev_time = current_time;
}

//PID

float Kp, Ki, Kd, Ts, Outmin, Outmax, anti_windup_error;
int enable_anti_windup;
uint16_t time,last_time=0;
float error, prev_error=0, error_sum=0;
int8_t is_sat, is_same_sign;

void pid_init(PIDParams_st *p)
{
	Kp=p->Kp;
	Ki=p->Ki;
	Kd=p->Kd;
	anti_windup_error=p->Anti_windup_error;
	Outmin=p->Outmin;
	Outmax=p->Outmax;
	enable_anti_windup=p->enable_anti_windup;
	is_sat = p->is_sat;
	is_same_sign = p->is_same_sign;

	if(p->Anti_windup_error==0){anti_windup_error=10;}
}

float pid_calculation(float set_point,float current)
{
	error=(set_point - current);
	time = HAL_GetTick(); // this should be converted to time
	Ts = time - last_time;
	error_sum+=error*Ts;

	float out;
	//	Anti windup v2
//	if(is_sat == 1 && is_same_sign ==1 ){
//		error_sum = 0;
//	}

	if(enable_anti_windup==1){
		if(anti_windup_error < abs(error)){
			out=Kp*(error)+Kd*(error-prev_error)/Ts;
		}
		else{
			out=(Kp*(error)) +( Ki*(error_sum)) + (Kd*(error-prev_error)/Ts);
		}
	}else{
		out=Kp*(error) + Ki*(error_sum) + Kd*(error-prev_error)/Ts;
	}

//	if((error>=0)^(out<0)){
//		is_same_sign =1;
//	}else{
//		is_same_sign=0;
//	}

	if (out > Outmax){
		is_sat = 1;
		out = Outmax;
	} else if(out < Outmin){
		is_sat = 1;
		out = Outmin;
	}else{
		is_sat = 0;
	}

	prev_error=error;
	last_time = time;
	return out;
}

void motor_calculateSpeed(motorParams_st* mp){
	mp->speed =  mp->rpm * mp->wheel_radius;
}
