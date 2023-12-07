
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

void motor_calculateSpeed(motorParams_st* mp){
	mp->speed =  mp->rpm * mp->wheel_radius;
}
