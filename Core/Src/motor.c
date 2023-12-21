
#include "motor.h"
#include "constants.h"
#include <stdlib.h>

//Motor
static void motor_setDutyCycle(motorParams_st* mp, uint32_t duty_cycle){
	float pw_resolution = (((float)(*mp->htim).Init.Period + 1.0f) / 100.0f);
	uint16_t pw_desired = pw_resolution * duty_cycle;
	__HAL_TIM_SET_COMPARE(mp->htim, mp->pwm_channel, pw_desired);
}

void motor_init(motorParams_st* mp){
	//HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET); //Turn on driver
//	HAL_GPIO_WritePin(mp->in_port, mp->in1_pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(mp->in_port, mp->in2_pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(mp->htim, mp->pwm_channel);
	motor_setDutyCycle(mp,0);
}

void motor_stopMotor(motorParams_st* mp){
	HAL_GPIO_WritePin(mp->in_port, mp->in1_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mp->in_port, mp->in2_pin, GPIO_PIN_RESET);
	motor_setDutyCycle(mp,0);
}

void motor_standByMode(void){
//	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_RESET);
}

void motor_setMotorPWM(motorParams_st* mp, int16_t duty_cycle_percentage){
// 100 means full speed forward; -100 means full speed backward
//	HAL_GPIO_WritePin(STBY_PORT, STBY_PIN, GPIO_PIN_SET);
	if(duty_cycle_percentage>0){
		if (duty_cycle_percentage>100){
			duty_cycle_percentage =100;
		};
		motor_setDutyCycle(mp, duty_cycle_percentage);
	}else if (duty_cycle_percentage<0){
		if (duty_cycle_percentage<-100){
			duty_cycle_percentage=-100;
		};
		HAL_GPIO_WritePin(mp->in_port, mp->in1_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mp->in_port, mp->in2_pin, GPIO_PIN_RESET);
		motor_setDutyCycle(mp,-duty_cycle_percentage);
	}
}
void motor_readEncoder(motorParams_st* mp){
	mp->ucount = __HAL_TIM_GET_COUNTER(mp->enc_htim);
	mp->scount = (int16_t)mp->ucount;
	mp->position = (uint16_t)(mp->ucount/4);
}

void motor_calculateRPM(motorParams_st* mp, int32_t current_time){
// Time based implement
//	mp->rpm = 60.0f/(mp->pulse_per_rev*1e-5*abs(current_time - mp->prev_time));
//	mp->prev_time = current_time;

// Pulse based implement

	uint16_t delta_pos;
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(mp->enc_htim)){
		if(mp->position < mp->prev_pos){
			delta_pos = mp->prev_pos - mp->position;
		}else{
			delta_pos = (mp->enc_htim)->Init.Period + 1 - mp->position + mp->prev_pos;
		}
	}else{
	if (mp->prev_pos > mp->position){
		delta_pos = (mp->enc_htim)->Init.Period + 1 + mp->position - mp->prev_pos;
	}else{
		delta_pos = mp->position - mp->prev_pos;
	}
}
	mp->rpm = (float)(60.0f/24e-3) *((float)delta_pos/(float)mp->pulse_per_rev);
	mp->prev_pos = mp->position;
}

void motor_odometry(motorParams_st* p){
	// Don't worry about overflow, as calculated the car can move 27 meter before overflow (assume the wheels not slipped)
	float distance = ((float)p->position / p->pulse_per_rev)* WHEEL_RADIUS * 2 * PI;;
	p->distance = distance * 1000; // To mm
}
void motor_resetEncoderCount(motorParams_st* p){
	p-> position = 0;
	p->scount = 0;
	p->ucount =0;
	p->prev_pos = 0;
	p->distance = 0;
}

