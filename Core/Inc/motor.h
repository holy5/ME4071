/*
 * motor.h
 *
 *  Created on: Oct 8, 2023
 *      Author: lvquang
 */
#include "main.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

typedef struct{
	uint16_t pulse_per_rev;
	uint16_t ucount;
	uint16_t scount;
	uint16_t position;
	uint32_t prev_time;
	uint16_t prev_pos;
	uint16_t delta_pos;
	float rpm;
	uint16_t pwm_channel;
	uint16_t in1_pin;
	uint16_t in2_pin;
	GPIO_TypeDef* in_port;
	TIM_HandleTypeDef* htim;
	TIM_HandleTypeDef* enc_htim;
	float distance;
}motorParams_st;

void motor_init(motorParams_st* motor_params);

void motor_stopMotor(motorParams_st* motor_params);

void motor_standByMode(void);

void motor_odometry(motorParams_st* motor_params);

void motor_setMotorPWM(motorParams_st* motor_params, int16_t duty_cycle_percentage);

void motor_readEncoder(motorParams_st* motor_params);

void motor_calculateRPM(motorParams_st* motor_params, int32_t current_time);

void motor_resetEncoderCount(motorParams_st* p);

#endif /* INC_MOTOR_H_ */
