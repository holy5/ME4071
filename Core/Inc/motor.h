/*
 * motor.h
 *
 *  Created on: Oct 8, 2023
 *      Author: lvquang
 */
#include "main.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#define STBY_PIN GPIO_PIN_14
#define STBY_PORT GPIOB

typedef struct{
	uint16_t pulse_per_rev;
	uint32_t ucount;
	int16_t scount;
	int16_t position;
	int32_t prev_time;
	uint16_t rpm;
	int8_t direction;
	uint16_t pwm_channel;
	uint16_t in1_pin;
	uint16_t in2_pin;
	GPIO_TypeDef* in_port;
	float speed;
	float wheel_radius;
	TIM_HandleTypeDef* htim;
	TIM_HandleTypeDef* enc_htim;
}motorParams_st;

void motor_init(motorParams_st* motor_params);

void motor_stopMotor(motorParams_st* motor_params);

void motor_standByMode(void);

//void motor_setDutyCycle(motorParams_st* motor_params,TIM_HandleTypeDef* htim, uint32_t duty_cycle);

void motor_setMotorPWM(motorParams_st* motor_params, int16_t duty_cycle_percentage);

void motor_readEncoder(motorParams_st* motor_params);

void motor_calculateRPM(motorParams_st* motor_params, int32_t current_time);

void motor_calculateSpeed(motorParams_st* motor_params);

#endif /* INC_MOTOR_H_ */
