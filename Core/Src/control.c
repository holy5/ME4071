/*
 * control.c
 *
 *  Created on: Dec 7, 2023
 *      Author: ADMIN
 */

#include "control.h"

static wheelSpeed_st control_calculateWheelsRPM(control_st* ctrlp,float yError, float angleError){
	float vel = VEL_REF * cos(angleError);
	float omega = OMEGA_REF + ctrlp->k2 * yError * VEL_REF + ctrlp->k3 * sin(angleError);

	float left_omega = (1 / WHEEL_RADIUS) * (vel+ CAR_WIDTH * omega/2); // omega left
	float right_omega = (1 / WHEEL_RADIUS) * (vel - CAR_WIDTH * omega/2); // omega right

	wheelSpeed_st s;
	s.left_wheel_speed = utils_radPerSecToRPM(left_omega);
	s.right_wheel_speed = utils_radPerSecToRPM(right_omega);

	return s;
}

static void control_steerToBranch(control_st* ctrlp, motorParams_st* mps[]){
	if(ctrlp->branch == UPPER){
		motor_setMotorPWM(mps[1],90);
		motor_setMotorPWM(mps[0],20);
	}else if (ctrlp->branch == LOWER){
		motor_setMotorPWM(mps[0], 50);
		motor_setMotorPWM(mps[1],20);
	}
}

uint16_t count=0;
uint16_t red_count = 0;
uint16_t green_count = 0;
float current_pos;
uint16_t hinch_flag=0;


void control_lineTracking(control_st* ctrlp,PIDControl* pidps[], motorParams_st* mps[], lineSensorParams_st* lp, colorSensor_st* cp){

	uint16_t threshold = 200;
	uint8_t count_white = 0;
	uint8_t count_white_ambient = 0;
	int length = sizeof(lp->sensor_values) / sizeof(lp->sensor_values[0]);

	for(int i = 0; i < length ;i++){
		if(lp->sensor_values[i] < threshold){
			count_white++;
		}
		else if (lp->sensor_values[i] > 3500){
			count_white_ambient++;
		}
	}
	if (count_white == 5 || count_white_ambient == 5){ // All 5 sensors read white, end of the line
		motor_stopMotor(mps[0]);
		motor_stopMotor(mps[1]);
	}else{
		motor_odometry(mps[0]);
		motor_odometry(mps[1]);
		float avg_distance = (mps[0]->distance + mps[1]->distance)/2;
		float eol_stop_distance = 2000 + 500 + 500 * PI +300 + 2000 + 30;
		if(avg_distance > eol_stop_distance){
			motor_stopMotor(mps[0]);
			motor_stopMotor(mps[1]);
		}else{
		if (utils_inRange(avg_distance, 1970, 2020)){
			if (ctrlp->stopTime == 0){
				motor_stopMotor(mps[0]);
				motor_stopMotor(mps[1]);
				ctrlp->state = STOP;
				if(cp->color == C_RED){
					red_count++;
				}else if(cp->color == C_GREEN){
					green_count++;
				}
				if(green_count > 10 || red_count > 10){
					if(green_count > red_count){
						ctrlp->branch = LOWER;
					}else{
						ctrlp->branch = UPPER;
					}
					red_count = 0;
					green_count = 0;
				}
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//				motor_resetEncoderCount(mps[0]);
//				motor_resetEncoderCount(mps[1]);
				if(ctrlp->branch != EMPTY){
					hinch_flag = 1;
					ctrlp->state = RUN;
					ctrlp->stopTime += 1;
				}
				if(hinch_flag == 1){
					motor_setMotorPWM(mps[0],50);
					motor_setMotorPWM(mps[1],50);
				}
			}
			}
			else{
				hinch_flag = 0;
				wheelSpeed_st speeds = control_calculateWheelsRPM(ctrlp, lp->y_error , lp->angle_error);
				pidps[0]->setpoint = speeds.left_wheel_speed;
				pidps[1]->setpoint = speeds.right_wheel_speed;
				pidps[0]->input = mps[0]->rpm;
				pidps[1]->input = mps[1]->rpm;
				PIDCompute(pidps[0]);
				PIDCompute(pidps[1]);
				float pos = 2000 + 500 + 500 * PI + 200;
				uint16_t range = 70;
				uint16_t offset = 230;
				uint16_t range_2 = 90;
				uint16_t offset_2 = 210;
				if (utils_inRange(avg_distance,pos - offset,pos - offset+range) && ctrlp->branch == LOWER){
					control_steerToBranch(ctrlp, mps);

//					motor_stopMotor(mps[0]);
//					motor_stopMotor(mps[1]);
				}else if (utils_inRange(avg_distance,pos - offset_2,pos - offset_2+range_2) && ctrlp->branch == UPPER){
					control_steerToBranch(ctrlp, mps);
//					motor_stopMotor(mps[0]);
//					motor_stopMotor(mps[1]);
				}else{
				motor_setMotorPWM(mps[0],pidps[0]->output);
				motor_setMotorPWM(mps[1],pidps[1]->output);
//				motor_setMotorPWM(mps[0],50);
//				motor_setMotorPWM(mps[1],50);
				}
			}
	}
	}
}




