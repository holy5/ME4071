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

void control_steerToBranch(control_st* ctrlp, motorParams_st* mp,lineSensorParams_st* lp){
	enum Branch upper = UPPER;
	enum Branch lower = LOWER;

	if(ctrlp->branch == upper){
//		To be implemented
	}else if (ctrlp->branch == lower){
//		To be implemented
	}
}

void control_lineTracking(control_st* ctrlp,PIDControl* pidps[], motorParams_st* mps[], lineSensorParams_st* lp, colorSensor_st* cp){

	uint16_t threshold = 300;
	uint8_t count = 0;
	int length = sizeof(lp->adc_values) / sizeof(lp->adc_values[0]);

	for(int i = 0; i < length ;i++){
		if(lp->adc_values[i] < threshold){
			count++;
		}
	}
	if (count==5){ // All 5 sensors read white, end of the line
		motor_stopMotor(mps[0]);
		motor_stopMotor(mps[1]);
	}else{
		float distance_avg = (mps[0]->distance + mps[1]->distance)/2;
			if (utils_inRange(distance_avg, 2000-20, 2000+20) && ctrlp->stopTime == 0){
				motor_stopMotor(mps[0]);
				motor_stopMotor(mps[1]);
				if(cp->color == C_RED){
					ctrlp->branch = UPPER;
				} else if(cp->color == C_GREEN){
					ctrlp->branch = LOWER;
				}
				if(ctrlp->branch != EMPTY){
					motor_resetEncoderCount(mps[0]);
					motor_resetEncoderCount(mps[1]);
					ctrlp->stopTime = 1;
				}
			}else{
				wheelSpeed_st speeds = control_calculateWheelsRPM(ctrlp, lp->y_error , lp->angle_error);
				pidps[0]->setpoint = speeds.left_wheel_speed;
				pidps[1]->setpoint = speeds.right_wheel_speed;
				pidps[0]->input = mps[0]->rpm;
				pidps[1]->input = mps[1]->rpm;
				PIDCompute(pidps[0]);
				PIDCompute(pidps[1]);
				motor_setMotorPWM(mps[0],pidps[0]->output);
				motor_setMotorPWM(mps[1],pidps[1]->output);
			}
	}

}




