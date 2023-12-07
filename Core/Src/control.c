/*
 * control.c
 *
 *  Created on: Dec 7, 2023
 *      Author: ADMIN
 */

#include "control.h"
#include "constants.h"
#include "math.h"
#include "utils.h"
#include "pid.h"

static wheelSpeed_st control_calculateWheelsRPM(control_st* ctrlp,float yError, float angleError){
	float vel = VEL_REF * cos(angleError);
	float omega = OMEGA_REF + ctrlp->k2 * yError * VEL_REF + ctrlp->k3 * sin(angleError);

	float left_omega = (0.5 / WHEEL_RADIUS) * (2 * vel+ CAR_WIDTH * omega); // omega left
	float right_omega = (0.5 / WHEEL_RADIUS) * (2 * vel - CAR_WIDTH * omega); // omega right

	wheelSpeed_st s;
	s.left_wheel_speed = utils_omegaToRPM(left_omega);
	s.right_wheel_speed = utils_omegaToRPM(right_omega);

	return s;
}

void control_lineTracking(control_st* ctrlp,PIDParams_st* pidps[], motorParams_st* mps[], lineSensorParams_st* lp){
  wheelSpeed_st speeds = control_calculateWheelsRPM(ctrlp, lp->y_error , lp->angle_error);
  pidps[0]->setpoint = speeds.left_wheel_speed;
  pidps[1]->setpoint = speeds.right_wheel_speed;
  float lw_pwm =pid_calculate(pidps[0]);
  float rw_pwm = pid_calculate(pidps[1]);

  motor_setMotorPWM(mps[0],lw_pwm);
  motor_setMotorPWM(mps[1],rw_pwm);
}




