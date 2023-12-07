/*
 * control.h
 *
 *  Created on: Dec 7, 2023
 *      Author: ADMIN
 */

#include "main.h"
#include "line_sensor.h"
#include "motor.h"
#include "pid.h"

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

typedef struct{
	float left_wheel_speed,right_wheel_speed;
}wheelSpeed_st;

typedef struct{
	float k1,k2,k3;
}control_st;


void control_lineTracking(control_st* ctrlp,PIDParams_st* pidps[], motorParams_st* mps[], lineSensorParams_st* lplp);

//wheelSpeed_st control_calculateWheelsRPM(control_st* ctrlp,float yError, float angleError);


#endif /* INC_CONTROL_H_ */
