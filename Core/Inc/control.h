/*
 * control.h
 *
 *  Created on: Dec 7, 2023
 *      Author: ADMIN
 */

#include "main.h"
#include "line_sensor.h"
#include "motor.h"
#include "constants.h"
#include "math.h"
#include "utils.h"
#include "pid_controller.h"
#include "color_sensor.h"

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

enum RobotState {STOP, RUN};
enum Branch {UPPER, LOWER, EMPTY};

typedef struct{
	float left_wheel_speed,right_wheel_speed;
}wheelSpeed_st;

typedef struct{
	float k1,k2,k3;
	enum Branch branch;
	uint8_t stopTime;
	enum RobotState state;
}control_st;



void control_lineTracking(control_st* ctrlp,PIDControl* pidps[], motorParams_st* mps[], lineSensorParams_st* lp, colorSensor_st* cp);

void control_steerToBranch(control_st* ctrlp, motorParams_st* mp,lineSensorParams_st* lp);

//wheelSpeed_st control_calculateWheelsRPM(control_st* ctrlp,float yError, float angleError);


#endif /* INC_CONTROL_H_ */
