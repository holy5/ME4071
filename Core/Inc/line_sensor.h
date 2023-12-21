/*
 * line_sensor.h
 *
 *  Created on: Dec 5, 2023
 *      Author: ADMIN
 */
#include "main.h"
#include "motor.h"

#ifndef INC_LINE_SENSOR_H_
#define INC_LINE_SENSOR_H_

typedef struct{
	uint16_t in_pins[5];
	GPIO_TypeDef* in_port;
	ADC_HandleTypeDef* hadc;
	float y_error;
	float angle_error; // in rad
	float angle_error_prev;
	uint16_t sensor_values[5];
	uint16_t adc_values[5];
	uint16_t time;
	uint16_t time_prev;
}lineSensorParams_st;

float linesensor_calculateYError(lineSensorParams_st* p);
//float linesensor_calulateAngleError(lineSensorParams_st* p);
void  linesensor_read(lineSensorParams_st* p);

#endif /* INC_LINE_SENSOR_H_ */
