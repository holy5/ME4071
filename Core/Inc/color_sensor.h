/*
 * color_sensor.h
 *
 *  Created on: Dec 5, 2023
 *      Author: ADMIN
 */

#include "main.h"

#ifndef INC_COLOR_SENSOR_H_
#define INC_COLOR_SENSOR_H_


typedef struct{
	uint16_t in_pins[4];
	uint16_t out_pin;
	GPIO_TypeDef* in_port_1;
	GPIO_TypeDef* in_port_2; // S3 using another port ??? so confused
	GPIO_TypeDef* out_port;
	TIM_HandleTypeDef* htim;
	uint16_t tim_channel;
	uint16_t rgb[3];
	char color;
}colorSensor_st;


void color_sensor_init(colorSensor_st* p);

void colorsensor_read(colorSensor_st* p);

void colorsensor_detectColor(colorSensor_st* p);

#endif /* INC_COLOR_SENSOR_H_ */
