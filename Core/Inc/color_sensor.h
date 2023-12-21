/*
 * color_sensor.h
 *
 *  Created on: Dec 5, 2023
 *      Author: ADMIN
 */

#include "main.h"

#ifndef INC_COLOR_SENSOR_H_
#define INC_COLOR_SENSOR_H_

enum ColorFilter{F_RED, F_BLUE, F_GREEN, F_CLEAR};
enum Color {C_RED, C_BLUE, C_GREEN, C_NONE};

typedef struct{
	uint16_t in_pins[4];
	uint16_t out_pin;
	GPIO_TypeDef* in_port_1;
	GPIO_TypeDef* in_port_2; // S3 using another port ??? so confused
	GPIO_TypeDef* out_port;
	TIM_HandleTypeDef* htim;
	uint16_t tim_channel;
	uint16_t rgb[3];
	enum Color color;
}colorSensor_st;

void color_sensor_init(colorSensor_st* p);

void colorsensor_setFilter(colorSensor_st* p,uint8_t filter);

void colorsensor_detectColor(colorSensor_st* p);

#endif /* INC_COLOR_SENSOR_H_ */
