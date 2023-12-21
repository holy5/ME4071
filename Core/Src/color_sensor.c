/*
 * color_sensor.c
 *
 *  Created on: Dec 5, 2023
 *      Author: ADMIN
 */

#include "color_sensor.h"

void color_sensor_init(colorSensor_st* p){
	// Scaling 20%
	  HAL_GPIO_WritePin(p->in_port_1, p->in_pins[0], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(p->in_port_1, p->in_pins[1], GPIO_PIN_RESET);
	  HAL_TIM_IC_Start_IT(p->htim, p->tim_channel);
}

void colorsensor_setFilter(colorSensor_st* p, uint8_t filter){
	switch(filter){
		case(F_RED):
			HAL_GPIO_WritePin(p->in_port_1, p->in_pins[2], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(p->in_port_2, p->in_pins[3], GPIO_PIN_RESET);
			break;
		case(F_GREEN):
			HAL_GPIO_WritePin(p->in_port_1, p->in_pins[2], GPIO_PIN_SET);
			HAL_GPIO_WritePin(p->in_port_2, p->in_pins[3], GPIO_PIN_SET);
			break;
		case(F_BLUE):
			HAL_GPIO_WritePin(p->in_port_1, p->in_pins[2], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(p->in_port_2, p->in_pins[3], GPIO_PIN_SET);
			break;
		case(F_CLEAR):
			HAL_GPIO_WritePin(p->in_port_1, p->in_pins[2], GPIO_PIN_SET);
			HAL_GPIO_WritePin(p->in_port_2, p->in_pins[3], GPIO_PIN_RESET);
			break;
	}

}

void colorsensor_detectColor(colorSensor_st* p){
	uint16_t r =p->rgb[0];
	uint16_t g =p->rgb[1];
	uint16_t b =p->rgb[2];

	enum Color red = C_RED;
	enum Color green = C_GREEN;
	enum Color none = C_NONE;

	if(g>6000 && g>r && g>b){
		p->color= green;
	}
	else if (g>3000 && b<2000 && r<2000){
		p->color = red;
	}else{
		p->color = none;
	}

};

