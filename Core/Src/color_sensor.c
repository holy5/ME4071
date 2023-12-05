/*
 * color_sensor.c
 *
 *  Created on: Dec 5, 2023
 *      Author: ADMIN
 */

#include "color_sensor.h"


void color_sensor_init(colorSensor_st* p){
	  HAL_GPIO_WritePin(p->in_port_1, p->in_pins[0], GPIO_PIN_SET);
	  HAL_GPIO_WritePin(p->in_port_1, p->in_pins[1], GPIO_PIN_SET);
	  HAL_TIM_IC_Start_IT(p->htim, p->tim_channel);
}

void colorsensor_detectColor(colorSensor_st* p){
	   for(int i = 0; i < 3; i++){
	         switch (i){
	            case 0:
	               HAL_GPIO_WritePin(p->in_port_1, p->in_pins[2], GPIO_PIN_RESET);
	               HAL_GPIO_WritePin(p->in_port_2, p->in_pins[3], GPIO_PIN_RESET);
	               p->rgb[0] = HAL_TIM_ReadCapturedValue(p->htim, p->tim_channel);
	               HAL_Delay(1);
	               break;
	            case 1:
	               HAL_GPIO_WritePin(p->in_port_1, p->in_pins[2], GPIO_PIN_RESET);
	               HAL_GPIO_WritePin(p->in_port_2, p->in_pins[3], GPIO_PIN_SET);
	               p->rgb[2] = HAL_TIM_ReadCapturedValue(p->htim, p->tim_channel);
	               HAL_Delay(1);
	               break;
	            case 2:
	               HAL_GPIO_WritePin(p->in_port_1, p->in_pins[2], GPIO_PIN_SET);
	               HAL_GPIO_WritePin(p->in_port_2, p->in_pins[3], GPIO_PIN_SET);
	               p->rgb[1] = HAL_TIM_ReadCapturedValue(p->htim, p->tim_channel);
	               HAL_Delay(1);
	         }
	      }
	   uint16_t r = p->rgb[0];
	   uint16_t g = p->rgb[1];
	   uint16_t b = p->rgb[2];

	   if ((r> 800 && r< 2100) &&  (b > 350 && b < 780) &&  (g > 650 && g < 1700) && ((r> g)) && (g > b)){
	   		p->color = 'r';
	   	}
	   	      // Green
	   	else if ((r> 400 && r< 900) &&  (b > 450 && b < 780) &&  (g > 650 && g < 1750)&& ((g > r) && (g > b))){
	   		p->color = 'g';
	   	}
	   	      //Blue
	   	else if((r> 590 && r< 1250) &&  (b > 1000 && b < 2000) &&  (g > 280 && g < 980) && ((b < r)) && (r> g)){
	   		p->color = 'b';
	   	}
	   	else {
	   		p->color='n'; // null
	   	}
};

