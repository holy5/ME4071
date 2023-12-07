/*
 * line_sensor.c
 *
 *  Created on: Dec 5, 2023
 *      Author: ADMIN
 */
#include "line_sensor.h"
#include "math.h"

float linesensor_calculateYError(lineSensorParams_st* p)
{
   float num = (-34 * (p->sensor_values[4] - p->sensor_values[0])) + (-17 * (p->sensor_values[3] - p->sensor_values[1])) ;
   float denom = p->sensor_values[0] + p->sensor_values[1] + p->sensor_values[2] + p->sensor_values[3] + p->sensor_values[4];
   float error = num / denom;
   return error;
}

static float linesensor_calulateAngleError(lineSensorParams_st* p){
	return (linesensor_calculateYError(p) - 0.6986)/(float)0.7703;
}

static float linesensor_calculateDs(motorParams_st* mp, lineSensorParams_st* p){
	uint16_t now = HAL_GetTick();
	uint16_t delta_time = now - p->time_prev;
	p->time_prev = now;

	return mp->speed * delta_time;
}

void linesensor_read(lineSensorParams_st* p,motorParams_st* mp){
      HAL_ADC_Start_DMA(p->hadc,(uint32_t*)p->adc_values,5);
      p->sensor_values[0]=233+(3037.0f/3154.0f*(p->adc_values[0]-235));
      p->sensor_values[1]=233+(3037.0f/3028.0f*(p->adc_values[1]-230));
      p->sensor_values[2]=233+(3037.0f/3021.0f*(p->adc_values[2]-226));
      p->sensor_values[3]=233+(3037.0f/3068.0f*(p->adc_values[3]-234));
      p->sensor_values[4]=233+(3037.0f/3068.0f*(p->adc_values[4]-240));

      p->time = HAL_GetTick(); // this should be converted to time

      p->y_error = linesensor_calulateAngleError(p); //mm

      float ds = linesensor_calculateDs(mp,p);

      p->angle_error = atan((p->angle_error-p->angle_error_prev)/ds);
      //e3 = 0;
      p->angle_error_prev = p->y_error;

      p->time_prev = p->time;
}



