/*
 * utils.h
 *
 *  Created on: Dec 7, 2023
 *      Author: ADMIN
 */
#include "main.h"
#include "constants.h"

#ifndef INC_UTILS_H_
#define INC_UTILS_H_


float utils_radPerSecToRPM(float omega);
uint8_t utils_inRange(float number, float min, float max);

#endif /* INC_UTILS_H_ */
