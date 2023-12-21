#include "utils.h"


float utils_radPerSecToRPM(float omega){
	return omega * (60 / (2 * PI));
}

uint8_t utils_inRange(float number,float min, float max){
	if (number > min && number < max){
		return 1;
	}
	return 0;
};
