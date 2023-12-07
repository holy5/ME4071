#include "utils.h"
#include "math.h"

float utils_omegaToRPM(float omega){
	return omega * 60 / 2 / PI;
}
