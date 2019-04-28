#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#include "common.h"

typedef struct {
	float b0;
    float b1;
    float out_min;
    float out_max;

	float desired;
	float output;
	float ek1;

} pi_controller_t;

void pi_controller_initialize(pi_controller_t *pi, uint32_t period_us, uint32_t Ti_us, float Kc, float out_min, float out_max);

void pi_controller_update(pi_controller_t *pi, float measured);

#endif /* PI_CONTROLLER_H_ */
