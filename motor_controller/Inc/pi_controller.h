#ifndef PI_CONTROLLER_H_
#define PI_CONTROLLER_H_

#include <micro/utils/types.h>

typedef struct {
    uint32_t period_us;
    uint32_t Ti_us;
    float Kc;
    float deadband;
    float out_min;
    float out_max;
    float max_delta;

    float b0;
    float b1;
    float desired;
    float output;
    float ek1;

} pi_controller_t;

void pi_controller_initialize(pi_controller_t *pi, uint32_t period_us, uint32_t Ti_us, float Kc, float deadband, float out_min, float out_max, float max_delta);

void pi_controller_set_Ti(pi_controller_t *pi, uint32_t Ti_us);

void pi_controller_set_Kc(pi_controller_t *pi, float Kc);

void pi_controller_update(pi_controller_t *pi, float measured);

#endif /* PI_CONTROLLER_H_ */
