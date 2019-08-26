#include "pi_controller.h"

#include <stdlib.h>

static void update_coeffs(pi_controller_t *pi) {
    pi->b0 = pi->Kc * (1 + (float)pi->Ti_us / pi->period_us);
    pi->b1 = -pi->Kc;
}


void pi_controller_initialize(pi_controller_t *pi, uint32_t period_us, uint32_t Ti_us, float Kc, float deadband, float out_min, float out_max, float max_delta) {
    pi->period_us = period_us;
    pi->Ti_us     = Ti_us;
    pi->Kc        = Kc;
    pi->deadband  = deadband;
    pi->out_min   = out_min;
    pi->out_max   = out_max;
    pi->max_delta = max_delta;
    pi->desired   = pi->output = pi->ek1 = 0.0f;

    update_coeffs(pi);
}

void pi_controller_set_Ti(pi_controller_t *pi, uint32_t Ti_us) {
    pi->Ti_us = Ti_us;
    update_coeffs(pi);
}

void pi_controller_set_Kc(pi_controller_t *pi, float Kc) {
    pi->Kc = Kc;
    update_coeffs(pi);
}

void pi_controller_update(pi_controller_t *pi, float measured) {
	if (0.0f == pi->desired && abs(measured) < pi->deadband) {
	    pi->output = 0.0f;
	    pi->ek1 = 0.0f;
	} else {
        const float ek = pi->desired - measured;
        const float prev_out = pi->output;
        pi->output += pi->b0 * ek + pi->b1 * pi->ek1;
        pi->output = clamp(pi->output, prev_out - pi->max_delta, prev_out + pi->max_delta);
        pi->output = clamp(pi->output, pi->out_min, pi->out_max);
	}
}
