#include <pi_controller.h>

void pi_controller_initialize(pi_controller_t *pi, uint32_t period_us, uint32_t Ti_us, float Kc, float out_min, float out_max) {
	pi->b0 = Kc * (1 + (float)Ti_us / period_us);
	pi->b1 = -Kc;
	pi->out_min = out_min;
	pi->out_max = out_max;
	pi->desired = pi->output = pi->ek1 = 0.0f;
}

void pi_controller_update(pi_controller_t *pi, float measured) {
	const float ek = pi->desired - measured;
	pi->output += clamp(pi->b0 * ek + pi->b1 * pi->ek1, pi->out_min, pi->out_max);
	pi->ek1 = ek;
}
