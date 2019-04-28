#include "dc_motor.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"

extern uint32_t rc_recv_in_speed;
extern TIM_HandleTypeDef * const tim_motor;
extern const uint32_t chnl_fwd;
extern const uint32_t chnl_bwd;

#define PWM_PERIOD (48 * 20)

#define chnl2_pwm(fwd_pwm) (PWM_PERIOD - fwd_pwm)

#define PWM_FWD_MAX (PWM_PERIOD * 0.90f)
#define PWM_BWD_MAX chnl2_pwm(PWM_FWD_MAX)
#define PWM_STOP    ((PWM_FWD_MAX + PWM_BWD_MAX) / 2)

#define ERROR_LIMIT 3

void dc_motor_initialize() {
	dc_motor_write(0.0f, 0);
}

void dc_motor_write(float duty, uint8_t use_safety_enable_signal) {
	static uint8_t err_cntr = 0;

	int32_t pwm = PWM_STOP;

	if (use_safety_enable_signal) {
	    int32_t recvPwm = (int32_t)rc_recv_in_speed;

	    // after a given number of errors, stops motor
	    if (recvPwm < 900 || recvPwm > 2100) {
	        if (++err_cntr >= ERROR_LIMIT) {
	            recvPwm = 1500;
	        } // else: does not change motor pwm value
	    } else {
	        err_cntr = 0;

	        if (recvPwm > 1200 && recvPwm < 1800) {
	            recvPwm = 1500;
	        }

	        if (recvPwm <= 1500) {
	            pwm = map_int_to_int(recvPwm, 1500L, 1000L, PWM_STOP, PWM_BWD_MAX);
	        } else {
	            pwm = map_float_to_int(duty, -1.0f, 1.0f, PWM_BWD_MAX, PWM_FWD_MAX);
	        }
	    }
	} else {
	    pwm = map_float_to_int(duty, -1.0f, 1.0f, PWM_BWD_MAX, PWM_FWD_MAX);
	}

    pwm = clamp(pwm, PWM_BWD_MAX, PWM_FWD_MAX);
    const int32_t pwm2 = chnl2_pwm(pwm);

    __HAL_TIM_SET_COMPARE(tim_motor, chnl_fwd, pwm);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bwd, pwm2);
}
