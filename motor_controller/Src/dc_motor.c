#include "dc_motor.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"

extern TIM_HandleTypeDef * const tim_motor;
extern const uint32_t            chnl_fwd;
extern const uint32_t            chnl_bwd;

#define PWM_PERIOD (48 * 20)

#define chnl2_pwm(fwd_pwm) (PWM_PERIOD - fwd_pwm)

#define PWM_FWD_MAX (PWM_PERIOD * 0.92f)
#define PWM_BWD_MAX chnl2_pwm(PWM_FWD_MAX)

void dc_motor_initialize() {
	dc_motor_write(0.0f, 0);
}

void dc_motor_write(float duty, uint8_t use_safety_enable_signal) {

    const int32_t pwm1 = map_float_to_int(duty, -1.0f, 1.0f, PWM_BWD_MAX, PWM_FWD_MAX);
    const int32_t pwm2 = chnl2_pwm(pwm1);

    // sets motor pwm atomically
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_fwd, pwm1);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bwd, pwm2);
    if (primask) {
        __enable_irq();
    }
}
