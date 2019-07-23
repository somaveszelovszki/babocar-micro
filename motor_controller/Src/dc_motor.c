#include "dc_motor.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"

extern TIM_HandleTypeDef * const tim_motor;
extern const uint32_t chnl_fwd_high;
extern const uint32_t chnl_fwd_low;
extern const uint32_t chnl_bwd_high;
extern const uint32_t chnl_bwd_low;

#define PWM_PERIOD (48 * 20 - 1)

#define DEAD_TIME_TICK 2 // TODO

#define chnl2_pwm(fwd_pwm) (PWM_PERIOD - fwd_pwm)

#define PWM_MAX ((int)(PWM_PERIOD * 0.92f))
#define PWM_MIN chnl2_pwm(PWM_MAX)

void dc_motor_initialize() {
	dc_motor_write(0.0f, 0);
}

void dc_motor_write(float duty, uint8_t use_safety_enable_signal) {

    const int32_t pwm1 = map_float_to_int(duty, -1.0f, 1.0f, PWM_MIN, PWM_MAX);
    const int32_t pwm2 = chnl2_pwm(pwm1);

    // sets motor pwm atomically
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_fwd_high, pwm1);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bwd_high, pwm2);
    // TODO set all 4 timer values
    if (primask) {
        __enable_irq();
    }
}
