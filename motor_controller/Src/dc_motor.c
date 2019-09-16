#include "dc_motor.h"
#include "config.h"
#include "common.h"

#include "stm32f0xx_hal_tim.h"

#define chnl2_pwm(fwd_pwm) (motor_PWM_PERIOD - fwd_pwm)

void dc_motor_initialize() {
	dc_motor_write(0.0f);
}

void dc_motor_write(float duty) {

    static const int32_t PWM_MAX         = motor_HARD_MAX;
    static const int32_t PWM_MIN         = chnl2_pwm(PWM_MAX);
    static const int32_t DEAD_TIME_DELTA = motor_DEAD_TIME_TICK / 2;

    const int32_t pwm1 = map(duty, -1.0f, 1.0f, PWM_MIN, PWM_MAX);
    const int32_t pwm2 = chnl2_pwm(pwm1);

    // sets motor pwm atomically
    __disable_irq();

    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_1_high, pwm1 - DEAD_TIME_DELTA);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_1_low,  pwm1 + DEAD_TIME_DELTA);

    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_2_high, pwm2 - DEAD_TIME_DELTA);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_2_low,  pwm2 + DEAD_TIME_DELTA);

    __enable_irq();
}
