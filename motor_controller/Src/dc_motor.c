#include "dc_motor.h"
#include "config.h"

#include "stm32f0xx_hal_tim.h"
#include "stm32f0xx_hal_rcc.h"

#define chnl2_pwm(fwd_pwm) (motor_PWM_PERIOD - fwd_pwm)

#define PWM_MAX ((int32_t)(motor_PWM_PERIOD * 0.92f))
#define PWM_MIN chnl2_pwm(PWM_MAX)

void dc_motor_initialize() {
	dc_motor_write(0.0f, 0);
}

void dc_motor_write(float duty, uint8_t use_safety_enable_signal) {

    const int32_t pwm1 = map_float_to_int(duty, -1.0f, 1.0f, PWM_MIN, PWM_MAX);
    const int32_t pwm2 = chnl2_pwm(pwm1);

    // sets motor pwm atomically
    __disable_irq();

    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_1_high, pwm1 - motor_DEAD_TIME_TICK / 2);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_1_low, pwm1 + motor_DEAD_TIME_TICK / 2);

    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_2_high, pwm2 - motor_DEAD_TIME_TICK / 2);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_2_low, pwm2 + motor_DEAD_TIME_TICK / 2);

    __enable_irq();
}
