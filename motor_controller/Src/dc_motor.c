#include "dc_motor.h"
#include "config.h"
#include "common.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_tim.h"
#include "tim.h"

#if !FACTORY_MOTOR_CONTROLLER
#define chnl2_pwm(fwd_pwm) (motor_PWM_PERIOD - fwd_pwm)
#endif // !FACTORY_MOTOR_CONTROLLER

void dc_motor_initialize() {
	dc_motor_write(0.0f);
}

void dc_motor_write(float duty) {

#if FACTORY_MOTOR_CONTROLLER

    const int32_t PWM_MAX  = motor_HARD_MAX;
    const int32_t PWM_STOP = motor_PWM_STOP;
    const int32_t PWM_MIN  = motor_HARD_MIN;

    const int32_t pwm = duty >= 0.0f ? map(duty, 0.0f, 1.0f, motor_PWM_STOP, PWM_MAX) : map(duty, 0.0f, -1.0f, motor_PWM_STOP, PWM_MIN);

    __HAL_TIM_SET_COMPARE(tim_motor, chnl_motor, (uint32_t)pwm);

#else

    static const int32_t DEAD_TIME_DELTA = motor_DEAD_TIME_TICK / 2;

    const int32_t PWM_MAX = motor_HARD_MAX;
    const int32_t PWM_MIN = chnl2_pwm(PWM_MAX);

    const int32_t pwm1 = map(duty, -1.0f, 1.0f, PWM_MIN, PWM_MAX);
    const int32_t pwm2 = chnl2_pwm(pwm1);

    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_1_high, pwm1 - DEAD_TIME_DELTA);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_1_low,  pwm1 + DEAD_TIME_DELTA);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_2_high, pwm2 - DEAD_TIME_DELTA);
    __HAL_TIM_SET_COMPARE(tim_motor, chnl_bridge_2_low,  pwm2 + DEAD_TIME_DELTA);

#endif // FACTORY_MOTOR_CONTROLLER

}
