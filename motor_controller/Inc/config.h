#ifndef CONFIG_H_
#define CONFIG_H_

#include "tim.h"
#include "usart.h"

#define SPEED_CTRL_PERIOD_US    500u    // TODO [us]
#define SPEED_CTRL_Ti_us        500u    // TODO [us]
#define SPEED_CTRL_Kc           0.0f    // TODO
#define SPEED_CTRL_OUT_MAX      0.90f   // Speed controller maximum output duty
#define SPEED_CTRL_DEADBAND_MPS 0.005f  // If measured speed is less than this value [m/s], speed controller output will be zero

#define ENCODER_MAX_VALUE             65536
#define ENCODER_TO_MPS_RATIO          1.0f  // TODO
#define MAX_CMD_DELAY_MS              50u   // If no command is received for this amount of time, motor needs to be stopped

#define SAFETY_SIGNAL_CHECK_PERIOD_MS 10u   // Period of safety signal checking

#define SPEED_SEND_PERIOD_MS          10u   // Period of speed sending

#define RX_SIZE                       5     // RX message buffer size
#define TX_SIZE                       5     // TX message buffer size

#define uart_cmd (&huart1)

#define gpio_user_led     GPIOB
#define gpio_pin_user_led GPIO_PIN_5

#define tim_motor            (&htim1)
#define chnl_bridge_2_high   TIM_CHANNEL_1
#define chnl_bridge_2_low    TIM_CHANNEL_2
#define chnl_bridge_1_high   TIM_CHANNEL_3
#define chnl_bridge_1_low    TIM_CHANNEL_4
#define motor_PWM_PERIOD     (48 * 20)
#define motor_DEAD_TIME_TICK 20

#define tim_encoder (&htim3)

#define tim_rc_recv  (&htim14)
#define chnl_rc_recv TIM_CHANNEL_1

#define tim_speedControllerPeriod (&htim17)

#endif /* CONFIG_H_ */
