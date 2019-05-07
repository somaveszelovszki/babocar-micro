#ifndef CONFIG_H_
#define CONFIG_H_

#define SPEED_CTRL_PERIOD_US    500u    // TODO [us]
#define SPEED_CTRL_Ti_us        500u    // TODO [us]
#define SPEED_CTRL_Kc           0.0f    // TODO
#define SPEED_CTRL_OUT_MAX      0.90f   // Speed controller maximum output duty
#define SPEED_CTRL_DEADBAND_MPS 0.005f  // If measured speed is less than this value [m/s], speed controller output will be zero

#define ENCODER_MAX_VALUE           65536
#define ENCODER_TO_MPS_RATIO        1.0f    // TODO
#define MAX_CMD_DELAY_MS            50u     // If no command is received for this amount of time, motor needs to be stopped

#define SPEED_SEND_PERIOD_MS        10u     // Period of speed sending

#define RX_SIZE                     5       // RX message buffer size
#define TX_SIZE                     5       // TX message buffer size

#endif /* CONFIG_H_ */
