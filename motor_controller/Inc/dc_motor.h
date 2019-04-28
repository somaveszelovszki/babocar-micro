#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include "common.h"

void dc_motor_initialize();

void dc_motor_write(float duty, uint8_t use_safety_enable_signal);

#endif /* DC_MOTOR_H_ */
