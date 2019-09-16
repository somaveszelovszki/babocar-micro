#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

#include <micro/utils/types.h>

void dc_motor_initialize();

void dc_motor_write(float duty);

#endif /* DC_MOTOR_H_ */
