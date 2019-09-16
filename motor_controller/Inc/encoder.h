#ifndef ENCODER_H_
#define ENCODER_H_

#include <micro/utils/types.h>

typedef struct {
	int32_t prev_pos;
	int32_t max_value;
} encoder_t;

void encoder_initialize(encoder_t *enc, int32_t max_value);

int32_t encoder_get_diff(encoder_t *enc);

#endif /* ENCODER_H_ */
