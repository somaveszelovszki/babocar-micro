#include "encoder.h"
#include "config.h"

#include <stdlib.h>

void encoder_initialize(encoder_t *enc, int32_t max_value) {
	enc->prev_pos = 0;
	enc->max_value = max_value;
}

int32_t encoder_get_diff(encoder_t *enc) {
	const int32_t pos = __HAL_TIM_GET_COUNTER(tim_encoder);
	int32_t diff = pos - enc->prev_pos;

	if (abs(diff) > enc->max_value / 2) {
		const int32_t dp = diff + enc->max_value, dm = diff - enc->max_value;
		diff = abs(dp) <= abs(dm) ? dp : dm;
	}

	enc->prev_pos = pos;
	return diff;
}
