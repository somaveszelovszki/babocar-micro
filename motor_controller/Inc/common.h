#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

int32_t round(float value);

int32_t clamp(int32_t value, int32_t min_val, int32_t max_val);

float clampf(float value, float min_val, float max_val);

int32_t map_int_to_int(int32_t value, int32_t from_low, int32_t from_high, int32_t to_low, int32_t to_high);

int32_t map_float_to_int(float value, float from_low, float from_high, int32_t to_low, int32_t to_high);

#endif /* COMMON_H_ */
