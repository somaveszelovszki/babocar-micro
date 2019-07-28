#include "common.h"

int32_t round_to_int(float value) {
    return (int32_t)(value + 0.5f);
}

int32_t clamp(int32_t value, int32_t min_val, int32_t max_val) {
	int32_t result = value;
    if (value <= min_val) result = min_val;
    if (value >= max_val) result = max_val;
    return result;
}

float clampf(float value, float min_val, float max_val) {
	float result = value;
    if (value <= min_val) result = min_val;
    if (value >= max_val) result = max_val;
    return result;
}

int32_t map_int_to_int(int32_t value, int32_t from_low, int32_t from_high, int32_t to_low, int32_t to_high) {
    return to_low + (clamp(value, from_low, from_high) - from_low) * (to_high - to_low) / (from_high - from_low);
}

int32_t map_float_to_int(float value, float from_low, float from_high, int32_t to_low, int32_t to_high) {
    return to_low + (clampf(value, from_low, from_high) - from_low) * (to_high - to_low) / (from_high - from_low);
}
