#ifndef COMMON_H_
#define COMMON_H_

#include <micro/utils/types.h>

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

int32_t round_to_int(float value);

#define clamp(value, bound1, bound2)    \
((bound1) <= (bound2) ?                 \
max(min((value), (bound2)), (bound1)) : \
min(max((value), (bound2)), (bound1)))

#define map(value, from_low, from_high, to_low, to_high) \
((to_low) + (clamp((value), (from_low), (from_high)) - (from_low)) * ((to_high) - (to_low)) / ((from_high) - (from_low)))

#endif /* COMMON_H_ */
