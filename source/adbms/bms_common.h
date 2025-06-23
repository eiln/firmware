
#pragma once

#include <assert.h>
#include "stdint.h"
#include "stdbool.h"
#include <limits.h>

typedef float float32_t;
typedef double float64_t;
_Static_assert(sizeof(float32_t)*CHAR_BIT == 32, "float 32");
_Static_assert(sizeof(float64_t)*CHAR_BIT == 64, "float 64");

// Helper: clamp a float between min and max
static inline float32_t clampf(float32_t x, float32_t min, float32_t max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// Higher the alpha, slower the response/smoother the curve
static inline float32_t ema_filter(float32_t new_sample, float32_t prev_filtered, float32_t alpha)
{
	return alpha * new_sample + (1.0f - alpha) * prev_filtered;
}
