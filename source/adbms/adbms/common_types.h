
#pragma once

#include <assert.h>
#include "stdint.h"
#include "stdbool.h"
#include <limits.h>

typedef      float  float32_t;
typedef      double float64_t;

_Static_assert(sizeof(float32_t)*CHAR_BIT == 32, "float 32");
_Static_assert(sizeof(float64_t)*CHAR_BIT == 64, "float 64");
