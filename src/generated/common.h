#pragma once
#include "survive.h"
#include <linmath.h>
#include <math.h>
static inline double __safe_asin(double x) { return asin(linmath_enforce_range(x, -1, 1)); }
#define asin __safe_asin
#ifndef WIN32
#include <complex.h>
static inline double __safe_pow(double x, double y) { return x >= 0 ? pow(x, y) : creal(cpow(x, y)); }
#define pow __safe_pow
#endif
#define GEN_FLT FLT
