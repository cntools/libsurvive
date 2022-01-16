#pragma once
#include "survive.h"
#include <linmath.h>
#include <math.h>
static inline double __safe_sqrt(double x) { return x > 0 ? sqrt(x) : 0; }
#undef sqrt
#define sqrt __safe_sqrt
static inline double __safe_asin(double x) { return asin(linmath_enforce_range(x, -1, 1)); }
#undef asin
#define asin __safe_asin
#ifndef WIN32
#ifndef ANDROID
#include <complex.h>
static inline double __safe_pow(double x, double y) { return x >= 0 ? pow(x, y) : creal(cpow(x, y)); }
#undef pow
#define pow __safe_pow
#endif
#endif
#define GEN_FLT FLT
