#include "opencv2/core/core_c.h"
#ifndef ANDROID
#include "opencv2/core/fast_math.hpp"
#endif

#ifdef ANDROID
#define SV_FLT_PTR(m) ((FLT *)((m)->data.ptr))
#define DECOMP_LU 2
#endif
