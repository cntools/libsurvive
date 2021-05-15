#ifdef __cplusplus
extern "C" {
#endif

#include "assert.h"
#include "stdint.h"
#include "stdlib.h"

#define SV_Error(code, msg) assert(0 && msg); // cv::error( code, msg, SV_Func, __FILE__, __LINE__ )

#define SV_32FC1 SV_32F
#define SV_64FC1 SV_64F

#define SV_MAGIC_MASK 0xFFFF0000
#define SV_MAT_MAGIC_VAL 0x42420000

#define SV_CN_MAX 512
#define SV_CN_SHIFT 3
#define SV_DEPTH_MAX (1 << SV_CN_SHIFT)

#define SV_MAT_DEPTH_MASK (SV_DEPTH_MAX - 1)
#define SV_MAT_DEPTH(flags) ((flags)&SV_MAT_DEPTH_MASK)

#define SV_MAKETYPE(depth, cn) (SV_MAT_DEPTH(depth) + (((cn)-1) << SV_CN_SHIFT))
#define SV_MAKE_TYPE SV_MAKETYPE

#define SV_MAT_CN_MASK ((SV_CN_MAX - 1) << SV_CN_SHIFT)
#define SV_MAT_CN(flags) ((((flags)&SV_MAT_CN_MASK) >> SV_CN_SHIFT) + 1)
#define SV_MAT_TYPE_MASK (SV_DEPTH_MAX * SV_CN_MAX - 1)
#define SV_MAT_TYPE(flags) ((flags)&SV_MAT_TYPE_MASK)
#define SV_MAT_CONT_FLAG_SHIFT 14
#define SV_MAT_CONT_FLAG (1 << SV_MAT_CONT_FLAG_SHIFT)
#define SV_IS_MAT_CONT(flags) ((flags)&SV_MAT_CONT_FLAG)
#define SV_IS_CONT_MAT SV_IS_MAT_CONT
#define SV_SUBMAT_FLAG_SHIFT 15
#define SV_SUBMAT_FLAG (1 << SV_SUBMAT_FLAG_SHIFT)
#define SV_IS_SUBMAT(flags) ((flags)&SV_MAT_SUBMAT_FLAG)

#define SV_IS_MATND_HDR(mat) ((mat) != NULL && (((const SvMat *)(mat))->type & SV_MAGIC_MASK) == SV_MATND_MAGIC_VAL)

#define SV_IS_MATND(mat) (SV_IS_MATND_HDR(mat) && ((const SvMat *)(mat))->data.ptr != NULL)
#define SV_MATND_MAGIC_VAL 0x42430000

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define SV_ELEM_SIZE(type)                                                                                             \
	(SV_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> SV_MAT_DEPTH(type) * 2) & 3))

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define SV_ELEM_SIZE(type)                                                                                             \
	(SV_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> SV_MAT_DEPTH(type) * 2) & 3))

//#include "shim_types_c.h"

#ifdef __cplusplus
}
#endif