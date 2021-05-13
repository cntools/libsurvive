#include "sv_matrix.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SvMat {
	int type;
	int step;

	/* for internal use only */
	int *refcount;
	int hdr_refcount;

	union {
		unsigned char *ptr;
		short *s;
		int *i;
		float *fl;
		double *db;
	} data;

	int rows;
	int cols;

} SvMat;
// SvMat A = cvMat(num_pts, 3, SV_FLT, (FLT *)ptsA);

#define SV_FLT_PTR(m) ((FLT *)((m)->data.ptr))

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

#define SV_ELEM_SIZE(type)                                                                                             \
	(SV_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> SV_MAT_DEPTH(type) * 2) & 3))

/*
The function is a fast replacement for cvGetReal2D in the case of single-channel floating-point
matrices. It is faster because it is inline, it does fewer checks for array type and array element
type, and it checks for the row and column ranges only in debug mode.
@param mat Input matrix
@param row The zero-based index of row
@param col The zero-based index of column
 */
static inline double svMatrixGet(const SvMat *mat, int row, int col) {
	int type;

	type = SV_MAT_TYPE(mat->type);
	assert((unsigned)row < (unsigned)mat->rows && (unsigned)col < (unsigned)mat->cols);

	if (type == SV_32FC1)
		return ((float *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col];
	else {
		assert(type == SV_64FC1);
		return ((double *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col];
	}
}

/** @brief Sets a specific element of a single-channel floating-point matrix.

The function is a fast replacement for cvSetReal2D in the case of single-channel floating-point
matrices. It is faster because it is inline, it does fewer checks for array type and array element
type, and it checks for the row and column ranges only in debug mode.
@param mat The matrix
@param row The zero-based index of row
@param col The zero-based index of column
@param value The new value of the matrix element
 */
static inline void svMatrixSet(SvMat *mat, int row, int col, double value) {
	int type;
	type = SV_MAT_TYPE(mat->type);
	assert((unsigned)row < (unsigned)mat->rows && (unsigned)col < (unsigned)mat->cols);

	if (type == SV_32FC1)
		((float *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col] = (float)value;
	else {
		assert(type == SV_64FC1);
		((double *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col] = value;
	}
}

#ifdef __cplusplus
}
#endif