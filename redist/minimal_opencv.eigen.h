#include "minimal_opencv.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CvMat {
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

} CvMat;
// CvMat A = cvMat(num_pts, 3, CV_FLT, (FLT *)ptsA);

#define CV_FLT_PTR(m) ((FLT *)((m)->data.ptr))

#define CV_32FC1 CV_32F
#define CV_64FC1 CV_64F

#define CV_MAGIC_MASK 0xFFFF0000
#define CV_MAT_MAGIC_VAL 0x42420000

#define CV_CN_MAX 512
#define CV_CN_SHIFT 3
#define CV_DEPTH_MAX (1 << CV_CN_SHIFT)

#define CV_MAT_DEPTH_MASK (CV_DEPTH_MAX - 1)
#define CV_MAT_DEPTH(flags) ((flags)&CV_MAT_DEPTH_MASK)

#define CV_MAKETYPE(depth, cn) (CV_MAT_DEPTH(depth) + (((cn)-1) << CV_CN_SHIFT))
#define CV_MAKE_TYPE CV_MAKETYPE

#define CV_MAT_CN_MASK ((CV_CN_MAX - 1) << CV_CN_SHIFT)
#define CV_MAT_CN(flags) ((((flags)&CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1)
#define CV_MAT_TYPE_MASK (CV_DEPTH_MAX * CV_CN_MAX - 1)
#define CV_MAT_TYPE(flags) ((flags)&CV_MAT_TYPE_MASK)
#define CV_MAT_CONT_FLAG_SHIFT 14
#define CV_MAT_CONT_FLAG (1 << CV_MAT_CONT_FLAG_SHIFT)
#define CV_IS_MAT_CONT(flags) ((flags)&CV_MAT_CONT_FLAG)
#define CV_IS_CONT_MAT CV_IS_MAT_CONT
#define CV_SUBMAT_FLAG_SHIFT 15
#define CV_SUBMAT_FLAG (1 << CV_SUBMAT_FLAG_SHIFT)
#define CV_IS_SUBMAT(flags) ((flags)&CV_MAT_SUBMAT_FLAG)

#define CV_ELEM_SIZE(type)                                                                                             \
	(CV_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> CV_MAT_DEPTH(type) * 2) & 3))

/*
The function is a fast replacement for cvGetReal2D in the case of single-channel floating-point
matrices. It is faster because it is inline, it does fewer checks for array type and array element
type, and it checks for the row and column ranges only in debug mode.
@param mat Input matrix
@param row The zero-based index of row
@param col The zero-based index of column
 */
static inline double cvmGet(const CvMat *mat, int row, int col) {
	int type;

	type = CV_MAT_TYPE(mat->type);
	assert((unsigned)row < (unsigned)mat->rows && (unsigned)col < (unsigned)mat->cols);

	if (type == CV_32FC1)
		return ((float *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col];
	else {
		assert(type == CV_64FC1);
		return ((double *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col];
	}
}

/** Inline constructor. No data is allocated internally!!!
 * (Use together with cvCreateData, or use cvCreateMat instead to
 * get a matrix with allocated data):
 */
static inline CvMat cvMat(int rows, int cols, int type, void *data) {
	CvMat m;

	assert((unsigned)CV_MAT_DEPTH(type) <= CV_64F);
	type = CV_MAT_TYPE(type);
	m.type = CV_MAT_MAGIC_VAL | CV_MAT_CONT_FLAG | type;
	m.cols = cols;
	m.rows = rows;
	m.step = m.cols * CV_ELEM_SIZE(type);
	m.data.ptr = (unsigned char *)data;
	m.refcount = 0;
	m.hdr_refcount = 0;

#if SURVIVE_ASAN_CHECKS
	volatile double v = cvmGet(&m, rows - 1, cols - 1);
	(void)v;
#endif

	return m;
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
static inline void cvmSet(CvMat *mat, int row, int col, double value) {
	int type;
	type = CV_MAT_TYPE(mat->type);
	assert((unsigned)row < (unsigned)mat->rows && (unsigned)col < (unsigned)mat->cols);

	if (type == CV_32FC1)
		((float *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col] = (float)value;
	else {
		assert(type == CV_64FC1);
		((double *)(void *)(mat->data.ptr + (size_t)mat->step * row))[col] = value;
	}
}

#ifdef __cplusplus
}
#endif