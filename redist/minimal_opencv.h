#pragma once

#ifdef USE_OPENCV
#include "opencv2/core/core_c.h"
#include "opencv2/core/fast_math.hpp"
#else // NOT USE_OPENCV
#ifdef __cplusplus
extern "C" {
#endif

#include "assert.h"
#include "stdint.h"
#include "stdlib.h"

#define CV_Error(code, msg) assert(0 && msg); // cv::error( code, msg, CV_Func, __FILE__, __LINE__ )

#define CV_8U 0
#define CV_8S 1
#define CV_16U 2
#define CV_16S 3
#define CV_32S 4
#define CV_32F 5
#define CV_64F 6

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

typedef uint8_t uchar;

#define CV_IS_MAT_HDR(mat)                                                                                             \
	((mat) != NULL && (((const CvMat *)(mat))->type & CV_MAGIC_MASK) == CV_MAT_MAGIC_VAL &&                            \
	 ((const CvMat *)(mat))->cols > 0 && ((const CvMat *)(mat))->rows > 0)

#define CV_IS_MAT_HDR_Z(mat)                                                                                           \
	((mat) != NULL && (((const CvMat *)(mat))->type & CV_MAGIC_MASK) == CV_MAT_MAGIC_VAL &&                            \
	 ((const CvMat *)(mat))->cols >= 0 && ((const CvMat *)(mat))->rows >= 0)

#define CV_IS_MAT(mat) (CV_IS_MAT_HDR(mat) && ((const CvMat *)(mat))->data.ptr != NULL)

#define CV_IS_MASK_ARR(mat) (((mat)->type & (CV_MAT_TYPE_MASK & ~CV_8SC1)) == 0)

#define CV_ARE_TYPES_EQ(mat1, mat2) ((((mat1)->type ^ (mat2)->type) & CV_MAT_TYPE_MASK) == 0)

#define CV_ARE_CNS_EQ(mat1, mat2) ((((mat1)->type ^ (mat2)->type) & CV_MAT_CN_MASK) == 0)

#define CV_ARE_DEPTHS_EQ(mat1, mat2) ((((mat1)->type ^ (mat2)->type) & CV_MAT_DEPTH_MASK) == 0)

#define CV_ARE_SIZES_EQ(mat1, mat2) ((mat1)->rows == (mat2)->rows && (mat1)->cols == (mat2)->cols)

#define CV_IS_MAT_CONST(mat) (((mat)->rows | (mat)->cols) == 1)

#define CV_IS_MATND_HDR(mat) ((mat) != NULL && (((const CvMat *)(mat))->type & CV_MAGIC_MASK) == CV_MATND_MAGIC_VAL)

#define CV_IS_MATND(mat) (CV_IS_MATND_HDR(mat) && ((const CvMat *)(mat))->data.ptr != NULL)
#define CV_MATND_MAGIC_VAL 0x42430000

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define CV_ELEM_SIZE(type)                                                                                             \
	(CV_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> CV_MAT_DEPTH(type) * 2) & 3))

#ifndef MIN
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#endif

#ifndef MAX
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#endif

/** Matrix elements are stored row by row. Element (i, j) (i - 0-based row index, j - 0-based column
index) of a matrix can be retrieved or modified using CV_MAT_ELEM macro:

	uchar pixval = CV_MAT_ELEM(grayimg, uchar, i, j)
	CV_MAT_ELEM(cameraMatrix, float, 0, 2) = image.width*0.5f;

To access multiple-channel matrices, you can use
CV_MAT_ELEM(matrix, type, i, j\*nchannels + channel_idx).

@deprecated CvMat is now obsolete; consider using Mat instead.
*/
typedef struct CvMat {
	int type;
	int step;

	/* for internal use only */
	int *refcount;
	int hdr_refcount;

	union {
		uchar *ptr;
		short *s;
		int *i;
		float *fl;
		double *db;
	} data;

	int rows;
	int cols;

} CvMat;

#define CV_FLT_PTR(m) ((FLT *)((m)->data.ptr))

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
	m.data.ptr = (uchar *)data;
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

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define CV_ELEM_SIZE(type)                                                                                             \
	(CV_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> CV_MAT_DEPTH(type) * 2) & 3))

//#include "shim_types_c.h"

void print_mat(const CvMat *M);

CvMat *cvCreateMat(int height, int width, int type);

double cvInvert(const CvMat *srcarr, CvMat *dstarr, int method);

// Special case dst = alpha * src2 * src1 * src2' + beta * src3
void mulBABt(const CvMat *src1, const CvMat *src2, double alpha, const CvMat *src3, double beta, CvMat *dst);

void cvGEMM(const CvMat *src1, const CvMat *src2, double alpha, const CvMat *src3, double beta, CvMat *dst, int tABC);

int cvSolve(const CvMat *Aarr, const CvMat *Barr, CvMat *xarr, int method);

void cvSetZero(CvMat *arr);

void cvCopy(const CvMat *src, CvMat *dest, const CvMat *mask);

CvMat *cvCloneMat(const CvMat *mat);

void cvReleaseMat(CvMat **mat);

void cvSVD(CvMat *aarr, CvMat *warr, CvMat *uarr, CvMat *varr, int flags);

void cvMulTransposed(const CvMat *src, CvMat *dst, int order, const CvMat *delta, double scale);

void cvTranspose(const CvMat *M, CvMat *dst);

void print_mat(const CvMat *M);

double cvDet(const CvMat *M);

#define CV_SVD 1
#define CV_SVD_MODIFY_A 1
#define CV_SVD_SYM 2
#define CV_SVD_U_T 2
#define CV_SVD_V_T 4
extern const int DECOMP_SVD;
extern const int DECOMP_LU;

#define CV_GEMM_A_T 1
#define CV_GEMM_B_T 2
#define CV_GEMM_C_T 4

#ifdef __cplusplus
}
#endif
#endif // NOT USE_OPENCV
