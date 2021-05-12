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

typedef uint8_t uchar;

#define SV_IS_MAT_HDR(mat)                                                                                             \
	((mat) != NULL && (((const SvMat *)(mat))->type & SV_MAGIC_MASK) == SV_MAT_MAGIC_VAL &&                            \
	 ((const SvMat *)(mat))->cols > 0 && ((const SvMat *)(mat))->rows > 0)

#define SV_IS_MAT_HDR_Z(mat)                                                                                           \
	((mat) != NULL && (((const SvMat *)(mat))->type & SV_MAGIC_MASK) == SV_MAT_MAGIC_VAL &&                            \
	 ((const SvMat *)(mat))->cols >= 0 && ((const SvMat *)(mat))->rows >= 0)

#define SV_IS_MAT(mat) (SV_IS_MAT_HDR(mat) && ((const SvMat *)(mat))->data.ptr != NULL)

#define SV_IS_MASK_ARR(mat) (((mat)->type & (SV_MAT_TYPE_MASK & ~SV_8SC1)) == 0)

#define SV_ARE_TYPES_EQ(mat1, mat2) ((((mat1)->type ^ (mat2)->type) & SV_MAT_TYPE_MASK) == 0)

#define SV_ARE_CNS_EQ(mat1, mat2) ((((mat1)->type ^ (mat2)->type) & SV_MAT_CN_MASK) == 0)

#define SV_ARE_DEPTHS_EQ(mat1, mat2) ((((mat1)->type ^ (mat2)->type) & SV_MAT_DEPTH_MASK) == 0)

#define SV_ARE_SIZES_EQ(mat1, mat2) ((mat1)->rows == (mat2)->rows && (mat1)->cols == (mat2)->cols)

#define SV_IS_MAT_CONST(mat) (((mat)->rows | (mat)->cols) == 1)

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

/** Matrix elements are stored row by row. Element (i, j) (i - 0-based row index, j - 0-based column
index) of a matrix can be retrieved or modified using SV_MAT_ELEM macro:

	uchar pixval = SV_MAT_ELEM(grayimg, uchar, i, j)
	SV_MAT_ELEM(cameraMatrix, float, 0, 2) = image.width*0.5f;

To access multiple-channel matrices, you can use
SV_MAT_ELEM(matrix, type, i, j\*nchannels + channel_idx).

@deprecated SvMat is now obsolete; consider using Mat instead.
*/
typedef struct SvMat {
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

} SvMat;

#define SV_FLT_PTR(m) ((FLT *)((m)->data.ptr))

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

/** Inline constructor. No data is allocated internally!!!
 * (Use together with cvCreateData, or use svCreateMat instead to
 * get a matrix with allocated data):
 */
static inline SvMat svMat(int rows, int cols, int type, void *data) {
	SvMat m;

	assert((unsigned)SV_MAT_DEPTH(type) <= SV_64F);
	type = SV_MAT_TYPE(type);
	m.type = SV_MAT_MAGIC_VAL | SV_MAT_CONT_FLAG | type;
	m.cols = cols;
	m.rows = rows;
	m.step = m.cols * SV_ELEM_SIZE(type);
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

/** 0x3a50 = 11 10 10 01 01 00 00 ~ array of log2(sizeof(arr_type_elem)) */
#define SV_ELEM_SIZE(type)                                                                                             \
	(SV_MAT_CN(type) << ((((sizeof(size_t) / 4 + 1) * 16384 | 0x3a50) >> SV_MAT_DEPTH(type) * 2) & 3))

//#include "shim_types_c.h"

#ifdef __cplusplus
}
#endif