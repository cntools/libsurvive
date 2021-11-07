#pragma once

#ifdef USE_OPENCV
#include "minimal_opencv.opencv.h"
#else

#include "linmath.h"
#include "math.h"
#include "string.h"

#define SV_FLT_PTR(m) ((FLT *)((m)->data))

typedef struct SvMat {
	int step;

	/* for internal use only */
	int *refcount;
	int hdr_refcount;

	FLT *data;

	int rows;
	int cols;

} SvMat;

#ifdef USE_EIGEN
#include "sv_matrix.eigen.h"
#else
#include "sv_matrix.blas.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
// Avoids missing-field-initializers errors
#define ZERO_INITIALIZATION                                                                                            \
	{}
#else
#define ZERO_INITIALIZATION                                                                                            \
	{ 0 }
#endif

void print_mat(const SvMat *M);

SvMat *svInitMatHeader(SvMat *arr, int rows, int cols);
SvMat *svCreateMat(int height, int width);

enum svInvertMethod {
	SV_INVERT_METHOD_UNKNOWN = 0,
	SV_INVERT_METHOD_SVD = 1,
	SV_INVERT_METHOD_LU = 2,
	SV_INVERT_METHOD_QR = 3,
};

double svInvert(const SvMat *srcarr, SvMat *dstarr, enum svInvertMethod method);

enum svGEMMFlags {
	SV_GEMM_FLAG_A_T = 1,
	SV_GEMM_FLAG_B_T = 2,
	SV_GEMM_FLAG_C_T = 4,
};

void svGEMM(const SvMat *src1, const SvMat *src2, double alpha, const SvMat *src3, double beta, SvMat *dst,
			enum svGEMMFlags tABC);

/**
 * xarr = argmin_x(Aarr * x - Barr)
 */
int svSolve(const SvMat *Aarr, const SvMat *Barr, SvMat *xarr, enum svInvertMethod method);

void svSetZero(SvMat *arr);

void svCopy(const SvMat *src, SvMat *dest, const SvMat *mask);

SvMat *svCloneMat(const SvMat *mat);

void svReleaseMat(SvMat **mat);

enum svSVDFlags { SV_SVD_MODIFY_A = 1, SV_SVD_U_T = 2, SV_SVD_V_T = 4 };

void svSVD(SvMat *aarr, SvMat *warr, SvMat *uarr, SvMat *varr, enum svSVDFlags flags);

void svMulTransposed(const SvMat *src, SvMat *dst, int order, const SvMat *delta, double scale);

void svTranspose(const SvMat *M, SvMat *dst);

void print_mat(const SvMat *M);

double svDet(const SvMat *M);

#ifdef SV_MATRIX_USE_MALLOC
#define SV_MATRIX_ALLOC(size) calloc(1, size)
#define SV_MATRIX_FREE(ptr) free(ptr)
#define SV_MATRIX_STACK_SCOPE_BEGIN {
#define SV_MATRIX_STACK_SCOPE_END }
#else
#define SV_MATRIX_ALLOC(size) (memset(alloca(size), 0, size))
#define SV_MATRIX_FREE(ptr)
#define SV_MATRIX_STACK_SCOPE_BEGIN
#define SV_MATRIX_STACK_SCOPE_END
#endif

#define SV_CREATE_STACK_MAT(name, rows, cols)                                                                          \
	SV_MATRIX_STACK_SCOPE_BEGIN                                                                                        \
	FLT *_##name = SV_MATRIX_ALLOC((rows) * (cols) * sizeof(FLT));                                                     \
	SvMat name = svMat(rows, cols, _##name);

#define SV_FREE_STACK_MAT(name)                                                                                        \
	SV_MATRIX_FREE(_##name);                                                                                           \
	SV_MATRIX_STACK_SCOPE_END

#ifndef SV_MATRIX_IS_COL_MAJOR
static inline int sv_stride(const struct SvMat *m) { return m->rows; }
#else
static inline int sv_stride(const struct SvMat *m) { return m->cols; }
#endif

static inline void sv_set_zero(struct SvMat *m) { memset(SV_FLT_PTR(m), 0, sizeof(FLT) * m->rows * m->cols); }
static inline void sv_set_constant(struct SvMat *m, FLT v) {
	for (int i = 0; i < m->rows * m->cols; i++)
		SV_FLT_PTR(m)[i] = v;
}

static inline bool sv_is_finite(const struct SvMat *m) {
	for (int i = 0; i < m->rows * m->cols; i++)
		if (!isfinite(SV_FLT_PTR(m)[i]))
			return false;
	return true;
}

static inline void sv_matrix_copy(struct SvMat *dst, const struct SvMat *src) {
	assert(dst->rows == src->rows);
	assert(dst->cols == src->cols);
	memcpy(SV_FLT_PTR(dst), SV_FLT_PTR(src), sizeof(FLT) * dst->cols * dst->rows);
}

static inline FLT *sv_as_vector(struct SvMat *m) {
	assert(m->rows == 1 || m->cols == 1);
	return SV_FLT_PTR(m);
}

static inline const FLT *sv_as_const_vector(const struct SvMat *m) {
	assert(m->rows == 1 || m->cols == 1);
	return SV_FLT_PTR(m);
}

/** Inline constructor. No data is allocated internally!!!
 * (Use together with svCreateData, or use svCreateMat instead to
 * get a matrix with allocated data):
 */
static inline SvMat svMat(int rows, int cols, FLT *data) {
	SvMat m = ZERO_INITIALIZATION;

	m.cols = cols;
	m.rows = rows;
#ifndef SV_MATRIX_IS_COL_MAJOR
	m.step = m.cols;
#else
	m.step = m.rows;
#endif

	if (!data) {
		m.data = (FLT *)calloc(m.cols * m.rows, sizeof(FLT));
	} else {
		m.data = (FLT *)data;
	}
	m.refcount = 0;
	m.hdr_refcount = 0;

#if SURVIVE_ASAN_CHECKS
	volatile double v = cvmGet(&m, rows - 1, cols - 1);
	(void)v;
#endif

	return m;
}

static inline size_t sv_matrix_idx(const SvMat *mat, int row, int col) {
	assert((unsigned)row < (unsigned)mat->rows && (unsigned)col < (unsigned)mat->cols);
#ifndef SV_MATRIX_IS_COL_MAJOR
	return (size_t)mat->step * row + col;
#else
	return (size_t)mat->step * col + row;
#endif
}

/*
The function is a fast replacement for cvGetReal2D in the case of single-channel floating-point
matrices. It is faster because it is inline, it does fewer checks for array type and array element
type, and it checks for the row and column ranges only in debug mode.
@param mat Input matrix
@param row The zero-based index of row
@param col The zero-based index of column
 */
static inline FLT svMatrixGet(const SvMat *mat, int row, int col) { return mat->data[sv_matrix_idx(mat, row, col)]; }

static inline FLT *svMatrixPtr(const SvMat *mat, int row, int col) { return &mat->data[sv_matrix_idx(mat, row, col)]; }
/** @brief Sets a specific element of a single-channel floating-point matrix.

The function is a fast replacement for cvSetReal2D in the case of single-channel floating-point
matrices. It is faster because it is inline, it does fewer checks for array type and array element
type, and it checks for the row and column ranges only in debug mode.
@param mat The matrix
@param row The zero-based index of row
@param col The zero-based index of column
@param value The new value of the matrix element
 */
static inline void svMatrixSet(SvMat *mat, int row, int col, FLT value) {
	mat->data[sv_matrix_idx(mat, row, col)] = value;
}

static inline void sv_get_diag(const struct SvMat *m, FLT *v, size_t cnt) {
	for (size_t i = 0; i < cnt; i++) {
		v[i] = svMatrixGet(m, i, i);
	}
}
static inline void sv_set_diag(struct SvMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			svMatrixSet(m, i, j, i == j ? (v ? v[i] : 1.) : 0.);
		}
	}
}

static inline void sv_set_diag_val(struct SvMat *m, FLT v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			svMatrixSet(m, i, j, i == j ? v : 0.);
		}
	}
}

static inline void sv_eye(struct SvMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			svMatrixSet(m, i, j, i == j ? (v ? v[i] : 1.) : 0.);
		}
	}
}

static inline void sv_copy_in_row_major_roi(struct SvMat *dst, const FLT *src, size_t src_stride, int start_i,
											int start_j, int end_i, int end_j) {
	for (int i = start_i; i < end_i; i++) {
		for (int j = start_j; j < end_j; j++) {
			svMatrixSet(dst, i, j, src[j + i * src_stride]);
		}
	}
}

static inline void sv_copy_in_row_major(struct SvMat *dst, const FLT *src, size_t src_stride) {
	sv_copy_in_row_major_roi(dst, src, src_stride, 0, 0, dst->rows, dst->cols);
}
static inline FLT sv_sum(const struct SvMat *A) {
	FLT rtn = 0;
	for (int i = 0; i < A->rows; i++) {
		for (int j = 0; j < A->cols; j++) {
			rtn += svMatrixGet(A, i, j);
		}
	}
	return rtn;
}
static inline FLT sv_trace(const struct SvMat *A) {
	FLT rtn = 0;
	int min_dim = A->rows;
	if (min_dim > A->cols)
		min_dim = A->cols;
	for (int i = 0; i < min_dim; i++) {
		for (int j = 0; j < min_dim; j++) {
			rtn += svMatrixGet(A, i, j);
		}
	}

	return rtn;
}

static inline void sv_copy_data_in(struct SvMat *A, bool isRowMajor, const FLT *d) {
#ifdef SV_MATRIX_IS_COL_MAJOR
	bool needsFlip = isRowMajor;
#else
	bool needsFlip = !isRowMajor;
#endif
	if (needsFlip) {
		SvMat t = svMat(A->cols, A->rows, (FLT *)d);
		svTranspose(&t, A);
	} else {
		memcpy(A->data, d, A->rows * A->cols * sizeof(FLT));
	}
}

static inline void sv_row_major_to_internal(struct SvMat *A, const FLT *d) { sv_copy_data_in(A, true, d); }

static inline void sv_col_major_to_internal(struct SvMat *A, const FLT *d) { sv_copy_data_in(A, false, d); }

static inline SvMat svMat_from_row_major(int rows, int cols, FLT *data) {
	SvMat rtn = svMat(rows, cols, data);
	sv_row_major_to_internal(&rtn, data);
	return rtn;
}

static inline SvMat svMat_from_col_major(int rows, int cols, FLT *data) {
	SvMat rtn = svMat(rows, cols, data);
	sv_col_major_to_internal(&rtn, data);
	return rtn;
}

static inline void sv_elementwise_subtract(struct SvMat *dst, const struct SvMat *A, const struct SvMat *B) {
	for (int i = 0; i < A->rows; i++) {
		for (int j = 0; j < A->cols; j++) {
			svMatrixSet(dst, i, j, svMatrixGet(A, i, j) - svMatrixGet(B, i, j));
		}
	}
}

#ifndef SV_MATRIX_IS_COL_MAJOR
static inline SvMat sv_row(struct SvMat *M, int r) {
	assert(r < M->rows);
	return svMat(1, M->cols, M->data + M->step * r);
}
#endif

#ifdef __cplusplus
}
#endif

#endif // NOT USE_OPENCV
