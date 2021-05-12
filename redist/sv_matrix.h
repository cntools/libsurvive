#pragma once

#ifdef USE_OPENCV
#include "minimal_opencv.opencv.h"

#else

#define SV_SVD 1
#define SV_SVD_MODIFY_A 1
#define SV_SVD_SYM 2
#define SV_SVD_U_T 2
#define SV_SVD_V_T 4

#define SV_GEMM_A_T 1
#define SV_GEMM_B_T 2
#define SV_GEMM_C_T 4

#define SV_8U 0
#define SV_8S 1
#define SV_16U 2
#define SV_16S 3
#define SV_32S 4
#define SV_32F 5
#define SV_64F 6

#ifdef USE_EIGEN
#include "sv_matrix.eigen.h"
#else
#include "sv_matrix.blas.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

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

void print_mat(const SvMat *M);

SvMat *svInitMatHeader(SvMat *arr, int rows, int cols, int type);
SvMat *svCreateMat(int height, int width, int type);

double svInvert(const SvMat *srcarr, SvMat *dstarr, int method);

void svGEMM(const SvMat *src1, const SvMat *src2, double alpha, const SvMat *src3, double beta, SvMat *dst, int tABC);

/**
 * xarr = argmin_x(Aarr * x - Barr)
 */
int svSolve(const SvMat *Aarr, const SvMat *Barr, SvMat *xarr, int method);

void svSetZero(SvMat *arr);

void svCopy(const SvMat *src, SvMat *dest, const SvMat *mask);

SvMat *svCloneMat(const SvMat *mat);

void svReleaseMat(SvMat **mat);

void svSVD(SvMat *aarr, SvMat *warr, SvMat *uarr, SvMat *varr, int flags);

void svMulTransposed(const SvMat *src, SvMat *dst, int order, const SvMat *delta, double scale);

void svTranspose(const SvMat *M, SvMat *dst);

void print_mat(const SvMat *M);

double svDet(const SvMat *M);

extern const int DECOMP_SVD;
extern const int DECOMP_LU;

#ifdef __cplusplus
}
#endif

#endif // NOT USE_OPENCV
