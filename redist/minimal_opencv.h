#pragma once

#ifdef USE_OPENCV
#include "minimal_opencv.opencv.h"

#else

#define CV_SVD 1
#define CV_SVD_MODIFY_A 1
#define CV_SVD_SYM 2
#define CV_SVD_U_T 2
#define CV_SVD_V_T 4

#define CV_GEMM_A_T 1
#define CV_GEMM_B_T 2
#define CV_GEMM_C_T 4

#define CV_8U 0
#define CV_8S 1
#define CV_16U 2
#define CV_16S 3
#define CV_32S 4
#define CV_32F 5
#define CV_64F 6

#ifdef USE_EIGEN
#include "minimal_opencv.eigen.h"
#else
#include "minimal_opencv.blas.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

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

void print_mat(const CvMat *M);

CvMat *cvInitMatHeader(CvMat *arr, int rows, int cols, int type);
CvMat *cvCreateMat(int height, int width, int type);

double cvInvert(const CvMat *srcarr, CvMat *dstarr, int method);

// Special case dst = alpha * src2 * src1 * src2' + beta * src3
void mulBABt(const CvMat *src1, const CvMat *src2, double alpha, const CvMat *src3, double beta, CvMat *dst);

void cvGEMM(const CvMat *src1, const CvMat *src2, double alpha, const CvMat *src3, double beta, CvMat *dst, int tABC);

/**
 * xarr = argmin_x(Aarr * x - Barr)
 */
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

extern const int DECOMP_SVD;
extern const int DECOMP_LU;

#ifdef __cplusplus
}
#endif

#endif // NOT USE_OPENCV
