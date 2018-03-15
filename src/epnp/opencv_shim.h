int cvRound(float f);
#define CV_Error(code, msg) assert(0 && msg); // cv::error( code, msg, CV_Func, __FILE__, __LINE__ )

#include "shim_types_c.h"

void print_mat(const CvMat *M);

CvMat *cvCreateMat(int height, int width, int type);
double cvInvert(const CvMat *srcarr, CvMat *dstarr, int method);
void cvGEMM(const CvMat *src1, const CvMat *src2, double alpha, const CvMat *src3, double beta, CvMat *dst, int tABC);
int cvSolve(const CvMat *Aarr, const CvMat *Barr, CvMat *xarr, int method);
void cvSetZero(CvMat *arr);
void cvCopyTo(const CvMat *src, CvMat *dest);
CvMat *cvCloneMat(const CvMat *mat);
void cvReleaseMat(CvMat **mat);
void cvSVD(CvMat *aarr, CvMat *warr, CvMat *uarr, CvMat *varr, int flags);
void cvMulTransposed(const CvMat *src, CvMat *dst, int order, const CvMat *delta, double scale);

#define CV_SVD 1
#define CV_SVD_MODIFY_A 1
#define CV_SVD_SYM 2
#define CV_SVD_U_T 2
#define CV_SVD_V_T 4
extern const int DECOMP_SVD;
extern const int DECOMP_LU;

#define GEMM_1_T 1
#define GEMM_2_T 2
#define GEMM_3_T 4
