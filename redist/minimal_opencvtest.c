#include "minimal_opencv.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void print_mat(const CvMat *M) {
	for (int i = 0; i < M->rows; i++) {
		for (int j = 0; j < M->cols; j++) {
			printf("%f,\t", cvmGet(M, i, j));
		}
		printf("\n");
	}
	printf("\n");
}

#define PRINT_MAT(name)                                                                                                \
	printf(#name "\n");                                                                                                \
	print_mat(&name);

void test_gemm() {
  double _2x3[2*3] = {1, 2, 3, 4, 5, 6};
  CvMat m2x3 = cvMat(2, 3, CV_64F, _2x3);
  
  double _3x2[2*3] = {1, 2, 3, 4, 5, 6};
  CvMat m3x2 = cvMat(3, 2, CV_64F, _3x2);
  
  double _2x2[2*2] = {1, 2, 3, 4};
  CvMat m2x2 = cvMat(2, 2, CV_64F, _2x2);

  double _3x3[3*3] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  CvMat m3x3 = cvMat(3, 3, CV_64F, _3x3);

  cvGEMM(&m2x3, &m3x2, 1, 0, 0, &m2x2, 0);
  cvGEMM(&m3x2, &m2x3, 1, 0, 0, &m3x3, 0);
  PRINT_MAT(m2x3);
  PRINT_MAT(m3x2);
  PRINT_MAT(m3x3);
  PRINT_MAT(m2x2);

  cvGEMM(&m2x3, &m2x3, 1, 0, 0, &m3x3, CV_GEMM_A_T);
  cvGEMM(&m2x3, &m2x3, 1, 0, 0, &m2x2, CV_GEMM_B_T);
  PRINT_MAT(m3x3);
  PRINT_MAT(m2x2);

  cvGEMM(&m2x3, &m3x2, 1, 0, 0, &m3x3, CV_GEMM_A_T | CV_GEMM_B_T);
  //  cvGEMM(&m3x2, &m2x3, 1, 0, 0, &m2x2, CV_GEMM_A_T | CV_GEMM_B_T);

  PRINT_MAT(m3x3);
}

static void test_solve() {
	{
		double _A[3] = {1, 2, 3};
		double _B[3] = {4, 8, 12};
		double _x[1] = {0};

		CvMat A = cvMat(3, 1, CV_64F, _A);
		CvMat B = cvMat(3, 1, CV_64F, _B);
		CvMat x = cvMat(1, 1, CV_64F, _x);

		cvSolve(&A, &B, &x, CV_SVD);

		assert(fabs(_x[0] - 4) < .001);
	}

	{
		double _A[3] = {1, 2, 3};
		double _B[9] = {4, 5, 6, 7, 8, 9, 10, 11, 12};
		double _x[3] = {0};

		CvMat A = cvMat(3, 1, CV_64F, _A);
		CvMat B = cvMat(3, 3, CV_64F, _B);
		CvMat x = cvMat(1, 3, CV_64F, _x);

		cvSolve(&A, &B, &x, CV_SVD);
	}
}

static void test_svd() {
	printf("SVD:\n");

	double _3x3[3 * 3] = {1, 2, 3, 4, 5, 6, 7, 8, 12};
	CvMat m3x3 = cvMat(3, 3, CV_64F, _3x3);

	double _w[3] = {0};
	CvMat w = cvMat(1, 3, CV_64F, _w);

	double _u[9] = {0};
	CvMat u = cvMat(3, 3, CV_64F, _u);

	double _v[9] = {0};
	CvMat v = cvMat(3, 3, CV_64F, _v);

	cvSVD(&m3x3, &w, &u, &v, 0);

	PRINT_MAT(w);
	PRINT_MAT(u);
	PRINT_MAT(v);
}

static void test_multrans() {
	double _A[3] = {1, 2, 3};
	CvMat A = cvMat(3, 1, CV_64F, _A);

	double _B[9];
	CvMat B = cvMat(3, 3, CV_64F, _B);

	double _C[1];
	CvMat C = cvMat(1, 1, CV_64F, _C);

	cvMulTransposed(&A, &B, 0, 0, 1);
	PRINT_MAT(B);
	double ans[] = {1, 2, 3, 2, 4, 6, 3, 6, 9};
	for (int i = 0; i < 9; i++) {
		assert(ans[i] == _B[i]);
	}

	cvMulTransposed(&A, &C, 1, 0, 10);
	PRINT_MAT(C);
	assert(_C[0] == 140);
}

int main()
{
  test_gemm();
  test_solve();
  test_svd();
  test_multrans();
  return 0;
}

