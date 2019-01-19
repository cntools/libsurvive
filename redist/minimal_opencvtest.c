#include "minimal_opencv.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void print_mat(const CvMat *M) {}

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

  cvGEMM(&m2x3, &m2x3, 1, 0, 0, &m3x3, GEMM_1_T);
  cvGEMM(&m2x3, &m2x3, 1, 0, 0, &m2x2, GEMM_2_T);

  cvGEMM(&m2x3, &m3x2, 1, 0, 0, &m3x3, GEMM_1_T | GEMM_2_T);
  //  cvGEMM(&m3x2, &m2x3, 1, 0, 0, &m2x2, GEMM_1_T | GEMM_2_T);
  
}

static void test_solve() {
	{
		double _A[3] = {1, 2, 3};
		double _B[1] = {4};
		double _x[3] = {};

		CvMat A = cvMat(1, 3, CV_64F, _A);
		CvMat B = cvMat(1, 1, CV_64F, _B);
		CvMat x = cvMat(3, 1, CV_64F, _x);

		cvSolve(&A, &B, &x, CV_SVD);

		double Ax = _A[0] * _x[0] + _A[1] * _x[1] + _A[2] * _x[2];
		assert(fabs(_B[0] - Ax) < .001);
	}

	{
		double _A[3] = {1, 2, 3};
		double _B[9] = {4, 5, 6, 7, 8, 9, 10, 11, 12};
		double _x[3] = {};

		CvMat A = cvMat(3, 1, CV_64F, _A);
		CvMat B = cvMat(3, 3, CV_64F, _B);
		CvMat x = cvMat(1, 3, CV_64F, _x);

		cvSolve(&A, &B, &x, CV_SVD);
	}
}

int main()
{
  test_gemm();
  test_solve();
  return 0;
}

