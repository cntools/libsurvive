#include "minimal_opencv.h"
#include "linmath.h"
#include "os_generic.h"
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

static inline void multiply(int N, const FLT *mat1, const FLT *mat2, FLT *res) {
	int i, j, k;
	for (i = 0; i < N; i++) {
		for (j = 0; j < N; j++) {
			res[i + j * N] = 0;
			for (k = 0; k < N; k++)
				res[i + j * N] += mat1[i + k * N] * mat2[k + j * N];
		}
	}
}

void test_sparse_matrix() {
	double _2x3[2 * 3] = {1, 2, 3, 0, 0, 6};
	CvMat m2x3 = cvMat(2, 3, CV_64F, _2x3);
	CvMat m3x2 = cvMat(3, 2, CV_64F, _2x3);
	ALLOC_SPARSE_MATRIX(sm2x3, 2, 3);
	create_sparse_matrix(&sm2x3, &m2x3);

	PRINT_MAT(m2x3);

	CREATE_STACK_MAT(D, 3, 3);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j <= i; j++) {
			_D[j + 3 * i] = _D[i + 3 * j] = linmath_normrand(0, 100);
		}
	}
	PRINT_MAT(D);

	{
		CREATE_STACK_MAT(Rd, 2, 3);
		CREATE_STACK_MAT(Rs, 2, 3);
		sparse_multiply_sparse_by_dense_sym(&Rs, &sm2x3, &D);
		PRINT_MAT(Rs);

		cvGEMM(&m2x3, &D, 1, 0, 0, &Rd, 0);
		PRINT_MAT(Rd);

		for (int i = 0; i < Rd.cols * Rd.rows; i++) {
			assert(fabs(_Rd[i] - _Rs[i]) < 1e-10);
		}
	}

	{
		CREATE_STACK_MAT(Rd, 2, 2);
		CREATE_STACK_MAT(Rs, 2, 2);

		gemm_ABAt_add(&Rd, &m2x3, &D, 0);
		PRINT_MAT(Rd);

		matrix_ABAt_add(&Rs, &m2x3, &D, 0);
		PRINT_MAT(Rs);

		bool valid = true;
		for (int i = 0; i < Rd.cols * Rd.rows; i++) {
			valid &= fabs(_Rd[i] - _Rs[i]) < 1e-10;
		}
		assert(valid);
		if (!valid)
			exit(-1);
	}
}

void test_speedN(int N) {
	CREATE_STACK_MAT(A, N, N);
	CREATE_STACK_MAT(B, N, N);
	CREATE_STACK_MAT(C, N, N);
	CREATE_STACK_MAT(Cs, N, N);
	CREATE_STACK_MAT(Ct, N, N);
	for (int i = 0; i < N * N; i++) {
		_A[i] = (i % 6 == 0) ? linmath_normrand(0, 100) : 0;
		//_A[i] = linmath_normrand(0, 100);
		_B[i] = linmath_normrand(0, 100);
	}
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < i; j++) {
			_B[i + j * N] = _B[j + i * N];
		}
	}

	/*
	PRINT_MAT(A);
	PRINT_MAT(B);
*/
	uint32_t cnts = 100000;
	double start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cnts; i++) {
		cvGEMM(&A, &B, 1, 0, 0, &C, 0);
	}
	double finish = OGGetAbsoluteTime();
	printf("Test speed %2d cvgemm: %10.2fkhz\n", N, cnts / (finish - start_gen) / 1000.);

	start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cnts; i++) {
		gemm_ABAt_add(&Ct, &A, &B, &A);
	}
	finish = OGGetAbsoluteTime();
	printf("Tran speed %2d cvgemm: %10.2fkhz\n", N, cnts / (finish - start_gen) / 1000.);

	start_gen = OGGetAbsoluteTime();
	ALLOC_SPARSE_MATRIX(s, N, N);
	for (int i = 0; i < cnts; i++) {
		create_sparse_matrix(&s, &A);
		sparse_multiply_sparse_by_dense_sym(&Cs, &s, &B);
	}

	for (int i = 0; i < C.cols * C.rows; i++) {
		assert(fabs(_Cs[i] - _C[i]) < 1e-10);
	}

	finish = OGGetAbsoluteTime();
	printf("Test speed %2d  naive: %10.2fkhz\n", N, cnts / (finish - start_gen) / 1000.);

	start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cnts; i++) {
		matrix_ABAt_add(&Cs, &A, &B, &A);
	}
	finish = OGGetAbsoluteTime();
	printf("Tran speed %2d  naive: %10.2fkhz\n\n", N, cnts / (finish - start_gen) / 1000.);

	bool valid = true;
	for (int i = 0; i < C.cols * C.rows; i++) {
		FLT diff = fabs(_Cs[i] - _Ct[i]);
		assert(diff < 1e-7);
		valid &= diff < 1e-7;
	}

	if (!valid) {
		PRINT_MAT(Cs);
		PRINT_MAT(Ct);
		assert(false);
		exit(-3);
	}
}

int main()
{
	test_gemm();
	test_solve();
	test_svd();
	test_multrans();

	test_sparse_matrix();
	for (int i = 1; i < 20; i++)
		test_speedN(i);

	return 0;
}

