#include "epnp.h"
#include "stdio.h"

/* Parameters */

void test_svd() {
	double wkopt;
	double *work;
/* Local arrays */
/* iwork dimension should be at least 8*min(m,n) */
#define COLS 4
#define ROWS 6

#define LDA ROWS
#define LDU ROWS
#define LDVT COLS

	double s[COLS], u[LDU * ROWS], vt[LDVT * COLS];
	double a[ROWS * COLS] = {7.52,  -1.10, -7.95, 1.08, -0.76, 0.62, 9.34,  -7.10, 5.13,  6.62,  -5.66, 0.87,
							 -4.75, 8.52,  5.75,  5.30, 1.33,  4.91, -5.49, -3.52, -2.40, -6.77, 2.34,  3.95};

	CvMat A = cvMat(ROWS, COLS, CV_64F, a);
	CvMat S = cvMat(1, COLS, CV_64F, s);
	CvMat U = cvMat(LDU, ROWS, CV_64F, u);
	CvMat VT = cvMat(LDVT, COLS, CV_64F, vt);

	cvSVD(&A, &S, &U, &VT, 0);

	print_mat(&A);
	print_mat(&S);
	print_mat(&U);
	print_mat(&VT);

	double n[LDVT * COLS];
	CvMat N = cvMat(LDVT, COLS, CV_64F, n);

	printf("Tf:\n");
	cvMulTransposed(&VT, &N, 1, 0, 1);
	print_mat(&N);
}

void test_solve() {
	int msize = 10;
	CvMat *A = cvCreateMat(msize, msize, CV_64F);
	for (unsigned i = 0; i < A->rows; i++) {
		for (unsigned j = 0; j < A->cols; j++) {
			cvmSet(A, i, j, 1000. * rand() / (double)RAND_MAX);
		}
	}

	int nsize = 1;
	CvMat *X = cvCreateMat(msize, nsize, CV_64F);
	for (unsigned i = 0; i < X->rows; i++) {
		for (unsigned j = 0; j < X->cols; j++) {
			cvmSet(X, i, j, 1000. * rand() / (double)RAND_MAX);
		}
	}

	double b_m[nsize * msize];
	CvMat B = cvMat(msize, nsize, CV_64F, b_m);
	cvSolve(A, X, &B, 0);

	double check_m[msize * nsize];
	CvMat check = cvMat(msize, nsize, CV_64F, check_m);

	cvGEMM(A, &B, 1, &check, 0, &check, 0);

	printf("A: \n");
	print_mat(A);
	printf("B: \n");
	print_mat(&B);

	printf("X: \n");
	print_mat(X);
	printf("A*B: \n");
	print_mat(&check);

	cvReleaseMat(&A);
	cvReleaseMat(&X);
}

void test_invert() {
	int msize = 10;
	CvMat *M = cvCreateMat(msize, msize, CV_64F);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			cvmSet(M, i, j, 1000. * rand() / (double)RAND_MAX);
		}
	}

	double inv_a[msize * msize];
	CvMat inv = cvMat(msize, msize, CV_64F, inv_a);
	cvInvert(M, &inv, CV_SVD);

	double check_m[msize * msize];
	CvMat check = cvMat(msize, msize, CV_64F, check_m);

	cvGEMM(&inv, M, 1, &check, 0, &check, 0);
	print_mat(M);
	print_mat(&inv);
	print_mat(&check);

	cvReleaseMat(&M);
}

void test_transpose_mult() {
	int msize = 10;
	CvMat *M = cvCreateMat(msize, msize, CV_64F);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			cvmSet(M, i, j, rand() / (double)RAND_MAX);
		}
	}

	double n[msize * msize];
	CvMat N = cvMat(msize, msize, CV_64F, n);

	cvMulTransposed(M, &N, 1, 0, 1);

	print_mat(&N);
	cvReleaseMat(&M);

	{
		double m[] = {1, 2, 0, 3};
		CvMat M = cvMat(2, 2, CV_64F, m);
		double m1[] = {1, 2, 0, 3};
		CvMat M1 = cvMat(2, 2, CV_64F, m1);

		cvMulTransposed(&M, &M1, 1, 0, 1);

		print_mat(&M);
		print_mat(&M1);
	}
}

int main() {
	test_invert();
	test_solve();
	test_svd();
	test_transpose_mult();
	return 0;
}
