// gcc -msse2 -O3 -ftree-vectorize test_dcl.c dclhelpers.c os_generic.c -DFLT=double -lpthread -lcblas && valgrind
// ./a.out

#include "dclhelpers.h"
#include "os_generic.h"
#include <assert.h>
#include <cblas.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "cnmatrix/matrix.h"

void fill_random(FLT *A, int ld, int m, int n) {
	assert(ld == n);
	for (int y = 0; y < m; y++)
		for (int x = 0; x < n; x++)
			A[y * ld + x] = (rand()) / (double)RAND_MAX;
}

void test_dcldgemm_speed(const char *name, char transA, char transB, int m, int n, int k, DCL_FLOAT alpha,
						 const DCL_FLOAT *A, int Ac, const DCL_FLOAT *B, int Bc, DCL_FLOAT beta) {
	printf("%s speed test:\n", name);
	double times[2];
	FLT emo[2][m][n];

	for (int z = 0; z < 2; z++) {
		double start = OGGetAbsoluteTime();
		for (int i = 0; i < 100000; i++) {
			dclZero(DMS(emo[z]), m, n);

			if (z == 0) {
				dcldgemm(transA, transB, m, n, k, alpha, A, Ac, B, Bc, beta, DMS(emo[z]));
			} else {

				cblas_dgemm(CblasRowMajor, transA == 1 ? CblasTrans : CblasNoTrans,
							transB == 1 ? CblasTrans : CblasNoTrans, m, n, k, alpha, A, Ac, B, Bc, beta, emo[z][0], n);
			}
		}

		times[z] = OGGetAbsoluteTime() - start;
	}

	dclPrint(DMS(emo[0]), m, n);
	dclPrint(DMS(emo[1]), m, n);
	printf("dcl  Elapsed: %f\n", times[0]);
	printf("cblas Elapsed: %f\n", times[1]);
	printf("%fx difference\n", times[0] / times[1]);

	for (int i = 0; i < m; i++) {
		for (int j = 0; j < k; j++) {
			assert(fabs(emo[0][i][j] - emo[1][i][j]) < .00001);
		}
	}
}

void compareToCblas() {
	srand(0);
	int m = 12;
	int n = 20;
	int k = 20;

	FLT em1[m][n];
	FLT em2[n][k];

	fill_random(DMS(em1), m, n);
	fill_random(DMS(em2), n, k);

	dclPrint(DMS(em1), m, n);
	dclPrint(DMS(em2), n, k);

	test_dcldgemm_speed("Simple", 0, 0, m, n, k, 1.0, DMS(em1), DMS(em2), .1);
}

void compareToCblasTrans() {
	srand(0);
	int m = 12;
	int n = 20;
	int k = n;

	FLT em1[m][n];

	fill_random(DMS(em1), m, n);

	dclPrint(DMS(em1), m, n);

	CnMat Em1 = cvMat(m, n, SV_FLT, em1);
	FLT em1tem1[n][n];
	CnMat Em1tEm1 = cvMat(n, n, SV_FLT, em1tem1);
	cvMulTransposed(&Em1, &Em1tEm1, 1, 0, 1);
	print_mat(&Em1tEm1);

	test_dcldgemm_speed("Trans", 0, 0,
						n, // # of rows in OP(A) == em1' -- 20
						n, // # of cols in OP(B) == em1 -- 20
						m, // # of cols in OP(A) == em1' -- 12
						1.0,
						DMS(em1), // Note that LD stays the same
						DMS(em1), 0);
}

int main() {
	FLT A[2][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}};
	FLT B[4][2];
	dclPrint(A[0], 4, 2, 4);
	dclTransp(B[0], 2, A[0], 4, 2, 4);
	dclPrint(B[0], 2, 4, 2);

	int i, j;
	for (i = 0; i < 8; i++) {
		printf("%f\n", ((float *)(B[0]))[i]);
	}

	FLT M[3][3] = {{.32, 1, 0}, {0, 1, 2}, {1, 0, 1}};
	FLT Mo[3][3];
	dclInv(Mo[0], 3, M[0], 3, 3);
	dclPrint(Mo[0], 3, 3, 3);

	FLT MM[3][3];
	dclMul(MM[0], 3, M[0], 3, Mo[0], 3, 3, 3, 3);

	printf("The following should be an identity matrix\n");
	dclPrint(MM[0], 3, 3, 3);

	{
		FLT A[3][4];
		dclIdentity(DMS(A), 3, 4);
		dclPrint(DMS(A), 3, 4);

		FLT x[4][2] = {
			{7, -7}, {8, -8}, {9, -9}, {10, -10},
		};
		FLT R[4][2];
		dclZero(DMS(R), 4, 2);

		// dclMul(R, 1, A[0], 4, x, 1, 4, 1, 3);
		dcldgemm(0, 0, 3, 4, 2, 1, A[0], 4, x[0], 2, 0, R[0], 2);

		dclPrint(DMS(x), 4, 2);
		dclPrint(DMS(R), 3, 2);

		for (int j = 0; j < 2; j++) {
			for (int i = 0; i < 3; i++) {
				printf("[%d][%d]\n", i, j);
				assert(R[i][j] == x[i][j]);
			}

			assert(fabs(R[3][j]) < .0000001);
		}
	}

	compareToCblas();
	compareToCblasTrans();
}
