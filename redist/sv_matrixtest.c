#include "linmath.h"
#include "os_generic.h"
#include "sv_matrix.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void print_mat(const SvMat *M) {
	for (int i = 0; i < M->rows; i++) {
		for (int j = 0; j < M->cols; j++) {
			printf("%f,\t", svMatrixGet(M, i, j));
		}
		printf("\n");
	}
	printf("\n");
}

bool assertFLTEquals(FLT a, FLT b) { return fabs(a - b) < 0.0001; }

int assertFLTAEquals(FLT *a, FLT *b, int length) {
	for (int i = 0; i < length; i++) {
		if (assertFLTEquals(a[i], b[i]) != true) {
			return i;
		}
	}
	return -1;
}

int checkMatFLTAEquals(const struct SvMat *a, const FLT *b) {
	for (int i = 0; i < a->rows; i++) {
		for (int j = 0; j < a->cols; j++) {
			if (assertFLTEquals(svMatrixGet(a, i, j), b[i * a->cols + j]) == false)
				return i * a->cols + j;
		}
	}
	return -1;
}

#define PRINT_MAT(name)                                                                                                \
	printf(#name "\n");                                                                                                \
	print_mat(&name);

void test_gemm() {
	FLT _2x3[2 * 3] = {1, 2, 3, 4, 5, 6};
	SvMat m2x3 = svMat_from_row_major(2, 3, _2x3);

	FLT _3x2[2 * 3] = {1, 3, 5, 2, 4, 6};
	SvMat m3x2 = svMat_from_col_major(3, 2, _3x2);

	FLT _2x2[2 * 2] = {0};
	SvMat m2x2 = svMat_from_row_major(2, 2, _2x2);

	FLT _3x3[3 * 3] = {0};
	SvMat m3x3 = svMat_from_row_major(3, 3, _3x3);

	svGEMM(&m2x3, &m3x2, 1, 0, 0, &m2x2, 0);
	svGEMM(&m3x2, &m2x3, 1, 0, 0, &m3x3, 0);

	PRINT_MAT(m2x3);
	PRINT_MAT(m3x2);
	PRINT_MAT(m3x3);
	PRINT_MAT(m2x2);

	{
		FLT m2x2_gt[] = {
			22.000000,
			28.000000,
			49.000000,
			64.000000,
		};
		assert(checkMatFLTAEquals(&m2x2, m2x2_gt) < 0);
	}

	FLT m2x3_gt[] = {
		1.000000, 2.000000, 3.000000, 4.000000, 5.000000, 6.000000,
	};
	assert(checkMatFLTAEquals(&m2x3, m2x3_gt) < 0);

	svGEMM(&m2x3, &m2x3, 1, 0, 0, &m3x3, SV_GEMM_FLAG_A_T);
	svGEMM(&m2x3, &m2x3, 1, 0, 0, &m2x2, SV_GEMM_FLAG_B_T);
	PRINT_MAT(m3x3);
	PRINT_MAT(m2x2);

	{
		FLT m3x3_gt[] = {
			17.000000, 22.000000, 27.000000, 22.000000, 29.000000, 36.000000, 27.000000, 36.000000, 45.000000,
		};
		assert(checkMatFLTAEquals(&m3x3, m3x3_gt) < 0);
	}

	FLT m2x2_gt[] = {
		14.000000,
		32.000000,
		32.000000,
		77.000000,
	};
	assert(checkMatFLTAEquals(&m2x2, m2x2_gt) < 0);

	svGEMM(&m2x3, &m3x2, 1, 0, 0, &m3x3, SV_GEMM_FLAG_A_T | SV_GEMM_FLAG_B_T);
	//  cvGEMM(&m3x2, &m2x3, 1, 0, 0, &m2x2, SV_GEMM_A_T | SV_GEMM_B_T);

	FLT m3x3_gt[] = {
		9.000000, 19.000000, 29.000000, 12.000000, 26.000000, 40.000000, 15.000000, 33.000000, 51.000000,
	};
	assert(checkMatFLTAEquals(&m3x3, m3x3_gt) < 0);
	PRINT_MAT(m3x3);
}

static void test_solve() {
	{
		FLT _A[3] = {1, 2, 3};
		FLT _B[3] = {4, 8, 12};
		FLT _x[1] = {0};

		SvMat A = svMat_from_row_major(3, 1, _A);
		SvMat B = svMat_from_row_major(3, 1, _B);
		SvMat x = svMat_from_row_major(1, 1, _x);

		svSolve(&A, &B, &x, SV_INVERT_METHOD_SVD);

		assert(fabs(_x[0] - 4) < .001);
	}

	{
		FLT _A[3] = {1, 2, 3};
		FLT _B[9] = {4, 5, 6, 7, 8, 9, 10, 11, 12};
		FLT _x[3] = {0};

		SvMat A = svMat_from_row_major(3, 1, _A);
		SvMat B = svMat_from_row_major(3, 3, _B);
		SvMat x = svMat_from_row_major(1, 3, _x);

		svSolve(&A, &B, &x, SV_INVERT_METHOD_SVD);
	}
}

static void test_invert() {
	printf("Invert:\n");

	FLT _3x3[3 * 3] = {1, 2, 3, 4, 5, 6, 7, 8, 12};
	SvMat m3x3 = svMat_from_row_major(3, 3, _3x3);

	FLT _d3x3[3 * 3] = {0};
	SvMat d3x3 = svMat_from_row_major(3, 3, _d3x3);

	FLT _i3x3[3 * 3] = {0};
	SvMat i3x3 = svMat_from_row_major(3, 3, _i3x3);

	svInvert(&m3x3, &d3x3, SV_INVERT_METHOD_LU);

	svGEMM(&m3x3, &d3x3, 1, 0, 0, &i3x3, 0);
	print_mat(&d3x3);
	print_mat(&i3x3);

	FLT t = sv_trace(&i3x3);
	FLT s = sv_sum(&i3x3);

	assertFLTEquals(t, 3.);
	assertFLTEquals(s - t, 0);
}

static void test_svd() {
	printf("SVD:\n");

	FLT _3x3[3 * 3] = {1, 2, 3, 4, 5, 6, 7, 8, 12};
	SvMat m3x3 = svMat_from_row_major(3, 3, _3x3);

	FLT _w[3] = {0};
	SvMat w = svMat_from_row_major(1, 3, _w);

	FLT _u[9] = {0};
	SvMat u = svMat_from_row_major(3, 3, _u);

	FLT _v[9] = {0};
	SvMat v = svMat_from_row_major(3, 3, _v);

	svSVD(&m3x3, &w, &u, &v, 0);

	PRINT_MAT(w);
	PRINT_MAT(u);
	PRINT_MAT(v);

	FLT wgt[] = {18.626945, 0.840495, 0.574865};
	assertFLTAEquals(sv_as_vector(&w), wgt, 3);
}

static void test_multrans() {
	FLT _A[3] = {1, 2, 3};
	SvMat A = svMat(3, 1, _A);

	FLT _B[9];
	SvMat B = svMat(3, 3, _B);

	FLT _C[1];
	SvMat C = svMat(1, 1, _C);

	svMulTransposed(&A, &B, 0, 0, 1);
	PRINT_MAT(B);
	FLT ans[] = {1, 2, 3, 2, 4, 6, 3, 6, 9};
	for (int i = 0; i < 9; i++) {
		assert(ans[i] == _B[i]);
	}

	svMulTransposed(&A, &C, 1, 0, 10);
	PRINT_MAT(C);
	assert(_C[0] == 140);
}
/*
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
	FLT _2x3[2 * 3] = {1, 2, 3, 0, 0, 6};
	SvMat m2x3 = svMat(2, 3, _2x3);
	SvMat m3x2 = svMat(3, 2, _2x3);
	ALLOC_SPARSE_MATRIX(sm2x3, 2, 3);
	create_sparse_matrix(&sm2x3, &m2x3);

	PRINT_MAT(m2x3);

	SV_CREATE_STACK_MAT(D, 3, 3);
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j <= i; j++) {
			_D[j + 3 * i] = _D[i + 3 * j] = linmath_normrand(0, 100);
		}
	}
	PRINT_MAT(D);

	{
		SV_CREATE_STACK_MAT(Rd, 2, 3);
		SV_CREATE_STACK_MAT(Rs, 2, 3);
		sparse_multiply_sparse_by_dense_sym(&Rs, &sm2x3, &D);
		PRINT_MAT(Rs);

		svGEMM(&m2x3, &D, 1, 0, 0, &Rd, 0);
		PRINT_MAT(Rd);

		for (int i = 0; i < Rd.cols * Rd.rows; i++) {
			assert(fabs(_Rd[i] - _Rs[i]) < 1e-10);
		}
	}

	{
		SV_CREATE_STACK_MAT(Rd, 2, 2);
		SV_CREATE_STACK_MAT(Rs, 2, 2);

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
	SV_CREATE_STACK_MAT(A, N, N);
	SV_CREATE_STACK_MAT(B, N, N);
	SV_CREATE_STACK_MAT(C, N, N);
	SV_CREATE_STACK_MAT(Cs, N, N);
	SV_CREATE_STACK_MAT(Ct, N, N);
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

	uint32_t cnts = 100000;
	double start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cnts; i++) {
		svGEMM(&A, &B, 1, 0, 0, &C, 0);
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
*/
int main()
{
	test_invert();
	test_gemm();
	test_solve();
	test_svd();
	test_multrans();

	/*
	test_sparse_matrix();
	for (int i = 1; i < 20; i++)
		test_speedN(i);
*/
	return 0;
}

