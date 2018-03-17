#include "dclhelpers.h"
#define FLOAT DCL_FLOAT
#define DYNAMIC_INDEX
#include "dclapack.h"
#include <alloca.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

void dclPrint(const DCL_FLOAT *PMATRIX, int PMATRIXc, int n, int m) { PRINT(PMATRIX, n, m); }

void dclIdentity(DCL_FLOAT *I, int Ic, int m, int n) { IDENTITY(I, m, n); }

/* Returns the zero matrix */
void dclZero(DCL_FLOAT *Z, int Zc, int m, int n) { memset(Z, 0, m * n * sizeof(DCL_FLOAT)); }

void dclTransp(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, int n, int m) { TRANSP(R, A, n, m); }

void dclLU(DCL_FLOAT *L, int Lc, DCL_FLOAT *U, int Uc, const DCL_FLOAT *A, int Ac, int *Piv, int n) {
	LU(L, U, A, Piv, n);
}

void dclPivot(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, int *Piv, int n, int m) { PIVOT(R, A, Piv, n, m); }

void dclLSub(DCL_FLOAT *X, int Xc, const DCL_FLOAT *L, int Lc, const DCL_FLOAT *B, int Bc, int n, int m) {
	L_SUB(X, L, B, n, m);
}

void dclUSub(DCL_FLOAT *X, int Xc, const DCL_FLOAT *U, int Uc, const DCL_FLOAT *B, int Bc, int n, int m) {
	U_SUB(X, U, B, n, m);
}

void dclInv(DCL_FLOAT *Ainv, int Ainvc, const DCL_FLOAT *A, int Ac, int n) { INV(Ainv, A, n, n); }

void dclMul(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, const DCL_FLOAT *B, int Bc, int n, int m, int p) {
	MUL(R, A, B, n, m, p);
}

void dclMulAdd(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, const DCL_FLOAT *B, int Bc, const DCL_FLOAT *C, int Cc,
			   int n, int m, int p) {
	MULADD(R, A, B, C, n, m, p);
}

void dclGMulAdd(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, const DCL_FLOAT *B, int Bc, const DCL_FLOAT *C,
				int Cc, DCL_FLOAT alpha, DCL_FLOAT beta, int n, int m, int p) {
	GMULADD(R, A, B, C, alpha, beta, n, m, p);
}

/* dclGMulAdd( R, ((transA)?TRANS(A):A, (transB)?TRANS(B):B), C, alpha, beta, n, m, p ); */
void dcldgemm(char transA, char transB, int m, int n, int k, DCL_FLOAT alpha, const DCL_FLOAT *A,
			  int Ac, // must be n
			  const DCL_FLOAT *B,
			  int Bc, // must be m
			  DCL_FLOAT beta, DCL_FLOAT *C,
			  int Cc // must be n
			  ) {
	const DCL_FLOAT *ta;
	const DCL_FLOAT *tb;
	int tac = Ac;
	int tbc = Bc;
	if (transA) {
		DCL_FLOAT *la = alloca(sizeof(DCL_FLOAT) * n * m);
		const int lac = m;
		TRANSP(la, A, n, m);
		ta = la;
		tac = lac;
	} else
		ta = A;

	if (transB) {
		DCL_FLOAT *lb = alloca(sizeof(DCL_FLOAT) * n * m);
		const int lbc = m;
		TRANSP(lb, B, n, m);
		tb = lb;
		tbc = lbc;
	} else
		tb = B;

	GMULADD(C, ta, tb, C, alpha, beta, m, n, k);
}
