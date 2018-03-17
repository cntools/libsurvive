#ifndef _DCL_HELPERS_H
#define _DCL_HELPERS_H

#define DCL_FLOAT FLT

// Use this macro to safely
#define DMS(m) ((m)[0]), (sizeof((m)[0]) / sizeof((m)[0][0]))

/* Prints matrix A of size[n][m] */
void dclPrint(const DCL_FLOAT *A, int Ac, int n, int m);

/* Returns the identity matrix */
void dclIdentity(DCL_FLOAT *I, int Ic, int m, int n);

/* Returns the zero matrix */
void dclZero(DCL_FLOAT *I, int Ic, int m, int n);

/* R = Transpose(A)
	 A is (n by m)
	 R is (m by n) */
void dclTransp(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, int n, int m);

/* Calculate L,U of a matrix A with pivot table; the pivot table is output. */
void dclLU(DCL_FLOAT *L, int Lc, DCL_FLOAT *U, int Uc, const DCL_FLOAT *A, int Ac, int *Piv, int n);

/* Pivots a matrix to a different matrix
	R = Pivot(A) given table 'Piv'
	A and R are (n by m) */
void dclPivot(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, int *Piv, int n, int m);

/* Solve LX=B for matrix X and B
	L is n by n (lower triangular)
	B is n by m */
void dclLSub(DCL_FLOAT *X, int Xc, const DCL_FLOAT *L, int Lc, const DCL_FLOAT *B, int Bc, int n, int m);

/* Solve UX=B for matrix X and B
	U is n by n (upper triangular)
	B is n by m */
void dclUSub(DCL_FLOAT *X, int Xc, const DCL_FLOAT *U, int Uc, const DCL_FLOAT *B, int Bc, int n, int m);

/* Inverts a matrix X (n by n) using the method of LU decomposition */
void dclInv(DCL_FLOAT *Ainv, int Ainvc, const DCL_FLOAT *A, int Ac, int n);

/* Matrix Multiply R = A * B
	A (n by m)
	B (m by p)
	R (n by p) */
void dclMul(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, const DCL_FLOAT *B, int Bc, int n, int m, int p);

/* Matrix Multiply R = A * B + C
	A (n by m)
	B (m by p)
	C (n by p)
	R (n by p) */
void dclMulAdd(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, const DCL_FLOAT *B, int Bc, const DCL_FLOAT *C, int Cc,
			   int n, int m, int p);

/* Matrix Multiply R = alpha * A * B + beta * C
	A (n by m)
	B (m by p)
	C (n by p)
	R (n by p) */
void dclGMulAdd(DCL_FLOAT *R, int Rc, const DCL_FLOAT *A, int Ac, const DCL_FLOAT *B, int Bc, const DCL_FLOAT *C,
				int Cc, DCL_FLOAT alpha, DCL_FLOAT beta, int n, int m, int p);

/********************************
 * Auxiliary functionality in C *
 ********************************/

// Matches dgemm from lapack.
void dcldgemm(char transA, char transB, int m, int n, int k, DCL_FLOAT alpha, const DCL_FLOAT *A, int Ac,
			  const DCL_FLOAT *B, int Bc, DCL_FLOAT beta, DCL_FLOAT *C, int Cc);

#endif
