#ifndef __DCLAPACK_H__
#define __DCLAPACK_H__

#ifndef FLOAT
#define FLOAT float
#endif

#include<stdio.h>

#define _ABS(a)  ( (a)<=0 ? 0-(a) : (a) )

// Tricky: If you want to use this with pointers, instead of 2D arrays, you will
// need to #define DYNAMIC_INDEX, as well as, for all arrays, suffix their name
// with 'c'

#ifdef DYNAMIC_INDEX
#define _(AM, O, P) AM[O * AM##c + P]
#else
#define _(AM, O, P) AM[O][P]
#endif

/*
 * Prints a matrix A (m by n)  (with witdth Ac)
 */
#define PRINT(A, m, n)                                                                                                 \
	{                                                                                                                  \
		printf(#A " %dx%d\n", m, n);                                                                                   \
		for (int _i = 0; _i < (m); _i++) {                                                                             \
			for (int _j = 0; _j < (n); _j++) {                                                                         \
				printf("%4.3f ", _(A, _i, _j));                                                                        \
			}                                                                                                          \
			printf("\n");                                                                                              \
		}                                                                                                              \
		printf("\n");                                                                                                  \
	}

/*
 * Returns the identity matrix (with size n x n, but width Ic)
 */
#define IDENTITY(I, m, n)                                                                                              \
	{                                                                                                                  \
		for (int _i = 0; _i < (m); _i++) {                                                                             \
			for (int _j = 0; _j < _i; _j++) {                                                                          \
				_(I, _i, _j) = 0.0f;                                                                                   \
			}                                                                                                          \
			_(I, _i, _i) = 1.0f;                                                                                       \
			for (int _j = _i + 1; _j < (n); _j++) {                                                                    \
				_(I, _i, _j) = 0.0f;                                                                                   \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Returns the identity matrix (with size n x n, but width Ic)
 */
#define ZERO(Z, m, n)                                                                                                  \
	{                                                                                                                  \
		for (int _i = 0; _i < (m); _i++)                                                                               \
			for (int _j = 0; _j < (n); _j++)                                                                           \
				_(Z, _i, _j) = 0.0f;                                                                                   \
	}

/*
 * R = Transpose(A)
 *  A is (m by n)
 *  R is (n by m)
 */
#define TRANSP(R, A, m, n)                                                                                             \
	{                                                                                                                  \
		for (int _i = 0; _i < (m); _i++) {                                                                             \
			for (int _j = 0; _j < (n); _j++) {                                                                         \
				_(R, _j, _i) = _(A, _i, _j);                                                                           \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Calculate L,U of a matrix A with pivot table
 */
#define LU(L, U, A, Piv, n)                                                                                            \
	{                                                                                                                  \
		int _i, _j, _k, _tempi;                                                                                        \
		float _tempf;                                                                                                  \
		for (_i = 0; _i < (n); _i++) {                                                                                 \
			Piv[_i] = _i;                                                                                              \
		}                                                                                                              \
		for (_i = 0; _i < (n); _i++) {                                                                                 \
			for (_j = 0; _j < (n); _j++) {                                                                             \
				_(U, _i, _j) = _(A, _i, _j);                                                                           \
			}                                                                                                          \
		}                                                                                                              \
		IDENTITY(L, n, n);                                                                                             \
                                                                                                                       \
		for (_i = 0; _i < (n)-1; _i++) {                                                                               \
                                                                                                                       \
			int max = _i;                                                                                              \
			for (_j = _i + 1; _j < (n); _j++) {                                                                        \
				if (_ABS(_(U, _j, _i)) > _ABS(_(U, max, _i))) {                                                        \
					max = _j;                                                                                          \
				}                                                                                                      \
			}                                                                                                          \
			_tempi = Piv[_i];                                                                                          \
			Piv[_i] = Piv[max];                                                                                        \
			Piv[max] = _tempi;                                                                                         \
			for (_k = _i; _k < (n); _k++) {                                                                            \
				_tempf = _(U, _i, _k);                                                                                 \
				_(U, _i, _k) = _(U, max, _k);                                                                          \
				_(U, max, _k) = _tempf;                                                                                \
			}                                                                                                          \
			for (_k = 0; _k < _i; _k++) {                                                                              \
				_tempf = _(L, _i, _k);                                                                                 \
				_(L, _i, _k) = _(L, max, _k);                                                                          \
				_(L, max, _k) = _tempf;                                                                                \
			}                                                                                                          \
                                                                                                                       \
			FLOAT invDiag = 1.0 / _(U, _i, _i);                                                                        \
			for (_j = _i + 1; _j < (n); _j++) {                                                                        \
				FLOAT scale = _(U, _j, _i) * invDiag;                                                                  \
				_(U, _j, _i) = 0.0;                                                                                    \
				for (_k = _i + 1; _k < (n); _k++) {                                                                    \
					_(U, _j, _k) -= _(U, _i, _k) * scale;                                                              \
				}                                                                                                      \
				_(L, _j, _i) = scale;                                                                                  \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Pivots a matrix to a different matrix
 *  R = Pivot(A) given table 'Piv'
 *  A and R are (m by n)
 */
#define PIVOT(R, A, Piv, m, n)                                                                                         \
	{                                                                                                                  \
		for (int _i = 0; _i < (m); _i++) {                                                                             \
			for (int _j = 0; _j < (n); _j++) {                                                                         \
				_(R, _i, _j) = _(A, Piv[_i], _j);                                                                      \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Solve LX=B for matrix X and B
 *  L is m by m (lower triangular)
 *  B is m by n
 */
#define L_SUB(X, L, B, m, n)                                                                                           \
	{                                                                                                                  \
		for (int _i = 0; _i < (n); _i++) {                                                                             \
			for (int _j = 0; _j < (m); _j++) {                                                                         \
				float sum = 0.0;                                                                                       \
				for (int _k = 0; _k < _j; _k++) {                                                                      \
					sum += _(L, _j, _k) * _(X, _k, _i);                                                                \
				}                                                                                                      \
				_(X, _j, _i) = (_(B, _j, _i) - sum) / _(L, _j, _j);                                                    \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Solve UX=B for matrix X and B
 *  U is m by m (upper triangular)
 *  B is m by n
 */
#define U_SUB(X, U, B, m, n)                                                                                           \
	{                                                                                                                  \
		for (int _i = 0; _i < (n); _i++) {                                                                             \
			for (int _j = m - 1; _j >= 0; _j--) {                                                                      \
				float sum = 0.0;                                                                                       \
				for (int _k = n - 1; _k > _j; _k--) {                                                                  \
					sum += _(U, _j, _k) * _(X, _k, _i);                                                                \
				}                                                                                                      \
				_(X, _j, _i) = (_(B, _j, _i) - sum) / _(U, _j, _j);                                                    \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Inverts a matrix X (n by n) using the method of LU decomposition
 */

#ifdef DYNAMIC_INDEX
#define INV_SETUP(ORDER)                                                                                               \
	FLOAT Ipiv[ORDER * ORDER];                                                                                         \
	const int Ipivc = ORDER;                                                                                           \
	FLOAT L[ORDER * ORDER];                                                                                            \
	const int Lc = ORDER;                                                                                              \
	FLOAT U[ORDER * ORDER];                                                                                            \
	const int Uc = ORDER;                                                                                              \
	FLOAT I[ORDER * ORDER];                                                                                            \
	const int Ic = ORDER;                                                                                              \
	FLOAT C[ORDER * ORDER];                                                                                            \
	const int Cc = ORDER;
#else
#define INV_SETUP(ORDER)                                                                                               \
	FLOAT Ipiv[ORDER][ORDER];                                                                                          \
	FLOAT L[ORDER][ORDER];                                                                                             \
	FLOAT U[ORDER][ORDER];                                                                                             \
	FLOAT I[ORDER][ORDER];                                                                                             \
	FLOAT C[ORDER][ORDER];
#endif

#define INV(Ainv, A, n, ORDER)                                                                                         \
	{                                                                                                                  \
		INV_SETUP(ORDER)                                                                                               \
		int Piv[ORDER];                                                                                                \
		IDENTITY(I, n, n);                                                                                             \
		LU(L, U, A, Piv, n);                                                                                           \
		PIVOT(Ipiv, I, Piv, n, n);                                                                                     \
		L_SUB(C, L, Ipiv, n, n);                                                                                       \
		U_SUB(Ainv, U, C, n, n);                                                                                       \
	}

/*
PRINT(A,n,n); \
PRINT(L,n,n); \
PRINT(U,n,n); \
MUL(L,U,LU,n,n,n);\
PRINT(LU,n,n);\
PRINT(C,n,n); \
PRINT(Ainv,n,n); \
*/

/*
 * Matrix Multiply R = A * B
 *  R (n by p)
 *  A (m by n)
 *  B (m by p)
 */
#define MUL(R, A, B, m, n, p)                                                                                          \
	{                                                                                                                  \
		for (int _i = 0; _i < (m); _i++) {                                                                             \
			for (int _j = 0; _j < p; _j++) {                                                                           \
				_(R, _i, _j) = 0.0f;                                                                                   \
				for (int _k = 0; _k < (n); _k++) {                                                                     \
					_(R, _i, _j) += _(A, _i, _k) * _(B, _k, _j);                                                       \
				}                                                                                                      \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Matrix Multiply R = A * B + C
 *  R (m by p)
 *  A (m by n)
 *  B (n by p)
 *  C (m by p)
 */
#define MULADD(R, A, B, C, m, n, p)                                                                                    \
	{                                                                                                                  \
		for (int _i = 0; _i < (m); _i++) {                                                                             \
			for (int _j = 0; _j < p; _j++) {                                                                           \
				_(R, _i, _j) = _(C, _i, _j);                                                                           \
				for (int _k = 0; _k < (n); _k++) {                                                                     \
					_(R, _i, _j) += _(A, _i, _k) * _(B, _k, _j);                                                       \
				}                                                                                                      \
			}                                                                                                          \
		}                                                                                                              \
	}

/*
 * Matrix Multiply R = alpha * A * B + beta * C
 *  R (m by p)
 *  A (m by n)
 *  B (n by p)
 *  C (m by p)
 */
#define GMULADD(R, A, B, C, alpha, beta, m, n, p)                                                                      \
	{                                                                                                                  \
		FLOAT sum;                                                                                                     \
		for (int _i = 0; _i < m; _i++) {                                                                               \
			for (int _j = 0; _j < p; _j++) {                                                                           \
				sum = 0.0f;                                                                                            \
				for (int _k = 0; _k < n; _k++) {                                                                       \
					sum += _(A, _i, _k) * _(B, _k, _j);                                                                \
				}                                                                                                      \
				_(R, _i, _j) = alpha * sum + beta * _(C, _i, _j);                                                      \
			}                                                                                                          \
		}                                                                                                              \
	}

#endif
