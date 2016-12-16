#ifndef __DCLAPACK_H__
#define __DCLAPACK_H__

#ifndef ORDER
#define ORDER 50
#endif

#ifndef FLOAT
#define FLOAT float
#endif

#include<stdio.h>

/*
 * Prints a matrix A (n by m)
 */
#define PRINT(A,n,m) { \
    int i,j; \
    printf(#A "\n"); \
    for (i=0; i<n; i++) { \
        for (j=0; j<m; j++) { \
            printf("%4.3f\t", A[i][j]); \
        } \
        printf("\n"); \
    } \
    printf("\n"); \
} 

/*
 * Returns the identity matrix
 */
#define IDENTITY(I,n) { \
    int i,j; \
    for (i=0; i<n; i++) { \
        for (j=0; j<i; j++) { I[i][j]=0.0f; } \
        I[i][i] = 1.0f; \
        for (j=i+1; j<n; j++) { I[i][j]=0.0f; } \
    } \
}

/*
 * Calculate L,U of a matrix A with pivot table
 */
#define LU(A,L,U,Piv,n) { \
    int i,j,k,_tempi; float _tempf; \
    for (i=0; i<n; i++) { Piv[i]=i; } \
    for (i=0; i<n; i++) { \
        for (j=0; j<n; j++) { \
            U[i][j] = A[i][j]; \
        } \
    } \
    IDENTITY(L,n); \
    \
    for (i=0; i<n-1; i++) { \
        \
        int max=i; \
	for (j=i+1; j<n; j++) { \
		if (U[j][i] > U[max][i]) { max = j; } \
	} \
	_tempi=Piv[i]; Piv[i]=Piv[max]; Piv[max]=_tempi; \
	for (k=i; k<n; k++) { \
		_tempf=U[i][k]; U[i][k]=U[max][k]; U[max][k]=_tempf; \
	} \
	for (k=0; k<i; k++) { \
		_tempf=L[i][k]; L[i][k]=L[max][k]; L[max][k]=_tempf; \
	} \
        \
        FLOAT invDiag = 1.0 / U[i][i]; \
        for (j=i+1; j<n; j++) { \
            FLOAT scale = U[j][i] * invDiag; \
            U[j][i] = 0.0; \
            for (k=i+1; k<n; k++) { U[j][k] -= U[i][k]*scale; } \
            L[j][i] = scale; \
        } \
    } \
}

/*
 * Pivots a matrix to a different matrix
 *  B = Pivot(A) given table 'Piv'
 *  A and B are (n by m)
 */
#define PIVOT(A,B,Piv,n,m) { \
    int i,j; \
    for (j=0; j<n; j++) { \
        for (i=0; i<m; i++) { \
            B[j][i] = A[Piv[j]][i]; \
        } \
    } \
}

/*
 * Solve LX=B for matrix X and B
 *  L is n by n (lower triangular)
 *  B is n by m
 */
#define L_SUB(L,X,B,n,m) { \
    int i,j,k; \
    for (i=0; i<m; i++) { \
        for (j=0; j<n; j++) { \
            float sum=0.0; \
            for (k=0; k<j; k++) { sum += L[j][k]*X[k][i]; } \
            X[j][i] = (B[j][i] - sum) / L[j][j]; \
        } \
    } \
}

/*
 * Solve UX=B for matrix X and B
 *  U is n by n (upper triangular)
 *  B is n by m
 */
#define U_SUB(U,X,B,n,m) { \
    int i,j,k; \
    for (i=0; i<m; i++) { \
        for (j=n-1; j>=0; j--) { \
            float sum=0.0; \
            for (k=n-1; k>j; k--) { sum += U[j][k]*X[k][i]; } \
            X[j][i] = (B[j][i] - sum) / U[j][j]; \
        } \
    } \
}

/*
 * Inverts a matrix X (n by n) using the method of LU decomposition
 */
#define INV(A,Ainv,n) { \
    FLOAT Ipiv[ORDER][ORDER]; \
    FLOAT L[ORDER][ORDER]; \
    FLOAT U[ORDER][ORDER]; \
    FLOAT I[ORDER][ORDER]; \
    FLOAT C[ORDER][ORDER]; \
    int Piv[ORDER]; \
    IDENTITY(I,n); \
    LU(A,L,U,Piv,n); \
    PIVOT(I,Ipiv,Piv,n,n); \
    L_SUB(L,C,Ipiv,n,n); \
    U_SUB(U,Ainv,C,n,n); \
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
 * Matrix Multiply C = A * B
 *  A (n by m)
 *  B (m by p)
 *  C (n by p)
 */
#define MUL(A,B,C,n,m,p) { \
    int i,j,k; \
    for (i=0; i<n; i++) { \
        for (j=0; j<p; j++) { \
            C[i][j] = 0.0f; \
            for (k=0; k<m; k++) { \
                C[i][j] += A[i][k] * B[k][j]; \
            } \
        } \
    } \
}

/*
 * Matrix Multiply D = A * B + C
 *  A (n by m)
 *  B (m by p)
 *  C (n by p)
 *  D (n by p)
 */
#define MULADD(A,B,C,D,n,m,p) { \
    int i,j,k; \
    for (i=0; i<n; i++) { \
        for (j=0; j<p; j++) { \
            D[i][j] = C[i][j]; \
            for (k=0; k<m; k++) { \
                D[i][j] += A[i][k] * B[k][j]; \
            } \
        } \
    } \
}

#endif
