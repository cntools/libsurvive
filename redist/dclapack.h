#ifndef __DCLAPACK_H__
#define __DCLAPACK_H__

#ifndef FLOAT
#define FLOAT float
#endif

#include<stdio.h>

#define _ABS(a)  ( (a)<=0 ? 0-(a) : (a) )

#ifdef DYNAMIC_INDEX
	#define _(A,O,P) A[O*n+P]
#else
	#define _(A,O,P) A[O][P]
#endif


/*
 * Prints a matrix A (n by m)
 */
#define PRINT(A,n,m) { \
    int i,j; \
    printf(#A "\n"); \
    for (i=0; i<n; i++) { \
        for (j=0; j<m; j++) { \
            printf("%4.3f\t", _(A,i,j)); \
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
        for (j=0; j<i; j++) { _(I,i,j)=0.0f; } \
        _(I,i,i) = 1.0f; \
        for (j=i+1; j<n; j++) { _(I,i,j)=0.0f; } \
    } \
}

/*
 * B = Transpose(A)
 *  A is (n by m)
 *  B is (m by n)
 */
#define TRANSP(A,B,n,m) { \
   int i,j; \
   for (i=0; i<n; i++) { \
      for (j=0; j<m; j++) { \
         _(B,j,i) = _(A,i,j); \
      } \
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
            _(U,i,j) = _(A,i,j); \
        } \
    } \
    IDENTITY(L,n); \
    \
    for (i=0; i<n-1; i++) { \
        \
        int max=i; \
	for (j=i+1; j<n; j++) { \
		if (_ABS( _(U,j,i)) > _ABS(_(U,max,i))) { max = j; } \
	} \
	_tempi=Piv[i]; Piv[i]=Piv[max]; Piv[max]=_tempi; \
	for (k=i; k<n; k++) { \
		_tempf=_(U,i,k); _(U,i,k)=_(U,max,k); _(U,max,k)=_tempf; \
	} \
	for (k=0; k<i; k++) { \
		_tempf=_(L,i,k); _(L,i,k)=_(L,max,k); _(L,max,k)=_tempf; \
	} \
        \
        FLOAT invDiag = 1.0 / _(U,i,i); \
        for (j=i+1; j<n; j++) { \
            FLOAT scale = _(U,j,i) * invDiag; \
            _(U,j,i) = 0.0; \
            for (k=i+1; k<n; k++) { _(U,j,k) -= _(U,i,k)*scale; } \
            _(L,j,i) = scale; \
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
            _(B,j,i) = _(A,Piv[j],i); \
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
            for (k=0; k<j; k++) { sum += _(L,j,k)*_(X,k,i); } \
            _(X,j,i) = (_(B,j,i) - sum) / _(L,j,j); \
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
            for (k=n-1; k>j; k--) { sum += _(U,j,k)*_(X,k,i); } \
            _(X,j,i) = (_(B,j,i) - sum) / _(U,j,j); \
        } \
    } \
}

/*
 * Inverts a matrix X (n by n) using the method of LU decomposition
 */

#ifdef DYNAMIC_INDEX
	#define INV_SETUP(ORDER) \
    FLOAT Ipiv[ORDER*ORDER]; \
    FLOAT L[ORDER*ORDER]; \
    FLOAT U[ORDER*ORDER]; \
    FLOAT I[ORDER*ORDER]; \
    FLOAT C[ORDER*ORDER];
#else
	#define INV_SETUP(ORDER) \
    FLOAT Ipiv[ORDER][ORDER]; \
    FLOAT L[ORDER][ORDER]; \
    FLOAT U[ORDER][ORDER]; \
    FLOAT I[ORDER][ORDER]; \
    FLOAT C[ORDER][ORDER];
#endif

#define INV(A,Ainv,n,ORDER) { \
	INV_SETUP(ORDER) \
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
            _(C,i,j) = 0.0f; \
            for (k=0; k<m; k++) { \
                _(C,i,j) += _(A,i,k) * _(B,k,j); \
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
            _(D,i,j) = _(C,i,j); \
            for (k=0; k<m; k++) { \
                _(D,i,j) += _(A,i,k) * _(B,k,j); \
            } \
        } \
    } \
}

/*
 * Matrix Multiply D = alpha * A * B + beta * C
 *  A (n by m)
 *  B (m by p)
 *  C (n by p)
 *  D (n by p)
 */
#define GMULADD(A,B,C,D,alpha,beta,n,m,p) { \
    int i,j,k; \
    float sum; \
    for (i=0; i<n; i++) { \
        for (j=0; j<p; j++) { \
            sum = 0.0f; \
            for (k=0; k<m; k++) { \
                sum += _(A,i,k) * _(B,k,j); \
            } \
            _(D,i,j) = alpha * sum + beta * _(C,i,j); \
        } \
    } \
}

#endif
