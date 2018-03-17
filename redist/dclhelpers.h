#ifndef _DCL_HELPERS_H
#define _DCL_HELPERS_H

#define DCL_FLOAT FLT

//XXX XXX XXX WARNING XXX XXX XXX The argument order may be changing!!!

/* Prints matrix A of size[n][m] */
void dclPrint( const DCL_FLOAT * A, int n, int m );

/* Returns the identity matrix */
void dclIdentity( DCL_FLOAT * A, int n );

/* B = Transpose(A)
     A is (n by m)
     B is (m by n) */
void dclTransp( const DCL_FLOAT * A, DCL_FLOAT * B, int n, int m );

/* Calculate L,U of a matrix A with pivot table; the pivot table is output. */
void dclLU( const DCL_FLOAT * A, DCL_FLOAT * L, DCL_FLOAT * U, int * Piv, int n );

/* Pivots a matrix to a different matrix
    B = Pivot(A) given table 'Piv'
    A and B are (n by m) */
void dclPivot( const DCL_FLOAT * A, DCL_FLOAT * B, int * Piv, int n, int m );

/* Solve LX=B for matrix X and B
    L is n by n (lower triangular)
    B is n by m */
void dclLSub( const DCL_FLOAT * L, DCL_FLOAT * X, const DCL_FLOAT * B, int n, int m );

/* Solve UX=B for matrix X and B
    U is n by n (upper triangular)
    B is n by m */
void dclUSub( const DCL_FLOAT * U, DCL_FLOAT * X, const DCL_FLOAT * B, int n, int m );

/* Inverts a matrix X (n by n) using the method of LU decomposition */
void dclInv( const DCL_FLOAT * A, DCL_FLOAT * Ainv, int n );

/* Matrix Multiply C = A * B
    A (n by m)
    B (m by p)
    C (n by p) */
void dclMul( const DCL_FLOAT * A, const DCL_FLOAT * B, DCL_FLOAT * C, int n, int m, int p );

/* Matrix Multiply D = A * B + C
    A (n by m)
    B (m by p)
    C (n by p)
    D (n by p) */
void dclMulAdd( const DCL_FLOAT * A, const DCL_FLOAT * B, const DCL_FLOAT * C, DCL_FLOAT * D, int n, int m, int p );

/* Matrix Multiply D = alpha * A * B + beta * C
    A (n by m)
    B (m by p)
    C (n by p)
    D (n by p) */
void dclGMulAdd( const DCL_FLOAT * A, const DCL_FLOAT * B, const DCL_FLOAT * C, DCL_FLOAT * D, DCL_FLOAT alpha, DCL_FLOAT beta, int n, int m, int p );

#endif

