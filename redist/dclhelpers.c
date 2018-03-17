#include "dclhelpers.h"
#define FLOAT DCL_FLOAT
#define DYNAMIC_INDEX
#include <stdio.h>
#include "dclapack.h"


void dclPrint( const DCL_FLOAT * A, int n, int m )
{
	PRINT( A, n, m );
}

void dclIdentity( DCL_FLOAT * A, int n )
{
	IDENTITY( A, n );
}

void dclTransp( DCL_FLOAT * R, const DCL_FLOAT * A, int n, int m )
{
	TRANSP(R,A,n,m);
}

void dclLU( DCL_FLOAT * L, DCL_FLOAT * U, const DCL_FLOAT * A, int * Piv, int n )
{
	LU(L,U,A,Piv,n);
}

void dclPivot( DCL_FLOAT * R, const DCL_FLOAT * A, int * Piv, int n, int m )
{
	PIVOT(R,A,Piv,n,m);
}

void dclLSub( DCL_FLOAT * X, const DCL_FLOAT * L, const DCL_FLOAT * B, int n, int m )
{
	L_SUB(X,L,B,n,m);
}

void dclUSub( DCL_FLOAT * X, const DCL_FLOAT * U, const DCL_FLOAT * B, int n, int m )
{
	U_SUB(X,U,B,n,m);
}

void dclInv( DCL_FLOAT * Ainv, const DCL_FLOAT * A, int n )
{
	INV(Ainv,A,n,n);
}

void dclMul( DCL_FLOAT * R, const DCL_FLOAT * A, const DCL_FLOAT * B, int n, int m, int p )
{
	MUL(R,A,B,n,m,p);
}

void dclMulAdd( DCL_FLOAT * R, const DCL_FLOAT * A, const DCL_FLOAT * B, const DCL_FLOAT * C, int n, int m, int p )
{
	MULADD(R,A,B,C,n,m,p);
}

void dclGMulAdd( DCL_FLOAT * R, const DCL_FLOAT * A, const DCL_FLOAT * B, const DCL_FLOAT * C, DCL_FLOAT alpha, DCL_FLOAT beta, int n, int m, int p )
{
	GMULADD(R,A,B,C,alpha,beta,n,m,p);
}

/* dclGMulAdd( R, ((transA)?TRANS(A):A, (transB)?TRANS(B):B), C, alpha, beta, n, m, p ); */
void dcldgemm(
	char transA,
	char transB,
	int m,
	int n,
	int k,
	DCL_FLOAT alpha,
	const DCL_FLOAT* A,
	int lda, //must be n
	const DCL_FLOAT* B,
	int ldb, //must be m
	DCL_FLOAT beta,
	const DCL_FLOAT * C,
	int ldc //must be n
	 )
{
	DCL_FLOAT * ta;
	DCL_FLOAT * tb;
	if( transA )
	{
		ta = alloca( sizeof( DCL_FLOAT ) * n * m );
		
	}
}

