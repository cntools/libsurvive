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

void dclTransp( const DCL_FLOAT * A, DCL_FLOAT * B, int n, int m)
{
	TRANSP(A,B,n,m);
}

void dclLU( const DCL_FLOAT * A, DCL_FLOAT * L, DCL_FLOAT * U, int * Piv, int n )
{
	LU(A,L,U,Piv,n);
}

void dclPivot( const DCL_FLOAT * A, DCL_FLOAT * B, int * Piv, int n, int m )
{
	PIVOT(A,B,Piv,n,m);
}

void dclLSub( const DCL_FLOAT * L, DCL_FLOAT * X, const DCL_FLOAT * B, int n, int m )
{
	L_SUB(L,X,B,n,m);
}

void dclUSub( const DCL_FLOAT * U, DCL_FLOAT * X, const DCL_FLOAT * B, int n, int m )
{
	U_SUB(U,X,B,n,m);
}

void dclInv( const DCL_FLOAT * A, DCL_FLOAT * Ainv, int n )
{
	INV(A,Ainv,n,n);
}

void dclMul( const DCL_FLOAT * A, const DCL_FLOAT * B, DCL_FLOAT * C, int n, int m, int p )
{
	MUL(A,B,C,n,m,p);
}

void dclMulAdd( const DCL_FLOAT * A, const DCL_FLOAT * B, const DCL_FLOAT * C, DCL_FLOAT * D, int n, int m, int p )
{
	MULADD(A,B,C,D,n,m,p);
}

void dclGMulAdd( const DCL_FLOAT * A, const DCL_FLOAT * B, const DCL_FLOAT * C, DCL_FLOAT * D, DCL_FLOAT alpha, DCL_FLOAT beta, int n, int m, int p )
{
	GMULADD(A,B,C,D,alpha,beta,n,m,p);
}


