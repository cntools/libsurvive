//gcc -msse2 -O3 -ftree-vectorize test_dcl.c dclhelpers.c os_generic.c -DFLT=double -lpthread -lcblas && valgrind ./a.out

#include "dclhelpers.h"
#include "os_generic.h"
#include <assert.h>
#include <cblas.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

void compareToCblas() {
	FLT em1[12][20];
	FLT em2[20][20];
	FLT emo[2][20][20] = {};
	int x, y;

	for (y = 0; y < 12; y++)
		for (x = 0; x < 20; x++)
			em1[y][x] = (rand() % 1000) / 1000.0;

	for (y = 0; y < 20; y++)
		for (x = 0; x < 20; x++)
			em2[y][x] = (rand() % 1000) / 1000.0;

	int m = 12;
	int n = 20;
	int k = 20;

	dclPrint(DMS(em1), 12, 20);
	dclPrint(DMS(em2), 20, 12);

	double times[2];
	for (int z = 0; z < 2; z++) {
		double start = OGGetAbsoluteTime();
		for (int i = 0; i < 100000; i++) {
			dclZero(DMS(emo[z]), 20, 20);

			if (z == 0) {
				dcldgemm(0, 0, m, n, k, 1.0, DMS(em1), DMS(em2), .1, DMS(emo[z]));
			} else {
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, m, n, k, 1.0, DMS(em1), DMS(em2), .1,
							DMS(emo[z]));
			}
		}

		printf("%s Elapsed: %f\n", z ? "CBlas" : "dcl", times[z] = OGGetAbsoluteTime() - start);
	}
	printf("%fx difference\n", times[0] / times[1]);
	dclPrint(emo[0][0], 12, 20, 12);
	dclPrint(emo[1][0], 12, 20, 12);

	for (int i = 0; i < m; i++) {
		for (int j = 0; j < k; j++) {
			assert(fabs(emo[0][i][j] - emo[1][i][j]) < .00001);
		}
	}
}

int main()
{
	FLT A[2][4]	= { { 0, 1, 2, 3 }, { 4, 5, 6, 7} };
	FLT B[4][2];
	dclPrint( A[0], 4, 2, 4 );
	dclTransp( B[0], 2, A[0], 4, 2, 4 );
	dclPrint( B[0], 2, 4, 2 );

	int i, j;
	for( i = 0; i < 8; i++ )
	{
		printf( "%f\n", ((float*)(B[0]))[i] );
	}

	FLT M[3][3] = {
		{ .32, 1, 0 },
		{ 0, 1, 2 },
		{ 1, 0, 1 } };
	FLT Mo[3][3];
	dclInv( Mo[0], 3, M[0], 3, 3 );
	dclPrint( Mo[0], 3, 3, 3 );

	FLT MM[3][3];
	dclMul( MM[0], 3, M[0], 3, Mo[0], 3, 3, 3, 3 );

	printf( "The following should be an identity matrix\n" );
	dclPrint( MM[0], 3, 3, 3 );

	{
		FLT A[3][4];
		dclIdentity( DMS(A), 3, 4);
		dclPrint( DMS(A), 3, 4);

		FLT x[4][2] = {
			{7, -7}, {8, -8}, {9, -9}, {10, -10},
		};
		FLT R[4][2];
		dclZero( DMS(R), 4, 2 );

		// dclMul(R, 1, A[0], 4, x, 1, 4, 1, 3);
		dcldgemm(0, 0, 3, 4, 2, 1, A[0], 4, x[0], 2, 0, R[0], 2);

		dclPrint(DMS(x), 4, 2);
		dclPrint(DMS(R), 3, 2);

		for (int j = 0; j < 2; j++) {
			for (int i = 0; i < 3; i++)
			{
				printf( "[%d][%d]\n", i, j );
				assert(R[i][j] == x[i][j]);
			}

			assert(fabs(R[3][j]) < .0000001);
		}
	}

	compareToCblas();
}

