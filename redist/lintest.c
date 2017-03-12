#include "linmath.h"
#include <stdio.h>

int main()
{
	
	FLT e[3] = { 1,1,3.14 };
	FLT q[4];
	FLT m[16];
	FLT pt[3] = { 1, 1, 1 };

	q[0] = 0;
	q[1] = 0;
	q[2] = 0;
	q[3] = 1;

	quatrotatevector( pt, q, pt );
	printf( "%f %f %f\n", PFTHREE( pt ) );
	printf( "\n" );

	quatfromeuler( q, e );
	printf( "%f %f %f %f\n\n", PFFOUR( q ) );
	quattomatrix(m,q);
	printf( "%f %f %f %f\n", PFFOUR( &m[0] ) );
	printf( "%f %f %f %f\n", PFFOUR( &m[4] ) );
	printf( "%f %f %f %f\n", PFFOUR( &m[8] ) );
	printf( "%f %f %f %f\n\n", PFFOUR( &m[12] ) );
	quatfrommatrix(q,m );
	printf( "%f %f %f %f\n\n", PFFOUR( q ) );
	quattoeuler( e,q );
	printf( "E: %f %f %f\n", e[0], e[1], e[2] );


	FLT pfromlh[3] = { 0, 1, 0 };
	quatrotatevector( p, q, p );
	printf( "%f %f %f\n", PFTHREE( p ) );
	printf( "Flipping rotation\n" );
	q[0] *= -1; //Wow that was easy.
	quatrotatevector( p, q, p );
	printf( "%f %f %f\n", PFTHREE( p ) );


	//Try setting up a pose.
//	FLT mypose[7] = { 0, 0, 10, q[0], q[1], q[2], q[3] );
//	ApplyPoseToPoint( FLT * pout, const FLT * pin, const FLT * pose );
//void InvertPose( FLT * poseout, const FLT * pose );
	
	
}

