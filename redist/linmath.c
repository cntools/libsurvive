//Copyright 2013,2016 <>< C. N. Lohr.  This file licensed under the terms of the MIT license.

#include "linmath.h"
#include <math.h>

void cross3d( FLT * out, const FLT * a, const FLT * b )
{
	out[0] = a[1]*b[2] - a[2]*b[1];
	out[1] = a[2]*b[0] - a[0]*b[2];
	out[2] = a[0]*b[1] - a[1]*b[0];
}

void sub3d( FLT * out, const FLT * a, const FLT * b )
{
	out[0] = a[0] - b[0];
	out[1] = a[1] - b[1];
	out[2] = a[2] - b[2];
}

void add3d( FLT * out, const FLT * a, const FLT * b )
{
	out[0] = a[0] + b[0];
	out[1] = a[1] + b[1];
	out[2] = a[2] + b[2];
}

void scale3d( FLT * out, const FLT * a, FLT scalar )
{
	out[0] = a[0] * scalar;
	out[1] = a[1] * scalar;
	out[2] = a[2] * scalar;
}

void normalize3d( FLT * out, const FLT * in )
{
	FLT r = 1./sqrtf( in[0] * in[0] + in[1] * in[1] + in[2] * in[2] );
	out[0] = in[0] * r;
	out[1] = in[1] * r;
	out[2] = in[2] * r;
}

FLT dot3d( const FLT * a, const FLT * b )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

int compare3d( const FLT * a, const FLT * b, FLT epsilon )
{
	if( !a || !b ) return 0;
	if( a[2] - b[2] > epsilon ) return 1;
	if( b[2] - a[2] > epsilon ) return -1;
	if( a[1] - b[1] > epsilon ) return 1;
	if( b[1] - a[1] > epsilon ) return -1;
	if( a[0] - b[0] > epsilon ) return 1;
	if( b[0] - a[0] > epsilon ) return -1;
	return 0;
}

void copy3d( FLT * out, const FLT * in )
{
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}

FLT magnitude3d( FLT * a )
{
	return sqrt( a[0]*a[0] + a[1]*a[1] + a[2]*a[2] );
}

FLT anglebetween3d( FLT * a, FLT * b )
{
	FLT an[3];
	FLT bn[3];
	normalize3d( an, a );
	normalize3d( bn, b );
	FLT dot = dot3d( a, b );
	if( dot < -0.9999999 ) return LINMATHPI;
	if( dot >  0.9999999 ) return 0;
	return acos( dot );
}

/////////////////////////////////////QUATERNIONS//////////////////////////////////////////
//Originally from Mercury (Copyright (C) 2009 by Joshua Allen, Charles Lohr, Adam Lowman)
//Under the mit/X11 license.




void quatsetnone( FLT * q )
{
	q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
}

void quatcopy( FLT * qout, const FLT * qin )
{
	qout[0] = qin[0];
	qout[1] = qin[1];
	qout[2] = qin[2];
	qout[3] = qin[3];
}

void quatfromeuler( FLT * q, const FLT * euler )
{
	FLT X = euler[0]/2.0f; //roll
	FLT Y = euler[1]/2.0f; //pitch
	FLT Z = euler[2]/2.0f; //yaw

	FLT cx = cosf(X);
	FLT sx = sinf(X);
	FLT cy = cosf(Y);
	FLT sy = sinf(Y);
	FLT cz = cosf(Z);
	FLT sz = sinf(Z);

	//Correct according to
	//http://en.wikipedia.org/wiki/Conversion_between_MQuaternions_and_Euler_angles
	q[0] = cx*cy*cz+sx*sy*sz;//q1
	q[1] = sx*cy*cz-cx*sy*sz;//q2
	q[2] = cx*sy*cz+sx*cy*sz;//q3
	q[3] = cx*cy*sz-sx*sy*cz;//q4
	quatnormalize( q, q );
}

void quattoeuler( FLT * euler, const FLT * q )
{
	//According to http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles (Oct 26, 2009)
	euler[0] = atan2( 2 * (q[0]*q[1] + q[2]*q[3]), 1 - 2 * (q[1]*q[1] + q[2]*q[2] ) );
	euler[1] = asin( 2 * (q[0] *q[2] - q[3]*q[1] ) );
	euler[2] = atan2( 2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]*q[2] + q[3]*q[3] ) );
}

void quatfromaxisangle( FLT * q, const FLT * axis, FLT radians )
{
	FLT v[3];
	normalize3d( v, axis );
	
	FLT sn = sin(radians/2.0f);
	q[0] = cos(radians/2.0f);
	q[1] = sn * v[0];
	q[2] = sn * v[1];
	q[3] = sn * v[2];

	quatnormalize( q, q );
}

FLT quatmagnitude( const FLT * q )
{
	return sqrt((q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])+(q[3]*q[3]));
}

FLT quatinvsqmagnitude( const FLT * q )
{
	return 1./((q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])+(q[3]*q[3]));
}


void quatnormalize( FLT * qout, const FLT * qin )
{
	FLT imag = quatinvsqmagnitude( qin );
	quatscale( qout, qin, imag );
}

void quattomatrix( FLT * matrix44, const FLT * qin )
{
	FLT q[4];
	quatnormalize( q, qin );
	
	//Reduced calulation for speed
	FLT xx = 2*q[0]*q[0];
	FLT xy = 2*q[0]*q[1];
	FLT xz = 2*q[0]*q[2];
	FLT xw = 2*q[0]*q[3];
	
	FLT yy = 2*q[1]*q[1];
	FLT yz = 2*q[1]*q[2];
	FLT yw = 2*q[1]*q[3];
	
	FLT zz = 2*q[2]*q[2];
	FLT zw = 2*q[2]*q[3];

	//opengl major
	matrix44[0] = 1-yy-zz;
	matrix44[1] = xy-zw;
	matrix44[2] = xz+yw;
	matrix44[3] = 0;

	matrix44[4] = xy+zw;
	matrix44[5] = 1-xx-zz;
	matrix44[6] = yz-xw;
	matrix44[7] = 0;

	matrix44[8] = xz-yw;
	matrix44[9] = yz+xw;
	matrix44[10] = 1-xx-yy;
	matrix44[11] = 0;

	matrix44[12] = 0;
	matrix44[13] = 0;
	matrix44[14] = 0;
	matrix44[15] = 1;
}

void quatgetconjugate( FLT * qout, const FLT * qin )
{
	qout[0] = qin[0];
	qout[1] = -qin[1];
	qout[2] = -qin[2];
	qout[3] = -qin[3];
}

void quatgetreciprocal( FLT * qout, const FLT * qin )
{
	FLT m = quatinvsqmagnitude(qin);
	quatgetconjugate( qout, qin );
	quatscale( qout, qout, m );
}

void quatsub( FLT * qout, const FLT * a, const FLT * b )
{
	qout[0] = a[0] - b[0];
	qout[1] = a[1] - b[1];
	qout[2] = a[2] - b[2];
	qout[3] = a[3] - b[3];
}

void quatadd( FLT * qout, const FLT * a, const FLT * b )
{
	qout[0] = a[0] + b[0];
	qout[1] = a[1] + b[1];
	qout[2] = a[2] + b[2];
	qout[3] = a[3] + b[3];
}

void quatrotateabout( FLT * qout, const FLT * a, const FLT * b )
{
	FLT q1[4];
	FLT q2[4];

	quatnormalize( q1, a );
	quatnormalize( q2, b );

	qout[0] = (q1[0]*q2[0])-(q1[1]*q2[1])-(q1[2]*q2[2])-(q1[3]*q2[3]);
	qout[1] = (q1[0]*q2[1])+(q1[1]*q2[0])+(q1[2]*q2[3])-(q1[3]*q2[2]);
	qout[2] = (q1[0]*q2[2])-(q1[1]*q2[3])+(q1[2]*q2[0])+(q1[3]*q2[1]);
	qout[3] = (q1[0]*q2[3])+(q1[1]*q2[2])-(q1[2]*q2[1])+(q1[3]*q2[0]);
}

void quatscale( FLT * qout, const FLT * qin, FLT s )
{
	qout[0] = qin[0] * s;
	qout[1] = qin[1] * s;
	qout[2] = qin[2] * s;
	qout[3] = qin[3] * s;
}


FLT quatinnerproduct( const FLT * qa, const FLT * qb )
{
	return (qa[0]*qb[0])+(qa[1]*qb[1])+(qa[2]*qb[2])+(qa[3]*qb[3]);
}

void quatouterproduct( FLT * outvec3, FLT * qa, FLT * qb )
{
	outvec3[0] = (qa[0]*qb[1])-(qa[1]*qb[0])-(qa[2]*qb[3])+(qa[3]*qb[2]);
	outvec3[1] = (qa[0]*qb[2])+(qa[1]*qb[3])-(qa[2]*qb[0])-(qa[3]*qb[1]);
	outvec3[2] = (qa[0]*qb[3])-(qa[1]*qb[2])+(qa[2]*qb[1])-(qa[3]*qb[0]);
}

void quatevenproduct( FLT * q, FLT * qa, FLT * qb )
{
	q[0] = (qa[0]*qb[0])-(qa[1]*qb[1])-(qa[2]*qb[2])-(qa[3]*qb[3]);
	q[1] = (qa[0]*qb[1])+(qa[1]*qb[0]);
	q[2] = (qa[0]*qb[2])+(qa[2]*qb[0]);
	q[3] = (qa[0]*qb[3])+(qa[3]*qb[0]);
}

void quatoddproduct( FLT * outvec3, FLT * qa, FLT * qb )
{
	outvec3[0] = (qa[2]*qb[3])-(qa[3]*qb[2]);
	outvec3[1] = (qa[3]*qb[1])-(qa[1]*qb[3]);
	outvec3[2] = (qa[1]*qb[2])-(qa[2]*qb[1]);
}

void quatslerp( FLT * q, const FLT * qa, const FLT * qb, FLT t )
{
	FLT an[4];
	FLT bn[4];
	quatnormalize( an, qa );
	quatnormalize( bn, qb );
	FLT cosTheta = quatinnerproduct(an,bn);
	FLT sinTheta;

	//Careful: If cosTheta is exactly one, or even if it's infinitesimally over, it'll
	// cause SQRT to produce not a number, and screw everything up.
	if ( 1 - (cosTheta*cosTheta) <= 0 )
		sinTheta = 0;
	else
		sinTheta = sqrt(1 - (cosTheta*cosTheta));

	FLT Theta = acos(cosTheta); //Theta is half the angle between the 2 MQuaternions

	if(fabs(Theta) < DEFAULT_EPSILON )
		quatcopy( q, qa );
	else if(fabs(sinTheta) < DEFAULT_EPSILON )
	{
		quatadd( q, qa, qb );
		quatscale( q, q, 0.5 );
	}
	else
	{
		FLT aside[4];
		FLT bside[4];
		quatscale( bside, qb, sin( t * Theta ) );
		quatscale( aside, qa, sin((1-t)*Theta) );
		quatadd( q, aside, bside );
		quatscale( q, q, 1./sinTheta );
	}
}

void quatrotatevector( FLT * vec3out, const FLT * quat, const FLT * vec3in )
{
	FLT tquat[4];
	FLT vquat[4];
	FLT qrecp[4];
	vquat[0] = 0;
	vquat[1] = vec3in[0];
	vquat[2] = vec3in[1];
	vquat[3] = vec3in[2];

	quatrotateabout( tquat, quat, vquat );
	quatgetreciprocal( qrecp, quat );
	quatrotateabout( vquat, tquat, qrecp );

	vec3out[0] = vquat[1];
	vec3out[1] = vquat[2];
	vec3out[2] = vquat[3];
}



