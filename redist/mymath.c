//Copyright 2013,2016 <>< C. N. Lohr.  This file licensed under the terms of the MIT license.

#include <math.h>
#include <float.h>
#include <string.h>
#include "linmath.h"

#define FLT float
#define LINMATHPI ((FLT)3.141592653589)
#define DEFAULT_EPSILON 0.001
void quatnormalize( FLT * qout, const FLT * qin );
void quatscale( FLT * qout, const FLT * qin, FLT s );
void quatfrom2vectors(FLT *q, const FLT *src, const FLT *dest);


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
	FLT r = ((FLT)1.) / FLT_SQRT(in[0] * in[0] + in[1] * in[1] + in[2] * in[2]);
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

FLT magnitude3d(const FLT * a )
{
	return FLT_SQRT(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

FLT anglebetween3d( FLT * a, FLT * b )
{
	FLT an[3];
	FLT bn[3];
	normalize3d( an, a );
	normalize3d( bn, b );
	FLT dot = dot3d(an, bn);
	if( dot < -0.9999999 ) return LINMATHPI;
	if( dot >  0.9999999 ) return 0;
	return FLT_ACOS(dot);
}

/////////////////////////////////////QUATERNIONS//////////////////////////////////////////
//Originally from Mercury (Copyright (C) 2009 by Joshua Allen, Charles Lohr, Adam Lowman)
//Under the mit/X11 license.




void quatsetnone(FLT * q)
{
	q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
}

void quatcopy(FLT * qout, const FLT * qin)
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

	FLT cx = FLT_COS(X);
	FLT sx = FLT_SIN(X);
	FLT cy = FLT_COS(Y);
	FLT sy = FLT_SIN(Y);
	FLT cz = FLT_COS(Z);
	FLT sz = FLT_SIN(Z);

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
	euler[0] = FLT_ATAN2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	euler[1] = FLT_ASIN(2 * (q[0] * q[2] - q[3] * q[1]));
	euler[2] = FLT_ATAN2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
}

void quatfromaxisangle( FLT * q, const FLT * axis, FLT radians )
{
	FLT v[3];
	normalize3d( v, axis );
	
	FLT sn = FLT_SIN(radians / 2.0f);
	q[0] = FLT_COS(radians / 2.0f);
	q[1] = sn * v[0];
	q[2] = sn * v[1];
	q[3] = sn * v[2];

	quatnormalize( q, q );
}

FLT quatmagnitude( const FLT * q )
{
	return FLT_SQRT((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
}

FLT quatinvsqmagnitude( const FLT * q )
{
	return ((FLT)1.)/FLT_SQRT((q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])+(q[3]*q[3]));
}


void quatnormalize( FLT * qout, const FLT * qin )
{
	FLT imag = quatinvsqmagnitude( qin );
	quatscale( qout, qin, imag );
}

void quattomatrix(FLT * matrix44, const FLT * qin)
{
	FLT q[4];
	quatnormalize(q, qin);

	//Reduced calulation for speed
	FLT xx = 2 * q[1] * q[1];
	FLT xy = 2 * q[1] * q[2];
	FLT xz = 2 * q[1] * q[3];
	FLT xw = 2 * q[1] * q[0];

	FLT yy = 2 * q[2] * q[2];
	FLT yz = 2 * q[2] * q[3];
	FLT yw = 2 * q[2] * q[0];

	FLT zz = 2 * q[3] * q[3];
	FLT zw = 2 * q[3] * q[0];

	//opengl major
	matrix44[0] = 1 - yy - zz;
	matrix44[1] = xy - zw;
	matrix44[2] = xz + yw;
	matrix44[3] = 0;

	matrix44[4] = xy + zw;
	matrix44[5] = 1 - xx - zz;
	matrix44[6] = yz - xw;
	matrix44[7] = 0;

	matrix44[8] = xz - yw;
	matrix44[9] = yz + xw;
	matrix44[10] = 1 - xx - yy;
	matrix44[11] = 0;

	matrix44[12] = 0;
	matrix44[13] = 0;
	matrix44[14] = 0;
	matrix44[15] = 1;
}


void quatfrommatrix( FLT * q, const FLT * matrix44 )
{
	//Algorithm from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	float tr = matrix44[0] + matrix44[5] + matrix44[10];

	if (tr > 0) {
		float S = sqrt(tr+1.0) * 2; // S=4*qw
		q[0] = 0.25 * S;
		q[1] = (matrix44[9] - matrix44[6]) / S;
		q[2] = (matrix44[2] - matrix44[8]) / S;
		q[3] = (matrix44[4] - matrix44[1]) / S;
	} else if ((matrix44[0] > matrix44[5])&(matrix44[0] > matrix44[10])) {
		float S = sqrt(1.0 + matrix44[0] - matrix44[5] - matrix44[10]) * 2; // S=4*qx
		q[0] = (matrix44[9] - matrix44[6]) / S;
		q[1] = 0.25 * S;
		q[2] = (matrix44[1] + matrix44[4]) / S;
		q[3] = (matrix44[2] + matrix44[8]) / S;
	} else if (matrix44[5] > matrix44[10]) {
		float S = sqrt(1.0 + matrix44[5] - matrix44[0] - matrix44[10]) * 2; // S=4*qy
		q[0] = (matrix44[2] - matrix44[8]) / S;
		q[1] = (matrix44[1] + matrix44[4]) / S;
		q[2] = 0.25 * S;
		q[3] = (matrix44[6] + matrix44[9]) / S;
	} else {
		float S = sqrt(1.0 + matrix44[10] - matrix44[0] - matrix44[5]) * 2; // S=4*qz
		q[0] = (matrix44[4] - matrix44[1]) / S;
		q[1] = (matrix44[2] + matrix44[8]) / S;
		q[2] = (matrix44[6] + matrix44[9]) / S;
		q[3] = 0.25 * S;
	}
}


void quattomatrix33(FLT * matrix33, const FLT * qin)
{
	FLT q[4];
	quatnormalize(q, qin);

	//Reduced calulation for speed
	FLT xx = 2 * q[0] * q[0];
	FLT xy = 2 * q[0] * q[1];
	FLT xz = 2 * q[0] * q[2];
	FLT xw = 2 * q[0] * q[3];

	FLT yy = 2 * q[1] * q[1];
	FLT yz = 2 * q[1] * q[2];
	FLT yw = 2 * q[1] * q[3];

	FLT zz = 2 * q[2] * q[2];
	FLT zw = 2 * q[2] * q[3];

	//opengl major
	matrix33[0] = 1 - yy - zz;
	matrix33[1] = xy - zw;
	matrix33[2] = xz + yw;

	matrix33[3] = xy + zw;
	matrix33[4] = 1 - xx - zz;
	matrix33[5] = yz - xw;

	matrix33[6] = xz - yw;
	matrix33[7] = yz + xw;
	matrix33[8] = 1 - xx - yy;
}

void quatgetconjugate(FLT * qout, const FLT * qin)
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

void quatrotateabout( FLT * qout, const FLT * q1, const FLT * q2 )
{
	//NOTE: Does not normalize
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
		sinTheta = FLT_SQRT(1 - (cosTheta*cosTheta));

	FLT Theta = FLT_ACOS(cosTheta); //Theta is half the angle between the 2 MQuaternions

	if (FLT_FABS(Theta) < DEFAULT_EPSILON)
		quatcopy( q, qa );
	else if (FLT_FABS(sinTheta) < DEFAULT_EPSILON)
	{
		quatadd( q, qa, qb );
		quatscale( q, q, 0.5 );
	}
	else
	{
		FLT aside[4];
		FLT bside[4];
		quatscale( bside, qb, FLT_SIN(t * Theta));
		quatscale( aside, qa, FLT_SIN((1 - t)*Theta));
		quatadd( q, aside, bside );
		quatscale( q, q, ((FLT)1.)/sinTheta );
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
	quatgetconjugate( qrecp, quat );
	quatrotateabout( vquat, tquat, qrecp );

	vec3out[0] = vquat[1];
	vec3out[1] = vquat[2];
	vec3out[2] = vquat[3];
}


// Matrix Stuff

Matrix3x3 inverseM33(const Matrix3x3 mat)
{
	Matrix3x3 newMat;
	for (int a = 0; a < 3; a++)
	{
		for (int b = 0; b < 3; b++)
		{
			newMat.val[a][b] = mat.val[a][b];
		}
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = i + 1; j < 3; j++)
		{
			FLT tmp = newMat.val[i][j];
			newMat.val[i][j] = newMat.val[j][i];
			newMat.val[j][i] = tmp;
		}
	}

	return newMat;
}

void rotation_between_vecs_to_m3(Matrix3x3 *m, const FLT v1[3], const FLT v2[3])
{
	FLT q[4];

	quatfrom2vectors(q, v1, v2);

	quattomatrix33(&(m->val[0][0]), q);
}

void rotate_vec(FLT *out, const FLT *in, Matrix3x3 rot)
{
	out[0] = rot.val[0][0] * in[0] + rot.val[1][0] * in[1] + rot.val[2][0] * in[2];
	out[1] = rot.val[0][1] * in[0] + rot.val[1][1] * in[1] + rot.val[2][1] * in[2];
	out[2] = rot.val[0][2] * in[0] + rot.val[1][2] * in[1] + rot.val[2][2] * in[2];

	return;
}


// This function based on code from Object-oriented Graphics Rendering Engine
// Copyright(c) 2000 - 2012 Torus Knot Software Ltd
// under MIT license
// http://www.ogre3d.org/docs/api/1.9/_ogre_vector3_8h_source.html

/** Gets the shortest arc quaternion to rotate this vector to the destination
vector.
@remarks
If you call this with a dest vector that is close to the inverse
of this vector, we will rotate 180 degrees around a generated axis if
since in this case ANY axis of rotation is valid.
*/
void quatfrom2vectors(FLT *q, const FLT *src, const FLT *dest)
{
	// Based on Stan Melax's article in Game Programming Gems

	// Copy, since cannot modify local
	FLT v0[3];
	FLT v1[3];
	normalize3d(v0, src);
	normalize3d(v1, dest);

	FLT d = dot3d(v0, v1);// v0.dotProduct(v1);
	// If dot == 1, vectors are the same
	if (d >= 1.0f)
	{
		quatsetnone(q);
		return;
	}
	if (d < (1e-6f - 1.0f))
	{
		// Generate an axis
		FLT unitX[3] = { 1, 0, 0 };
		FLT unitY[3] = { 0, 1, 0 };
		
		FLT axis[3];
		cross3d(axis, unitX, src); // pick an angle
		if ((axis[0] < 1.0e-35f) &&
			(axis[1] < 1.0e-35f) &&
			(axis[2] < 1.0e-35f)) // pick another if colinear
		{
			cross3d(axis, unitY, src);
		}
		normalize3d(axis, axis);
		quatfromaxisangle(q, axis, LINMATHPI);
	}
	else
	{
		FLT s = FLT_SQRT((1 + d) * 2);
		FLT invs = 1 / s;

		FLT c[3];
		//cross3d(c, v0, v1);
		cross3d(c, v1, v0);

		q[0] = c[0] * invs;
		q[1] = c[1] * invs;
		q[2] = c[2] * invs;
		q[3] = s * 0.5f;

		quatnormalize(q, q);
	}

}

void matrix44copy(FLT * mout, const FLT * minm )
{
	memcpy( mout, minm, sizeof( FLT ) * 16 );
}

void matrix44transpose(FLT * mout, const FLT * minm )
{
	mout[0] = minm[0];
	mout[1] = minm[4];
	mout[2] = minm[8];
	mout[3] = minm[12];

	mout[4] = minm[1];
	mout[5] = minm[5];
	mout[6] = minm[9];
	mout[7] = minm[13];

	mout[8] = minm[2];
	mout[9] = minm[6];
	mout[10] = minm[10];
	mout[11] = minm[14];

	mout[12] = minm[3];
	mout[13] = minm[7];
	mout[14] = minm[11];
	mout[15] = minm[15];

}

