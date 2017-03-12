//Copyright 2013,2016 <>< C. N. Lohr.  This file licensed under the terms of the MIT/x11 license.

#ifndef _LINMATH_H
#define _LINMATH_H

//Yes, I know it's kind of arbitrary.
#define DEFAULT_EPSILON 0.001

//For printf
#define PFTHREE(x) (x)[0], (x)[1], (x)[2]
#define PFFOUR(x) (x)[0], (x)[1], (x)[2], (x)[3]

#define LINMATHPI ((FLT)3.141592653589)

//uncomment the following line to use double precision instead of single precision.
//#define USE_DOUBLE

#ifdef USE_DOUBLE

#define FLT double
#define FLT_SQRT sqrt
#define FLT_TAN tan
#define FLT_SIN  sin
#define FLT_COS  cos
#define FLT_ACOS  acos
#define FLT_ASIN  asin
#define FLT_ATAN2  atan2
#define FLT_FABS fabs

#else

#define FLT float
#define FLT_SQRT sqrtf
#define FLT_TAN tanf
#define FLT_SIN  sinf
#define FLT_COS  cosf
#define FLT_ACOS  acosf
#define FLT_ASIN  asinf
#define FLT_ATAN2  atan2f
#define FLT_FABS fabsf

#endif



//NOTE: Inputs may never be output with cross product.
void cross3d( FLT * out, const FLT * a, const FLT * b );

void sub3d( FLT * out, const FLT * a, const FLT * b );

void add3d( FLT * out, const FLT * a, const FLT * b );

void scale3d( FLT * out, const FLT * a, FLT scalar );

void normalize3d( FLT * out, const FLT * in );

FLT dot3d( const FLT * a, const FLT * b );

//Returns 0 if equal.  If either argument is null, 0 will ALWAYS be returned.
int compare3d( const FLT * a, const FLT * b, FLT epsilon );

void copy3d( FLT * out, const FLT * in );

FLT magnitude3d(const FLT * a );

FLT anglebetween3d( FLT * a, FLT * b );

//Quaternion things...

void quatsetnone( FLT * q );
void quatcopy( FLT * qout, const FLT * qin );
void quatfromeuler( FLT * q, const FLT * euler );
void quattoeuler( FLT * euler, const FLT * q );
void quatfromaxisangle( FLT * q, const FLT * axis, FLT radians );
FLT quatmagnitude( const FLT * q );
FLT quatinvsqmagnitude( const FLT * q );
void quatnormalize( FLT * qout, const FLT * qin );  //Safe for in to be same as out.
void quattomatrix( FLT * matrix44, const FLT * q );
void quatfrommatrix( FLT * q, const FLT * matrix44 );
void quatgetconjugate( FLT * qout, const FLT * qin );
void quatgetreciprocal( FLT * qout, const FLT * qin );
void quatsub( FLT * qout, const FLT * a, const FLT * b );
void quatadd( FLT * qout, const FLT * a, const FLT * b );
void quatrotateabout( FLT * qout, const FLT * a, const FLT * b );  //same as quat multiply, not piecewise multiply.
void quatscale( FLT * qout, const FLT * qin, FLT s );
FLT quatinnerproduct( const FLT * qa, const FLT * qb );
void quatouterproduct( FLT * outvec3, FLT * qa, FLT * qb );
void quatevenproduct( FLT * q, FLT * qa, FLT * qb );
void quatoddproduct( FLT * outvec3, FLT * qa, FLT * qb );
void quatslerp( FLT * q, const FLT * qa, const FLT * qb, FLT t );
void quatrotatevector( FLT * vec3out, const FLT * quat, const FLT * vec3in );
void quatfrom2vectors(FLT *q, const FLT *src, const FLT *dest);

//Poses are Position: [x, y, z]  Quaternion: [q, x, y, z]
//XXX TODO Write these!
void ApplyPoseToPoint( FLT * pout, const FLT * pin, const FLT * pose );
void InvertPose( FLT * poseout, const FLT * pose );


// Matrix Stuff

typedef struct
{
	FLT val[3][3]; // row, column
} Matrix3x3;

void rotate_vec(FLT *out, const FLT *in, Matrix3x3 rot);
void rotation_between_vecs_to_m3(Matrix3x3 *m, const FLT v1[3], const FLT v2[3]);
Matrix3x3 inverseM33(const Matrix3x3 mat);


void matrix44copy(FLT * mout, const FLT * minm );



#endif



