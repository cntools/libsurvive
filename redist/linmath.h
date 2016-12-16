//Copyright 2013,2016 <>< C. N. Lohr.  This file licensed under the terms of the MIT/x11 license.

#ifndef _LINMATH_H
#define _LINMATH_H

//Yes, I know it's kind of arbitrary.
#define DEFAULT_EPSILON 0.001

//For printf
#define PFTHREE(x) x[0], x[1], x[2]
#define PFFOUR(x) x[0], x[1], x[2], x[3]

#define LINMATHPI 3.141592653589

//If you want, you can define FLT to be double for double precision.
#ifndef FLT
#define FLT float
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

FLT magnitude3d( FLT * a );

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


#endif



