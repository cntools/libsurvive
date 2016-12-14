//Copyright 2013 <>< C. N. Lohr.  This file licensed under the terms of the MIT license.

#ifndef _LINMATH_H
#define _LINMATH_H

//Yes, I know it's kind of arbitrary.
#define DEFAULT_EPSILON 0.001


//NOTE: Inputs may never be output with cross product.
void cross3d( float * out, const float * a, const float * b );

void sub3d( float * out, const float * a, const float * b );

void add3d( float * out, const float * a, const float * b );

void scale3d( float * out, const float * a, float scalar );

void normalize3d( float * out, const float * in );

float dot3d( const float * a, const float * b );

//Returns 0 if equal.  If either argument is null, 0 will ALWAYS be returned.
int compare3d( const float * a, const float * b, float epsilon );

void copy3d( float * out, const float * in );




//Quaternion things...

void quatsetnone( float * q );
void quatcopy( float * qout, const float * qin );
void quatfromeuler( float * q, const float * euler );
void quattoeuler( float * euler, const float * q );
void quatfromaxisangle( float * q, const float * axis, float radians );
float quatmagnitude( const float * q );
float quatinvsqmagnitude( const float * q );
void quatnormalize( float * qout, const float * qin );  //Safe for in to be same as out.
void quattomatrix( float * matrix44, const float * q );
void quatgetconjugate( float * qout, const float * qin );
void quatgetreciprocal( float * qout, const float * qin );
void quatsub( float * qout, const float * a, const float * b );
void quatadd( float * qout, const float * a, const float * b );
void quatrotateabout( float * qout, const float * a, const float * b );  //same as quat multiply, not piecewise multiply.
void quatscale( float * qout, const float * qin, float s );
float quatinnerproduct( const float * qa, const float * qb );
void quatouterproduct( float * outvec3, float * qa, float * qb );
void quatevenproduct( float * q, float * qa, float * qb );
void quatoddproduct( float * outvec3, float * qa, float * qb );
void quatslerp( float * q, const float * qa, const float * qb, float t );
void quatrotatevector( float * vec3out, const float * quat, const float * vec3in );


#endif



