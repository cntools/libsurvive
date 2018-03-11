//Copyright 2012-2017 <>< Charles Lohr
//You may license this file under the MIT/x11, NewBSD, or any GPL license.

//This is a series of tools useful for software rendering.
//Use of this file with OpenGL is untested.

#ifndef _3D_H
#define _3D_H

#include <math.h>
#define tdCOS cosf
#define tdSIN sinf
#define tdTAN tanf
#define tdSQRT sqrtf
#define tdMATCOPY(x,y) memcpy( x, y, 16*sizeof(float))
#define tdQ_PI 3.141592653589
#define tdDEGRAD (tdQ_PI/180.)
#define tdRADDEG (180./tdQ_PI)


//General Matrix Functions
void tdIdentity( float * f );
void tdZero( float * f );
void tdTranslate( float * f, float x, float y, float z );		//Operates ON f
void tdScale( float * f, float x, float y, float z );			//Operates ON f
void tdRotateAA( float * f, float angle, float x, float y, float z ); 	//Operates ON f
void tdRotateEA( float * f, float x, float y, float z );		//Operates ON f
void tdMultiply( float * fin1, float * fin2, float * fout );		//Operates ON f
void tdPrint( float * f );
void tdTransposeSelf( float * f );

//Specialty Matrix Functions
void tdPerspective( float fovy, float aspect, float zNear, float zFar, float * out ); //Sets, NOT OPERATES. (FOVX=degrees)
void tdLookAt( float * m, float * eye, float * at, float * up );	//Operates ON f

//General point functions
#define tdPSet( f, x, y, z ) { f[0] = x; f[1] = y; f[2] = z; }
void tdPTransform( const float * pin, float * f, float * pout );
void tdVTransform( const float * vin, float * f, float * vout );
void td4Transform( float * kin, float * f, float * kout );
void td4RTransform( float * kin, float * f, float * kout );
void tdNormalizeSelf( float * vin );
void tdCross( float * va, float * vb, float * vout );
float tdDistance( float * va, float * vb );
float tdDot( float * va, float * vb );
#define tdPSub( x, y, z ) { (z)[0] = (x)[0] - (y)[0]; (z)[1] = (x)[1] - (y)[1]; (z)[2] = (x)[2] - (y)[2]; }
#define tdPAdd( x, y, z ) { (z)[0] = (x)[0] + (y)[0]; (z)[1] = (x)[1] + (y)[1]; (z)[2] = (x)[2] + (y)[2]; }

//Stack Functionality
#define tdMATRIXMAXDEPTH 32
extern float * gSMatrix;
void tdPush();
void tdPop();
void tdMode( int mode );
#define tdMODELVIEW 0
#define tdPROJECTION 1

//Final stage tools
void tdSetViewport( float leftx, float topy, float rightx, float bottomy, float pixx, float pixy );
void tdFinalPoint( float * pin, float * pout );


float tdFLerp( float a, float b, float t );
float tdPerlin2D( float x, float y );

#endif
