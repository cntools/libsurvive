//
//  glutil.h
//  
//
//  Created by user on 2/4/17.
//
//

#ifndef glutil_h
#define glutil_h

#include <stdio.h>
//#include "quaternion.h"
#include "linmath.h"

#include <stdio.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>   // The GL Header File
#include <GLUT/glut.h>   // The GL Utility Toolkit (Glut) Header
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

void DrawGrid(
	float minX, float maxX,
	float minY, float maxY,
	float minZ, float maxZ,
	float stepX, float stepY, float stepZ);

void DrawCoordinateSystem(
	float x, float y, float z,
	float qx, float qy, float qz, float qr);

#endif /* glutil_h */
