//
//  glutil.c
//  
//
//  Created by user on 2/4/17.
//
//

#include "glutil.h"

void DrawGrid(
	float minX, float maxX,
	float minY, float maxY,
	float minZ, float maxZ,
	float stepX, float stepY, float stepZ)
{
	float x,y,z;
	
	glBegin(GL_LINES);

	// X grid stripes
	for (y=minY; y<maxY; y+=stepY) {
		for (z=minZ; z<maxZ; z+=stepZ) {
			glVertex3f(minX, y, z);
			glVertex3f(maxX, y, z);
		}
	}

	// Y grid stripes
	for (x=minX; x<maxX; x+=stepX) {
		for (z=minZ; z<maxZ; z+=stepZ) {
			glVertex3f(x, minY, z);
			glVertex3f(x, maxY, z);
		}
	}

	// Z grid stripes
	for (y=minY; y<maxY; y+=stepY) {
		for (x=minX; x<maxX; x+=stepX) {
			glVertex3f(x, y, minZ);
			glVertex3f(x, y, maxZ);
		}
	}

	glEnd();
}


void DrawCoordinateSystem(
	float x, float y, float z,
	float qx, float qy, float qz, float qr)
{
	Quaternion i0,j0,k0;
	Quaternion i, j, k;
	Quaternion q;

	// Calculate the i, j, and k vectors
	QuaternionSet(i0, 1, 0, 0, 0);
	QuaternionSet(j0, 0, 1, 0, 0);
	QuaternionSet(k0, 0, 0, 1, 0);
	QuaternionSet(q, qx, qy, qz, qr);
	QuaternionRot(i, q, i0);
	QuaternionRot(j, q, j0);
	QuaternionRot(k, q, k0);
	
	// Draw the coordinate system i red, j green, k blue
	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(x,z,y); glVertex3f(x+i.i,z+i.k,y+i.j);
	glColor3f(0, 1, 0); glVertex3f(x,z,y); glVertex3f(x+j.i,z+j.k,y+j.j);
	glColor3f(0, 0, 1); glVertex3f(x,z,y); glVertex3f(x+k.i,z+k.k,y+k.j);
	glEnd();
}



