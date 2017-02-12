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
	FLT i0[3],j0[3],k0[3];
	FLT i[3],j[3],k[3];
	FLT q[4];
	
	i0[0]=1.0; i0[1]=0.0; i0[2]=0.0;
	j0[0]=0.0; j0[1]=1.0; j0[2]=0.0;
	k0[0]=0.0; k0[1]=0.0; k0[2]=1.0;
	q [0]=qr;  q [1]=qx;  q [2]=qy;  q [3]=qz;
	
	quatrotatevector(i, q, i0);
	quatrotatevector(j, q, j0);
	quatrotatevector(k, q, k0);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(x,z,y); glVertex3f(x+i[0],z+i[2],y+i[1]);
	glColor3f(0, 1, 0); glVertex3f(x,z,y); glVertex3f(x+j[0],z+j[2],y+j[1]);
	glColor3f(0, 0, 1); glVertex3f(x,z,y); glVertex3f(x+k[0],z+k[2],y+k[1]);
	glEnd();
}



