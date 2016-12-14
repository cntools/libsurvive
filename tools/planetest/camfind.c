#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <math.h>

#define PTS 32
#define MAX_CHECKS 20000

double hmd_points[PTS*3];
double hmd_point_angles[PTS*2];
int    hmd_point_counts[PTS*2];
int LoadData( char Camera );

//Values used for RunTest()
float LighthousePos[3] = { 0, 0, 0 };
float LighthouseQuat[4] = { 1, 0, 0, 0 };

//Actual values
float xyzypr[6] = { 0, 0, 0, 0, 0, 0 };


float RunOpti( int axis );
float RunTest( int print );
void PrintOpti();

int main()
{
	int i;

	if( LoadData( 'R' ) ) return 5;

	int opti = 0;
	int cycle = 0;
	int axis = 0;

	float dx, dy, dz;

	float bestxyzypr[6];
	memcpy( bestxyzypr, xyzypr, sizeof( xyzypr ) );

	float fullrange = 5;
	for( cycle = 0; cycle < 4; cycle ++ )
	{
//		for( axis = 0; axis < 3; axis++ )
//		{
			float bestxyzyprrunning[6];
			float beste = 1e20;
			for( dz = -fullrange; dz < fullrange; dz += fullrange/5 )
			{
			for( dy = -fullrange; dy < fullrange; dy += fullrange/5 )
			{
			for( dx = -fullrange; dx < fullrange; dx += fullrange/5 )
			{
				memcpy( xyzypr, bestxyzypr, sizeof( xyzypr ) );
				xyzypr[0] += dx;
				xyzypr[1] += dy;
				xyzypr[2] += dz;
				xyzypr[3] = 0;
				xyzypr[4] = 0;
				xyzypr[5] = 0;
				float ft;
				int reopt = 0;
				for( reopt = 0; reopt < 10; reopt++ )
				for( opti = 3; opti < 6; opti++ )
				{
					ft = RunOpti( opti );
				}
				if( ft < beste ) { beste = ft; memcpy( bestxyzyprrunning, xyzypr, sizeof( xyzypr ) ); }
				//printf( "%f %f %f %f\n", xyzypr[0], xyzypr[1], xyzypr[2], ft );
			}}}
			memcpy( bestxyzypr, bestxyzyprrunning, sizeof( bestxyzypr ) );
			printf( "%f %f %f %f %f %f = %f\n", bestxyzypr[0], bestxyzypr[1], bestxyzypr[2], bestxyzypr[3], bestxyzypr[4], bestxyzypr[5], beste );
//		}

		fullrange *= 0.3;
	}

	//Use bestxyzypr
	memcpy( xyzypr, bestxyzypr, sizeof( xyzypr ) );
	PrintOpti();
}

void PrintOpti()
{
	float xyzyprchange[6];
	memcpy( xyzyprchange, xyzypr, sizeof( xyzypr ) );
	quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
	memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
	float ft = RunTest(1);
}


float RunOpti( int axis )
{
	float xyzyprchange[6];
	memcpy( xyzyprchange, xyzypr, sizeof( xyzypr ) );
	int i;
	float minv = 1e20;
	float fv = -1.3;
	float bv = 0;


	//Coarse linear search
	for( i = 0; i < 26; i++ )
	{
		xyzyprchange[axis] = fv;
		quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
		memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
		float ft = RunTest(0);
		if( ft < minv ) { minv = ft; bv = fv; }
		fv += 0.1;
	}

	xyzyprchange[axis] = bv;

	//Now, do a more precise binary search.
	float jumpamount = 0.1;
	for( i = 0; i < 30; i++ )
	{
		float orig = xyzyprchange[axis];

		minv = 1e20;

		fv = xyzyprchange[axis] = orig;
		quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
		memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
		float ft = RunTest(0);
		if( ft < minv ) { minv = ft; bv = fv; }

		fv = xyzyprchange[axis] = orig + jumpamount;
		quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
		memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
		ft = RunTest(0);
		if( ft < minv ) { minv = ft; bv = fv; }

		fv = xyzyprchange[axis] = orig - jumpamount;
		quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
		memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
		ft = RunTest(0);
		if( ft < minv ) { minv = ft; bv = fv; }

		xyzyprchange[axis] = bv;

		jumpamount *= 0.5;
	}



	xyzypr[axis] = bv;
	return minv;
}


float RunTest( int print )
{
	int k;
	float totprob = 0.0;
	int ict = 0;
	for( k = 0; k < 64; k++ )
	{
		if( hmd_point_counts[k] == 0 ) continue;
		int axis = k%2;
		int pt = k/2;
		float angle = (hmd_point_angles[k] - 200000) / 200000 * 3.1415926535/2;  //XXX XXX WRONG??? OR SOMETHING??? WHY DIV2 MAKE GOOD?
		if( axis == 1) angle = -angle;  //Flip coordinate systems
		float thiseuler[3] = { 0, 0, 0 };
		thiseuler[axis] = angle;  

		if( print ) printf( "%d %d : angle: %f / ", axis, pt, angle );
		//axis = 0: x changes.  +y is rotated plane normal.
		//axis = 1: y changes.  +x is rotated plane normal.
		float planequat[4];
		quatfromeuler( planequat, thiseuler );
		quatrotateabout( planequat, LighthouseQuat, planequat );  //Order of rotations may be wrong.
		float plane_normal[3] = { 0, 0, 0 };
		plane_normal[!axis] = 1;
		quatrotatevector( plane_normal, planequat, plane_normal );

		//plane_normal is our normal / LighthousePos is our point.
		float w0[] = { hmd_points[pt*3+0], hmd_points[pt*3+1], hmd_points[pt*3+2] };
		//float w0[] = { 0, 0, 0 };
		float d = -(plane_normal[0] * LighthousePos[0] + plane_normal[1] * LighthousePos[1] + plane_normal[2] * LighthousePos[2]);
		float D =   plane_normal[0] * w0[0]            + plane_normal[1] * w0[1]            + plane_normal[2] * w0[2] + d;
			//Point line distance assuming ||normal|| = 1.

		if( print ) printf( " %f %f -\n", d, D );
		totprob += (D*D);
		ict++;
		if( print )
		{
			//printf( "%d %d ", pt, axis, hmd_points[ ); ///..x.x.x  XXX TODO check line intersection and planequat thing
		}

		//printf( "  : %f %f %f %f  %f %f\n", plane_normal[0], plane_normal[1], plane_normal[2], 1.0, angle, hmd_point_angles[k] );
	}
	return sqrt(totprob/ict);
}


int LoadData( char Camera )
{
	int calpts[MAX_CHECKS*4]; //X (0) or Y (1), ID, offset
	int calptscount;

	FILE * f = fopen( "correct_hmd_points.csv", "r" );
	int pt = 0;
	if( !f ) { fprintf( stderr, "error: can't open hmd points.\n" ); return -5; }
	while(!feof(f) && !ferror(f) && pt < PTS)
	{
		float fa, fb, fc;
		int r = fscanf( f,"%g,%g,%g\n", &fa, &fb, &fc );
		hmd_points[pt*3+0] = fa;
		hmd_points[pt*3+1] = fb;
		hmd_points[pt*3+2] = fc;
		pt++;
		if( r != 3 )
		{
			fprintf( stderr, "Not enough entries on line %d\n", pt );
			return -8;
		}
	}
	if( pt < PTS )
	{
		fprintf( stderr, "Not enough points.\n" );
		return -9;
	}
	fclose( f );
	printf( "Loaded %d points\n", pt );


	int xck = 0;
	f = fopen( "third_test_with_time_lengths.csv", "r" );
	if( !f ) { fprintf( stderr, "Error: can't open two lighthouses test data.\n" ); return -11; }
	while( !feof(f) && !ferror(f) )
	{
		char * lineptr;
		size_t n;
		lineptr = 0;
		n = 0;
		ssize_t r = getline( &lineptr, &n, f );
		char lf[10];
		char xory[10];
		char dev[10];
		int timestamp;
		int sensorid;
		int offset;
		int code;
		int length;
		int rk = sscanf( lineptr, "%9s %9s %9s %d %d %d %d %d\n", lf, xory, dev, &timestamp, &sensorid, &code, &offset, &length );
		if( lf[0] == Camera && rk == 8 )
		{
			//calpts
			if( xory[0] == 'X' )
			{
				calpts[xck*3+0] = 0;
			}
			else if( xory[0] == 'Y' )
			{
				calpts[xck*3+0] = 1;
			}
			else
			{
				printf( "Confusing line\n" );
				continue;
			}

			calpts[xck*3+1] = sensorid;
			calpts[xck*3+2] = offset;
			xck++;
		}
		if( lineptr ) free( lineptr );
	}
	printf( "Cal points: %d\n", xck );
	calptscount = xck;

	int i;
	for( i = 0; i < calptscount; i++ )
	{
		int isy = calpts[i*3+0];
		int pt = calpts[i*3+1];
		int ofs = calpts[i*3+2];
		hmd_point_counts[pt*2+isy]++;
		hmd_point_angles[pt*2+isy]+=ofs;
	}
	for( i = 0; i < 32; i++ )
	{
		if( hmd_point_counts[i*2+0] < 100 ) hmd_point_counts[i*2+0] = 0;
		if( hmd_point_counts[i*2+1] < 100 ) hmd_point_counts[i*2+1] = 0;

		hmd_point_angles[i*2+0]/=hmd_point_counts[i*2+0];
		hmd_point_angles[i*2+1]/=hmd_point_counts[i*2+1];
	}
	return 0;
}
