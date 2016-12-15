#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <math.h>

#define PTS 32
#define MAX_CHECKS 20000

FLT hmd_points[PTS*3];
FLT hmd_norms[PTS*3];
FLT hmd_point_angles[PTS*2];
int    hmd_point_counts[PTS*2];
int LoadData( char Camera );

//Values used for RunTest()
FLT LighthousePos[3] = { 0, 0, 0 };
FLT LighthouseQuat[4] = { 1, 0, 0, 0 };

//Actual values
FLT xyzypr[6] = { 0, 2, 2, 0, 0, 0 };


FLT RunOpti( int axis );
FLT RunTest( int print );
void PrintOpti();

int main()
{
	int i;

	//Load either 'L' (LH1) or 'R' (LH2) data.
	if( LoadData( 'R' ) ) return 5;

	int opti = 0;
	int cycle = 0;
	int axis = 0;

	FLT dx, dy, dz;

	FLT bestxyzypr[6];
	memcpy( bestxyzypr, xyzypr, sizeof( xyzypr ) );

	FLT fullrange = 5; //Maximum search space for positions.  (Relative to HMD)
	for( cycle = 0; cycle < 20; cycle ++ )
	{

		//Adjust position, one axis at a time, over and over until we zero in.
#define FULLSEARCH

#ifndef FULLSEARCH
		for( axis = 0; axis < 3; axis++ )
#endif
		{
			FLT bestxyzyprrunning[6];
			FLT beste = 1e20;

			//Try a bunch of points in this axis.
#ifdef FULLSEARCH
			for( dz = -fullrange; dz <= fullrange; dz += fullrange/3 )
			for( dy = -fullrange; dy <= fullrange; dy += fullrange/3 )
			for( dx = -fullrange; dx <= fullrange; dx += fullrange/3 )
			{
#else
			for( dx = -fullrange; dx <= fullrange; dx += fullrange/5 )
			{
#endif
				//Specificially adjust one axis at a time, searching for the best.
				memcpy( xyzypr, bestxyzypr, sizeof( xyzypr ) );
#ifdef FULLSEARCH
				xyzypr[0] += dx;
				xyzypr[1] += dy;
				xyzypr[2] += dz;
#else
				xyzypr[axis] += dx;
#endif
				//if( axis == 2 && xyzypr[2] < 0 ) continue;

				xyzypr[3] = 0;
				xyzypr[4] = 0;
				xyzypr[5] = 0;
				FLT ft;
				int reopt = 0;

				//Try refining the search for the best orientation several times.
				for( reopt = 0; reopt < 4; reopt++ )
				{
					//XXX This search mechanism is insufficient :(
					//Search through each axis every time.
					for( opti = 3; opti < 6; opti++ )
					{
						ft = RunOpti( opti );
					}
					if( ft < beste ) { beste = ft; memcpy( bestxyzyprrunning, xyzypr, sizeof( xyzypr ) ); }
				}
				//printf( "  %f %f %f %f\n", xyzypr[0], xyzypr[1], xyzypr[2], ft );
#ifndef FULLSEARCH
				memcpy( bestxyzypr, bestxyzyprrunning, sizeof( bestxyzypr ) );
#endif
			}
#ifdef FULLSEARCH
				memcpy( bestxyzypr, bestxyzyprrunning, sizeof( bestxyzypr ) );
#endif

			//Print out the quality of the lock this time.
			FLT dist = sqrt(bestxyzypr[0]*bestxyzypr[0] + bestxyzypr[1]*bestxyzypr[1] + bestxyzypr[2]*bestxyzypr[2]);
			printf( "%f %f %f (%f) %f %f %f = %f\n", bestxyzypr[0], bestxyzypr[1], bestxyzypr[2], dist, bestxyzypr[3], bestxyzypr[4], bestxyzypr[5], beste );
		}
/*
		if( bestxyzypr[2] < 0 )
		{
			bestxyzypr[0] *= -1;
			bestxyzypr[1] *= -1;
			bestxyzypr[2] *= -1;
		}
*/
		//Every cycle, tighten up the search area.
		fullrange *= 0.6;
	}

	//Use bestxyzypr
	memcpy( xyzypr, bestxyzypr, sizeof( xyzypr ) );


	//Print out plane accuracies with these settings.
	PrintOpti();
}

void PrintOpti()
{
	FLT xyzyprchange[6];
	memcpy( xyzyprchange, xyzypr, sizeof( xyzypr ) );
	quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
	memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
	FLT ft = RunTest(1);
	printf( "Final RMS: %f\n", ft );
}


FLT RunOpti( int axis )
{
	FLT xyzyprchange[6];
	memcpy( xyzyprchange, xyzypr, sizeof( xyzypr ) );
	int i;
	FLT minv = 1e20;
	FLT fv = -3.3;
	FLT bv = 0;


	//Coarse linear search first, try to figure out about where
	for( i = 0; i < 33*2; i++ )
	{
		xyzyprchange[axis] = fv;
		quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
		memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
		FLT ft = RunTest(0);
		//printf( " %d / %f = %f\n", ft, fv );
		if( ft < minv ) { minv = ft; bv = fv; }
		fv += 0.1;
	}

	xyzyprchange[axis] = bv;
#if 1
	//Now, do a more precise binary-ish search.
	FLT jumpamount = 0.15;
	for( i = 0; i < 20; i++ )
	{
		FLT orig = xyzyprchange[axis];

		minv = 1e20;

		//When searching, consider 'less' 'this' and 'more'

		fv = xyzyprchange[axis] = orig;
		quatfromeuler( LighthouseQuat, &xyzyprchange[3] );
		memcpy( LighthousePos, &xyzyprchange[0], sizeof( LighthousePos ) );
		FLT ft = RunTest(0);
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

		//Tighten up search area.  Doing by 0.5 can cause unusual instabilities.
		jumpamount *= 0.6;
	}
#endif
	xyzypr[axis] = bv;
	return minv;
}


FLT RunTest( int print )
{
	int k;
	FLT totprob = 0.0;
	int ict = 0;
	for( k = 0; k < 64; k++ )
	{
		if( hmd_point_counts[k] == 0 ) continue;
		int axis = k%2;
		int pt = k/2;
		FLT angle = hmd_point_angles[k];
		if( print ) printf( "%d %d : angle: %f / ", axis, pt, angle );

		//XXX TODO: This is critical.  We need to properly define the planes. 
		FLT plane_normal[3] = { 0, 0, 0 };
		if( axis == 0 )
		{
			plane_normal[1] = cos(angle);
			plane_normal[2] =-sin(angle);
		}
		else
		{
			plane_normal[0] =-cos(angle);
			plane_normal[2] =-sin(angle);
		}

		quatrotatevector( plane_normal, LighthouseQuat, plane_normal ); //Rotate plane normal by concatenated rotation.

		//plane_normal is our normal / LighthousePos is our point.  
		FLT w0[] = { hmd_points[pt*3+0], hmd_points[pt*3+1], hmd_points[pt*3+2] };

//May be able to remove this.
//		FLT w0[] = { hmd_points[pt*3+0], hmd_points[pt*3+2],-hmd_points[pt*3+1] };  //XXX WRONG Why does this produce the right answers?

		//FLT w0[] = { 0, 0, 0 };
		FLT d = -(plane_normal[0] * LighthousePos[0] + plane_normal[1] * LighthousePos[1] + plane_normal[2] * LighthousePos[2]);
		FLT D =   plane_normal[0] * w0[0]            + plane_normal[1] * w0[1]            + plane_normal[2] * w0[2] + d;
			//Point line distance assuming ||normal|| = 1.

		FLT delta[] = { LighthousePos[0]-hmd_points[pt*3+0],LighthousePos[1]-hmd_points[pt*3+1],LighthousePos[2]-hmd_points[pt*3+2] };
		FLT dot = delta[0] * hmd_norms[pt*3+0] + delta[1] * hmd_norms[pt*3+1] + delta[2] * hmd_norms[pt*3+2];
//		if( dot < -0.04 ) totprob+=10000000000;
		if( print ) printf( " %f %f N:%f\n", d, D, dot );
		totprob += (D*D);  //Calculate RMS distance of incorrectitude.

		ict++;
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


	f = fopen( "hmd_normals.csv", "r" );
	int nrm = 0;
	if( !f ) { fprintf( stderr, "error: can't open hmd points.\n" ); return -5; }
	while(!feof(f) && !ferror(f) && nrm < PTS)
	{
		float fa, fb, fc;
		int r = fscanf( f,"%g,%g,%g\n", &fa, &fb, &fc );
		hmd_norms[nrm*3+0] = fa;
		hmd_norms[nrm*3+1] = fb;
		hmd_norms[nrm*3+2] = fc;
		nrm++;
		if( r != 3 )
		{
			fprintf( stderr, "Not enough entries on line %d\n", nrm );
			return -8;
		}
	}
	if( nrm < PTS )
	{
		fprintf( stderr, "Not enough points.\n" );
		return -9;
	}
	if( nrm != pt )
	{
		fprintf( stderr, "point/normal counts disagree.\n" );
		return -9;
	}
	fclose( f );
	printf( "Loaded %d norms\n", nrm );


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
		hmd_point_angles[i*2+0] = (hmd_point_angles[i*2+0] - 200000) / 200000 * 3.1415926535/2; // Is it really 800,000 ticks per revolution?
		hmd_point_angles[i*2+1] = (hmd_point_angles[i*2+1] - 200000) / 200000 * 3.1415926535/2; // Is it really 800,000 ticks per revolution?
	}




	return 0;
}
