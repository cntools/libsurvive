#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <math.h>

#define PTS 32
#define MAX_CHECKS 20000
#define MIN_HITS_FOR_VALID 10

FLT hmd_points[PTS*3];
FLT hmd_norms[PTS*3];
FLT hmd_point_angles[PTS*2];
int hmd_point_counts[PTS*2];
int best_hmd_target = 0;
int LoadData( char Camera );

//Values used for RunTest()
FLT LighthousePos[3] = { 0, 0, 0 };
FLT LighthouseQuat[4] = { 1, 0, 0, 0 };
FLT BarrelRotate[4];  //XXX TODO: Concatenate these.


FLT RunOpti( int print );
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

	FLT bestxyz[3];
	memcpy( bestxyz, LighthousePos, sizeof( LighthousePos ) );

	//STAGE1 1: Detemine vectoral position from lighthouse to target.  Does not determine lighthouse-target distance.
	//This also is constantly optimizing the lighthouse quaternion for optimal spotting.
	FLT fullrange = 5; //Maximum search space for positions.  (Relative to HMD)
	for( cycle = 0; cycle < 30; cycle ++ )
	{

		//Adjust position, one axis at a time, over and over until we zero in.
		{
			FLT bestxyzrunning[3];
			FLT beste = 1e20;

			FLT splits = 2;
			if( cycle == 0 ) splits = 25;

			//XXX TODO: Switch to polar coordinate search
			for( dz = 0; dz <= fullrange; dz += fullrange/splits )
			for( dy = -fullrange; dy <= fullrange; dy += fullrange/splits )
			for( dx = -fullrange; dx <= fullrange; dx += fullrange/splits )
			{
				//Specificially adjust one axis at a time, searching for the best.
				memcpy( LighthousePos, bestxyz, sizeof( LighthousePos ) );
				LighthousePos[0] += dx;
				LighthousePos[1] += dy;
				LighthousePos[2] += dz;

				FLT ft;
				//Try refining the search for the best orientation several times.
				ft = RunOpti(0);
				if( ft < beste ) { beste = ft; memcpy( bestxyzrunning, LighthousePos, sizeof( LighthousePos ) ); }

				//printf( "  %f %f %f %f\n", LighthousePos[0], LighthousePos[1], LighthousePos[2], ft );
			}
			memcpy( bestxyz, bestxyzrunning, sizeof( bestxyz ) );

			//Print out the quality of the lock this time.
			FLT dist = sqrt(bestxyz[0]*bestxyz[0] + bestxyz[1]*bestxyz[1] + bestxyz[2]*bestxyz[2]);
			printf( "%f %f %f (%f) = %f\n", bestxyz[0], bestxyz[1], bestxyz[2], dist, beste );
		}

		//Every cycle, tighten up the search area.
		fullrange *= 0.6;
	}

	//Use bestxyz
	memcpy( LighthousePos, bestxyz, sizeof( LighthousePos ) );

	//Optimize the quaternion for lighthouse rotation
	RunOpti(1);

	//STAGE 2: Determine optimal distance from target
	{
		FLT dist = 0.1;
		FLT best_dist = 0;
		FLT best_err = 1e20;
		for( ; dist < 10; dist+=0.01 )
		{
			FLT nrmvect[3];
			normalize3d( nrmvect, bestxyz );
			scale3d( LighthousePos, nrmvect, dist );
			FLT res = RunTest( 0 );
			if( res < best_err )
			{
				best_dist = dist;
				best_err = res;
			}
		}

		//Apply the best res.
		normalize3d( LighthousePos, bestxyz );
		scale3d( LighthousePos, LighthousePos, best_dist );
		printf( "Best distance: %f\n", best_dist );
	}

	//Print out plane accuracies with these settings.
	FLT ft = RunTest(1);
	printf( "Final RMS: %f\n", ft );
}

FLT RunOpti( int print )
{
	int i;
	FLT UsToTarget[3];

	{
		//XXX TODO: We should use several points to determine initial rotation optimization to object.
		//By using only one point the "best" we could be rotating somewhere wrong.  We do use
		//several points for the rotate-about-this-vector rotation in stage 2.

		//Find out where our ray shoots forth from.
		FLT ax = hmd_point_angles[best_hmd_target*2+0];
		FLT ay = hmd_point_angles[best_hmd_target*2+1];

		//NOTE: Inputs may never be output with cross product.
		//Create a fictitious normalized ray.  Imagine the lighthouse is pointed
		//straight in the +z direction, this is the lighthouse ray to the point.
		FLT RayShootOut[3] = { sin(ax), sin(ay), 0 };
		RayShootOut[2] = sqrt( 1 - (RayShootOut[0]*RayShootOut[0] + RayShootOut[1]*RayShootOut[1]) );

		//Find a ray from us to the target point.
		sub3d( UsToTarget, &hmd_points[best_hmd_target*3], LighthousePos );
		normalize3d( UsToTarget, UsToTarget );

		FLT AxisToRotate[3];
		cross3d( AxisToRotate, RayShootOut, UsToTarget );
		//Rotate the lighthouse around this axis to point at the HMD.

		FLT RotateAmount = -acos( dot3d( RayShootOut, UsToTarget ) );  //XXX TODO How to determine if negative.
		quatfromaxisangle( LighthouseQuat, AxisToRotate, RotateAmount ); //Tested, working!
	}

	//Now our lighthouse's ray is pointed at the HMD's dot, but, we need to
	//rotate the lighthouse such that it is oriented optimally.  We do this
	//by finding what the optimal rotation aroud our face would be for all
	//remaining points.
	FLT rotate_radians = 0;
	int points_found_to_rotate = 0;

	float rots[PTS];
	for( i = 0; i < PTS; i++ )
	{
		if( i == best_hmd_target ) continue;
		int xhits = hmd_point_counts[i*2+0];
		int yhits = hmd_point_counts[i*2+1];
		int xyhits = (xhits<yhits)?xhits:yhits;
		if( xyhits < MIN_HITS_FOR_VALID ) continue;

		//Find a point on the object to point at.
		FLT HMDRayPoint[3];
		sub3d( HMDRayPoint, &hmd_points[i*3], LighthousePos );
		normalize3d( HMDRayPoint, HMDRayPoint );

		FLT RayShootOut[3] = { sin(hmd_point_angles[i*2+0]), sin(hmd_point_angles[i*2+1]), 0 };
		RayShootOut[2] = sqrt( 1 - (RayShootOut[0]*RayShootOut[0] + RayShootOut[1]*RayShootOut[1]) );
		//Find the corresponding vector from camera out if it was rotated by "RotateAmount"
		quatrotatevector( RayShootOut, LighthouseQuat, RayShootOut );

		//Figure out rotation to rotate around UsToTarget, from RayShootOut to HMDRayPoint
		FLT SphSurf_Point[3]; //Relative to our fixed point, how much to rotate?
		FLT SphSurf_Ray[3];
		sub3d( SphSurf_Point, HMDRayPoint, UsToTarget  );
		sub3d( SphSurf_Ray,   RayShootOut, UsToTarget  );
		normalize3d( SphSurf_Point, SphSurf_Point );
		normalize3d( SphSurf_Ray, SphSurf_Ray );

		FLT rotate = acos( dot3d( SphSurf_Point, SphSurf_Ray ) );

		//XXX TODO: How to determine if rotate amount should be negative!!!

		if( print ) printf( " %f ", rotate );
		rotate_radians += rotate;
		rots[points_found_to_rotate++] = rotate;
	}

	//Get average barrel rotation
	rotate_radians/=points_found_to_rotate;

	//Find the standard of deviation of our data.
	float stddev = 0.0;
	for( i = 0; i < points_found_to_rotate; i++ )
	{
		float dr = rots[i]-rotate_radians;
		stddev += dr*dr;
	}
	stddev/=points_found_to_rotate;
	stddev = sqrt(stddev);
	if( print ) printf( " = %f\n", stddev );

	quatfromaxisangle( BarrelRotate, UsToTarget, rotate_radians ); //Rotate around barrel
	//quatrotateabout( LighthouseQuat, LighthouseQuat, BarrelRotate ); //Concatenate the rotation into the lighthouse quat rotation.

	return stddev;
}


FLT RunTest( int print )
{
	int k;
	FLT totprob = 0.0;
	int ict = 0;
	for( k = 0; k < PTS*2; k++ )
	{
		if( hmd_point_counts[k] == 0 ) continue;
		int axis = k%2;
		int pt = k/2;
		FLT angle = hmd_point_angles[k];
		if( print ) printf( "%d %d : angle: %f / ", axis, pt, angle );

		//XXX TODO: This is critical.  We need to properly define the planes. 
		FLT plane_normal[3] = { 0, 0, 0 };
		if( axis == 1 )
		{
			plane_normal[1] = cos(angle);
			plane_normal[2] =-sin(angle);
		}
		else
		{
			plane_normal[0] = cos(angle);
			plane_normal[2] =-sin(angle);
		}


		quatrotatevector( plane_normal, LighthouseQuat, plane_normal ); //Rotate plane normal by concatenated rotation.
		quatrotatevector( plane_normal, BarrelRotate, plane_normal ); //Rotate plane normal by concatenated rotation.

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
	for( i = 0; i < PTS; i++ )
	{
		if( hmd_point_counts[i*2+0] < 100 ) hmd_point_counts[i*2+0] = 0;
		if( hmd_point_counts[i*2+1] < 100 ) hmd_point_counts[i*2+1] = 0;

		hmd_point_angles[i*2+0]/=hmd_point_counts[i*2+0];
		hmd_point_angles[i*2+1]/=hmd_point_counts[i*2+1];
		hmd_point_angles[i*2+0] = (hmd_point_angles[i*2+0] - 200000) / 200000 * 3.1415926535/2; // Is it really 800,000 ticks per revolution?
		hmd_point_angles[i*2+1] = (hmd_point_angles[i*2+1] - 200000) / 200000 * 3.1415926535/2; // Is it really 800,000 ticks per revolution?
	}

	int targpd;
	int maxhits = 0;

	for( targpd = 0; targpd < PTS; targpd++ )
	{
		int hits = hmd_point_counts[targpd*2+0];
		if( hits > hmd_point_counts[targpd*2+1] ) hits = hmd_point_counts[targpd*2+1];
		//Need an X and a Y lock.  

		if( hits > maxhits ) { maxhits = hits; best_hmd_target = targpd; }
	}
	if( maxhits < MIN_HITS_FOR_VALID )
	{
		fprintf( stderr, "Error: Not enough data for a primary fix.\n" );
	}



	return 0;
}
