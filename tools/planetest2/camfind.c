#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#define PTS 32
#define MAX_CHECKS 40000
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

			FILE * f;
			if( cycle == 0 )
			{
				f = fopen( "raw_data_lighthouse.dat", "wb" );
			}
			FLT splits = 4;
			if( cycle == 0 ) splits = 32;
			if( cycle == 1 ) splits = 13;
			if( cycle == 2 ) splits = 10;
			if( cycle == 3 ) splits = 8;
			if( cycle == 4 ) splits = 5;

			for( dz = 0; dz < fullrange; dz += fullrange/splits )
			for( dy = -fullrange; dy < fullrange; dy += fullrange/splits )
			for( dx = -fullrange; dx < fullrange; dx += fullrange/splits )
			{
				//Specificially adjust one axis at a time, searching for the best.
				memcpy( LighthousePos, bestxyz, sizeof( LighthousePos ) );
				LighthousePos[0] += dx;
				LighthousePos[1] += dy;
				LighthousePos[2] += dz;

				FLT ft;
				//Try refining the search for the best orientation several times.
				ft = RunOpti(0);
				if( cycle == 0 )
				{
					float sk = ft*10.;
					if( sk > 1 ) sk = 1;
					uint8_t cell = (1.0 - sk) * 255;
					FLT epsilon = 0.1;

					if( dz == 0 ) { /* Why is dz special? ? */
					  if ( dx > -epsilon && dx < epsilon )
					    cell =  255;
					  if ( dy > -epsilon && dy < epsilon )
                                            cell = 128;
					}

					fprintf( f, "%c", cell );
				}

				if( ft < beste ) { beste = ft; memcpy( bestxyzrunning, LighthousePos, sizeof( LighthousePos ) ); }
			}

			if( cycle == 0 )
			{
				fclose( f );
			}
			memcpy( bestxyz, bestxyzrunning, sizeof( bestxyz ) );

			//Print out the quality of the lock this time.
			FLT dist = sqrt(bestxyz[0]*bestxyz[0] + bestxyz[1]*bestxyz[1] + bestxyz[2]*bestxyz[2]);
			printf( "%f %f %f (%f) = %f\n", bestxyz[0], bestxyz[1], bestxyz[2], dist, beste );
		}

		//Every cycle, tighten up the search area.
		fullrange *= 0.25;
	}

	//Use bestxyz
	memcpy( LighthousePos, bestxyz, sizeof( LighthousePos ) );

	//Optimize the quaternion for lighthouse rotation
	RunOpti(1);

	printf( "Best Quat: %f %f %f %f\n", PFFOUR( LighthouseQuat ) );

	//Print out plane accuracies with these settings.
	FLT ft = RunTest(1);
	printf( "Final RMS: %f\n", ft );
}

FLT RunOpti( int print )
{
	int i, p;
	FLT UsToTarget[3];
	FLT LastUsToTarget[3];
	FLT mux = .9;
	quatsetnone( LighthouseQuat );

	int first = 1, second = 0;

	//First check to see if this is a valid viewpoint.

	for( p = 0; p < 32; p++ )
	{
		if( hmd_point_counts[p*2+0] < MIN_HITS_FOR_VALID || hmd_point_counts[p*2+1] < MIN_HITS_FOR_VALID ) continue;
		FLT me_to_dot[3];
		sub3d( me_to_dot, LighthousePos, &hmd_points[p*3] );
		float dot = dot3d( &hmd_norms[p*3], me_to_dot );
		if( dot < -.01 ) { 	return 1000; }
	}

	int iters = 6;

	for( i = 0; i < iters; i++ )
	{
		first = 1;
		for( p = 0; p < 32; p++ )
		{
			if( hmd_point_counts[p*2+0] < MIN_HITS_FOR_VALID || hmd_point_counts[p*2+1] < MIN_HITS_FOR_VALID ) continue;

			//Find out where our ray shoots forth from.
			FLT ax = hmd_point_angles[p*2+0];
			FLT ay = hmd_point_angles[p*2+1];

			//NOTE: Inputs may never be output with cross product.
			//Create a fictitious normalized ray.  Imagine the lighthouse is pointed
			//straight in the +z direction, this is the lighthouse ray to the point.
			FLT RayShootOut[3] = { sin(ax), sin(ay), 0 };
			RayShootOut[2] = sqrt( 1 - (RayShootOut[0]*RayShootOut[0] + RayShootOut[1]*RayShootOut[1]) );
			FLT RayShootOutWorld[3];

			//Rotate that ray by the current rotation estimation.
			quatrotatevector( RayShootOutWorld, LighthouseQuat, RayShootOut );

			//Find a ray from us to the target point.
			sub3d( UsToTarget, &hmd_points[p*3], LighthousePos );
			if( magnitude3d( UsToTarget ) < 0.0001 ) { continue; }
			normalize3d( UsToTarget, UsToTarget );

			FLT RotatedLastUs[3];
			quatrotatevector( RotatedLastUs, LighthouseQuat, LastUsToTarget );

			//Rotate the lighthouse around this axis to point at the HMD.
			//If it's the first time, the axis is synthesized, if it's after that, use most recent point.
			FLT ConcatQuat[4];
			FLT AxisToRotate[3];
			if( first )
			{
				cross3d( AxisToRotate, RayShootOutWorld, UsToTarget );
				if( magnitude3d(AxisToRotate) < 0.0001 ) break;
				normalize3d( AxisToRotate, AxisToRotate );
				//Don't need to worry about being negative, cross product will fix it.
				FLT RotateAmount = anglebetween3d( RayShootOutWorld, UsToTarget );
				quatfromaxisangle( ConcatQuat, AxisToRotate, RotateAmount );
			}
			else
			{
				FLT Target[3];
				FLT Actual[3];

				copy3d( AxisToRotate, LastUsToTarget );
				//Us to target = normalized ray from us to where we should be.
				//RayShootOut = where we would be pointing.
				sub3d( Target, UsToTarget, AxisToRotate );  //XXX XXX XXX WARNING THIS MESSES STUFF UP.
				sub3d( Actual, RayShootOutWorld, AxisToRotate );
				if( magnitude3d( Actual ) < 0.0001 || magnitude3d( Target ) < 0.0001 ) { continue; }
				normalize3d( Target, Target );
				normalize3d( Actual, Actual );

				cross3d( AxisToRotate, Actual, Target );  //XXX Check: AxisToRotate should be equal to LastUsToTarget.
				if( magnitude3d( AxisToRotate ) < 0.000001 ) { continue; }
				normalize3d( AxisToRotate,AxisToRotate );

				//printf( "%f %f %f === %f %f %f : ", PFTHREE( AxisToRotate ), PFTHREE( LastUsToTarget ) );
				FLT RotateAmount = anglebetween3d( Actual, Target ) * mux;
				//printf( "FA: %f (O:%f)\n", acos( dot3d( Actual, Target ) ), RotateAmount );
				quatfromaxisangle( ConcatQuat, AxisToRotate, RotateAmount );
			}

			quatrotateabout( LighthouseQuat, ConcatQuat, LighthouseQuat );  //Chekcked.  This appears to be 

			mux = mux * 0.94;
			if( second ) { second = 0; }
			if( first ) { first = 0; second = 1; }
			copy3d( LastUsToTarget, RayShootOutWorld );
		}
	}

	//Step 2: Determine error.
	float errorsq = 0.0;
	int count = 0;
	for( p = 0; p < 32; p++ )
	{
		if( hmd_point_counts[p*2+0] < MIN_HITS_FOR_VALID || hmd_point_counts[p*2+1] < MIN_HITS_FOR_VALID ) continue;

		//Find out where our ray shoots forth from.
		FLT ax = hmd_point_angles[p*2+0];
		FLT ay = hmd_point_angles[p*2+1];
		FLT RayShootOut[3] = { sin(ax), sin(ay), 0 };
		RayShootOut[2] = sqrt( 1 - (RayShootOut[0]*RayShootOut[0] + RayShootOut[1]*RayShootOut[1]) );

		//Rotate that ray by the current rotation estimation.
		quatrotatevector( RayShootOut, LighthouseQuat, RayShootOut );

		//Point-line distance.
		//Line defined by LighthousePos & Direction: RayShootOut

		//Find a ray from us to the target point.
		sub3d( UsToTarget, &hmd_points[p*3], LighthousePos );
		FLT xproduct[3];
		cross3d( xproduct, UsToTarget, RayShootOut );
		FLT dist = magnitude3d( xproduct );
		errorsq += dist*dist;
		if( print ) printf( "%f (%d(%d/%d))\n", dist, p, hmd_point_counts[p*2+0], hmd_point_counts[p*2+1] );
	}
	if( print ) printf( " = %f\n", sqrt( errorsq ) );
	return sqrt(errorsq);
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
		if( print ) printf( "%3d %3d : angle: %10f / ", axis, pt, angle );

		//XXX TODO: This is critical.  We need to properly define the planes. 
		FLT plane_normal[3] = { 0, 0, 0 };
		if( axis == 0 )
		{
			plane_normal[0] = cos(angle);
			plane_normal[2] =-sin(angle);
		}
		else
		{
			plane_normal[1] = cos(angle);
			plane_normal[2] =-sin(angle);
		}


		quatrotatevector( plane_normal, LighthouseQuat, plane_normal ); //Rotate plane normal by concatenated rotation.

		//plane_normal is our normal / LighthousePos is our point.  
		FLT w0[] = { hmd_points[pt*3+0], hmd_points[pt*3+1], hmd_points[pt*3+2] };
		FLT d = -(plane_normal[0] * LighthousePos[0] + plane_normal[1] * LighthousePos[1] + plane_normal[2] * LighthousePos[2]);
		FLT D =   plane_normal[0] * w0[0]            + plane_normal[1] * w0[1]            + plane_normal[2] * w0[2] + d;
			//Point line distance assuming ||normal|| = 1.

		FLT delta[] = { LighthousePos[0]-hmd_points[pt*3+0],LighthousePos[1]-hmd_points[pt*3+1],LighthousePos[2]-hmd_points[pt*3+2] };
		FLT dot = delta[0] * hmd_norms[pt*3+0] + delta[1] * hmd_norms[pt*3+1] + delta[2] * hmd_norms[pt*3+2];
//		if( dot < -0.04 ) totprob+=10000000000;
		if( print ) printf( " %10f %10f N:%f\n", d, D, dot );
		totprob += (D*D);  //Calculate RMS distance of incorrectitude.

		ict++;
	}

	if( print )
	{
		int p;
		printf( "POS:  %f %f %f %f\n", PFFOUR(LighthousePos ) );
		printf( "QUAT: %f %f %f %f\n", PFFOUR(LighthouseQuat ) );
		printf( "Imagespace comparison:\n" );
		for( p = 0; p < 32; p++ )
		{
			if( hmd_point_counts[p*2+0] < MIN_HITS_FOR_VALID || hmd_point_counts[p*2+1] < MIN_HITS_FOR_VALID ) continue;

			FLT us_to_targ[3];
			sub3d( us_to_targ, &hmd_points[p*3] , LighthousePos );

			//Unrotate us_to_targ.
			FLT unrotate[4];
			quatgetconjugate( unrotate, LighthouseQuat );

			quatrotatevector( us_to_targ, unrotate, us_to_targ );
			normalize3d( us_to_targ, us_to_targ );
			FLT x = asin( us_to_targ[0] );
			FLT y = asin( us_to_targ[1] );

			printf( "%f %f %f %f\n", hmd_point_angles[p*2+0], hmd_point_angles[p*2+1], x, y );
		}
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
	f = fopen( "livestream_test_2_x_axis.csv", "r" );
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

