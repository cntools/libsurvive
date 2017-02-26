#include "survive_cal.h"
#include <math.h>
#include <string.h>
#include "linmath.h"

#define MAX_CHECKS 40000
#define MIN_HITS_FOR_VALID 10


FLT static RunOpti(  struct SurviveCalData * cd, int lh, int print, FLT * LighthousePos, FLT * LighthouseQuat );

//Values used for RunTest()

int survive_cal_lhfind( struct SurviveCalData * cd )
{
	struct SurviveContext * ctx = cd->ctx;
	int cycle, i;
	int lh = 0;
	FLT dx, dy, dz;

	//Use the following:
	//	FLT avgsweeps[MAX_CAL_PT_DAT];
	//	FLT avglens[MAX_CAL_PT_DAT];
	//	FLT stdsweeps[MAX_CAL_PT_DAT];
	//	FLT stdlens[MAX_CAL_PT_DAT];
	//	int ctsweeps[MAX_CAL_PT_DAT];
	//
	// Check your solution against point: senid_of_checkpt's data.



	for( lh = 0; lh < 2; lh++ )
	{
		FLT beste = 1e20;

		FLT LighthousePos[3];
		FLT LighthouseQuat[4];

		LighthousePos[0] = 0;
		LighthousePos[1] = 0;
		LighthousePos[2] = 0;
		LighthouseQuat[0] = 1;
		LighthouseQuat[1] = 1;
		LighthouseQuat[2] = 1;
		LighthouseQuat[3] = 1;

		FLT bestxyz[3];
		memcpy( bestxyz, LighthousePos, sizeof( LighthousePos ) );

		//STAGE1 1: Detemine vectoral position from lighthouse to target.  Does not determine lighthouse-target distance.
		//This also is constantly optimizing the lighthouse quaternion for optimal spotting.
		FLT fullrange = 5; //Maximum search space for positions.  (Relative to HMD)
	

		//Sweep whole area 30 times
		for( cycle = 0; cycle < 30; cycle ++ )
		{

			//Adjust position, one axis at a time, over and over until we zero in.
			{
				FLT bestxyzrunning[3];
				beste = 1e20;

				FILE * f;
				if( cycle == 0 )
				{
					char filename[1024];
					sprintf( filename, "calinfo/%d_lighthouse.dat", lh );
					f = fopen( filename, "wb" );
				}

				//We split the space into this many groups (times 2) and
				//if we're on the first cycle, we want to do a very linear
				//search.  As we refine our search we can then use a more
				//binary search technique.
				FLT splits = 4;
				if( cycle == 0 ) splits = 32;
				if( cycle == 1 ) splits = 13;
				if( cycle == 2 ) splits = 10;
				if( cycle == 3 ) splits = 8;
				if( cycle == 4 ) splits = 5;

				//Wwe search throug the whole space.
				for( dz = 0; dz < fullrange; dz += fullrange/splits )
				for( dy = -fullrange; dy < fullrange; dy += fullrange/splits )
				for( dx = -fullrange; dx < fullrange; dx += fullrange/splits )
				{
					//Specificially adjust one axis at a time, searching for the best.
					memcpy( LighthousePos, bestxyz, sizeof( LighthousePos ) );
					LighthousePos[0] += dx;  //These are adjustments to the "best" from last frame.
					LighthousePos[1] += dy;
					LighthousePos[2] += dz;

					FLT ft;
					//Try refining the search for the best orientation several times.
					ft = RunOpti(cd, lh, 0, LighthousePos, LighthouseQuat);
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

		if( beste > 0.005 )
		{
			//Error too high
			SV_ERROR( "LH: %d / Best E %f Error too high\n", lh, beste );
			return -1;
		}

		cd->ctx->bsd[lh].PositionSet = 1;
		copy3d( cd->ctx->bsd[lh].Position, LighthousePos );
		quatcopy( cd->ctx->bsd[lh].Quaternion, LighthouseQuat );
	}

	return 0; //Return 0 if success.
}






static FLT RunOpti( struct SurviveCalData * cd, int lh, int print, FLT * LighthousePos, FLT * LighthouseQuat )
{
	int i, p;
	FLT UsToTarget[3];
	FLT LastUsToTarget[3];
	FLT mux = .9;
	quatsetnone( LighthouseQuat );
	struct SurviveObject * hmd = cd->hmd;
	FLT * hmd_points  = hmd->sensor_locations;
	FLT * hmd_normals = hmd->sensor_normals;

	int first = 1, second = 0;

	//First check to see if this is a valid viewpoint.
	//If a sensor is pointed away from where we are testing a possible lighthouse position.
	//BUT We get data from that light house, then we KNOW this is not a possible
	//lighthouse position.
	for( p = 0; p < 32; p++ )
	{
		int dataindex = p*(2*NUM_LIGHTHOUSES)+lh*2;
		if( cd->ctsweeps[dataindex+0] < MIN_HITS_FOR_VALID || cd->ctsweeps[dataindex+1] < MIN_HITS_FOR_VALID ) continue;
		FLT me_to_dot[3];
		sub3d( me_to_dot, LighthousePos, &hmd_points[p*3] );
		float dot = dot3d( &hmd_normals[p*3], me_to_dot );
		if( dot < -.01 ) { 	return 1000; }
	}
	int iters = 6;

	//Iterate over a refinement of the quaternion that constitutes the
	//lighthouse.
	for( i = 0; i < iters; i++ )
	{
		first = 1;
		for( p = 0; p < 32; p++ )
		{
			int dataindex = p*(2*NUM_LIGHTHOUSES)+lh*2;
			if( cd->ctsweeps[dataindex+0] < MIN_HITS_FOR_VALID || cd->ctsweeps[dataindex+1] < MIN_HITS_FOR_VALID ) continue;

			//Find out where our ray shoots forth from.
			FLT ax = cd->avgsweeps[dataindex+0];
			FLT ay = cd->avgsweeps[dataindex+1];

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
		int dataindex = p*(2*NUM_LIGHTHOUSES)+lh*2;
		if( cd->ctsweeps[dataindex+0] < MIN_HITS_FOR_VALID || cd->ctsweeps[dataindex+1] < MIN_HITS_FOR_VALID ) continue;

		//Find out where our ray shoots forth from.
		FLT ax = cd->avgsweeps[dataindex+0];
		FLT ay = cd->avgsweeps[dataindex+1];
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
		if( print ) printf( "%f (%d(%d/%d))\n", dist, p, cd->ctsweeps[dataindex+0], cd->ctsweeps[dataindex+1] );
	}
	if( print ) printf( " = %f\n", sqrt( errorsq ) );
	return sqrt(errorsq);
}

