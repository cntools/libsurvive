#include "survive_cal.h"
#include <math.h>
#include <string.h>
#include "linmath.h"
#include <survive.h>
#include <stdio.h>
#include <stdlib.h>
#include <dclapack.h>
#include <linmath.h>

typedef struct
{
	int something;
	//Stuff
} DummyData;


static FLT RunOpti( SurviveObject * so, PoserDataFullScene * fs, int lh, int print, FLT * LighthousePos, FLT * LighthouseQuat );

int PoserCharlesSlow( SurviveObject * so, PoserData * pd )
{
	PoserType pt = pd->pt;
	SurviveContext * ctx = so->ctx;
	DummyData * dd = so->PoserData;

	if( !dd ) so->PoserData = dd = malloc( sizeof( DummyData ) );

	switch( pt )
	{
	case POSERDATA_IMU:
	{
		PoserDataIMU * imu = (PoserDataIMU*)pd;
		//printf( "IMU:%s (%f %f %f) (%f %f %f)\n", so->codename, imu->accel[0], imu->accel[1], imu->accel[2], imu->gyro[0], imu->gyro[1], imu->gyro[2] );
		break;
	}
	case POSERDATA_LIGHT:
	{
		PoserDataLight * l = (PoserDataLight*)pd;
		//printf( "LIG:%s %d @ %f rad, %f s (AC %d) (TC %d)\n", so->codename, l->sensor_id, l->angle, l->length, l->acode, l->timecode );
		break;
	}
	case POSERDATA_FULL_SCENE:
	{
		PoserDataFullScene * fs = (PoserDataFullScene*)pd;



		int p;
		FLT * hmd_points  = so->sensor_locations;

		for( p = 0; p < so->nr_locations; p++ )
		{
			printf( "%f %f %f\n", hmd_points[p*3+0], hmd_points[p*3+1], hmd_points[p*3+2] );
		}


		int lh, cycle;
		FLT dz, dy, dx;
		for( lh = 0; lh < 2; lh++ )
		{
			FLT beste = 1e20;

			FLT LighthousePos[3];
			FLT LighthouseQuat[4];

			LighthousePos[0] = 0;
			LighthousePos[1] = 0;
			LighthousePos[2] = 0;
			LighthouseQuat[0] = 1;
			LighthouseQuat[1] = 0;
			LighthouseQuat[2] = 0;
			LighthouseQuat[3] = 0;

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
					for( dz = -fullrange; dz < fullrange; dz += fullrange/splits )
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
						ft = RunOpti(so, fs, lh, 0, LighthousePos, LighthouseQuat);
						if( cycle == 0 )
						{
							FLT sk = ft*10.;
							if( sk > 1 ) sk = 1;
							uint8_t cell = (uint8_t)((1.0 - sk) * 255);
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

			if( beste > 0.1 )
			{
				//Error too high
				SV_ERROR( "LH: %d / Best E %f Error too high\n", lh, beste );
				return -1;
			}

			RunOpti(so, fs, lh, 1, LighthousePos, LighthouseQuat);

			ctx->bsd[lh].PositionSet = 1;
			copy3d( ctx->bsd[lh].Pose.Pos, LighthousePos );
			quatcopy( ctx->bsd[lh].Pose.Rot, LighthouseQuat );			
#define ALT_COORDS

#ifdef ALT_COORDS
			so->FromLHPose[lh].Pos[0] = LighthousePos[0];
			so->FromLHPose[lh].Pos[1] = LighthousePos[1];
			so->FromLHPose[lh].Pos[2] = LighthousePos[2];
			so->FromLHPose[lh].Rot[0] =-LighthouseQuat[0];
			so->FromLHPose[lh].Rot[1] = LighthouseQuat[1];
			so->FromLHPose[lh].Rot[2] = LighthouseQuat[2];
			so->FromLHPose[lh].Rot[3] = LighthouseQuat[3];

			quatrotatevector( so->FromLHPose[lh].Pos, so->FromLHPose[lh].Rot, so->FromLHPose[lh].Pos );
#else
			so->FromLHPose[lh].Pos[0] = LighthousePos[0];
			so->FromLHPose[lh].Pos[1] = LighthousePos[1];
			so->FromLHPose[lh].Pos[2] = LighthousePos[2];
			so->FromLHPose[lh].Rot[0] = LighthouseQuat[0];
			so->FromLHPose[lh].Rot[1] = LighthouseQuat[1];
			so->FromLHPose[lh].Rot[2] = LighthouseQuat[2];
			so->FromLHPose[lh].Rot[3] = LighthouseQuat[3];
#endif

			const FLT rt[4] = {0, 0, 1, 0};
			FLT tmp[4];
			quatrotateabout(tmp, so->ctx->bsd[lh].Pose.Rot, rt);
			memcpy(so->ctx->bsd[lh].Pose.Rot, tmp, sizeof(FLT) * 4);
		}

		return 0;
	}
	case POSERDATA_DISASSOCIATE:
	{
		free( dd );
		so->PoserData = 0;
		//printf( "Need to disassociate.\n" );
		break;
	}
	}
	return -1;
}


REGISTER_LINKTIME( PoserCharlesSlow );



static FLT RunOpti( SurviveObject * hmd, PoserDataFullScene * fs, int lh, int print, FLT * LighthousePos, FLT * LighthouseQuat )
{
	int i, p;
	FLT UsToTarget[3];
	FLT LastUsToTarget[3];
	FLT mux = .9;
	quatsetnone( LighthouseQuat );
	FLT * hmd_points  = hmd->sensor_locations;
	FLT * hmd_normals = hmd->sensor_normals;
	int dpts = hmd->nr_locations;

	int first = 1, second = 0;

	//First check to see if this is a valid viewpoint.
	//If a sensor is pointed away from where we are testing a possible lighthouse position.
	//BUT We get data from that light house, then we KNOW this is not a possible
	//lighthouse position.
	for( p = 0; p < dpts; p++ )
	{
		int dataindex = p*(2*NUM_LIGHTHOUSES)+lh*2;
		if( fs->lengths[p][lh][0] < 0 || fs->lengths[p][lh][1] < 0 ) continue;
		FLT me_to_dot[3];
		sub3d( me_to_dot, LighthousePos, &hmd_points[p*3] );
		FLT dot = dot3d( &hmd_normals[p*3], me_to_dot );
		if( dot < -.01 ) { 	return 1000; }
	}
	int iters = 6;

	//Iterate over a refinement of the quaternion that constitutes the
	//lighthouse.
	for( i = 0; i < iters; i++ )
	{
		first = 1;
		for( p = 0; p < dpts; p++ )
		{
			int dataindex = p*(2*NUM_LIGHTHOUSES)+lh*2;
			if( fs->lengths[p][lh][0] < 0 || fs->lengths[p][lh][1] < 0 ) continue;

			//Find out where our ray shoots forth from.
			FLT ax = fs->angles[p][lh][0];
			FLT ay = fs->angles[p][lh][1];
			//NOTE: Inputs may never be output with cross product.
			//Create a fictitious normalized ray.  Imagine the lighthouse is pointed
			//straight in the +z direction, this is the lighthouse ray to the point.
			FLT RayShootOut[3] = { sin(ax), sin(ay), 0 };
			RayShootOut[2] = sqrt( 1 - (RayShootOut[0]*RayShootOut[0] + RayShootOut[1]*RayShootOut[1]) );
			FLT RayShootOutWorld[3];

			quatnormalize( LighthouseQuat, LighthouseQuat );
			//Rotate that ray by the current rotation estimation.
			quatrotatevector( RayShootOutWorld, LighthouseQuat, RayShootOut );

			//Find a ray from us to the target point.
			sub3d( UsToTarget, &hmd_points[p*3], LighthousePos );
			if( magnitude3d( UsToTarget ) < 0.0001 ) { continue; }
			normalize3d( UsToTarget, UsToTarget );

			FLT RotatedLastUs[3];
			quatnormalize( LighthouseQuat, LighthouseQuat );
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
				quatnormalize( ConcatQuat, ConcatQuat );
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
				quatnormalize( ConcatQuat, ConcatQuat );
			}


			quatnormalize( ConcatQuat, ConcatQuat );
			quatnormalize( LighthouseQuat, LighthouseQuat );
			quatrotateabout( LighthouseQuat, ConcatQuat, LighthouseQuat );  //Checked.  This appears to be 

			mux = mux * 0.94;
			if( second ) { second = 0; }
			if( first ) { first = 0; second = 1; }
			copy3d( LastUsToTarget, RayShootOutWorld );
		}
	}

	//Step 2: Determine error.
	FLT errorsq = 0.0;
	int count = 0;
	for( p = 0; p < dpts; p++ )
	{
		int dataindex = p*(2*NUM_LIGHTHOUSES)+lh*2;
		if( fs->lengths[p][lh][0] < 0 || fs->lengths[p][lh][1] < 0 ) continue;

		//Find out where our ray shoots forth from.
		FLT ax = fs->angles[p][lh][0];
		FLT ay = fs->angles[p][lh][1];
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
		//if( print ) printf( "%f (%d(%d/%d))\n", dist, p, cd->ctsweeps[dataindex+0], cd->ctsweeps[dataindex+1] );
	}
	if( print ) printf( " = %f\n", sqrt( errorsq ) );
	return sqrt(errorsq);
}
