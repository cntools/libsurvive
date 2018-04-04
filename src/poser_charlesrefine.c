//EXPERIMENTAL DRIVER - DO NOT USE

#include <poser.h>
#include <survive.h>
#include <survive_reproject.h>

#include "epnp/epnp.h"
#include "linmath.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define MAX_PT_PER_SWEEP 32


typedef struct
{
	int sweepaxis;
	int sweeplh;
	FLT normal_at_errors[MAX_PT_PER_SWEEP][3]; //Value is actually normalized, not just normal to sweep plane.
	FLT quantity_errors[MAX_PT_PER_SWEEP];
	FLT angles_at_pts[MAX_PT_PER_SWEEP];
	SurvivePose object_pose_at_hit[MAX_PT_PER_SWEEP];
	uint8_t sensor_ids[MAX_PT_PER_SWEEP];
	int ptsweep;
} CharlesPoserData;



int PoserCharlesRefine(SurviveObject *so, PoserData *pd) {
	CharlesPoserData * dd = so->PoserData;
	if( !dd ) so->PoserData = dd = calloc( sizeof(CharlesPoserData), 1 );

	SurviveSensorActivations *scene = &so->activations;
	switch (pd->pt) {
	case POSERDATA_IMU: {
		// Really should use this...
		PoserDataIMU *imuData = (PoserDataIMU *)pd;


		//TODO: Actually do Madgwick's algorithm
		LinmathQuat	applymotion;
		const SurvivePose * object_pose = &so->OutPose;
		imuData->gyro[0] *= -0.0005;
		imuData->gyro[1] *= -0.0005;
		imuData->gyro[2] *= 0.0005;
		quatfromeuler( applymotion, imuData->gyro );
		//printf( "%f %f %f\n", imuData->gyro [0], imuData->gyro [1], imuData->gyro [2] );
		SurvivePose object_pose_out;
		quatrotateabout(object_pose_out.Rot, object_pose->Rot, applymotion );
		copy3d( object_pose_out.Pos, object_pose->Pos );
		PoserData_poser_raw_pose_func(pd, so, 0, &object_pose_out);

		return 0;
	}
	case POSERDATA_LIGHT: {
		int i;
		PoserDataLight *ld = (PoserDataLight *)pd;
		int lhid = ld->lh;
		int senid = ld->sensor_id;
		BaseStationData * bsd = &so->ctx->bsd[ld->lh];
		if( !bsd->PositionSet ) break;
		SurvivePose * lhp = &bsd->Pose;
		FLT angle = ld->angle;
		int sensor_id = ld->sensor_id;
		int axis = dd->sweepaxis;
		const SurvivePose * object_pose = &so->OutPose;
		dd->sweeplh = lhid;

		//FOR NOW, drop LH1.
		//if( lhid == 1 ) break;


//		const FLT * sensor_normal = &so->sensor_normals[senid*3];
//		FLT sensor_normal_worldspace[3];
//		ApplyPoseToPoint(sensor_normal_worldspace,   object_pose, sensor_inpos);

		const FLT * sensor_inpos = &so->sensor_locations[senid*3];
		FLT sensor_position_worldspace[3];
		//XXX Once I saw this get pretty wild (When in playback)
		//I had to invert the values of sensor_inpos.  Not sure why.
		ApplyPoseToPoint(sensor_position_worldspace, object_pose, sensor_inpos);


		//printf( "%f %f %f  == > %f %f %f\n", sensor_inpos[0], sensor_inpos[1], sensor_inpos[2], sensor_position_worldspace[0], sensor_position_worldspace[1], sensor_position_worldspace[2] );
		// = sensor position, relative to lighthouse center.
		FLT sensorpos_rel_lh[3];	
		sub3d( sensorpos_rel_lh, sensor_position_worldspace, lhp->Pos );

		//Next, define a normal in global space of the plane created by the sweep hit.
		//Careful that this must be normalized.
		FLT sweep_normal[3];

		//If 1, the "y" axis. //XXX Check me.
		if( axis )  		//XXX Just FYI this should include account for skew
		{
			sweep_normal[0] = 0;
			sweep_normal[1] = cos(angle );
			sweep_normal[2] = sin( angle );
			//printf( "+" );
		}
		else
		{
			sweep_normal[0] = cos( angle );
			sweep_normal[1] = 0;
			sweep_normal[2] = -sin( angle );
			//printf( "-" );
		}

		//Need to apply the lighthouse's transformation to the sweep's normal.
		quatrotatevector( sweep_normal, lhp->Rot, sweep_normal);

		//Compute point-line distance between sensorpos_rel_lh and the plane defined by sweep_normal.
		//Do this by projecting sensorpos_rel_lh (w) onto sweep_normal (v).
		//You can do this by |v dot w| / |v| ... But we know |v| is 1. So...
		FLT dist = dot3d( sensorpos_rel_lh, sweep_normal );

		if( (i = dd->ptsweep) < MAX_PT_PER_SWEEP )
		{
			memcpy( dd->normal_at_errors[i], sweep_normal, sizeof(FLT)*3 );
			dd->quantity_errors[i] = dist;
			dd->angles_at_pts[i] = angle;
			dd->sensor_ids[i] = sensor_id;
			memcpy( &dd->object_pose_at_hit[i], object_pose, sizeof(SurvivePose) );
			dd->ptsweep++;
		}

		return 0;
	}

	case POSERDATA_SYNC: {
		PoserDataLight *l = (PoserDataLight *)pd;
		int lhid = l->lh;


		//you can get sweepaxis and sweeplh.
		if( dd->ptsweep )
		{
			int i;
			int lhid = dd->sweeplh;
			int axis = dd->sweepaxis;
			int pts = dd->ptsweep;
			const SurvivePose * object_pose = &so->OutPose; //XXX TODO Should pull pose from approximate time when LHs were scanning it.

			BaseStationData * bsd = &so->ctx->bsd[lhid];
			SurvivePose * lh_pose = &bsd->Pose;

			int validpoints = 0;
			int ptvalid[MAX_PT_PER_SWEEP];
			FLT avgerr = 0.0;
			FLT vec_correct[3] = { 0., 0. , 0. };
			FLT avgang = 0.0;

//Tunable parameters:
#define MIN_HIT_QUALITY 0.5			//Determines which hits to cull.
#define HIT_QUALITY_BASELINE 0.0001	//Determines which hits to cull.  Actually SQRT(baseline) if 0.0001, it is really 1cm

#define CORRECT_LATERAL_POSITION_COEFFICIENT 0.8		//Explodes if you exceed 1.0
#define CORRECT_TELESCOPTION_COEFFICIENT 8.0			//Converges even as high as 10.0 and doesn't explode.
#define CORRECT_ROTATION_COEFFICIENT 1.0 //This starts to fall apart above 5.0, but for good reason.  It is amplified by the number of points seen.
#define ROTATIONAL_CORRECTION_MAXFORCE 0.10

			//Step 1: Determine standard of deviation, and average in order to
			// drop points that are likely in error.
			{
				//Calculate average
				FLT avgerr_orig = 0.0;
				FLT stddevsq = 0.0;
				for( i = 0; i < pts; i++ )
					avgerr_orig += dd->quantity_errors[i];
				avgerr_orig/=pts;

				//Calculate standard of deviation.
				for( i = 0; i < pts; i++ )
				{
					FLT diff = dd->quantity_errors[i]-avgerr_orig;
					stddevsq += diff*diff;
				}
				stddevsq/=pts;

				for( i = 0; i < pts; i++ )
				{
					FLT err = dd->quantity_errors[i];
					FLT diff = err-avgerr_orig;
					diff *= diff;
					int isptvalid = (diff * MIN_HIT_QUALITY <= stddevsq + HIT_QUALITY_BASELINE)?1:0;
					ptvalid[i] = isptvalid;
					if( isptvalid )
					{
						avgang += dd->angles_at_pts[i];
						avgerr += err;
						validpoints ++;
					}
				}
				avgang /= validpoints;
				avgerr /= validpoints;
			}

			//Step 2: Determine average lateral error.
			//We can actually always perform this operation.  Even with only one point.
			{
				FLT avg_err[3] = { 0, 0, 0 }; //Positional error.
				for( i = 0; i < pts; i++ )
				{
					if( !ptvalid[i] ) continue;
					FLT * nrm = dd->normal_at_errors[i];
					FLT err = dd->quantity_errors[i];
					avg_err[0] = avg_err[0] + nrm[0] * err;
					avg_err[1] = avg_err[1] + nrm[1] * err;
					avg_err[2] = avg_err[2] + nrm[2] * err;
				}

				//NOTE: The "avg_err" is not geometrically centered.  This is actually
				//probably okay, since if you have sevearl data points to one side, you
				//can probably trust that more.
				scale3d(avg_err, avg_err, 1./validpoints);

				//We have "Average error" now.  A vector in worldspace.
				//This can correct for lateral error, but not distance from camera.

				//XXX TODO: Should we check to see if we only have one or
				//two points to make sure the error on this isn't unusually high?
				//If calculated error is unexpectedly high, then we should probably
				//Not apply the transform.
				scale3d( avg_err, avg_err, -CORRECT_LATERAL_POSITION_COEFFICIENT );
				add3d( vec_correct, vec_correct, avg_err );
			}

			//Step 3: Control telecoption from lighthouse.
			// we need to find out what the weighting is to determine "zoom"
			if( validpoints > 1 )	//Can't correct "zoom" with only one point.
			{
				FLT zoom = 0.0;
				FLT rmsang = 0.0;
				for( i = 0; i < pts; i++ )
				{
					if( !ptvalid[i] ) continue;
					FLT delang = dd->angles_at_pts[i] - avgang;
					FLT delerr = dd->quantity_errors[i] - avgerr;
					if( axis ) delang *= -1;	//Flip sign on alternate axis because it's measured backwards.
					zoom += delerr * delang;
					rmsang += delang * delang;
				}

				//Control into or outof lighthouse.
				//XXX Check to see if we need to sqrt( the rmsang), need to check convergance behavior close to lighthouse.
				//This is a questionable step.
 				zoom /= sqrt(rmsang);

				zoom *= CORRECT_TELESCOPTION_COEFFICIENT;

				FLT veccamalong[3];
				sub3d( veccamalong, lh_pose->Pos, object_pose->Pos );
				normalize3d( veccamalong, veccamalong );
				scale3d( veccamalong, veccamalong, zoom );
				add3d( vec_correct, veccamalong, vec_correct );
			}


			SurvivePose object_pose_out;
			add3d(object_pose_out.Pos, vec_correct, object_pose->Pos);

			quatcopy( object_pose_out.Rot, object_pose->Rot );

			//Stage 4: "Tug" on the rotation of the object, from all of the sensor's pov.
			//If we were able to determine likliehood of a hit in the sweep instead of afterward
			//we would actually be able to perform this on a per-hit basis.
			if( 1 ) {
				LinmathQuat correction;
				quatcopy( correction, LinmathQuat_Identity );
				for( i = 0; i < pts; i++ )
				{
					if( !ptvalid[i] ) continue;
					FLT dist = dd->quantity_errors[i]-avgerr;
					FLT angle = dd->angles_at_pts[i];
					int sensor_id = dd->sensor_ids[i];
					FLT * normal =  dd->normal_at_errors[i];
					const SurvivePose * object_pose_at_hit = &dd->object_pose_at_hit[i];
					const FLT * sensor_inpos = &so->sensor_locations[sensor_id*3];

					LinmathQuat world_to_object_space;
					quatgetreciprocal(world_to_object_space, object_pose_at_hit->Rot);
					FLT correction_in_object_space[3];  //The amount across the surface of the object the rotation should happen.

					quatrotatevector(correction_in_object_space, world_to_object_space, normal );
					dist *= CORRECT_ROTATION_COEFFICIENT;
					if( dist > ROTATIONAL_CORRECTION_MAXFORCE ) dist = ROTATIONAL_CORRECTION_MAXFORCE;
					if( dist <-ROTATIONAL_CORRECTION_MAXFORCE ) dist =-ROTATIONAL_CORRECTION_MAXFORCE;

					//Now, we have a "tug" vector in object-local space.  Need to apply the torque.
					FLT vector_from_center_of_object[3];
					normalize3d( vector_from_center_of_object, sensor_inpos );
					//scale3d(vector_from_center_of_object, sensor_inpos, 10.0 );
		//			vector_from_center_of_object[2]*=-1;
	//				vector_from_center_of_object[1]*=-1;
//					vector_from_center_of_object[0]*=-1;
					//vector_from_center_of_object
					scale3d(vector_from_center_of_object,vector_from_center_of_object, 1);


					FLT new_vector_in_object_space[3];
					//printf( "%f %f %f %f\n", object_pose_at_hit->Rot[0], object_pose_at_hit->Rot[1], object_pose_at_hit->Rot[2], object_pose_at_hit->Rot[3] );
					//printf( "%f %f %f  // %f %f %f // %f\n", vector_from_center_of_object[0], vector_from_center_of_object[1], vector_from_center_of_object[2], correction_in_object_space[0], correction_in_object_space[1], correction_in_object_space[2], dist );
					scale3d( correction_in_object_space, correction_in_object_space, -dist );
					add3d( new_vector_in_object_space, vector_from_center_of_object, correction_in_object_space );

					normalize3d( new_vector_in_object_space, new_vector_in_object_space );

					LinmathQuat corrective_quaternion;
					quatfrom2vectors(corrective_quaternion, vector_from_center_of_object, new_vector_in_object_space );
					quatrotateabout( correction, correction, corrective_quaternion );
					//printf( "%f -> %f %f %f => %f %f %f [%f %f %f %f]\n", dist, vector_from_center_of_object[0], vector_from_center_of_object[1], vector_from_center_of_object[2],
						//correction_in_object_space[0], correction_in_object_space[1], correction_in_object_space[2],
						//corrective_quaternion[0],corrective_quaternion[1],corrective_quaternion[1],corrective_quaternion[3]);
				}
				//printf( "Applying: %f %f %f %f\n", correction[0], correction[1], correction[2], correction[3] );
				//Apply our corrective quaternion to the output.
				quatrotateabout( object_pose_out.Rot, object_pose_out.Rot, correction );
				quatnormalize( object_pose_out.Rot, object_pose_out.Rot );
			}

			//Janky need to do this somewhere else...  This initializes the pose estimator.
			if( so->PoseConfidence < .01 )
			{
				memcpy( &object_pose_out, &LinmathPose_Identity, sizeof( LinmathPose_Identity ) );
				object_pose_out.Pos[0] = -0.14372776;
				object_pose_out.Pos[1] = 0.06856518;
				object_pose_out.Pos[2] = 0.01960009;
				object_pose_out.Rot[0] = 1.0;
				object_pose_out.Rot[1] = -0.0;
				object_pose_out.Rot[2] = 0.0;
				object_pose_out.Rot[3] = 0.0;
				so->PoseConfidence = 1.0;
			}

			PoserData_poser_raw_pose_func(pd, so, lhid, &object_pose_out);

			dd->ptsweep = 0;
		}

		dd->sweepaxis = l->acode & 1;
		//printf( "SYNC %d %p\n", l->acode, dd );
		break;
	}
	case POSERDATA_FULL_SCENE: {
		//return opencv_solver_fullscene(so, (PoserDataFullScene *)(pd));
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserCharlesRefine);
