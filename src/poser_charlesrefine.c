#include <poser.h>
#include <survive.h>
#include <survive_reproject.h>

#include "epnp/epnp.h"
#include "linmath.h"
#include "survive_cal.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <survive_imu.h>

#define MAX_PT_PER_SWEEP SENSORS_PER_OBJECT



// Tunable parameters:
#define MIN_HIT_QUALITY 0.5 // Determines which hits to cull.
#define HIT_QUALITY_BASELINE                                                                                           \
	0.0001 // Determines which hits to cull.  Actually SQRT(baseline) if 0.0001, it is really 1cm

#define LIGHTCAP_DESCALE 0.2		//DO NOT EXCEED 0.7

#define CORRECT_LATERAL_POSITION_COEFFICIENT LIGHTCAP_DESCALE // Explodes if you exceed 1.0  (Normally 0.7 for snappy non-IMU response)
#define CORRECT_TELESCOPTION_COEFFICIENT (10.f*LIGHTCAP_DESCALE)	 // Converges even as high as 10.0 and doesn't explode. (Normally 7.0 for non-IMU respone)

#define CORRECT_ROTATION_COEFFICIENT                                                                                   \
	0.2 // This starts to fall apart above 5.0, but for good reason.  It is amplified by the number of points seen.
#define ROTATIONAL_CORRECTION_MAXFORCE 0.01
#define MINIMUM_CONFIDENCE_TO_CORRECT_POSITION 0.02 // 0.2
#define POSE_CONFIDENCE_FOR_HANDLING_LINEAR_IMU .9 //0.9
#define MAX_JUMP_DISTANCE 0.5   //0.4


//Grand todo:
//  Update global "Up" vector from LH's PoV based on "Up" from the HMD.

typedef struct {
	int sweepaxis;
	int sweeplh;

	FLT normal_at_errors[MAX_PT_PER_SWEEP][3]; // Value is actually normalized, not just normal to sweep plane.
	FLT quantity_errors[MAX_PT_PER_SWEEP];		//Dot product of error offset.
	FLT angles_at_pts[MAX_PT_PER_SWEEP];
	SurvivePose object_pose_at_hit[MAX_PT_PER_SWEEP];
	uint8_t sensor_ids[MAX_PT_PER_SWEEP];

	LinmathPoint3d MixingPositions[NUM_LIGHTHOUSES][2];
	LinmathPoint3d mixed_output;

	//Super high speed vibratey and terrible.
	//Also, big deal:  this does NOT have the "re-righting" vector
	//from the accelerometer applied to it.
	SurvivePose InteralPoseUsedForCalc;

	FLT MixingConfidence[NUM_LIGHTHOUSES][2];
	FLT last_angle_lh_axis[NUM_LIGHTHOUSES][2];
	int ptsweep;

	SurviveIMUTracker tracker;
	SurvivePose * lastlhp;


	//Additional flags, used when we start to try to use the accelerometer
	LinmathPoint3d average_accelerometer_up_vector_in_world_space;
	LinmathPoint3d velocity_according_to_accelerometer;
} CharlesPoserData;


void AdjustRotation( SurviveObject *so, LinmathQuat adjustment, int is_imu, int is_coarse )
{
	CharlesPoserData *dd = so->PoserData;
	//LinmathQuat invert_adjust;
	//quatgetreciprocal( invert_adjust, adjustment );
	quatrotateabout(dd->InteralPoseUsedForCalc.Rot, dd->InteralPoseUsedForCalc.Rot, adjustment );
	quatnormalize(dd->InteralPoseUsedForCalc.Rot, dd->InteralPoseUsedForCalc.Rot);

}

void AdjustPosition( SurviveObject * so, LinmathPoint3d adjustment, int is_imu, float descale )
{
	CharlesPoserData *dd = so->PoserData;

	add3d( dd->InteralPoseUsedForCalc.Pos, adjustment, dd->InteralPoseUsedForCalc.Pos);
	add3d( dd->mixed_output, adjustment, dd->mixed_output);

	if( descale > 0.0001 )	//Coming from lightcap.
	{
		LinmathPoint3d backflow;
		scale3d( backflow, adjustment, 1.0/descale );
		CharlesPoserData *dd = so->PoserData;
		//XXX TODO figure out how to dampen velocity.
		add3d( dd->velocity_according_to_accelerometer, dd->velocity_according_to_accelerometer, backflow );
		scale3d( backflow, backflow, .001 );
		add3d( dd->average_accelerometer_up_vector_in_world_space, dd->average_accelerometer_up_vector_in_world_space, backflow );
	}

}

//Emits "dd->mixed_output" for position and dd->InteralPoseUsedForCalc.Rot for rotation.
void EmitPose( SurviveObject *so, PoserData *pd )
{
	CharlesPoserData *dd = so->PoserData;

	SurvivePose object_pose_out;
	copy3d(   object_pose_out.Pos, dd->mixed_output );

	//average_accelerometer_up_vector_in_world_space  should be "up"
	LinmathVec3d true_up = { 0, 0, 1 };
	LinmathVec3d dist_up;
	normalize3d( dist_up, dd->average_accelerometer_up_vector_in_world_space );  //NOTE: Average vector probably won't be normalized.
	LinmathQuat adjustment_from_rerighting_up;
	quatfrom2vectors( adjustment_from_rerighting_up, dist_up, true_up );

	quatcopy( object_pose_out.Rot, dd->InteralPoseUsedForCalc.Rot );
	quatrotateabout( object_pose_out.Rot, adjustment_from_rerighting_up, dd->InteralPoseUsedForCalc.Rot );
	PoserData_poser_pose_func(pd, so, &object_pose_out);
}



int PoserCharlesRefine(SurviveObject *so, PoserData *pd) {
	CharlesPoserData *dd = so->PoserData;
	if (!dd)
	{
		so->PoserData = dd = calloc(sizeof(CharlesPoserData), 1);
		SurvivePose object_pose_out;
		memcpy(&object_pose_out, &LinmathPose_Identity, sizeof(LinmathPose_Identity));
		memcpy(&dd->InteralPoseUsedForCalc, &LinmathPose_Identity, sizeof(LinmathPose_Identity));
		so->PoseConfidence = 0.0;
		PoserData_poser_pose_func(pd, so, &object_pose_out);
	}

	SurviveSensorActivations *scene = &so->activations;
	switch (pd->pt) {
	case POSERDATA_IMU: {
		float imu_time = 1./ so->imu_freq;
		// Really should use this...
		PoserDataIMU *imuData = (PoserDataIMU *)pd;

		{
			LinmathQuat	applygyro;
			imuData->gyro[0] *= imu_time;
			imuData->gyro[1] *= imu_time;
			imuData->gyro[2] *= imu_time;
			quatfromeuler( applygyro, imuData->gyro );
			AdjustRotation( so, applygyro, 1, 0 );
		}


		printf( "ACCEL %f %f %f\n", PFTHREE( imuData->accel ) );

		{
			LinmathPoint3d rotated_out;
			quatrotatevector( rotated_out, dd->InteralPoseUsedForCalc.Rot, imuData->accel );

			if( so->PoseConfidence > 0.9999 )
			{
				LinmathPoint3d correction;

				//XXX Danger, will robinson.
				//We are doing an IIR on the acceleration.  Tests have shown THIS IS BAD.  We should try to correct based on the lightcap data.
				scale3d( dd->average_accelerometer_up_vector_in_world_space, dd->average_accelerometer_up_vector_in_world_space, .999  );
				scale3d( correction, rotated_out, .001  );
				add3d( dd->average_accelerometer_up_vector_in_world_space, dd->average_accelerometer_up_vector_in_world_space, correction );

				LinmathPoint3d deviation;
				sub3d( deviation, rotated_out, dd->average_accelerometer_up_vector_in_world_space );

				LinmathPoint3d acc;
				scale3d( acc, deviation, 9.8*imu_time );
				add3d( dd->velocity_according_to_accelerometer, acc, dd->velocity_according_to_accelerometer );
				scale3d( dd->velocity_according_to_accelerometer, dd->velocity_according_to_accelerometer, .999 ); //XXX Danger! We are doing an IIR on velocity.  This is dangerous.

				LinmathPoint3d posdiff;
				scale3d( posdiff, dd->velocity_according_to_accelerometer, imu_time );
				AdjustPosition( so, posdiff, 1, 0 );
			}
			else
			{
				copy3d( dd->average_accelerometer_up_vector_in_world_space, rotated_out );
				scale3d( dd->velocity_according_to_accelerometer, dd->velocity_according_to_accelerometer, 0 );
			}
		}

		EmitPose( so, pd );
		return 0;
	}
	case POSERDATA_LIGHT: {
		int i;
		PoserDataLight *ld = (PoserDataLight *)pd;
		int lhid = ld->lh;
		int senid = ld->sensor_id;
		BaseStationData *bsd = &so->ctx->bsd[ld->lh];
		if (!bsd->PositionSet)
			break;
		SurvivePose *lhp = dd->lastlhp = &bsd->Pose;
		FLT inangle = ld->angle;
		int sensor_id = ld->sensor_id;
		int axis = dd->sweepaxis;

		dd->sweeplh = lhid;

		const FLT *sensor_inpos = &so->sensor_locations[senid * 3];
		FLT sensor_position_worldspace[3];
		// XXX Once I saw this get pretty wild (When in playback)
		// I had to invert the values of sensor_inpos.  Not sure why.
		ApplyPoseToPoint(sensor_position_worldspace, &dd->InteralPoseUsedForCalc, sensor_inpos);

		FLT sensorpos_rel_lh[3];
		sub3d(sensorpos_rel_lh, sensor_position_worldspace, lhp->Pos);

		// Next, define a normal in global space of the plane created by the sweep hit.
		// Careful that this must be normalized.
		FLT sweep_normal[3];
		FLT inangles[2];
		FLT outangles[2];
		inangles[axis] = inangle;
		inangles[!axis] = dd->last_angle_lh_axis[lhid][!axis];
		survive_apply_bsd_calibration(so->ctx, lhid, inangles, outangles );
		FLT angle = outangles[axis];
		

		// If 1, the "y" axis. //XXX Check me.
		if (axis) // XXX Just FYI this should include account for skew
		{
			sweep_normal[0] = 0;
			sweep_normal[1] = cos(angle);
			sweep_normal[2] = sin(angle);
			// printf( "+" );
		} else {
			sweep_normal[0] = cos(angle);
			sweep_normal[1] = 0;
			sweep_normal[2] = -sin(angle);
			// printf( "-" );
		}

		// Need to apply the lighthouse's transformation to the sweep's normal.
		quatrotatevector(sweep_normal, lhp->Rot, sweep_normal);

		// Compute point-line distance between sensorpos_rel_lh and the plane defined by sweep_normal.
		// Do this by projecting sensorpos_rel_lh (w) onto sweep_normal (v).
		// You can do this by |v dot w| / |v| ... But we know |v| is 1. So...
		FLT dist = dot3d(sensorpos_rel_lh, sweep_normal);

		if ((i = dd->ptsweep) < MAX_PT_PER_SWEEP) {
			int repeat = 0;
			int k;

			//Detect repeated hits.  a rare problem but can happen with lossy sources of pose.
			for( k = 0; k < dd->ptsweep; k++ )
			{
				if( dd->sensor_ids[k] == sensor_id )
				{
					repeat = 1;
					i = k;
				}
			}
			memcpy(dd->normal_at_errors[i], sweep_normal, sizeof(FLT) * 3);
			dd->quantity_errors[i] = dist;
			dd->angles_at_pts[i] = angle;
			dd->sensor_ids[i] = sensor_id;
			memcpy(&dd->object_pose_at_hit[i], &dd->InteralPoseUsedForCalc, sizeof(SurvivePose));
			if( !repeat )
				dd->ptsweep++;
		}

		dd->last_angle_lh_axis[lhid][axis] = inangle;

		return 0;
	}

	case POSERDATA_SYNC: {
		PoserDataLight *l = (PoserDataLight *)pd;
		int lhid = l->lh;
		// you can get sweepaxis and sweeplh.
		if (dd->ptsweep) {
			int i;
			int applied_corrections = 0;
			int normal_faults = 0; 
			int lhid = dd->sweeplh;
			int axis = dd->sweepaxis;
			int pts = dd->ptsweep;

			BaseStationData *bsd = &so->ctx->bsd[lhid];
			SurvivePose *lh_pose = &bsd->Pose;

			int validpoints = 0;
			int ptvalid[MAX_PT_PER_SWEEP];
			FLT avgerr = 0.0;
			FLT avgang = 0.0;

			{
				FLT vec_correct[3] = {0., 0., 0.};
				// Step 1: Determine standard of deviation, and average in order to
				// drop points that are likely in error.
				{
					// Calculate average
					FLT avgerr_orig = 0.0;
					FLT stddevsq = 0.0;
					for (i = 0; i < pts; i++)
						avgerr_orig += dd->quantity_errors[i];
					avgerr_orig /= pts;

					// Calculate standard of deviation.
					for (i = 0; i < pts; i++) {
						FLT diff = dd->quantity_errors[i] - avgerr_orig;
						stddevsq += diff * diff;
					}
					stddevsq /= pts;

					for (i = 0; i < pts; i++) {
						FLT err = dd->quantity_errors[i];
						FLT diff = err - avgerr_orig;
						diff *= diff;
						int isptvalid = (diff * MIN_HIT_QUALITY <= stddevsq + HIT_QUALITY_BASELINE) ? 1 : 0;
						ptvalid[i] = isptvalid;
						if (isptvalid) {
							avgang += dd->angles_at_pts[i];
							avgerr += err;
							validpoints++;
						}
					}
					avgang /= validpoints;
					avgerr /= validpoints;
				}


				// Step 2: Determine average lateral error.
				// We can actually always perform this operation.  Even with only one point.
				if ( so->PoseConfidence > MINIMUM_CONFIDENCE_TO_CORRECT_POSITION )
				{
					FLT avg_err[3] = {0, 0, 0}; // Positional error.
					for (i = 0; i < pts; i++) {
						if (!ptvalid[i])
							continue;
						FLT *nrm = dd->normal_at_errors[i];
						FLT err = dd->quantity_errors[i];
						avg_err[0] = avg_err[0] + nrm[0] * err;
						avg_err[1] = avg_err[1] + nrm[1] * err;
						avg_err[2] = avg_err[2] + nrm[2] * err;
					}

					// NOTE: The "avg_err" is not geometrically centered.  This is actually
					// probably okay, since if you have sevearl data points to one side, you
					// can probably trust that more.
					scale3d(avg_err, avg_err, 1. / validpoints);

					// We have "Average error" now.  A vector in worldspace.
					// This can correct for lateral error, but not distance from camera.

					// XXX TODO: Should we check to see if we only have one or
					// two points to make sure the error on this isn't unusually high?
					// If calculated error is unexpectedly high, then we should probably
					// Not apply the transform.

					if( ( magnitude3d( avg_err ) < MAX_JUMP_DISTANCE  || so->PoseConfidence < 0.8 ) ) 
					{
						scale3d(avg_err, avg_err, -CORRECT_LATERAL_POSITION_COEFFICIENT);
						add3d(vec_correct, vec_correct, avg_err);
						applied_corrections++;
					}
					else
					{
						so->PoseConfidence *= 0.9;
					}
				}



				// Step 3: Control telecoption from lighthouse.
				// we need to find out what the weighting is to determine "zoom"
				if (validpoints > 1 && so->PoseConfidence > MINIMUM_CONFIDENCE_TO_CORRECT_POSITION ) // Can't correct "zoom" with only one point.
				{
					FLT zoom = 0.0;
					FLT rmsang = 0.0;
					for (i = 0; i < pts; i++) {
						if (!ptvalid[i])
							continue;
						FLT delang = dd->angles_at_pts[i] - avgang;
						FLT delerr = dd->quantity_errors[i] - avgerr;
						if (axis)
							delang *= -1; // Flip sign on alternate axis because it's measured backwards.
						zoom += delerr * delang;
						rmsang += delang * delang;
					}

					// Control into or outof lighthouse.
					// XXX Check to see if we need to sqrt( the rmsang), need to check convergance behavior close to
					// lighthouse.
					// This is a questionable step.
					zoom /= sqrt(rmsang);

					zoom *= CORRECT_TELESCOPTION_COEFFICIENT;

					//Don't apply completely wild zoom's unless our confidence is awful.
					if( ( zoom < MAX_JUMP_DISTANCE || so->PoseConfidence < 0.8 ) ) 
					{
						FLT veccamalong[3];
						sub3d(veccamalong, lh_pose->Pos, dd->InteralPoseUsedForCalc.Pos);
						normalize3d(veccamalong, veccamalong);
						scale3d(veccamalong, veccamalong, zoom);
						add3d(vec_correct, veccamalong, vec_correct);
						applied_corrections++;
					}
					else
					{
						so->PoseConfidence *= 0.9;
					}
				}
				AdjustPosition( so, vec_correct, 0, (applied_corrections==2)?LIGHTCAP_DESCALE:0 );
			}

			// Stage 4: "Tug" on the rotation of the object, from all of the sensor's pov.
			// If we were able to determine likliehood of a hit in the sweep instead of afterward
			// we would actually be able to perform this on a per-hit basis.
			if (validpoints > 1) {
				LinmathQuat correction;
				quatcopy(correction, LinmathQuat_Identity);
				for (i = 0; i < pts; i++) {
					if (!ptvalid[i])
						continue;

					FLT dist = dd->quantity_errors[i] - avgerr; //Relative dot-product error.
					FLT angle = dd->angles_at_pts[i];
					int sensor_id = dd->sensor_ids[i];
					FLT * normal = dd->normal_at_errors[i];
					FLT * sensornormal = &so->sensor_normals[sensor_id*3];
					SurvivePose * lhp = dd->lastlhp;

					const SurvivePose * object_pose_at_hit = &dd->object_pose_at_hit[i];
					const FLT * sensor_inpos = &so->sensor_locations[sensor_id * 3];
					LinmathQuat world_to_object_space;
					quatgetreciprocal(world_to_object_space, object_pose_at_hit->Rot);

					//4A: First, check to see if this hit is a sensor that is facing the lighthouse.
					//This is for coarse corrections early on in the calibration.
					//If one of these happens it means the orientation/pose is totally impossible.
					if( so->PoseConfidence < 0.9 ) {
						LinmathPoint3d vector_to_lighthouse;
						sub3d( vector_to_lighthouse, lhp->Pos, object_pose_at_hit->Pos ); //Get vector in world space.
						normalize3d( vector_to_lighthouse, vector_to_lighthouse );
						quatrotatevector( vector_to_lighthouse, world_to_object_space, vector_to_lighthouse );
						float facingness = dot3d( sensornormal, vector_to_lighthouse );
						if( facingness < -.1 )
						{
							//This is an impossible sensor hit. 
							so->PoseConfidence *= 0.8;

							//If our pose confidence is low, apply a torque.
							if( so->PoseConfidence < 0.8 )
							{
								LinmathPoint3d rotateaxis;
								cross3d( rotateaxis, vector_to_lighthouse, sensornormal );
								LinmathQuat correction;
								quatfromaxisangle(correction, rotateaxis, facingness*.2 );
								normal_faults ++;
								AdjustRotation( so, correction, 0, 1 );
							}
						}
					}

					//Apply the normal tug.
					{

						FLT correction_in_object_space[3]; // The amount across the surface of the object the rotation
														   // should happen.

						quatrotatevector(correction_in_object_space, world_to_object_space, normal);
						dist *= CORRECT_ROTATION_COEFFICIENT;
						if (dist > ROTATIONAL_CORRECTION_MAXFORCE)
							dist = ROTATIONAL_CORRECTION_MAXFORCE;
						if (dist < -ROTATIONAL_CORRECTION_MAXFORCE)
							dist = -ROTATIONAL_CORRECTION_MAXFORCE;

						// Now, we have a "tug" vector in object-local space.  Need to apply the torque.
						FLT vector_from_center_of_object[3];
						normalize3d(vector_from_center_of_object, sensor_inpos);
						scale3d(vector_from_center_of_object, vector_from_center_of_object, 1);

						FLT new_vector_in_object_space[3];
						scale3d(correction_in_object_space, correction_in_object_space, -dist);
						add3d(new_vector_in_object_space, vector_from_center_of_object, correction_in_object_space);

						normalize3d(new_vector_in_object_space, new_vector_in_object_space);

						LinmathQuat corrective_quaternion;
						quatfrom2vectors(corrective_quaternion, vector_from_center_of_object, new_vector_in_object_space);
						quatrotateabout(correction, correction, corrective_quaternion);
					}

				}
				AdjustRotation( so, correction, 0, 0 );
			}

			memcpy( dd->MixingPositions[lhid][axis], dd->InteralPoseUsedForCalc.Pos, sizeof( dd->InteralPoseUsedForCalc.Pos ) );
			dd->MixingConfidence[lhid][axis] = (validpoints)?((validpoints>1)?1.0:0.5):0;

			//Box filter all of the guesstimations, and emit the new guess.
			{
				FLT MixedAmount = 0;
				LinmathPoint3d MixedPosition = { 0, 0, 0 };
				int l = 0, a = 0;
				if( lhid == 0 && axis == 0 ) for( l = 0; l < NUM_LIGHTHOUSES; l++ ) for( a = 0; a < 2; a++ ) dd->MixingConfidence[l][a] -= 0.1;
				for( l = 0; l < NUM_LIGHTHOUSES; l++ ) for( a = 0; a < 2; a++ )
				{
					LinmathPoint3d MixThis = { 0, 0, 0 };
					FLT Confidence = dd->MixingConfidence[l][a];
					if(Confidence < 0 ) Confidence = 0;
					scale3d( MixThis, dd->MixingPositions[l][a], Confidence );
					add3d( MixedPosition, MixedPosition, MixThis );
					MixedAmount += Confidence;
					//printf( "%f ", Confidence );
				}
				scale3d( dd->mixed_output, MixedPosition, 1./MixedAmount );
				EmitPose( so, pd );

			}

			dd->ptsweep = 0;

			if( validpoints > 1 && applied_corrections > 1 && !normal_faults)
			{
				so->PoseConfidence += (1-so->PoseConfidence)*.04;
			}
			else if( validpoints > 1 && so->PoseConfidence < 0.5 && !normal_faults )
			{
				so->PoseConfidence += (1-so->PoseConfidence)*.01;
			}
		}

		dd->sweepaxis = l->acode & 1;
		// printf( "SYNC %d %p\n", l->acode, dd );
		break;
	}
	case POSERDATA_FULL_SCENE: {
		// return opencv_solver_fullscene(so, (PoserDataFullScene *)(pd));
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserCharlesRefine);
