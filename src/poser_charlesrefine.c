//EXPERIMENTAL DRIVER - DO NOT USE

#include <poser.h>
#include <survive.h>
#include <survive_reproject.h>

#include "epnp/epnp.h"
#include "linmath.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MAX_PT_PER_SWEEP 32


typedef struct
{
	int sweepaxis;
	int sweeplh;
	FLT normal_at_errors[MAX_PT_PER_SWEEP][3];
	FLT quantity_errors[MAX_PT_PER_SWEEP]
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
		if( lhid == 1 ) break;


//		const FLT * sensor_normal = &so->sensor_normals[senid*3];
//		FLT sensor_normal_worldspace[3];
//		ApplyPoseToPoint(sensor_normal_worldspace,   object_pose, sensor_inpos);

		const FLT * sensor_inpos = &so->sensor_locations[senid*3];
		FLT sensor_position_worldspace[3];
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
			dd->normal_at_errors[i] = sweep_normal;
			dd->quantity_errors[i] = dist;
			dd->sensor_ids[i] = sensor_id;
			dd->ptsweep++:
		}

#if 0
		printf( "D %d %d: %f   [%f %f %f]\n", lhid, axis, dist, sweep_normal[0], sweep_normal[1], sweep_normal[2] );


		//Naieve approach...  Push it in the right direction
		SurvivePose object_pose_out;
		quatcopy( object_pose_out.Rot, object_pose->Rot );
		scale3d(sweep_normal, sweep_normal, -0.1*dist);
		add3d(object_pose_out.Pos, sweep_normal, object_pose->Pos);

		if( so->PoseConfidence < .01 )
		{
			dd->average_nudge[0] = 0;
			dd->average_nudge[1] = 0;
			dd->average_nudge[2] = 0;

			memcpy( &object_pose_out, &LinmathPose_Identity, sizeof( LinmathPose_Identity ) );
			object_pose_out.Pos[1] = 2.5;
			object_pose_out.Pos[2] = 1.8;
			so->PoseConfidence = 1.0;
		}
//		printf( "%f %f %f %f\n", object_pose->Rot[0], object_pose->Rot[1], object_pose->Rot[2], object_pose->Rot[3] );

		PoserData_poser_raw_pose_func(pd, so, lhid, &object_pose_out);
#endif

#if 0
// = { 
			axis?0.0:sin(angle),
			axis?sin(angle):0.0,
			cos(angle) };

		 = sensor_locations;
		LinmathPoint3d 
	int8_t sensor_ct;	  // sensor count
	FLT *sensor_locations; // size is sensor_ct*3.  Contains x,y,z values for each sensor
	FLT *sensor_normals;   // size is nrlocations*3.  cointains normal vector for each sensor


		// This is the quat equivalent of 'pout = pose * pin' if pose were a 4x4 matrix in homogenous space
void ApplyPoseToPoint(LinmathPoint3d pout, const LinmathPose *pose, const LinmathPoint3d pin);



		SurvivePose obj2world;
//		ApplyPoseToPose(&obj2world, &lh2world, &objpose);
		memcpy( &obj2world, &LinmathPose_Identity, sizeof( obj2world ) );
		obj2world.Pos[1] = 1;
		PoserData_poser_raw_pose_func(pd, so, lhid, &obj2world);
//		PoserData_poser_raw_pose_func(pd, so, ld->lh, &posers[lightData->lh]);


	// Pose Information, also "poser" field.
	///from SurviveObject
	//		FLT PoseConfidence;						 // 0..1
	//		SurvivePose OutPose;					 // Final pose? (some day, one can dream!)
	//		SurvivePose FromLHPose[NUM_LIGHTHOUSES]; // Filled out by poser, contains computed position from each lighthouse.
	//		void *PoserData; // Initialized to zero, configured by poser, can be anything the poser wants.
	//		PoserCB PoserFn;


       // printf( "%d %d %f\n", ld->sensor_id, ld->lh, ld->angle );

/*
						   SurvivePose *object_pose, void *user);

typedef struct
{
	PoserType pt;
	poser_raw_pose_func rawposeproc;
	poser_lighthouse_pose_func lighthouseposeproc;
	void *userdata;
} PoserData;

	PoserData hdr;
	int sensor_id;
	int acode;			//OOTX Code associated with this sweep. bit 1 indicates vertical(1) or horizontal(0) sweep
	int lh;             //Lighthouse making this sweep
	uint32_t timecode;  //In object-local ticks.
	FLT length;			//In seconds
	FLT angle;			//In radians from center of lighthouse.
} PoserDataLight;



*/
#if 0
		SurvivePose posers[2];
		int meas[2] = {0, 0};
		for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (so->ctx->bsd[lh].PositionSet) {
				epnp pnp = {.fu = 1, .fv = 1};
				epnp_set_maximum_number_of_correspondences(&pnp, so->sensor_ct);

				add_correspondences(so, &pnp, scene, lightData->timecode, lh);
				static int required_meas = -1;
				if (required_meas == -1)
					required_meas = survive_configi(so->ctx, "epnp-required-meas", SC_GET, 4);

				if (pnp.number_of_correspondences > required_meas) {

					SurvivePose objInLh = solve_correspondence(so, &pnp, false);
					if (quatmagnitude(objInLh.Rot) != 0) {
						SurvivePose *lh2world = &so->ctx->bsd[lh].Pose;

						SurvivePose txPose = {.Rot = {1}};
						ApplyPoseToPose(&txPose, lh2world, &objInLh);
						posers[lh] = txPose;
						meas[lh] = pnp.number_of_correspondences;
					}
				}

				epnp_dtor(&pnp);
			}
		}

		if (meas[0] > 0 && meas[1] > 0) {
			SurvivePose interpolate = {0};
			bool winnerTakesAll = true; // Not convinced slerp does the right thing, will change this when i am

			if (winnerTakesAll) {
				int winner = meas[0] > meas[1] ? 0 : 1;
				PoserData_poser_raw_pose_func(pd, so, winner, &posers[winner]);
			} else {
				double a, b;
				a = meas[0] * meas[0];
				b = meas[1] * meas[1];

				double t = a + b;
				for (size_t i = 0; i < 3; i++) {
					interpolate.Pos[i] = (posers[0].Pos[i] * a + posers[1].Pos[i] * b) / (t);
				}
				quatslerp(interpolate.Rot, posers[0].Rot, posers[1].Rot, b / (t));
				PoserData_poser_raw_pose_func(pd, so, lightData->lh, &interpolate);
			}
		} else {
			if (meas[lightData->lh])
				PoserData_poser_raw_pose_func(pd, so, lightData->lh, &posers[lightData->lh]);
		}
#endif

#endif
		return 0;
	}

	case POSERDATA_SYNC: {
		PoserDataLight *l = (PoserDataLight *)pd;
		int lhid = l->lh;

		//you can get sweepaxis and sweeplh.

		if( dd->ptsweep )
		{
			int i;
			int lhid = dd->lhid;
			int pts = dd->ptsweep;
			const SurvivePose * object_pose = &so->OutPose;

			FLT avg_err[3] = { 0, 0, 0 };
			FLT avgtot = 0.0;
			for( i = 0; i < pts; i++ )
			{
				FLT * nrm = dd->normal_at_errors[pts];
				FLT qty = quantity_errors[pts];
				avgtot += qty;
				avg_err[0] = avg_err[0] + nrm[0] * qty;
				avg_err[1] = avg_err[1] + nrm[1] * qty;
				avg_err[2] = avg_err[2] + nrm[2] * qty;
			}
			scale3d(avg_err, avg_err, 1./pts);
			//We have "Average error" now.  This is a world space value.
			//This can correct for lateral error, but not distance from camera.

			//Next we need to find out what the weighting is to determine "zoom"
			//How do we do this?  ??? Too tired to math.
			FLT weight = 0.0;
			for( i = 0; i < pts; i++ )
			{
				//??!?!?  Sturfff
			}

			dd->ptsweep = 0;

			//Update		PoserData_poser_raw_pose_func(pd, so, lhid, &object_pose_out);
		}

		dd->nextaxis = l->acode & 1;
		printf( "SYNC %d %p\n", l->acode, dd );
		break;
	}
	case POSERDATA_FULL_SCENE: {
		//return opencv_solver_fullscene(so, (PoserDataFullScene *)(pd));
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserCharlesRefine);
