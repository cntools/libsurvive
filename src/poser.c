#include "math.h"
#include <assert.h>
#include <linmath.h>
#include <stdint.h>
#include <stdio.h>
#include <survive.h>

#define _USE_MATH_DEFINES // for C
#include <math.h>
#include <poser.h>
#include <stdlib.h>
#include <string.h>

static survive_timecode PoserData_timecode(const PoserData *poser_data) { return poser_data->timecode; }

SURVIVE_EXPORT int32_t PoserData_size(const PoserData *poser_data) {
	switch (poser_data->pt) {
	case POSERDATA_FULL_SCENE:
		return sizeof(PoserDataFullScene);
	case POSERDATA_DISASSOCIATE:
		return sizeof(PoserData);
	case POSERDATA_IMU:
		return sizeof(PoserDataIMU);
	case POSERDATA_LIGHT:
	case POSERDATA_SYNC:
		return sizeof(PoserDataLightGen1);
	case POSERDATA_LIGHT_GEN2:
	case POSERDATA_SYNC_GEN2:
		return sizeof(PoserDataLightGen2);
	}
	assert(false);
	return 0;
}

STATIC_CONFIG_ITEM(CENTER_ON_LH0, "center-on-lh0", 'i',
				   "Alternative scheme for setting initial position; LH0 is 0, 0 looking in the +X direction", 0)
STATIC_CONFIG_ITEM(REPORT_IN_IMU, "report-in-imu", 'i', "Debug option to output poses in IMU space.", 0)
void PoserData_poser_pose_func(PoserData *poser_data, SurviveObject *so, const SurvivePose *imu2world) {
	SurviveContext *ctx = so->ctx;
	for (int i = 0; i < 3; i++) {
		assert(!isnan(imu2world->Pos[i]));
		if (fabs(imu2world->Pos[i]) > 20) {
			SV_WARN("Squelching reported pose of " SurvivePose_format " for %s; values are invalid",
					SURVIVE_POSE_EXPAND(*imu2world), so->codename);
			return;
		}
	}
	if (poser_data->poseproc) {
		poser_data->poseproc(so, PoserData_timecode(poser_data), imu2world, poser_data->userdata);
	} else {
		static int report_in_imu = -1;
		if (report_in_imu == -1) {
			report_in_imu = survive_configi(so->ctx, REPORT_IN_IMU_TAG, SC_GET, 0);
		}

		SurvivePose head2world;
		so->OutPoseIMU = *imu2world;
		if (!report_in_imu) {
			ApplyPoseToPose(&head2world, imu2world, &so->head2imu);
		} else {
			head2world = *imu2world;
		}

		for (int i = 0; i < 7; i++)
			assert(!isnan(((FLT *)imu2world)[i]));

		so->ctx->poseproc(so, PoserData_timecode(poser_data), &head2world);
	}
}
void PoserData_poser_pose_func_with_velocity(PoserData *poser_data, SurviveObject *so, const SurvivePose *imu2world,
											 const SurviveVelocity *velocity) {
	so->ctx->velocityproc(so, PoserData_timecode(poser_data), velocity);
	PoserData_poser_pose_func(poser_data, so, imu2world);
}

void PoserData_lighthouse_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse,
									SurvivePose *lighthouse_pose, SurvivePose *object_pose) {
	if (poser_data->lighthouseposeproc) {
		for (int i = 0; i < 7; i++)
			assert(!isnan(((FLT *)lighthouse_pose)[i]));

		assert(!quatiszero(lighthouse_pose->Rot));

		if (quatiszero(object_pose->Rot)) {
			*object_pose = (SurvivePose){.Rot = {1.}};
		}
		poser_data->lighthouseposeproc(so, lighthouse, lighthouse_pose, object_pose, poser_data->userdata);
	} else {
		const FLT up[3] = {0, 0, 1};
		SurviveContext *ctx = so->ctx;

		if (quatmagnitude(lighthouse_pose->Rot) == 0) {
			SV_INFO("Pose func called with invalid pose.");
			return;
		}

		// Assume that the space solved for is valid but completely arbitrary. We are going to do a few things:
		// a) Using the gyro data, normalize it so that gravity is pushing straight down along Z
		// c) Assume the object is at origin
		// b) Place the first lighthouse on the X axis by rotating around Z
		//
		// This calibration setup has the benefit that as long as the user is calibrating on the same flat surface,
		// calibration results will be roughly identical between all posers no matter the orientation the object is
		// lying
		// in.
		//
		// We might want to go a step further and affix the first lighthouse in a given pose that preserves up so that
		// it doesn't matter where on that surface the object is.
		bool worldEstablished = quatmagnitude(object_pose->Rot) != 0;
		SurvivePose object2arb = {.Rot = {1.}};
		if (object_pose && !quatiszero(object_pose->Rot))
			object2arb = *object_pose;
		SurvivePose lighthouse2arb = *lighthouse_pose;

		SurvivePose obj2world, lighthouse2world;
		// Purposefully only set this once. It should only depend on the first (calculated) lighthouse
		if (!worldEstablished) {
			bool centerOnLh0 = survive_configi(so->ctx, "center-on-lh0", SC_GET, 0);

			// Start by just moving from whatever arbitrary space into object space.
			SurvivePose arb2object;
			InvertPose(&arb2object, &object2arb);

			SurvivePose lighthouse2obj;
			ApplyPoseToPose(&lighthouse2obj, &arb2object, &lighthouse2arb);
			SurvivePose arb2world = arb2object;

			// Find the poses that map to the above

			// Find the space with the same origin, but rotated so that gravity is up
			SurvivePose lighthouse2objUp = {0}, object2objUp = {0};
			FLT accel_mag = quatmagnitude(so->activations.accel);
			if (accel_mag != 0.0 && !isnan(accel_mag)) {
				quatfrom2vectors(object2objUp.Rot, so->activations.accel, up);
			} else {
				SV_WARN("Calibration didn't have valid IMU data for %s; couldn't establish 'up' vector.", so->codename);
				object2objUp.Rot[0] = 1.0;
			}

			// Calculate the pose of the lighthouse in this space
			ApplyPoseToPose(&lighthouse2objUp, &object2objUp, &lighthouse2obj);
			ApplyPoseToPose(&arb2world, &object2objUp, &arb2world);

			// Find what angle we need to rotate about Z by to get to 90 degrees.
			FLT ang = atan2(lighthouse2objUp.Pos[1], lighthouse2objUp.Pos[0]);
			FLT ang_target = M_PI / 2.;
			FLT euler[3] = {0, 0, ang_target - ang};
			SurvivePose objUp2World = {0};
			quatfromeuler(objUp2World.Rot, euler);

			ApplyPoseToPose(&arb2world, &objUp2World, &arb2world);
			ApplyPoseToPose(&obj2world, &arb2world, &object2arb);
			ApplyPoseToPose(&lighthouse2world, &arb2world, &lighthouse2arb);

			if (centerOnLh0) {
				sub3d(obj2world.Pos, obj2world.Pos, lighthouse2world.Pos);
				lighthouse2world.Pos[0] = lighthouse2world.Pos[1] = lighthouse2world.Pos[2] = 0.0;

				LinmathPoint3d camFwd = {0, 0, -1}, worldFwd = {0};
				ApplyPoseToPoint(worldFwd, &lighthouse2world, camFwd);
				FLT ang = atan2(worldFwd[1], worldFwd[0]);
				FLT euler[3] = {0, 0, M_PI / 2 - ang};

				SurvivePose rotate2center = {0};
				quatfromeuler(rotate2center.Rot, euler);

				ApplyPoseToPose(&obj2world, &rotate2center, &obj2world);
				ApplyPoseToPose(&lighthouse2world, &rotate2center, &lighthouse2world);
			}

			*object_pose = obj2world;
		} else {
			lighthouse2world = *lighthouse_pose;
			obj2world = *object_pose;
		}

		for (int i = 0; i < 7; i++)
			assert(!isnan(((FLT *)&lighthouse2world)[i]));

		so->ctx->lighthouse_poseproc(so->ctx, lighthouse, &lighthouse2world, &obj2world);
	}
}

void PoserData_lighthouse_poses_func(PoserData *poser_data, SurviveObject *so, SurvivePose *lighthouse_pose,
									 uint32_t lighthouse_count, SurvivePose *object_pose) {

	if (poser_data->lighthouseposeproc) {
		for (int lighthouse = 0; lighthouse < lighthouse_count; lighthouse++) {
			if (quatiszero(lighthouse_pose[lighthouse].Rot))
				continue;

			if (object_pose && quatiszero(object_pose->Rot)) {
				*object_pose = (SurvivePose){.Rot = {1.}};
			}
			poser_data->lighthouseposeproc(so, lighthouse, &lighthouse_pose[lighthouse], object_pose,
										   poser_data->userdata);
		}
	} else {
		SurvivePose object2World;
		if (object_pose == 0 || quatiszero(object_pose->Rot))
			object2World = so->OutPoseIMU;
		else
			object2World = *object_pose;

		bool worldEstablished = !quatiszero(object2World.Rot);

		uint32_t lh_indices[NUM_GEN2_LIGHTHOUSES] = {0};
		uint32_t cnt = 0;

		uint32_t reference_basestation = survive_configi(so->ctx, "reference-basestation", SC_GET, 0);

		for (int lh = 0; lh < lighthouse_count; lh++) {
			SurvivePose lh2object = lighthouse_pose[lh];
			if (quatmagnitude(lh2object.Rot) != 0.0) {
				lh_indices[cnt] = lh;
				uint32_t lh0 = lh_indices[0];
				bool preferThisBSD = reference_basestation == 0
										 ? (so->ctx->bsd[lh].BaseStationID < so->ctx->bsd[lh0].BaseStationID)
										 : reference_basestation == so->ctx->bsd[lh].BaseStationID;
				if (preferThisBSD) {
					lh_indices[0] = lh;
					lh_indices[cnt] = lh0;
				}
				cnt++;
			}
		}

		struct SurviveContext *ctx = so->ctx;
		SV_INFO("Using LH %d (%08x) as reference lighthouse", lh_indices[0], so->ctx->bsd[lh_indices[0]].BaseStationID);
		for (int lh_idx = 0; lh_idx < cnt; lh_idx++) {
			int lh = lh_indices[lh_idx];

			SurvivePose lh2object = lighthouse_pose[lh];
			quatnormalize(lh2object.Rot, lh2object.Rot);

			SurvivePose lh2world = lh2object;
			if (!quatiszero(object2World.Rot) && worldEstablished == false) {
				ApplyPoseToPose(&lh2world, &object2World, &lh2object);
			}

			PoserData_lighthouse_pose_func(poser_data, so, lh, &lh2world, &object2World);
		}

		if (object_pose)
			*object_pose = object2World;
	}
}
void PoserDataFullScene2Activations(const PoserDataFullScene *pdfs, SurviveSensorActivations *activations) {
	SurviveSensorActivations_ctor(0, activations);
	for (int i = 0; i < SENSORS_PER_OBJECT * NUM_GEN1_LIGHTHOUSES * 2; i++) {
		FLT length = ((FLT *)pdfs->lengths)[i] * 48000000;
		if (length > 0)
			((survive_timecode *)activations->lengths)[i] = (survive_timecode)length;
	}

	for (int i = 0; i < SENSORS_PER_OBJECT * NUM_GEN2_LIGHTHOUSES * 2; i++) {
		((FLT *)activations->angles)[i] = ((FLT *)pdfs->angles)[i];
	}

	memcpy(activations->accel, pdfs->lastimu.accel, sizeof(activations->accel));
	memcpy(activations->gyro, pdfs->lastimu.gyro, sizeof(activations->gyro));
	memcpy(activations->mag, pdfs->lastimu.mag, sizeof(activations->mag));
}

SURVIVE_EXPORT void Activations2PoserDataFullScene(const struct SurviveSensorActivations_s *activations,
												   PoserDataFullScene *pdfs) {
	for (int i = 0; i < SENSORS_PER_OBJECT * NUM_GEN1_LIGHTHOUSES * 2; i++) {
		survive_timecode length = ((survive_timecode *)activations->lengths)[i];
		if (length > 0)
			((FLT *)pdfs->lengths)[i] = length / 48000000.;
	}

	for (int i = 0; i < SENSORS_PER_OBJECT * NUM_GEN2_LIGHTHOUSES * 2; i++) {
		((FLT *)pdfs->angles)[i] = ((FLT *)activations->angles)[i];
	}

	memcpy(pdfs->lastimu.accel, activations->accel, sizeof(activations->accel));
	memcpy(pdfs->lastimu.gyro, activations->gyro, sizeof(activations->gyro));
	memcpy(pdfs->lastimu.mag, activations->mag, sizeof(activations->mag));
}
