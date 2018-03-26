#include "math.h"
#include <linmath.h>
#include <stdint.h>
#include <stdio.h>
#include <survive.h>

void PoserData_poser_raw_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	if (poser_data->rawposeproc) {
		poser_data->rawposeproc(so, lighthouse, pose, poser_data->userdata);
	} else {
		so->ctx->rawposeproc(so, lighthouse, pose);
	}
}

void PoserData_lighthouse_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse,
									SurvivePose *objUp2world, SurvivePose *lighthouse_pose, SurvivePose *object_pose) {
	if (poser_data->lighthouseposeproc) {
		poser_data->lighthouseposeproc(so, lighthouse, lighthouse_pose, object_pose, poser_data->userdata);
	} else {
		const FLT up[3] = {0, 0, 1};

		if (quatmagnitude(lighthouse_pose->Rot) == 0) {
			SurviveContext *ctx = so->ctx;
			SV_ERROR("Pose func called with invalid pose.");
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

		SurvivePose object2arb = {.Rot = {1.}};
		if (object_pose)
			object2arb = *object_pose;
		SurvivePose lighthouse2arb = *lighthouse_pose;

		// Start by just moving from whatever arbitrary space into object space.
		SurvivePose arb2object;
		InvertPose(&arb2object, &object2arb);

		SurvivePose lighthouse2obj;
		ApplyPoseToPose(&lighthouse2obj, &arb2object, &lighthouse2arb);

		// Now find the space with the same origin, but rotated so that gravity is up
		SurvivePose lighthouse2objUp = {}, object2objUp = {};
		if (quatmagnitude(so->activations.accel)) {
			quatfrom2vectors(object2objUp.Rot, so->activations.accel, up);
		} else {
			object2objUp.Rot[0] = 1.0;
		}

		// Calculate the pose of the lighthouse in this space
		ApplyPoseToPose(&lighthouse2objUp, &object2objUp, &lighthouse2obj);

		// Purposefully only set this once. It should only depend on the first (calculated) lighthouse
		if (quatmagnitude(objUp2world->Rot) == 0) {
			// Find what angle we need to rotate about Z by to get to 90 degrees.
			FLT ang = atan2(lighthouse2objUp.Pos[1], lighthouse2objUp.Pos[0]);
			FLT euler[3] = {0, 0, M_PI / 2. - ang};

			quatfromeuler(objUp2world->Rot, euler);
		}

		// Find find the poses that map to the above
		SurvivePose obj2world, lighthouse2world;
		ApplyPoseToPose(&obj2world, objUp2world, &object2objUp);
		ApplyPoseToPose(&lighthouse2world, objUp2world, &lighthouse2objUp);

		so->ctx->lighthouseposeproc(so->ctx, lighthouse, &lighthouse2world, &obj2world);
	}
}
