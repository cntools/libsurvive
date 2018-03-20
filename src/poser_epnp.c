
#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <poser.h>
#include <survive.h>

#include "epnp/epnp.h"
#include "linmath.h"
#include "math.h"
#include "stdio.h"

static SurvivePose solve_correspondence(SurviveObject *so, epnp *pnp, bool cameraToWorld) {
	SurvivePose rtn = {};
	// std::cerr << "Solving for " << cal_imagePoints.size() << " correspondents" << std::endl;
	if (pnp->number_of_correspondences <= 3) {
		SurviveContext *ctx = so->ctx;
		SV_INFO("Can't solve for only %u points\n", pnp->number_of_correspondences);
		return rtn;
	}

	double r[3][3];

	double err = epnp_compute_pose(pnp, r, rtn.Pos);

	CvMat R = cvMat(3, 3, CV_64F, r);
	CvMat T = cvMat(3, 1, CV_64F, rtn.Pos);
	// Requested output is camera -> world, so invert
	if (cameraToWorld) {
		FLT tmp[3];
		CvMat Tmp = cvMat(3, 1, CV_64F, tmp);
		cvCopyTo(&T, &Tmp);

		// Flip the Rotation matrix
		cvTranspose(&R, &R);
		// Then 'tvec = -R * tvec'
		cvGEMM(&R, &Tmp, -1, 0, 0, &T, 0);
	}

	FLT tmp[4];
	quatfrommatrix33(tmp, r[0]);

	// Typical camera applications have Z facing forward; the vive is contrarian and has Z going out of the
	// back of the lighthouse. Think of this as a rotation on the Y axis a full 180 degrees -- the quat for that is
	// [0 0x 1y 0z]
	const FLT rt[4] = {0, 0, 1, 0};
	quatrotateabout(rtn.Rot, tmp, rt);
	if (!cameraToWorld) {
		// We have to pre-multiply the rt transform here, which means we have to also offset our position by
		quatrotateabout(rtn.Rot, rt, tmp);
		rtn.Pos[0] = -rtn.Pos[0];
		rtn.Pos[2] = -rtn.Pos[2];
	}

	return rtn;
}
/*
static int survive_standardize_calibration(SurviveObject* so,
										   FLT upvec[3],
										   SurvivePose* _object2world,
										   SurvivePose* _lighthouses2world0,
										   SurvivePose* _lighthouses2world1) {
	SurvivePose object2world = {};
	SurvivePose lighthouses2world0 = *_lighthouses2world0;
	SurvivePose lighthouses2world1 = *_lighthouses2world1;

	const FLT up[3] = {0, 0, 1};
	quatfrom2vectors(object2world.Rot, so->activations.accel, _object2world->Pos);

	FLT tx[4][4];
	quattomatrix(tx, object2world.Rot);

	SurvivePose additionalTx = {0};

	SurvivePose lighthouse2world = {};
	// Lighthouse is now a tx from camera -> object
	ApplyPoseToPose(lighthouse2world.Pos,  object2world.Pos, lighthouse2object.Pos);

	if(quatmagnitude(additionalTx.Rot) == 0) {
		SurvivePose desiredPose = lighthouse2world;
		desiredPose.Pos[0] = 0.;

		quatfrom2vectors(additionalTx.Rot, lighthouse2world.Pos, desiredPose.Pos);
	}
	SurvivePose finalTx = {};
	ApplyPoseToPose(finalTx.Pos,  additionalTx.Pos, lighthouse2world.Pos);

}
*/
static int opencv_solver_fullscene(SurviveObject *so, PoserDataFullScene *pdfs) {
	SurvivePose object2world = {//.Rot = { 0.7325378, 0.4619398, 0.1913417, 0.4619398}
								.Rot = {1.}};
	const FLT up[3] = {0, 0, 1};

	SurvivePose actual;
	// quatfrom2vectors(object2world.Rot, so->activations.accel, up);
	// quatfrom2vectors(object2world.Rot, up, so->activations.accel);

	SurvivePose additionalTx = {0};

	for (int lh = 0; lh < 2; lh++) {
		epnp pnp = {.fu = 1, .fv = 1};
		epnp_set_maximum_number_of_correspondences(&pnp, so->sensor_ct);

		for (size_t i = 0; i < so->sensor_ct; i++) {
			FLT *lengths = pdfs->lengths[i][lh];
			FLT *ang = pdfs->angles[i][lh];
			if (lengths[0] < 0 || lengths[1] < 0)
				continue;

			epnp_add_correspondence(&pnp, so->sensor_locations[i * 3 + 0], so->sensor_locations[i * 3 + 1],
									so->sensor_locations[i * 3 + 2], tan(ang[0]), tan(ang[1]));
		}

		SurviveContext *ctx = so->ctx;
		SV_INFO("Solving for %d correspondents", pnp.number_of_correspondences);
		if (pnp.number_of_correspondences <= 4) {
			SV_INFO("Can't solve for only %d points on lh %d\n", pnp.number_of_correspondences, lh);
			continue;
		}

		SurvivePose lighthouse2object = solve_correspondence(so, &pnp, true);
		FLT euler[3];
		quattoeuler(euler, lighthouse2object.Rot);
		SurvivePose lighthouse2world = {};
		// Lighthouse is now a tx from camera -> object
		ApplyPoseToPose(lighthouse2world.Pos, object2world.Pos, lighthouse2object.Pos);

		if (false && quatmagnitude(additionalTx.Rot) == 0) {
			SurvivePose desiredPose = lighthouse2world;
			desiredPose.Pos[0] = 0.;

			quatfrom2vectors(additionalTx.Rot, lighthouse2world.Pos, desiredPose.Pos);
		}
		SurvivePose finalTx = lighthouse2world;
		// ApplyPoseToPose(finalTx.Pos, additionalTx.Pos, lighthouse2world.Pos);

		PoserData_lighthouse_pose_func(&pdfs->hdr, so, lh, &additionalTx, &finalTx, &object2world);

		epnp_dtor(&pnp);
	}

	return 0;
}

static void add_correspondences(SurviveObject *so, epnp *pnp, SurviveSensorActivations *scene,
								const PoserDataLight *lightData) {
	int lh = lightData->lh;
	for (size_t sensor_idx = 0; sensor_idx < so->sensor_ct; sensor_idx++) {
		if (SurviveSensorActivations_isPairValid(scene, SurviveSensorActivations_default_tolerance, lightData->timecode,
												 sensor_idx, lh)) {
			double *angles = scene->angles[sensor_idx][lh];
			epnp_add_correspondence(pnp, so->sensor_locations[sensor_idx * 3 + 0],
									so->sensor_locations[sensor_idx * 3 + 1], so->sensor_locations[sensor_idx * 3 + 2],
									tan(angles[0]), tan(angles[1]));
		}
	}
}

int PoserEPNP(SurviveObject *so, PoserData *pd) {

	SurviveSensorActivations *scene = &so->activations;
	switch (pd->pt) {
	case POSERDATA_IMU: {
		// Really should use this...
		PoserDataIMU *imuData = (PoserDataIMU *)pd;
		return 0;
	}
	case POSERDATA_LIGHT: {
		PoserDataLight *lightData = (PoserDataLight *)pd;

		SurvivePose posers[2];
		bool hasData[2] = {0, 0};
		for (int lh = 0; lh < 1; lh++) {
			if (so->ctx->bsd[lh].PositionSet) {
				epnp pnp = {.fu = 1, .fv = 1};
				epnp_set_maximum_number_of_correspondences(&pnp, so->sensor_ct);

				add_correspondences(so, &pnp, scene, lightData);

				if (pnp.number_of_correspondences > 4) {

					SurvivePose pose = solve_correspondence(so, &pnp, false);

					SurvivePose txPose = {};
					quatrotatevector(txPose.Pos, so->ctx->bsd[lh].Pose.Rot, pose.Pos);
					for (int i = 0; i < 3; i++) {
						txPose.Pos[i] += so->ctx->bsd[lh].Pose.Pos[i];
					}

					quatrotateabout(txPose.Rot, so->ctx->bsd[lh].Pose.Rot, pose.Rot);

					posers[lh] = txPose;
					hasData[lh] = 1;
				}

				epnp_dtor(&pnp);
			}
		}

		if (hasData[0] && hasData[1]) {
			SurvivePose interpolate = {0};
			for (size_t i = 0; i < 3; i++) {
				interpolate.Pos[i] = (posers[0].Pos[i] + posers[1].Pos[i]) / 2.;
			}
			quatslerp(interpolate.Rot, posers[0].Rot, posers[1].Rot, .5);
			PoserData_poser_raw_pose_func(pd, so, lightData->lh, &interpolate);
		} else {
			if (hasData[lightData->lh])
				PoserData_poser_raw_pose_func(pd, so, lightData->lh, &posers[lightData->lh]);
		}
		return 0;
	}
	case POSERDATA_FULL_SCENE: {
		return opencv_solver_fullscene(so, (PoserDataFullScene *)(pd));
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserEPNP);
