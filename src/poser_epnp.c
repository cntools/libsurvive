#include <poser.h>
#include <survive.h>
#include <survive_reproject.h>

#include "epnp/epnp.h"
#include "linmath.h"
#include "math.h"
#include "stdio.h"

static SurvivePose solve_correspondence(SurviveObject *so, epnp *pnp, bool cameraToWorld) {
	SurvivePose rtn = {0};
	SurviveContext *ctx = so->ctx;
	// std::cerr << "Solving for " << cal_imagePoints.size() << " correspondents" << std::endl;
	if (pnp->number_of_correspondences <= 3) {

		SV_WARN("Can't solve for only %u points\n", pnp->number_of_correspondences);
		return rtn;
	}

	FLT r[3][3];

	FLT err = epnp_compute_pose(pnp, r, rtn.Pos);

	CnMat R = cvMat(3, 3, SV_FLT, r);
	CnMat T = cvMat(3, 1, SV_FLT, rtn.Pos);

	// Super degenerate inputs will project us basically right in the camera. Detect and reject
	if (err > 2 || magnitude3d(rtn.Pos) < 0.25 || magnitude3d(rtn.Pos) > 25) {
		// err = epnp_compute_pose(pnp, r, rtn.Pos);

		SV_WARN("EPNP pose is degenerate %d, err %f", pnp->number_of_correspondences, err);
		return (SurvivePose){0};
	}

	// SV_INFO("EPNP for %s has err %f " SurvivePose_format, so->codename, err, SURVIVE_POSE_EXPAND(rtn));

	// Requested output is camera -> world, so invert
	if (cameraToWorld) {
		FLT tmp[3];
		CnMat Tmp = cvMat(3, 1, SV_FLT, tmp);
		cvCopy(&T, &Tmp, 0);

		// Flip the Rotation matrix
		cvTranspose(&R, &R);
		// Then 'tvec = -R * tvec'
		cvGEMM(&R, &Tmp, -1, 0, 0, &T, 0);
	}

	LinmathQuat tmp;
	quatfrommatrix33(tmp, r[0]);

	// Typical camera applications have Z facing forward; the vive is contrarian and has Z going out of the
	// back of the lighthouse. Think of this as a rotation on the Y axis a full 180 degrees -- the quat for that is
	// [0 0x 1y 0z]
	const LinmathQuat rt = {0, 0, 1, 0};

	quatrotateabout(rtn.Rot, tmp, rt);
	if (!cameraToWorld) {
		// We have to pre-multiply the rt transform here, which means we have to also offset our position by
		quatrotateabout(rtn.Rot, rt, tmp);
		rtn.Pos[0] = -rtn.Pos[0];
		rtn.Pos[2] = -rtn.Pos[2];
	}

	return rtn;
}

static FLT get_u(const FLT *ang) { return tan(ang[0]); }
static FLT get_v(const FLT *ang) { return tan(ang[1]); }

static int opencv_solver_fullscene(SurviveObject *so, PoserDataFullScene *pdfs) {
	SurvivePose lh2object[NUM_GEN2_LIGHTHOUSES] = { 0 };
	for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
		epnp pnp = {.fu = 1, .fv = 1};
		epnp_set_maximum_number_of_correspondences(&pnp, so->sensor_ct);

		for (size_t i = 0; i < so->sensor_ct; i++) {
			FLT *_ang = pdfs->angles[i][lh];
			if (isnan(_ang[0]) || isnan(_ang[1]))
				continue;

			FLT ang[2];
			survive_apply_bsd_calibration(so->ctx, lh, _ang, ang);

			epnp_add_correspondence(&pnp, so->sensor_locations[i * 3 + 0], so->sensor_locations[i * 3 + 1],
									so->sensor_locations[i * 3 + 2], get_u(ang), get_v(ang));
		}

		SurviveContext *ctx = so->ctx;
		SV_INFO("Solving for %d correspondents", pnp.number_of_correspondences);
		if (pnp.number_of_correspondences <= 4) {
			SV_INFO("Can't solve for only %d points on lh %d\n", pnp.number_of_correspondences, lh);
			continue;
		}

		lh2object[lh] = solve_correspondence(so, &pnp, true);

		epnp_dtor(&pnp);
	}

	PoserData_lighthouse_poses_func(&pdfs->hdr, so, lh2object, so->ctx->activeLighthouses, 0);

	return 0;
}

static void add_correspondences(SurviveObject *so, epnp *pnp, SurviveSensorActivations *scene, uint32_t timecode,
								int lh) {
	for (size_t sensor_idx = 0; sensor_idx < so->sensor_ct; sensor_idx++) {
		if (SurviveSensorActivations_isPairValid(scene, SurviveSensorActivations_default_tolerance * 4, timecode,
												 sensor_idx, lh)) {
			FLT *_angles = scene->angles[sensor_idx][lh];
			FLT angles[2];
			survive_apply_bsd_calibration(so->ctx, lh, _angles, angles);

			epnp_add_correspondence(pnp, so->sensor_locations[sensor_idx * 3 + 0],
									so->sensor_locations[sensor_idx * 3 + 1], so->sensor_locations[sensor_idx * 3 + 2],
									get_u(angles), get_v(angles));
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
	case POSERDATA_SYNC: {
		PoserDataLight *lightData = (PoserDataLight *)pd;
		SurviveContext *ctx = so->ctx;

		SurvivePose posers[NUM_GEN2_LIGHTHOUSES] = {0};
		int meas[2] = {0, 0};
		for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (so->ctx->bsd[lh].PositionSet) {
				epnp pnp = {.fu = 1, .fv = 1};
				epnp_set_maximum_number_of_correspondences(&pnp, so->sensor_ct);

				add_correspondences(so, &pnp, scene, pd->timecode, lh);
				static int required_meas = -1;
				if (required_meas == -1)
					required_meas = survive_configi(so->ctx, "epnp-required-meas", SC_GET, 5);

				if (pnp.number_of_correspondences >= required_meas) {

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
				PoserData_poser_pose_func(pd, so, &posers[winner]);
			} else {
				double a, b;
				a = meas[0] * meas[0];
				b = meas[1] * meas[1];

				double t = a + b;
				for (size_t i = 0; i < 3; i++) {
					interpolate.Pos[i] = (posers[0].Pos[i] * a + posers[1].Pos[i] * b) / (t);
				}
				quatslerp(interpolate.Rot, posers[0].Rot, posers[1].Rot, b / (t));
				PoserData_poser_pose_func(pd, so, &interpolate);
			}
		} else {
			if (meas[lightData->lh])
				PoserData_poser_pose_func(pd, so, &posers[lightData->lh]);
			else if (meas[!lightData->lh]) {
				PoserData_poser_pose_func(pd, so, &posers[!lightData->lh]);
			}
		}
		return 0;
	}
	case POSERDATA_FULL_SCENE: {
		return opencv_solver_fullscene(so, (PoserDataFullScene *)(pd));
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserEPNP)
