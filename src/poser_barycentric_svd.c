#include "barycentric_svd/barycentric_svd.h"
#include "math.h"
#include "minimal_opencv.h"
#include <stdio.h>
#include <stdlib.h>
#include <survive.h>
#include <survive_reproject.h>

typedef struct {
	SurviveObject *so;
	uint32_t required_meas;

	FLT max_error_obj;
	FLT max_error_cal;

	bc_svd bc;
} PoserDataSVD;

typedef FLT LinmathPoint2d[2];

static void survive_fill_m(void *user, double *eq, int axis, FLT angle) {
	SurviveObject *so = user;

	double sv = sin(angle), cv = cos(angle);
	switch (so->ctx->lh_version) {
	case 0: {
		switch (axis) {
		case 0:
			eq[0] = cv;
			eq[1] = 0;
			eq[2] = -sv;
			break;
		case 1:
			eq[0] = 0;
			eq[1] = cv;
			eq[2] = -sv;
			break;
		}

	} break;
	case 1: {
		FLT tan30 = 0.57735026919;
		switch (axis) {
		case 0:
			eq[0] = cv;
			eq[1] = -tan30;
			eq[2] = -sv;
			break;
		case 1:
			eq[0] = cv;
			eq[1] = tan30;
			eq[2] = -sv;
			break;
		}
	} break;
	default:
		assert(false);
	}
}

static PoserDataSVD *PoserDataSVD_new(SurviveObject *so) {
	PoserDataSVD *rtn = calloc(sizeof(PoserDataSVD), 1);
	rtn->so = so;
	rtn->required_meas = survive_configi(so->ctx, "epnp-required-meas", SC_GET, 10);

	survive_attach_configf(so->ctx, "max-error", &rtn->max_error_obj);
	survive_attach_configf(so->ctx, "max-cal-error", &rtn->max_error_cal);

	bc_svd_bc_svd(&rtn->bc, so, survive_fill_m, (LinmathPoint3d *)so->sensor_locations, so->sensor_ct);

	return rtn;
}

static SurvivePose solve_correspondence(PoserDataSVD *dd, bool cameraToWorld) {
	SurviveObject *so = dd->so;
	SurvivePose rtn = {0};
	SurviveContext *ctx = so->ctx;
	// std::cerr << "Solving for " << cal_imagePoints.size() << " correspondents" << std::endl;
	if (dd->bc.meas_cnt <= 6) {
		SV_WARN("Can't solve for only %u points\n", (int)dd->bc.meas_cnt);
		return rtn;
	}

	double r[3][3];

	double err = bc_svd_compute_pose(&dd->bc, r, rtn.Pos);
	if (err < 0) {
		return rtn;
	}

	CvMat R = cvMat(3, 3, CV_64F, r);
	CvMat T = cvMat(3, 1, CV_64F, rtn.Pos);

	// Super degenerate inputs will project us basically right in the camera. Detect and reject
	if (err > 1 || magnitude3d(rtn.Pos) < 0.25 || magnitude3d(rtn.Pos) > 25) {
		SV_WARN("pose is degenerate %d %f %f", (int)dd->bc.meas_cnt, err, magnitude3d(rtn.Pos));
		return rtn;
	}

	// SV_INFO("BaryCentricSVD for %s has err %f " SurvivePose_format, so->codename, err, SURVIVE_POSE_EXPAND(rtn));

	FLT allowable_error = cameraToWorld ? dd->max_error_cal : dd->max_error_obj;
	if (allowable_error < err) {
		if (cameraToWorld) {
			SV_WARN("Camera reprojection error was too high: %f for %d meas", err, dd->bc.meas_cnt);
		}
		return rtn;
	}

	// Requested output is camera -> world, so invert
	if (cameraToWorld) {
		FLT tmp[3];
		CvMat Tmp = cvMat(3, 1, CV_64F, tmp);
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

static int solve_fullscene(PoserDataSVD *dd, PoserDataFullScene *pdfs) {
	SurviveObject *so = dd->so;

	SurvivePose object2World = so->OutPoseIMU;

	// If a LH has its position; wait until we have a position on the object we can use.
	if (quatiszero(object2World.Rot)) {
		for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (so->ctx->bsd[lh].PositionSet) {
				return 0;
			}
		}
	}

	SurvivePose lh2objects[NUM_GEN2_LIGHTHOUSES] = { 0 };

	for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
		if (so->ctx->bsd[lh].PositionSet) {
			continue;
		}

		bc_svd_reset_correspondences(&dd->bc);
		for (size_t i = 0; i < so->sensor_ct; i++) {
			FLT *_ang = pdfs->angles[i][lh];
			bc_svd_add_correspondence(&dd->bc, i, (_ang[0]), (_ang[1]));
		}

		SurviveContext *ctx = so->ctx;
		if (dd->bc.meas_cnt <= 8) {
			continue;
		}

		SV_INFO("Solving for %d correspondents on lh %d", (int)dd->bc.meas_cnt, lh);
		lh2objects[lh] = solve_correspondence(dd, true);
	}

	PoserData_lighthouse_poses_func(&pdfs->hdr, so, lh2objects, so->ctx->activeLighthouses, 0);

	return 0;
}

static void add_correspondences(SurviveObject *so, bc_svd *bc, uint32_t timecode, int lh) {
	SurviveSensorActivations *scene = &so->activations;
	bc_svd_reset_correspondences(bc);
	for (size_t sensor_idx = 0; sensor_idx < so->sensor_ct; sensor_idx++) {
		FLT angles[2] = {NAN, NAN};
		for (uint8_t axis = 0; axis < 2; axis++) {
			bool isReadingValue = SurviveSensorActivations_isReadingValid(
				scene, SurviveSensorActivations_default_tolerance * 2, timecode, sensor_idx, lh, axis);

			if (isReadingValue) {
				angles[axis] = scene->angles[sensor_idx][lh][axis];
			}
		}

		survive_apply_bsd_calibration(so->ctx, lh, angles, angles);
		bc_svd_add_correspondence(bc, sensor_idx, (angles[0]), (angles[1]));
	}
}

int PoserBaryCentricSVD(SurviveObject *so, PoserData *pd) {
	PoserType pt = pd->pt;
	SurviveContext *ctx = so->ctx;
	PoserDataSVD *dd = so->PoserData;

	if (!dd)
		so->PoserData = dd = PoserDataSVD_new(so);

	switch (pt) {
	case POSERDATA_SYNC:
	case POSERDATA_SYNC_GEN2: {
		PoserDataLight *lightData = (PoserDataLight *)pd;
		SurviveContext *ctx = so->ctx;

		SurvivePose obj2world = { 0 };
		bool allowSolveLH = !lightData->no_lighthouse_solve;
		if (lightData->assume_current_pose == false) {
			bool canSolveLH = lightData->assume_current_pose;
			SurvivePose objs2world[NUM_GEN2_LIGHTHOUSES] = {0};
			int meas[NUM_GEN2_LIGHTHOUSES] = {0};
			bool hasLighthousePoses = false;
			bool hasUnsolvedLighthousePoses = false;
			for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
				if (so->ctx->bsd[lh].PositionSet) {
					hasLighthousePoses = true;
					add_correspondences(so, &dd->bc, lightData->hdr.timecode, lh);

					if (dd->bc.meas_cnt >= dd->required_meas) {

						SurvivePose obj2Lh = solve_correspondence(dd, false);
						if (quatmagnitude(obj2Lh.Rot) != 0) {
							SurvivePose *lh2world = &so->ctx->bsd[lh].Pose;

							ApplyPoseToPose(&objs2world[lh], lh2world, &obj2Lh);
							meas[lh] = dd->bc.meas_cnt;
						}
					}
				} else {
					hasUnsolvedLighthousePoses = true;
				}
			}

			int maxMeas = 0;
			int maxMeasIdx = 0;
			for (int lh = 0; lh < NUM_GEN2_LIGHTHOUSES; lh++) {
				if (quatiszero(objs2world[lh].Rot))
					continue;

				if (maxMeas < meas[lh]) {
					maxMeas = meas[lh];
					maxMeasIdx = lh;
				}
			}

			canSolveLH = !hasLighthousePoses;
			if (maxMeas > 0 && !quatiszero(objs2world[maxMeasIdx].Rot)) {
				obj2world = objs2world[maxMeasIdx];
				PoserData_poser_pose_func(pd, so, &obj2world);
				canSolveLH = hasUnsolvedLighthousePoses;
			}

			allowSolveLH &= canSolveLH;
		} else {
			obj2world = so->OutPoseIMU;
		}

		// If we haven't moved for a second; try to solve for unsolved lighthouses
		if (allowSolveLH && SurviveSensorActivations_stationary_time(&so->activations) > so->timebase_hz) {
			SurvivePose lh2world[NUM_GEN2_LIGHTHOUSES] = { 0 };
			int solved = 0;
			for (int lh = 0; lh < ctx->activeLighthouses; lh++) {
				if (!so->ctx->bsd[lh].PositionSet && so->ctx->bsd[lh].OOTXSet) {
					add_correspondences(so, &dd->bc, lightData->hdr.timecode, lh);

					if (dd->bc.meas_cnt >= dd->required_meas) {
						SurvivePose lh2obj = solve_correspondence(dd, true);
						if (quatmagnitude(lh2obj.Rot) != 0) {
							solved++;
							SV_VERBOSE(5, "Possible SVD solution for %d", lh);
							if (quatiszero(obj2world.Rot))
								lh2world[lh] = lh2obj;
							else
								ApplyPoseToPose(&lh2world[lh], &obj2world, &lh2obj);
						}
					}
				}
			}

			if (solved) {
				PoserData_lighthouse_poses_func(pd, so, lh2world, so->ctx->activeLighthouses, &obj2world);
			}
		}

		return 0;
	}

	case POSERDATA_FULL_SCENE: {
		return solve_fullscene(dd, (PoserDataFullScene *)(pd));
	}
	case POSERDATA_DISASSOCIATE: {
		free(dd);
		so->PoserData = 0;
		// printf( "Need to disassociate.\n" );
		break;
	}
	}
	return 0;
}

REGISTER_LINKTIME(PoserBaryCentricSVD);
