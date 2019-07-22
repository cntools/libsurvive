#include "barycentric_svd/barycentric_svd.h"
#include "math.h"
#include "minimal_opencv.h"
#include <stdio.h>
#include <stdlib.h>
#include <survive.h>
#include <survive_reproject.h>

typedef struct {
	SurviveObject *so;

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
		SV_WARN("pose is degenerate %d %f", (int)dd->bc.meas_cnt, err);
		double err = bc_svd_compute_pose(&dd->bc, r, rtn.Pos);
		return rtn;
	}

	static SurvivePose p = {0};
	if (!quatiszero(p.Rot)) {
		if (dist3d(rtn.Pos, p.Pos) > 1) {
			p = (SurvivePose){0};
		}
	} else {
		p = rtn;
		p.Rot[0] = 1;
	}

	// SV_INFO("BaryCentricSVD for %s has err %f " SurvivePose_format, so->codename, err, SURVIVE_POSE_EXPAND(rtn));

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
	SurvivePose arb2world = {0};
	for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
		bc_svd_reset_correspondences(&dd->bc);
		for (size_t i = 0; i < so->sensor_ct; i++) {
			FLT *_ang = pdfs->angles[i][lh];
			bc_svd_add_correspondence(&dd->bc, i, (_ang[0]), (_ang[1]));
		}

		SurviveContext *ctx = so->ctx;
		SV_INFO("Solving for %d correspondents on lh %d", (int)dd->bc.meas_cnt, lh);
		if (dd->bc.meas_cnt <= 4) {
			SV_INFO("Can't solve for only %d points on lh %d", (int)dd->bc.meas_cnt, lh);
			continue;
		}

		SurvivePose lighthouse2object = solve_correspondence(dd, true);
		if (quatmagnitude(lighthouse2object.Rot) != 0.0) {
			PoserData_lighthouse_pose_func(&pdfs->hdr, so, lh, &arb2world, &lighthouse2object, 0);
		}
	}

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

		SurvivePose posers[NUM_GEN2_LIGHTHOUSES] = {0};
		int meas[NUM_GEN2_LIGHTHOUSES] = {0};

		for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (so->ctx->bsd[lh].PositionSet) {
				add_correspondences(so, &dd->bc, lightData->timecode, lh);
				static int required_meas = -1;
				if (required_meas == -1)
					required_meas = survive_configi(so->ctx, "epnp-required-meas", SC_GET, 10);

				if (dd->bc.meas_cnt >= required_meas) {

					SurvivePose objInLh = solve_correspondence(dd, false);
					if (quatmagnitude(objInLh.Rot) != 0) {
						SurvivePose *lh2world = &so->ctx->bsd[lh].Pose;

						SurvivePose txPose = {.Rot = {1}};
						ApplyPoseToPose(&txPose, lh2world, &objInLh);
						posers[lh] = txPose;
						meas[lh] = dd->bc.meas_cnt;
					} else {
						SV_INFO("Localization failed for lh %d", lh);
					}
				}
			}
		}

		int maxMeas = 0;
		int maxMeasIdx = 0;
		for (int lh = 0; lh < NUM_GEN2_LIGHTHOUSES; lh++) {
			if (quatiszero(posers[lh].Rot))
				continue;

			if (maxMeas < meas[lh]) {
				maxMeas = meas[lh];
				maxMeasIdx = lh;
			}
		}

		if (maxMeas > 0 && !quatiszero(posers[maxMeasIdx].Rot)) {
			PoserData_poser_pose_func(pd, so, &posers[maxMeasIdx]);
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
