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

static void survive_fill_m(void *user, double *eq1, double *eq2, const double u, const double v) {
	SurviveObject *so = user;
	double sinu = sin(u), cosu = cos(u);
	double sinv = sin(v), cosv = cos(v);

	if (so->ctx->lh_version == 0) {
		eq1[0] = cosu;
		eq1[1] = 0.0;
		eq1[2] = -sinu;

		eq2[0] = 0.0;
		eq2[1] = cosv;
		eq2[2] = -sinv;
	} else {
		eq1[0] = cosu;
		eq1[1] = 1.0;
		eq1[2] = -sinu;

		eq2[0] = 1.0;
		eq2[1] = cosv;
		eq2[2] = -sinv;
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
	if (dd->bc.meas_cnt <= 3) {

		SV_WARN("Can't solve for only %u points\n", (int)dd->bc.meas_cnt);
		return rtn;
	}

	double r[3][3];

	double err = bc_svd_compute_pose(&dd->bc, r, rtn.Pos);

	CvMat R = cvMat(3, 3, CV_64F, r);
	CvMat T = cvMat(3, 1, CV_64F, rtn.Pos);

	// Super degenerate inputs will project us basically right in the camera. Detect and reject
	if (err > 1 || magnitude3d(rtn.Pos) < 0.25 || magnitude3d(rtn.Pos) > 25) {
		SV_WARN("pose is degenerate %d", (int)dd->bc.meas_cnt);
		return rtn;
	}

	// SV_INFO("EPNP for %s has err %f " SurvivePose_format, so->codename, err, SURVIVE_POSE_EXPAND(rtn));

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
			if (isnan(_ang[0]) || isnan(_ang[1]))
				continue;

			FLT ang[2];
			survive_apply_bsd_calibration(so->ctx, lh, _ang, ang);
			bc_svd_add_correspondence(&dd->bc, i, (ang[0]), (ang[1]));
		}

		SurviveContext *ctx = so->ctx;
		SV_INFO("Solving for %d correspondents", (int)dd->bc.meas_cnt);
		if (dd->bc.meas_cnt <= 4) {
			SV_INFO("Can't solve for only %d points on lh %d\n", (int)dd->bc.meas_cnt, lh);
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
		if (SurviveSensorActivations_isPairValid(scene, SurviveSensorActivations_default_tolerance * 4, timecode,
												 sensor_idx, lh)) {
			FLT *_angles = scene->angles[sensor_idx][lh];
			FLT angles[2];
			survive_apply_bsd_calibration(so->ctx, lh, _angles, angles);

			bc_svd_add_correspondence(bc, sensor_idx, (angles[0]), (angles[1]));
		}
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

		SurvivePose posers[2] = {0};
		int meas[2] = {0, 0};
		for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (so->ctx->bsd[lh].PositionSet) {
				add_correspondences(so, &dd->bc, lightData->timecode, lh);
				static int required_meas = -1;
				if (required_meas == -1)
					required_meas = survive_configi(so->ctx, "epnp-required-meas", SC_GET, 5);

				if (dd->bc.meas_cnt >= required_meas) {

					SurvivePose objInLh = solve_correspondence(dd, false);
					if (quatmagnitude(objInLh.Rot) != 0) {
						SurvivePose *lh2world = &so->ctx->bsd[lh].Pose;

						SurvivePose txPose = {.Rot = {1}};
						ApplyPoseToPose(&txPose, lh2world, &objInLh);
						posers[lh] = txPose;
						meas[lh] = dd->bc.meas_cnt;
					}
				}
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
