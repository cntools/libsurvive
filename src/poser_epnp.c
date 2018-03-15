#include "persistent_scene.h"

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
	if (pnp->number_of_correspondences <= 4) {
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
		print_mat(&R);
		print_mat(&T);
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

static int opencv_solver_fullscene(SurviveObject *so, PoserDataFullScene *pdfs) {

	for (int lh = 0; lh < 2; lh++) {
		epnp pnp = {.fu = 1, .fv = 1};
		epnp_set_maximum_number_of_correspondences(&pnp, so->nr_locations);

		for (size_t i = 0; i < so->nr_locations; i++) {
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

		SurvivePose lighthouse = solve_correspondence(so, &pnp, true);
		PoserData_lighthouse_pose_func(&pdfs->hdr, so, lh, &lighthouse);
	}
	return 0;
}

struct add_correspondence_for_lh {
	epnp *pnp;
	int lh;
};

void add_correspondence_for_lh(SurviveObject *so, int lh, int sensor_idx, FLT *angles, void *_user) {
	struct add_correspondence_for_lh *user = (struct add_correspondence_for_lh *)_user;
	if (user->lh == lh)
		epnp_add_correspondence(user->pnp, so->sensor_locations[sensor_idx * 3 + 0],
								so->sensor_locations[sensor_idx * 3 + 1], so->sensor_locations[sensor_idx * 3 + 2],
								tan(angles[0]), tan(angles[1]));
}

int PoserEPNP(SurviveObject *so, PoserData *pd) {
	switch (pd->pt) {
	case POSERDATA_IMU: {
		// Really should use this...
		PoserDataIMU *imuData = (PoserDataIMU *)pd;
		return 0;
	}
	case POSERDATA_LIGHT: {
		static PersistentScene _scene = {.tolerance = 1500000};
		PersistentScene *scene = &_scene;
		PoserDataLight *lightData = (PoserDataLight *)pd;

		PersistentScene_add(scene, so, lightData);

		int lh = lightData->lh;
		if (so->ctx->bsd[lh].PositionSet) {
			epnp pnp = {.fu = 1, .fv = 1};
			epnp_set_maximum_number_of_correspondences(&pnp, so->nr_locations);

			struct add_correspondence_for_lh user = {.lh = lh, .pnp = &pnp};
			PersistentScene_ForEachCorrespondence(scene, add_correspondence_for_lh, so, lightData->timecode, &user);

			if (pnp.number_of_correspondences > 4) {

				SurvivePose pose = solve_correspondence(so, &pnp, false);

				SurvivePose txPose = {};
				quatrotatevector(txPose.Pos, so->ctx->bsd[lh].Pose.Rot, pose.Pos);
				for (int i = 0; i < 3; i++) {
					txPose.Pos[i] += so->ctx->bsd[lh].Pose.Pos[i];
				}

				quatrotateabout(txPose.Rot, so->ctx->bsd[lh].Pose.Rot, pose.Rot);
				PoserData_poser_raw_pose_func(pd, so, lh, &txPose);
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

REGISTER_LINKTIME(PoserEPNP);
