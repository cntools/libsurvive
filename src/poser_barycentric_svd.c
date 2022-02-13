#include "barycentric_svd/barycentric_svd.h"
#include "cnmatrix/cn_matrix.h"
#include "math.h"
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <survive.h>
#include <survive_reproject.h>

#pragma GCC diagnostic ignored "-Wpedantic"

typedef struct {
	SurviveObject *so;
	uint32_t required_meas;

	FLT max_error_obj;
	FLT max_error_cal;

	bc_svd bc;
} PoserDataSVD;

static void survive_fill_m(void *user, FLT *eq, int axis, FLT angle) {
	SurviveObject *so = user;

	FLT sv = sin(angle), cv = cos(angle);
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
	case 3: {
		eq[0] = eq[1] = eq[2] = 0;
		break;
	}
	default:
		assert(false);
	}
}

static void PoserDataSVD_destroy(PoserDataSVD *dd) {
	bc_svd_dtor(&dd->bc);
	survive_detach_config(dd->so->ctx, "max-error", &dd->max_error_obj);
	survive_detach_config(dd->so->ctx, "max-cal-error", &dd->max_error_cal);
	free(dd);
}

static PoserDataSVD *PoserDataSVD_new(SurviveObject *so) {
	PoserDataSVD *rtn = SV_CALLOC(sizeof(PoserDataSVD));
	rtn->so = so;
	rtn->required_meas = survive_configi(so->ctx, "epnp-required-meas", SC_GET, 8);

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
	if (dd->bc.meas_cnt < 2) {
		SV_WARN("Can't solve for only %u points", (int)dd->bc.meas_cnt);
		return rtn;
	}

	FLT r[3][3];

	FLT err = bc_svd_compute_pose(&dd->bc, r, rtn.Pos);
	if (err < 0) {
		return rtn;
	}

	CN_CREATE_STACK_MAT(R, 3, 3);
	cn_copy_in_row_major(&R, (FLT *)r, 3);

	CnMat T = cnMat(3, 1, rtn.Pos);

	// Super degenerate inputs will project us basically right in the camera. Detect and reject
	if (err > 1 || magnitude3d(rtn.Pos) < 0.25 || magnitude3d(rtn.Pos) > 25) {
		SV_VERBOSE(200, "pose is degenerate %d %f %f", (int)dd->bc.meas_cnt, err, magnitude3d(rtn.Pos));
		return rtn;
	}

	SV_VERBOSE(200, "BaryCentricSVD for %s has err %f " SurvivePose_format " (Solving for camera: %d at %f)",
			   so->codename, err, SURVIVE_POSE_EXPAND(rtn), cameraToWorld, survive_run_time(ctx));

	FLT allowable_error = (cameraToWorld ? (dd->max_error_cal) : dd->max_error_obj) * 100.0;
	if (allowable_error < err) {
		if (cameraToWorld) {
			SV_WARN("Camera reprojection error was too high: %f for %d meas", err, (int)dd->bc.meas_cnt);
		}
		goto cleanup;
	}

	// Requested output is camera -> world, so invert
	if (cameraToWorld) {
		FLT tmp[3];
		CnMat Tmp = cnMat(3, 1, tmp);
		cnCopy(&T, &Tmp, 0);

		// Flip the Rotation matrix
		cnTranspose(&R, &R);
		// Then 'tvec = -R * tvec'
		cnGEMM(&R, &Tmp, -1, 0, 0, &T, 0);
	}

	LinmathQuat tmp;
	quatfromcnMatrix(tmp, &R);

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

cleanup:
	CN_FREE_STACK_MAT(R);

	return rtn;
}

static void add_correspondences(SurviveObject *so, bc_svd *bc, uint32_t timecode, int lh) {
	SurviveSensorActivations *scene = &so->activations;
	bc_svd_reset_correspondences(bc);

	bool isStationary = SurviveSensorActivations_stationary_time(scene) > so->timebase_hz;
	survive_timecode sensor_time_window =
		isStationary ? (so->timebase_hz) : SurviveSensorActivations_default_tolerance * 2;

	for (size_t sensor_idx = 0; sensor_idx < so->sensor_ct; sensor_idx++) {
		FLT angles[2] = {NAN, NAN};
		for (uint8_t axis = 0; axis < 2; axis++) {
			bool isReadingValid =
				SurviveSensorActivations_is_reading_valid(scene, sensor_time_window, sensor_idx, lh, axis);

			if (isReadingValid) {
				angles[axis] = scene->angles[sensor_idx][lh][axis];
			}
		}

		survive_apply_bsd_calibration(so->ctx, lh, angles, angles);
		bc_svd_add_correspondence(bc, sensor_idx, (angles[0]), (angles[1]));
	}
}

void solve_global_scene(struct SurviveObject *so, PoserDataSVD *dd, PoserDataGlobalScenes *gss) {
	SurviveContext *ctx = so->ctx;

	if (gss->scenes == 0 || gss->scenes_cnt == 0)
		return;
	struct PoserDataGlobalScene *scene = gss->scenes;

	bool needsObject = quatiszero(scene->pose.Rot);

	for (int lh = 0; lh < ctx->activeLighthouses; lh++) {

		bool needsLH = quatiszero(gss->world2lhs[lh].Rot);

		if (needsObject && needsLH)
			continue;

		if (!needsObject && !needsLH)
			continue;

		bc_svd *bc = &dd->bc;
		bc_svd_reset_correspondences(bc);

		const BaseStationCal *cal = ctx->bsd[lh].fcal;
		for (size_t m = 0; m < scene->meas_cnt; m++) {
			if (scene->meas[m].lh == lh)
				bc_svd_add_single_correspondence(bc, scene->meas[m].sensor_idx, scene->meas[m].axis,
												 scene->meas[m].value + cal[scene->meas[m].axis].phase);
		}

		if (dd->bc.meas_cnt >= dd->required_meas) {
			SurvivePose lh2obj = solve_correspondence(dd, true);
			if (quatmagnitude(lh2obj.Rot) != 0) {
				if (needsLH) {
					SurvivePose obj2world = scene->pose;
					SurvivePose lh2world;
					ApplyPoseToPose(&lh2world, &obj2world, &lh2obj);

					PoserData_lighthouse_pose_func(&gss->hdr, so, lh, &lh2world, 0, 0);
				} else if (needsObject) {
					SurvivePose world2lh = gss->world2lhs[lh];
					SurvivePose lh2world = InvertPoseRtn(&world2lh);
					SurvivePose obj2lh = InvertPoseRtn(&lh2obj);
					SurvivePose obj2world;
					ApplyPoseToPose(&obj2world, &lh2world, &obj2lh);
					PoserData_poser_pose_func(&gss->hdr, so, &obj2world, -1, 0);
					scene->pose = obj2world;
					needsObject = false;
				}
			}
		}
	}
}
int PoserBaryCentricSVD(SurviveObject *so, PoserData *pd) {
	PoserType pt = pd->pt;
	SurviveContext *ctx = so->ctx;
	void **user = survive_object_plugin_data(so, PoserBaryCentricSVD);

	PoserDataSVD *dd = *user;

	if (pt == POSERDATA_DISASSOCIATE && dd == 0)
		return 0;

	if (!dd) {

		*user = dd = PoserDataSVD_new(so);
	}

	switch (pt) {
	case POSERDATA_GLOBAL_SCENES: {
		// dd->globalDataAvailable = true;
		PoserDataGlobalScenes *gs = (PoserDataGlobalScenes *)pd;
		solve_global_scene(so, dd, gs);
		return 0;
	}
	case POSERDATA_SYNC:
	case POSERDATA_SYNC_GEN2: {
		if (so->has_sensor_locations == false) {
			break;
		}

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

						survive_release_ctx_lock(so->ctx);
						SurvivePose obj2Lh = solve_correspondence(dd, false);
						survive_get_ctx_lock(so->ctx);

						if (quatmagnitude(obj2Lh.Rot) != 0) {
							const SurvivePose *lh2world = survive_get_lighthouse_position(so->ctx, lh);

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
				PoserData_poser_pose_func(pd, so, &obj2world, -1, 0);
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
							LinmathPoint3d up = {ctx->bsd[lh].accel[0], ctx->bsd[lh].accel[1], ctx->bsd[lh].accel[2]};
							FLT err = 0;

							// Some older replays don't have the accel
							if (norm3d(up) > 0) {
								normalize3d(up, up);
								LinmathPoint3d lhUpInObj;
								quatrotatevector(lhUpInObj, lh2obj.Rot, up);

								LinmathPoint3d objUp;
								normalize3d(objUp, so->activations.accel);

								LinmathQuat err_q;
								quatfind_between_vectors(err_q, objUp, lhUpInObj);
								err = 1. - err_q[0];
							}

							if (err < .25) {
								solved++;
								if (quatiszero(obj2world.Rot))
									lh2world[lh] = lh2obj;
								else
									ApplyPoseToPose(&lh2world[lh], &obj2world, &lh2obj);

								SV_VERBOSE(
									10,
									"Possible SVD solution for lighthouse %d, from object %s at " SurvivePose_format
									"; accel error is %6.5f",
									lh, so->codename, SURVIVE_POSE_EXPAND(lh2world[lh]), err);
							} else {
								SV_VERBOSE(5, "Discarding SVD solution; up vectors seemed wrong %f", err);
							}
						}
					} else {
						SV_WARN("Couldn't solve for LH %d with %d measures", lh, (int)dd->bc.meas_cnt);
					}
				}
			}

			if (solved) {
				PoserData_lighthouse_poses_func(pd, so, lh2world, 0, so->ctx->activeLighthouses, &obj2world);
			}
		}

		return 0;
	}
	case POSERDATA_DISASSOCIATE: {
		*user = 0;
		PoserDataSVD_destroy(dd);
		// printf( "Need to disassociate.\n" );
		break;
	}
	}
	return 0;
}

REGISTER_POSER(PoserBaryCentricSVD)
