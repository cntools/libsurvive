#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <malloc.h>

#include "mpfit/mpfit.h"
#include "poser.h"
#include <survive.h>
#include <survive_imu.h>

#include "assert.h"
#include "linmath.h"
#include "math.h"
#include "poser_general_optimizer.h"
#include "string.h"
#include "survive_cal.h"
#include "survive_config.h"
#include "survive_reproject.h"

typedef struct {
	SurviveObject *so;
	FLT *pts3d;
	int *lh;
	FLT *meas;
	SurvivePose camera_params[2];
} mpfit_context;

typedef struct MPFITData {
	GeneralOptimizerData opt;

	int last_acode;
	int last_lh;

	int sensor_time_window;
	int use_jacobian_function;
	int required_meas;

	SurviveIMUTracker tracker;
	bool useIMU;

	struct {
		int meas_failures;
	} stats;
} MPFITData;

static size_t construct_input_from_scene(MPFITData *d, PoserDataLight *pdl, SurviveSensorActivations *scene,
										 double *meas, double *sensors, int *lhs) {
	size_t rtn = 0;
	SurviveObject *so = d->opt.so;

	for (size_t sensor = 0; sensor < so->sensor_ct; sensor++) {
		for (size_t lh = 0; lh < 2; lh++) {
			if (SurviveSensorActivations_isPairValid(scene, d->sensor_time_window, pdl->timecode, sensor, lh)) {
				const double *a = scene->angles[sensor][lh];

				sensors[rtn * 3 + 0] = so->sensor_locations[sensor * 3 + 0];
				sensors[rtn * 3 + 1] = so->sensor_locations[sensor * 3 + 1];
				sensors[rtn * 3 + 2] = so->sensor_locations[sensor * 3 + 2];
				meas[rtn * 2 + 0] = a[0];
				meas[rtn * 2 + 1] = a[1];
				lhs[rtn] = lh;
				rtn++;
			}
		}
	}
	return rtn;
}

static void str_metric_function(int j, int i, double *bi, double *xij, void *adata) {
	SurvivePose obj = *(SurvivePose *)bi;
	int sensor_idx = j >> 1;
	int lh = j & 1;

	mpfit_context *ctx = (mpfit_context *)(adata);
	SurviveObject *so = ctx->so;

	assert(lh < 2);
	assert(sensor_idx < so->sensor_ct);

	// quatnormalize(obj.Rot, obj.Rot);

	// std::cerr << "Processing " << sensor_idx << ", " << lh << std::endl;
	SurvivePose *camera = &so->ctx->bsd[lh].Pose;
	survive_reproject_full(so->ctx->bsd[lh].fcal, camera, &obj, &so->sensor_locations[sensor_idx * 3], xij);
}

static void str_metric_function_jac(int j, int i, double *bi, double *xij, void *adata) {
	SurvivePose obj = *(SurvivePose *)bi;
	int sensor_idx = j >> 1;
	int lh = j & 1;

	mpfit_context *ctx = (mpfit_context *)(adata);
	SurviveObject *so = ctx->so;

	assert(lh < 2);
	assert(sensor_idx < so->sensor_ct);

	// quatnormalize(obj.Rot, obj.Rot);

	SurvivePose *camera = &so->ctx->bsd[lh].Pose;
	survive_reproject_full_jac_obj_pose(xij, &obj, &so->sensor_locations[sensor_idx * 3], camera,
										so->ctx->bsd[lh].fcal);
}

int mpfunc(int m, int n, double *p, double *deviates, double **derivs, void *private) {
	mpfit_context *mpfunc_ctx = private;

	SurvivePose *pose = (SurvivePose *)p;

	for (int i = 0; i < m / 2; i++) {
		FLT out[2];
		survive_reproject_full(mpfunc_ctx->so->ctx->bsd[mpfunc_ctx->lh[i]].fcal,
							   &mpfunc_ctx->camera_params[mpfunc_ctx->lh[i]], pose, mpfunc_ctx->pts3d + i * 3, out);
		assert(!isnan(out[0]));
		assert(!isnan(out[1]));
		deviates[i * 2 + 0] = out[0] - mpfunc_ctx->meas[i * 2 + 0];
		deviates[i * 2 + 1] = out[1] - mpfunc_ctx->meas[i * 2 + 1];

		if (derivs) {
			FLT out[7 * 2];
			survive_reproject_full_jac_obj_pose(out, pose, mpfunc_ctx->pts3d + i * 3,
												&mpfunc_ctx->camera_params[mpfunc_ctx->lh[i]],
												mpfunc_ctx->so->ctx->bsd[mpfunc_ctx->lh[i]].fcal);

			for (int j = 0; j < n; j++) {
				derivs[j][i * 2 + 0] = out[j];
				derivs[j][i * 2 + 1] = out[j + 7];
			}
		}
	}

	return 0;
}

static double run_mpfit_find_3d_structure(MPFITData *d, PoserDataLight *pdl, SurviveSensorActivations *scene,
										  int max_iterations /* = 50*/, double max_reproj_error /* = 0.005*/,
										  SurvivePose *out) {
	double *covx = 0;
	SurviveObject *so = d->opt.so;

	mpfit_context mpfitctx = {.so = so,
							  .camera_params = {so->ctx->bsd[0].Pose, so->ctx->bsd[1].Pose},
							  .meas = alloca(sizeof(double) * 2 * so->sensor_ct * NUM_LIGHTHOUSES),
							  .lh = alloca(sizeof(int) * so->sensor_ct * NUM_LIGHTHOUSES),
							  .pts3d = alloca(sizeof(double) * 3 * so->sensor_ct * NUM_LIGHTHOUSES)};

	size_t meas_size = 2 * construct_input_from_scene(d, pdl, scene, mpfitctx.meas, mpfitctx.pts3d, mpfitctx.lh);

	static int failure_count = 500;
	bool hasAllBSDs = true;
	for (int lh = 0; lh < so->ctx->activeLighthouses; lh++)
		hasAllBSDs &= so->ctx->bsd[lh].PositionSet;

	if (!hasAllBSDs || meas_size < d->required_meas) {
		if (hasAllBSDs && failure_count++ == 500) {
			SurviveContext *ctx = so->ctx;
			SV_INFO("Can't solve for position with just %u measurements", (unsigned int)meas_size);
			failure_count = 0;
		}
		if (meas_size < d->required_meas) {
			d->stats.meas_failures++;
		}
		return -1;
	}
	failure_count = 0;

	SurvivePose soLocation = {0};

	if (!general_optimizer_data_record_current_pose(&d->opt, &pdl->hdr, sizeof(*pdl), &soLocation)) {
		return -1;
	}

	mp_result result = {0};
	mp_par pars[7] = {0};

	const bool debug_jacobian = false;
	if (d->use_jacobian_function) {
		for (int i = 0; i < 7; i++) {
			if (debug_jacobian) {
				pars[i].side = 1;
				pars[i].deriv_debug = 1;
			} else {
				pars[i].side = 3;
			}
		}
	}

	int res = mpfit(mpfunc, meas_size, 7, soLocation.Pos, pars, 0, &mpfitctx, &result);

	double rtn = -1;
	bool status_failure = res <= 0;
	bool error_failure = !general_optimizer_data_record_success(&d->opt, result.bestnorm);
	if (!status_failure && !error_failure) {
		quatnormalize(soLocation.Rot, soLocation.Rot);
		*out = soLocation;
		rtn = result.bestnorm;
	}

	return rtn;
}

int PoserMPFIT(SurviveObject *so, PoserData *pd) {
	SurviveContext *ctx = so->ctx;
	if (so->PoserData == 0) {
		so->PoserData = calloc(1, sizeof(MPFITData));
		MPFITData *d = so->PoserData;

		general_optimizer_data_init(&d->opt, so);
		d->useIMU = (bool)survive_configi(ctx, "mpfit-use-imu", SC_GET, 1);
		d->required_meas = survive_configi(ctx, "mpfit-required-meas", SC_GET, 8);

		d->sensor_time_window =
			survive_configi(ctx, "mpfit-time-window", SC_GET, SurviveSensorActivations_default_tolerance * 2);
		d->use_jacobian_function = survive_configi(ctx, "mpfit-use-jacobian-function", SC_GET, 1.0);

		SV_INFO("Initializing MPFIT:");
		SV_INFO("\tmpfit-required-meas: %d", d->required_meas);
		SV_INFO("\tmpfit-time-window: %d", d->sensor_time_window);
		SV_INFO("\tmpfit-use-imu: %d", d->useIMU);
		SV_INFO("\tmpfit-use-jacobian-function: %d", d->use_jacobian_function);
	}
	MPFITData *d = so->PoserData;
	switch (pd->pt) {
	case POSERDATA_LIGHT: {
		// No poses if calibration is ongoing
		if (ctx->calptr && ctx->calptr->stage < 5)
			return 0;
		SurviveSensorActivations *scene = &so->activations;
		PoserDataLight *lightData = (PoserDataLight *)pd;
		SurvivePose estimate;

		// only process sweeps
		FLT error = -1;
		if (d->last_lh != lightData->lh || d->last_acode != lightData->acode) {
			error = run_mpfit_find_3d_structure(d, lightData, scene, 100, .5, &estimate);

			d->last_lh = lightData->lh;
			d->last_acode = lightData->acode;

			if (error > 0) {
				if (d->useIMU) {
					FLT var_meters = 0.5;
					FLT var_quat = error + .05;
					FLT var[7] = {error * var_meters, error * var_meters, error * var_meters, error * var_quat,
								  error * var_quat,   error * var_quat,   error * var_quat};

					survive_imu_tracker_integrate_observation(so, lightData->timecode, &d->tracker, &estimate, var);
					estimate = d->tracker.pose;
				}

				PoserData_poser_pose_func(&lightData->hdr, so, &estimate);
			}
		}
		return 0;
	}

	case POSERDATA_DISASSOCIATE: {
		SV_INFO("MPFIT stats:");
		SV_INFO("\tmeas failures %d", d->stats.meas_failures);
		general_optimizer_data_dtor(&d->opt);
		free(d);
		so->PoserData = 0;
		return 0;
	}
	case POSERDATA_IMU: {

		PoserDataIMU *imu = (PoserDataIMU *)pd;
		if (ctx->calptr && ctx->calptr->stage < 5) {
		} else if (d->useIMU) {
			survive_imu_tracker_integrate(so, &d->tracker, imu);
			PoserData_poser_pose_func(pd, so, &d->tracker.pose);
		}

		general_optimizer_data_record_imu(&d->opt, imu);
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserMPFIT);
