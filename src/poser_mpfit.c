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
	FLT value;
	uint8_t lh;
	uint8_t sensor_idx;
	uint8_t axis;
} mpfit_mea_entry;

typedef struct {
	SurviveObject *so;
	mpfit_mea_entry *meas;
	FLT current_bias;
	SurvivePose currentPose;
	SurvivePose world2camera[NUM_LIGHTHOUSES];
} mpfit_context;

typedef struct MPFITData {
	GeneralOptimizerData opt;

	int last_acode;
	int last_lh;

	int sensor_time_window;
	// > 0; use jacobian, 0 don't use, < 0 debug
	int use_jacobian_function;
	int required_meas;

	SurviveIMUTracker tracker;
	bool useIMU;

	struct {
		int meas_failures;
	} stats;
} MPFITData;

static size_t construct_input_from_scene(const MPFITData *d, const PoserDataLight *pdl,
										 const SurviveSensorActivations *scene, mpfit_mea_entry *meas) {
	size_t rtn = 0;
	SurviveObject *so = d->opt.so;
	const bool force_pair = false;
	for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
		for (uint8_t lh = 0; lh < 2; lh++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				bool isReadingValue = SurviveSensorActivations_isReadingValid(scene, d->sensor_time_window,
																			  pdl->timecode, sensor, lh, axis);
				if (force_pair) {
					isReadingValue =
						SurviveSensorActivations_isPairValid(scene, d->sensor_time_window, pdl->timecode, sensor, lh);
				}
				if (isReadingValue) {
					const double *a = scene->angles[sensor][lh];
					meas->axis = axis;
					meas->value = a[axis];
					meas->sensor_idx = sensor;
					meas->lh = lh;
					meas++;
					rtn++;
				}
			}
		}
	}
	return rtn;
}

typedef FLT (*reproject_axis_fn_t)(const BaseStationCal *, const SurviveAngleReading);
static const reproject_axis_fn_t reproject_axis_fns[] = {survive_reproject_axis_x, survive_reproject_axis_y};

typedef void (*reproject_axis_jacob_fn_t)(SurviveAngleReading, const SurvivePose *, const LinmathPoint3d,
										  const SurvivePose *, const BaseStationCal *);
static const reproject_axis_jacob_fn_t reproject_axis_jacob_fns[] = {survive_reproject_full_x_jac_obj_pose,
																	 survive_reproject_full_y_jac_obj_pose};

static int mpfunc(int m, int n, double *p, double *deviates, double **derivs, void *private) {
	mpfit_context *mpfunc_ctx = private;

	const SurvivePose *pose = (SurvivePose *)p;

	assert(n == 7);

	SurvivePose obj2lh[NUM_LIGHTHOUSES] = {};
	for (int lh = 0; lh < NUM_LIGHTHOUSES; lh++) {
		ApplyPoseToPose(&obj2lh[lh], &mpfunc_ctx->world2camera[lh], pose);
	}

	int meas_count = m;
	if (mpfunc_ctx->current_bias > 0) {
		meas_count -= 7;
		FLT *pp = (FLT *)mpfunc_ctx->currentPose.Pos;
		for (int i = 0; i < 7; i++) {
			deviates[i + meas_count] = (p[i] - pp[i]) * mpfunc_ctx->current_bias;
			if (derivs) {
				derivs[i][i + meas_count] = mpfunc_ctx->current_bias;
			}
		}
	}

	for (int i = 0; i < meas_count; i++) {
		const mpfit_mea_entry *meas = &mpfunc_ctx->meas[i];
		const int lh = meas->lh;
		const struct BaseStationCal *cal = mpfunc_ctx->so->ctx->bsd[lh].fcal;
		const SurvivePose *world2lh = &mpfunc_ctx->world2camera[lh];
		const FLT *pt = &mpfunc_ctx->so->sensor_locations[meas->sensor_idx * 3];

		// If the next two measurements are joined; handle the full pair. This lets us just calculate
		// sensorPtInLH once
		const bool nextIsPair =
			i + 1 < m && meas[0].axis == 0 && meas[1].axis == 1 && meas[0].sensor_idx == meas[1].sensor_idx;

		LinmathPoint3d sensorPtInLH;
		ApplyPoseToPoint(sensorPtInLH, &obj2lh[lh], pt);

		if (nextIsPair) {
			FLT out[2];

			survive_reproject_xy(cal, sensorPtInLH, out);

			deviates[i] = out[meas[0].axis] - meas[0].value;
			deviates[i + 1] = out[meas[1].axis] - meas[1].value;
		} else {
			FLT out = reproject_axis_fns[meas->axis](cal, sensorPtInLH);
			deviates[i] = out - meas->value;
		}

		if (derivs) {
			if (nextIsPair) {
				FLT out[7 * 2] = {};
				survive_reproject_full_jac_obj_pose(out, pose, pt, world2lh, cal);

				for (int j = 0; j < n; j++) {
					derivs[j][i] = out[j];
					derivs[j][i + 1] = out[j + 7];
				}
			} else {
				FLT out[7] = {};
				reproject_axis_jacob_fns[meas->axis](out, pose, pt, world2lh, cal);
				for (int j = 0; j < n; j++) {
					derivs[j][i] = out[j];
				}
			}
		}

		// Skip the next point -- we handled it already
		if (nextIsPair)
			i++;
	}

	return 0;
}

static double run_mpfit_find_3d_structure(MPFITData *d, PoserDataLight *pdl, SurviveSensorActivations *scene,
										  int max_iterations /* = 50*/, double max_reproj_error /* = 0.005*/,
										  SurvivePose *out) {
	double *covx = 0;
	SurviveObject *so = d->opt.so;

	mpfit_context mpfitctx = {
		.so = so, .meas = alloca(sizeof(mpfit_mea_entry) * 2 * so->sensor_ct * NUM_LIGHTHOUSES), .current_bias = .0001};

	for (int lh = 0; lh < NUM_LIGHTHOUSES; lh++)
		InvertPose(&mpfitctx.world2camera[lh], &so->ctx->bsd[lh].Pose);

	size_t meas_size = construct_input_from_scene(d, pdl, scene, mpfitctx.meas);

	if (mpfitctx.current_bias > 0) {
		meas_size += 7;
	}

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

	if (d->use_jacobian_function != 0) {
		for (int i = 0; i < 7; i++) {
			if (d->use_jacobian_function < 0) {
				pars[i].side = 1;
				pars[i].deriv_debug = 1;
				pars[i].deriv_abstol = .0001;
				pars[i].deriv_reltol = .0001;
			} else {
				pars[i].side = 3;
			}
		}
	}

	mpfitctx.currentPose = soLocation;
	int res = mpfit(mpfunc, meas_size, 7, soLocation.Pos, pars, 0, &mpfitctx, &result);

	double rtn = -1;
	bool status_failure = res <= 0;
	bool error_failure = !general_optimizer_data_record_success(&d->opt, result.bestnorm);
	if (!status_failure && !error_failure) {
		quatnormalize(soLocation.Rot, soLocation.Rot);
		*out = soLocation;
		rtn = result.bestnorm;
	} else {
		SurviveContext *ctx = so->ctx;
		SV_INFO("MPFIT failure %f", result.bestnorm);
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
			survive_configi(ctx, "mpfit-time-window", SC_GET, SurviveSensorActivations_default_tolerance);
		d->use_jacobian_function = survive_configi(ctx, "mpfit-use-jacobian-function", SC_GET, 1);

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
