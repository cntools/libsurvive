#include <assert.h>
#include <math.h>
#include <survive_optimizer.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#ifndef NOZLIB
#include <zlib.h>
#endif

#include "survive_optimizer.h"

#include "mpfit/mpfit.h"
#include "survive_default_devices.h"
#include <malloc.h>

STATIC_CONFIG_ITEM(OPTIMIZER_FTOL, "optimizer-ftol", 'f', "Relative chi-square convergence criterium", 0.)
STATIC_CONFIG_ITEM(OPTIMIZER_XTOL, "optimizer-xtol", 'f', "Relative parameter convergence criterium", 0.0001)
STATIC_CONFIG_ITEM(OPTIMIZER_GTOL, "optimizer-gtol", 'f', "Orthogonality convergence criterium", 0.)
STATIC_CONFIG_ITEM(OPTIMIZER_COVTOL, "optimizer-covtol", 'f', "Range tolerance for covariance calculation", 0.)
STATIC_CONFIG_ITEM(OPTIMIZER_EPSFCN, "optimizer-epsfcn", 'f', "Finite derivative step size", 0.)
STATIC_CONFIG_ITEM(OPTIMIZER_STEPFACTOR, "optimizer-stepfactor", 'f', "Initial step bound", 0.)
STATIC_CONFIG_ITEM(OPTIMIZER_DOUSERSCALE, "optimizer-douserscale", 'i', "Scale variables by user values", 0)
STATIC_CONFIG_ITEM(OPTIMIZER_MAXITER, "optimizer-maxiter", 'i', "Maximum iterations", 0)
STATIC_CONFIG_ITEM(OPTIMIZER_MAXFEV, "optimizer-maxfev", 'i', "Maximum function evals", 0)
STATIC_CONFIG_ITEM(OPTIMIZER_NORMTOL, "optimizer-normtol", 'f', "Convergence for norm", 0.00005)
STATIC_CONFIG_ITEM(OPTIMIZER_NPRINT, "optimizer-nprint", 'i', "", 0)

static char *object_parameter_names[] = {"Pose x",	 "Pose y",	 "Pose z",	"Pose Rot w",
										 "Pose Rot x", "Pose Rot y", "Pose Rot z"};

static void setup_pose_param_limits(survive_optimizer *mpfit_ctx, FLT *parameter,
									struct mp_par_struct *pose_param_info) {
	for (int i = 0; i < 7; i++) {
		pose_param_info[i].limited[0] = pose_param_info[i].limited[1] = (i >= 3 ? false : true);

		pose_param_info[i].limits[0] = -(i >= 3 ? 1.0001 : 20.);
		pose_param_info[i].limits[1] = -pose_param_info[i].limits[0];
	}
}

void survive_optimizer_setup_pose(survive_optimizer *mpfit_ctx, const SurvivePose *poses, bool isFixed,
								  int use_jacobian_function) {
	for (int i = 0; i < mpfit_ctx->poseLength; i++) {
		if (poses)
			survive_optimizer_get_pose(mpfit_ctx)[i] = poses[i];
		else
			survive_optimizer_get_pose(mpfit_ctx)[i] = (SurvivePose){.Rot = {1.}};

		setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + i * 7, mpfit_ctx->parameters_info + i * 7);
	}

	for (int i = 0; i < 7 * mpfit_ctx->poseLength; i++) {
		mpfit_ctx->parameters_info[i].fixed = isFixed;
		mpfit_ctx->parameters_info[i].parname = object_parameter_names[i % 7];

		if (use_jacobian_function != 0 &&
			mpfit_ctx->reprojectModel
				->reprojectAxisAngleFullJacObjPose /*mpfit_ctx->reprojectModel->reprojectFullJacObjPose*/) {
			if (use_jacobian_function < 0) {
				mpfit_ctx->parameters_info[i].side = 2;
				mpfit_ctx->parameters_info[i].deriv_debug = 1;
				mpfit_ctx->parameters_info[i].deriv_abstol = .0001;
				mpfit_ctx->parameters_info[i].deriv_reltol = .0001;
			} else {
				mpfit_ctx->parameters_info[i].side = 3;
			}
		}
	}
}

static char *lh_parameter_names[] = {"LH x",	 "LH y",	 "LH z",	"LH Rot w",
									 "LH Rot x", "LH Rot y", "LH Rot z"

};

void survive_optimizer_setup_camera(survive_optimizer *mpfit_ctx, int8_t lh, const SurvivePose *pose, bool isFixed,
									int use_jacobian_function) {
	SurvivePose *cameras = survive_optimizer_get_camera(mpfit_ctx);
	int start = survive_optimizer_get_camera_index(mpfit_ctx) + lh * 7;

	bool poseIsInvalid = pose == 0;
	if (pose && !quatiszero(pose->Rot)) {
		InvertPose(&cameras[lh], pose);
	} else {
		cameras[lh] = (SurvivePose){ 0 };
		poseIsInvalid = true;
	}

	setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + start, mpfit_ctx->parameters_info + start);

	for (int i = start; i < start + 7; i++) {
		mpfit_ctx->parameters_info[i].fixed = (isFixed || poseIsInvalid);
		mpfit_ctx->parameters_info[i].parname = lh_parameter_names[i - start];

		if (use_jacobian_function != 0 &&
			mpfit_ctx->reprojectModel
				->reprojectAxisAngleFullJacLhPose /*mpfit_ctx->reprojectModel->reprojectFullJacLhPose*/) {
			if (use_jacobian_function < 0) {
				mpfit_ctx->parameters_info[i].side = 2;
				mpfit_ctx->parameters_info[i].deriv_debug = 1;
				mpfit_ctx->parameters_info[i].deriv_abstol = .0001;
				mpfit_ctx->parameters_info[i].deriv_reltol = .0001;

			} else {
				mpfit_ctx->parameters_info[i].side = 3;
			}
		}
	}
}

void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed,
									 int use_jacobian_function) {
	for (int lh = 0; lh < mpfit_ctx->cameraLength; lh++) {
		if (ctx->bsd[lh].PositionSet)
			survive_optimizer_setup_camera(mpfit_ctx, lh, &ctx->bsd[lh].Pose, isFixed, use_jacobian_function);
		else {
			SurvivePose id = {.Rot[0] = 1.};
			survive_optimizer_setup_camera(mpfit_ctx, lh, &id, isFixed, use_jacobian_function);
		}
	}

	for (int lh = 0; lh < mpfit_ctx->cameraLength; lh++) {
		BaseStationCal *fcal = survive_optimizer_get_calibration(mpfit_ctx, lh);
		for (int axis = 0; axis < 2; axis++)
			fcal[axis] = ctx->bsd[lh].fcal[axis];
	}

	size_t start = survive_optimizer_get_calibration_index(mpfit_ctx);
	for (int i = start; i < start + 2 * sizeof(BaseStationCal) / sizeof(FLT) * mpfit_ctx->cameraLength; i++) {
		mpfit_ctx->parameters_info[i].parname = "Fcal parameter";
		mpfit_ctx->parameters_info[i].fixed = true;
	}
}

int survive_optimizer_get_parameters_count(const survive_optimizer *ctx) {
	return ctx->cameraLength * 7 + ctx->poseLength * 7 + ctx->ptsLength * 3 +
		   2 * ctx->cameraLength * sizeof(BaseStationCal) / sizeof(FLT);
}

FLT *survive_optimizer_get_sensors(survive_optimizer *ctx) {
	if (ctx->ptsLength == 0)
		return ctx->so->sensor_locations;

	return &ctx->parameters[survive_optimizer_get_sensors_index(ctx)];
}

int survive_optimizer_get_sensors_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_calibration_index(ctx) + 2 * ctx->cameraLength * sizeof(BaseStationCal) / sizeof(FLT);
}

BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh) {
	if (ctx->cameraLength <= lh)
		return ctx->so->ctx->bsd[lh].fcal;

	BaseStationCal *base = (BaseStationCal *)(&ctx->parameters[survive_optimizer_get_calibration_index(ctx)]);
	return &base[2 * lh];
}

int survive_optimizer_get_calibration_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_camera_index(ctx) + ctx->cameraLength * 7;
}

SurvivePose *survive_optimizer_get_camera(survive_optimizer *ctx) {
	assert(ctx->cameraLength > 0);
	return (SurvivePose *)&ctx->parameters[survive_optimizer_get_camera_index(ctx)];
}

int survive_optimizer_get_camera_index(const survive_optimizer *ctx) { return ctx->poseLength * 7; }

SurvivePose *survive_optimizer_get_pose(survive_optimizer *ctx) {
	if (ctx->poseLength)
		return (SurvivePose *)ctx->parameters;
	return &ctx->initialPose;
}
static inline void run_pair_measurement(survive_optimizer *mpfunc_ctx, size_t meas_idx,
										const survive_reproject_model_t *reprojectModel,
										const survive_optimizer_measurement *meas, const LinmathAxisAnglePose *pose,
										const LinmathAxisAnglePose *obj2lh, const LinmathAxisAnglePose *world2lh,
										FLT *deviates, FLT **derivs) {
	SurviveContext *ctx = mpfunc_ctx->so->ctx;
	const int lh = meas->lh;
	const FLT *sensor_points = survive_optimizer_get_sensors(mpfunc_ctx);
	const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
	const FLT *pt = &sensor_points[meas->sensor_idx * 3];

	LinmathPoint3d sensorPtInLH;
	ApplyAxisAnglePoseToPoint(sensorPtInLH, obj2lh, pt);

	FLT out[2];
	reprojectModel->reprojectXY(cal, sensorPtInLH, out);

	deviates[0] = (out[meas[0].axis] - meas[0].value) / meas[0].variance;
	deviates[1] = (out[meas[1].axis] - meas[1].value) / meas[1].variance;

	assert(isfinite(deviates[0]));
	assert(isfinite(deviates[1]));

	if (derivs) {
		int jac_offset_lh = (lh + mpfunc_ctx->poseLength) * 7;
		int jac_offset_obj = meas->object * 7;

		LinmathAxisAnglePose safe_pose = *pose, safe_world2lh = *world2lh;
		if (magnitude3d(safe_pose.AxisAngleRot) == 0)
			safe_pose.AxisAngleRot[0] = 1e-10;
		if (magnitude3d(safe_world2lh.AxisAngleRot) == 0)
			safe_world2lh.AxisAngleRot[0] = 1e-10;

		if (derivs[jac_offset_obj]) {
			FLT out[7 * 2] = {0};
			// reprojectModel->reprojectFullJacObjPose(out, pose, pt, world2lh, cal);
			reprojectModel->reprojectAxisAngleFullJacObjPose(out, pose, pt, world2lh, cal);
			// SV_INFO("Double obj %3d %3d %f", jac_offset_obj, meas_idx, out[0]);
			for (int j = 0; j < 6; j++) {
				assert(derivs[jac_offset_obj + j] && "all 7 parameters should be the same for jacobian calculation");
				derivs[jac_offset_obj + j][meas_idx] = out[j];
				derivs[jac_offset_obj + j][meas_idx + 1] = out[j + 6];
				assert(!isnan(out[j]));
				assert(!isnan(out[j + 6]));
			}
		}

		if (derivs[jac_offset_lh]) {
			FLT out[7 * 2] = {0};
			reprojectModel->reprojectAxisAngleFullJacLhPose(out, pose, pt, world2lh, cal);
			for (int j = 0; j < 6; j++) {
				assert(derivs[jac_offset_lh + j] && "all 7 parameters should be the same for jacobian calculation");
				derivs[jac_offset_lh + j][meas_idx] = out[j];
				derivs[jac_offset_lh + j][meas_idx + 1] = out[j + 6];
				assert(!isnan(out[j]));
				assert(!isnan(out[j + 6]));
			}
		}
	}
}
static void run_single_measurement(survive_optimizer *mpfunc_ctx, size_t meas_idx,
								   const survive_reproject_model_t *reprojectModel,
								   const survive_optimizer_measurement *meas, const LinmathAxisAnglePose *pose,
								   const LinmathAxisAnglePose *obj2lh, const LinmathAxisAnglePose *world2lh,
								   FLT *deviates, FLT **derivs) {
	SurviveContext *ctx = mpfunc_ctx->so->ctx;
	const int lh = meas->lh;
	const FLT *sensor_points = survive_optimizer_get_sensors(mpfunc_ctx);
	const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
	const FLT *pt = &sensor_points[meas->sensor_idx * 3];

	LinmathPoint3d sensorPtInLH;
	ApplyAxisAnglePoseToPoint(sensorPtInLH, obj2lh, pt);

	FLT out = reprojectModel->reprojectAxisFn[meas->axis](cal, sensorPtInLH);
	deviates[0] = (out - meas->value) / meas->variance;
	assert(isfinite(deviates[0]));

	if (derivs) {
		int jac_offset_lh = (lh + mpfunc_ctx->poseLength) * 7;
		int jac_offset_obj = meas->object * 7;

		FLT out[7] = {0};

		if (derivs[jac_offset_obj]) {
			reprojectModel->reprojectAxisAngleAxisJacobFn[meas->axis](out, pose, pt, world2lh, cal);
			for (int j = 0; j < 6; j++) {
				assert(derivs[jac_offset_obj + j]);
				derivs[jac_offset_obj + j][meas_idx] = out[j];
				assert(!isnan(out[j]));
			}
		}

		if (derivs[jac_offset_lh]) {
			reprojectModel->reprojectAxisAngleAxisJacobLhPoseFn[meas->axis](out, pose, pt, world2lh, cal);
			for (int j = 0; j < 6; j++) {
				assert(derivs[jac_offset_lh + j]);
				derivs[jac_offset_lh + j][meas_idx] = out[j];
				assert(!isnan(out[j]));
			}
		}
	}
}

static void filter_measurements(survive_optimizer *optimizer, FLT *deviates) {
	SurviveContext *ctx = optimizer->so ? optimizer->so->ctx : 0;

	FLT lh_deviates[NUM_GEN2_LIGHTHOUSES] = {0};
	size_t lh_meas_cnt[NUM_GEN2_LIGHTHOUSES] = {0};

	FLT avg_dev = 0, lh_avg_dev = 0;
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		lh_deviates[meas->lh] += fabs(deviates[i]);
		lh_meas_cnt[meas->lh]++;
	}

	size_t obs_lhs = 0;

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (lh_meas_cnt[i]) {
			lh_deviates[i] = lh_deviates[i] / (FLT)lh_meas_cnt[i];
			lh_avg_dev += lh_deviates[i];
			obs_lhs++;
		}
	}

	lh_avg_dev = lh_avg_dev / (FLT)obs_lhs;
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (lh_deviates[i] > 2. * lh_avg_dev) {
			SV_VERBOSE(500, "Data from LH %d seems suspect (%f/%f)", i, lh_deviates[i], lh_avg_dev);
			lh_meas_cnt[i] = 0;
		} else if (lh_deviates[i] > 0.) {
			SV_VERBOSE(1000, "Data from LH %d seems OK (%f/%f)", i, lh_deviates[i], lh_avg_dev);
		}
	}

	size_t valid_meas = 0;
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		if (lh_meas_cnt[meas->lh]) {
			avg_dev += fabs(deviates[i]);
			valid_meas++;
		}
	}

	avg_dev = avg_dev / (FLT)valid_meas;
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		if (lh_meas_cnt[meas->lh] == 0 || fabs(deviates[i]) > avg_dev * 10) {
			meas->invalid = true;
			deviates[i] = 0.;
			optimizer->stats.dropped_data_cnt++;

			SV_VERBOSE(500, "Ignoring noisy data at lh %d sensor %d axis %d val %f (%f/%f)", meas->lh, meas->sensor_idx,
					   meas->axis, meas->value, deviates[i], avg_dev);
		} else {
			SV_VERBOSE(1000, "Data at lh %d sensor %d axis %d val %f (%f/%f)", meas->lh, meas->sensor_idx, meas->axis,
					   meas->value, deviates[i], avg_dev);
		}
	}

	optimizer->needsFiltering = false;
}
static int mpfunc(int m, int n, FLT *p, FLT *deviates, FLT **derivs, void *private) {
	survive_optimizer *mpfunc_ctx = private;
	SurviveContext *ctx = mpfunc_ctx->so ? mpfunc_ctx->so->ctx : 0;

	const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
	mpfunc_ctx->parameters = p;

	SurvivePose *cameras = survive_optimizer_get_camera(mpfunc_ctx);

	int start = survive_optimizer_get_camera_index(mpfunc_ctx);
	for (int i = 0; i < mpfunc_ctx->cameraLength; i++) {
		if (!mpfunc_ctx->parameters_info[start + 7 * i].fixed) {
			// quatnormalize(cameras[i].Rot, cameras[i].Rot);
		}
	}
	const FLT *sensor_points = survive_optimizer_get_sensors(mpfunc_ctx);

	int pose_idx = -1;
	// SurvivePose *pose = 0;
	// SurvivePose obj2lh[NUM_GEN2_LIGHTHOUSES] = {0};
	LinmathAxisAnglePose *pose = 0;
	LinmathAxisAnglePose obj2lh[NUM_GEN2_LIGHTHOUSES] = {0};

	int meas_count = m;
	if (mpfunc_ctx->current_bias > 0) {
		meas_count -= 7;
		FLT *pp = (FLT *)mpfunc_ctx->initialPose.Pos;
		for (int i = 0; i < 7; i++) {
			deviates[i + meas_count] = (p[i] - pp[i]) * mpfunc_ctx->current_bias;
			if (derivs) {
				derivs[i][i + meas_count] = mpfunc_ctx->current_bias;
			}
		}
	}

	for (int i = 0; i < meas_count; i++) {
		const survive_optimizer_measurement *meas = &mpfunc_ctx->measurements[i];
		const int lh = meas->lh;

		if (meas->invalid) {
			deviates[i] = 0;
			if (derivs) {
				int jac_offset_lh = (lh + mpfunc_ctx->poseLength) * 7;
				int jac_offset_obj = meas->object * 7;

				for (int j = 0; j < 6; j++) {
					if (derivs[jac_offset_obj + j])
						derivs[jac_offset_obj + j][i] = 0;
					if (derivs[jac_offset_lh + j])
						derivs[jac_offset_lh + j][i] = 0;
				}
			}
			continue;
		}

		const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
		LinmathAxisAnglePose *world2lh = (LinmathAxisAnglePose *)&cameras[lh];
		const FLT *pt = &sensor_points[meas->sensor_idx * 3];

		if (pose_idx != meas->object) {
			pose_idx = meas->object;
			assert(pose_idx < mpfunc_ctx->poseLength);
			pose = (LinmathAxisAnglePose *)(&survive_optimizer_get_pose(mpfunc_ctx)[meas->object]);

			int lh_count =
				mpfunc_ctx->cameraLength > 0 ? mpfunc_ctx->cameraLength : mpfunc_ctx->so->ctx->activeLighthouses;
			for (int lh = 0; lh < lh_count; lh++) {
				ApplyAxisAnglePoseToPose(&obj2lh[lh], (const LinmathAxisAnglePose *)&cameras[lh], pose);
			}
		}

		// If the next two measurements are joined; handle the full pair. This lets us just calculate
		// sensorPtInLH once
		const bool nextIsPair = i + 1 < m && meas[0].axis == 0 && meas[1].axis == 1 &&
								meas[0].sensor_idx == meas[1].sensor_idx && !meas[1].invalid;

		LinmathPoint3d sensorPtInLH;
		ApplyAxisAnglePoseToPoint(sensorPtInLH, &obj2lh[lh], pt);

		if (nextIsPair) {
			run_pair_measurement(mpfunc_ctx, i, reprojectModel, meas, pose, &obj2lh[lh], world2lh, deviates + i,
								 derivs);
			i++;
		} else {
			run_single_measurement(mpfunc_ctx, i, reprojectModel, meas, pose, &obj2lh[lh], world2lh, deviates + i,
								   derivs);
		}
	}

	if (mpfunc_ctx->needsFiltering) {
		assert(derivs == 0);
		filter_measurements(mpfunc_ctx, deviates);
	}

	return 0;
}

const char *survive_optimizer_error(int status) {
#define CASE(x)                                                                                                        \
	case x:                                                                                                            \
		return #x

	switch (status) {
		CASE(MP_ERR_INPUT);
		CASE(MP_ERR_NAN);
		CASE(MP_ERR_FUNC);
		CASE(MP_ERR_NPOINTS);
		CASE(MP_ERR_NFREE);
		CASE(MP_ERR_MEMORY);
		CASE(MP_ERR_INITBOUNDS);
		CASE(MP_ERR_BOUNDS);
		CASE(MP_ERR_PARAM);
		CASE(MP_ERR_DOF);

		/* Potential success status codes */
		CASE(MP_OK_CHI);
		CASE(MP_OK_PAR);
		CASE(MP_OK_BOTH);
		CASE(MP_OK_DIR);
		CASE(MP_OK_NORM);

		CASE(MP_MAXITER);
		CASE(MP_FTOL);
		CASE(MP_GTOL);
		CASE(MP_XTOL);
	default:
		return "Unknown error";
	}
}

static SurviveContext *cachedCtx = 0;
static mp_config cachedCfg = {0};

static mp_config *survive_optimizer_get_cfg(SurviveContext *ctx) {
	if (ctx != cachedCtx) {
		cachedCfg = (mp_config){0};
		cachedCfg.maxiter = survive_configf(ctx, OPTIMIZER_MAXITER_TAG, SC_GET, 0);
		cachedCfg.maxfev = survive_configf(ctx, OPTIMIZER_MAXFEV_TAG, SC_GET, 0);
		cachedCfg.ftol = survive_configf(ctx, OPTIMIZER_FTOL_TAG, SC_GET, 0);
		cachedCfg.normtol = survive_configf(ctx, OPTIMIZER_NORMTOL_TAG, SC_GET, 0);
		cachedCfg.xtol = survive_configf(ctx, OPTIMIZER_XTOL_TAG, SC_GET, 0);
		cachedCfg.gtol = survive_configf(ctx, OPTIMIZER_GTOL_TAG, SC_GET, 0);
		cachedCfg.covtol = survive_configf(ctx, OPTIMIZER_COVTOL_TAG, SC_GET, 0);
		cachedCfg.epsfcn = survive_configf(ctx, OPTIMIZER_EPSFCN_TAG, SC_GET, 0);
		cachedCfg.stepfactor = survive_configf(ctx, OPTIMIZER_STEPFACTOR_TAG, SC_GET, 0);
		cachedCfg.douserscale = survive_configi(ctx, OPTIMIZER_DOUSERSCALE_TAG, SC_GET, 0);
		cachedCfg.nprint = survive_configi(ctx, OPTIMIZER_NPRINT_TAG, SC_GET, 0);
	}
	cachedCfg.iterproc = 0;
	return &cachedCfg;
}

mp_config precise_cfg = {0};
SURVIVE_EXPORT mp_config *survive_optimizer_precise_config() { return &precise_cfg; }

int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result) {
	SurviveContext *ctx = optimizer->so ? optimizer->so->ctx : 0;

	mp_config *cfg = optimizer->cfg;
	if (cfg == 0)
		cfg = survive_optimizer_get_cfg(ctx);

	SurvivePose *poses = survive_optimizer_get_pose(optimizer);
	for (int i = 0; i < optimizer->poseLength + optimizer->cameraLength; i++) {
		quattoaxisanglemag(poses[i].Rot, poses[i].Rot);
		poses[i].Rot[3] = 0; // NAN;
		optimizer->parameters_info[i * 7 + 6].fixed = true;
	}

#ifndef NDEBUG
	for (int i = 0; i < survive_optimizer_get_parameters_count(optimizer); i++) {
		if ((optimizer->parameters_info[i].limited[0] &&
			 optimizer->parameters[i] < optimizer->parameters_info[i].limits[0]) ||
			(optimizer->parameters_info[i].limited[1] &&
			 optimizer->parameters[i] > optimizer->parameters_info[i].limits[1]) ||
			isnan(optimizer->parameters[i])) {
			survive_optimizer_serialize(optimizer, "debug.opt");
			SurviveContext *ctx = optimizer->so->ctx;
			SV_GENERAL_ERROR("Parameter %s is invalid. %f <= %f <= %f should be true",
							 optimizer->parameters_info[i].parname, optimizer->parameters_info[i].limits[0],
							 optimizer->parameters[i], optimizer->parameters_info[i].limits[1])
		}
	}
#endif

	// MPFit runs on temporary storage; so parameters is manipulated in mpfunc. Save it and restore it here.
	FLT *params = optimizer->parameters;
	optimizer->needsFiltering = true;
	int rtn = mpfit(mpfunc, optimizer->measurementsCnt, survive_optimizer_get_parameters_count(optimizer),
					optimizer->parameters, optimizer->parameters_info, cfg, optimizer, result);
	optimizer->parameters = params;

	for (int i = 0; i < optimizer->poseLength + optimizer->cameraLength; i++) {
		quatfromaxisangle(poses[i].Rot, poses[i].Rot, norm3d(poses[i].Rot));
	}
	return rtn;
}

void survive_optimizer_set_reproject_model(survive_optimizer *optimizer,
										   const survive_reproject_model_t *reprojectModel) {
	optimizer->reprojectModel = reprojectModel;
}

#ifdef NOZLIB
#define gzFile FILE *
#define gzopen fopen
#define gzprintf fprintf
#define gzclose fclose
#endif

void survive_optimizer_serialize(const survive_optimizer *opt, const char *fn) {
	FILE *f = fopen(fn, "w");

	fprintf(f, "object       %s\n", opt->so->codename);
	fprintf(f, "currentBias  %+0.16f\n", opt->current_bias);
	fprintf(f, "initialPose " SurvivePose_format "\n", SURVIVE_POSE_EXPAND(opt->initialPose));
	fprintf(f, "model        %d\n", opt->reprojectModel != &survive_reproject_model);
	fprintf(f, "poseLength   %d\n", opt->poseLength);
	fprintf(f, "cameraLength %d\n", opt->cameraLength);
	fprintf(f, "ptsLength    %d\n", opt->ptsLength);

	fprintf(f, "\n");
	fprintf(f, "parameters   %d\n", survive_optimizer_get_parameters_count(opt));
	fprintf(f, "#	          <name>:        <idx>      <fixed>             <value>            <min>            <max> "
			   "<use_jacobian>\n");
	for (int i = 0; i < survive_optimizer_get_parameters_count(opt); i++) {
		struct mp_par_struct *info = &opt->parameters_info[i];
		fprintf(f, "\t%16s:", opt->parameters_info[i].parname);
		fprintf(f, " %12d", i);
		fprintf(f, " %12d", info->fixed);
		fprintf(f, " %+0.16f", opt->parameters[i]);
		fprintf(f, " %+16.f %+16.f", info->limits[0], info->limits[1]);
		fprintf(f, " %14d\n", info->side);
	}

	fprintf(f, "\n");
	fprintf(f, "measurementsCnt %ld\n", opt->measurementsCnt);
	fprintf(f, "\t#<lh> <axis> <sensor_idx> <object_idx> <value> <variance>\n");
	for (int i = 0; i < opt->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &opt->measurements[i];
		fprintf(f, "\t%d", meas->lh);
		fprintf(f, " %d", meas->axis);
		fprintf(f, " %2d", meas->sensor_idx);
		fprintf(f, " %d", meas->object);
		fprintf(f, " %+0.16f", meas->value);
		fprintf(f, " %+0.16f\n", meas->variance);
	}

	fclose(f);
}

survive_optimizer *survive_optimizer_load(const char *fn) {
	survive_optimizer *opt = calloc(sizeof(survive_optimizer), 1);

	FILE *f = fopen(fn, "r");
	int read_count = 0;

#ifndef LINE_MAX
#define LINE_MAX 2048
#endif
	char buffer[LINE_MAX] = { 0 };
	char device_name[LINE_MAX] = {0};
	read_count = fscanf(f, "object       %s\n", device_name);
	read_count = fscanf(f, "currentBias  " FLT_sformat "\n", &opt->current_bias);
	read_count = fscanf(f, "initialPose " SurvivePose_sformat "\n", SURVIVE_POSE_SCAN_EXPAND(opt->initialPose));
	int model = 0;
	read_count = fscanf(f, "model        %d\n", &model);
	opt->reprojectModel = model == 0 ? &survive_reproject_model : &survive_reproject_gen2_model;
	read_count = fscanf(f, "poseLength   %d\n", &opt->poseLength);
	read_count = fscanf(f, "cameraLength %d\n", &opt->cameraLength);
	read_count = fscanf(f, "ptsLength    %d\n", &opt->ptsLength);

	int param_count;
	read_count = fscanf(f, "parameters   %d\n", &param_count);
	char *success = fgets(buffer, LINE_MAX, f); // fscanf(f, "\t#<name>: <fixed> <value> <min> <max> <use_jacobian>\n");
	assert(success);

	(void)read_count;
	assert(param_count == survive_optimizer_get_parameters_count(opt));

	SURVIVE_OPTIMIZER_SETUP_HEAP_BUFFERS(*opt);

	for (int i = 0; i < survive_optimizer_get_parameters_count(opt); i++) {
		struct mp_par_struct *info = &opt->parameters_info[i];
		read_count = fscanf(f, "\t");

		opt->parameters_info[i].parname = calloc(128, 1);
		char *b = opt->parameters_info[i].parname;
		char c = fgetc(f);
		while (c != ':') {
			*(b++) = c;
			c = fgetc(f);
		}
		int idx;
		read_count = fscanf(f, "%d ", &idx);
		read_count = fscanf(f, " %d", &info->fixed);
		read_count = fscanf(f, " " FLT_sformat, &opt->parameters[i]);
		read_count = fscanf(f, " " FLT_sformat " " FLT_sformat, &info->limits[0], &info->limits[1]);
		read_count = fscanf(f, " %d\n", &info->side);
	}

	read_count = fscanf(f, "\n");
	read_count = fscanf(f, "measurementsCnt %lu\n", &opt->measurementsCnt);
	read_count = fscanf(f, "\t#<lh> <axis> <sensor_idx> <object_idx> <value> <variance>\n");
	for (int i = 0; i < opt->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &opt->measurements[i];
		read_count = fscanf(f, "\t%hhu", &meas->lh);
		read_count = fscanf(f, " %hhu", &meas->axis);
		read_count = fscanf(f, " %hhu", &meas->sensor_idx);
		read_count = fscanf(f, " %d", &meas->object);
		read_count = fscanf(f, " " FLT_sformat, &meas->value);
		read_count = fscanf(f, " " FLT_sformat "\n", &meas->variance);
	}

	fclose(f);

	SurviveObject *so = survive_create_device(0, "SLV", opt, "SV0", 0);
	char filename[FILENAME_MAX] = {0};
	snprintf(filename, FILENAME_MAX, "%s_config.json", device_name);
	FILE *fp = fopen(filename, "r");
	if (fp) {
		fseek(fp, 0L, SEEK_END);
		int len = ftell(fp);
		fseek(fp, 0L, SEEK_SET);
		if (len > 0) {
			char *ct0conf = (char *)malloc(len);
			fread(ct0conf, 1, len, fp);
			survive_default_config_process(so, ct0conf, len);
			fclose(fp);
		}
	}
	opt->so = so;

	return opt;
}

SURVIVE_EXPORT FLT survive_optimizer_current_norm(const survive_optimizer *opt) {
	int m = opt->measurementsCnt;
	int npar = survive_optimizer_get_parameters_count(opt);
	FLT *p = opt->parameters;
	FLT *deviates = alloca(sizeof(FLT) * m);
	FLT **derivs = 0;
	mpfunc(m, npar, p, deviates, 0, (void *)opt);

	FLT fnorm = 0;
	for (size_t i = 0; i < m; i++) {
		fnorm += deviates[i] * deviates[i];
	}
	return fnorm;
}

SURVIVE_EXPORT int survive_optimizer_nonfixed_cnt(const survive_optimizer *ctx) {
	int rtn = 0;
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		if (!ctx->parameters_info[i].fixed)
			rtn++;
	}
	return rtn;
}
SURVIVE_EXPORT void survive_optimizer_get_nonfixed(const survive_optimizer *ctx, FLT *params) {
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		if (!ctx->parameters_info[i].fixed)
			*(params++) = ctx->parameters[i];
	}
}

SURVIVE_EXPORT void survive_optimizer_set_nonfixed(survive_optimizer *ctx, FLT *params) {
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		if (!ctx->parameters_info[i].fixed)
			ctx->parameters[i] = *(params++);
	}
}

SURVIVE_EXPORT size_t survive_optimizer_get_total_buffer_size(const survive_optimizer *ctx) {
	size_t par_count = survive_optimizer_get_parameters_count(ctx);
	size_t sensor_cnt = ctx->so ? ctx->so->sensor_ct : 32;
	return par_count * (sizeof(FLT) +				  // parameters
						sizeof(struct mp_par_struct)) // parameters_info
		   + ctx->poseLength * sizeof(survive_optimizer_measurement) * 2 * sensor_cnt *
				 NUM_GEN2_LIGHTHOUSES; // measurements
}

SURVIVE_EXPORT void survive_optimizer_setup_buffers(survive_optimizer *ctx, void *parameter_buffer,
													void *parameter_info_buffer, void *measurements_buffer) {
	size_t par_count = survive_optimizer_get_parameters_count(ctx);
	ctx->parameters = (FLT *)parameter_buffer;
	for (int i = 0; i < par_count; i++)
		ctx->parameters[i] = NAN;
	size_t par_offset = par_count * sizeof(FLT);
	ctx->parameters_info = (struct mp_par_struct *)(parameter_info_buffer);
	size_t sensor_cnt = ctx->so ? ctx->so->sensor_ct : 32;

	size_t par_info_offset = par_offset + sizeof(struct mp_par_struct) * par_count;

	ctx->measurements = (survive_optimizer_measurement *)(measurements_buffer);
	memset(ctx->measurements, 0,
		   ctx->poseLength * sizeof(survive_optimizer_measurement) * 2 * sensor_cnt * NUM_GEN2_LIGHTHOUSES);
	memset(ctx->parameters_info, 0, sizeof(mp_par) * par_count);
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		ctx->parameters_info[i].fixed = 1;
	}
}

SURVIVE_EXPORT void *survive_optimizer_realloc(void *old_ptr, size_t size) { return realloc(old_ptr, size); }
