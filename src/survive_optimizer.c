#include <assert.h>
#include <math.h>
#include <survive_optimizer.h>
#include <survive_reproject.h>

#include "survive_optimizer.h"

#include "mpfit/mpfit.h"

static char *object_parameter_names[] = {"Pose x",	 "Pose y",	 "Pose z",	"Pose Rot w",
										 "Pose Rot x", "Pose Rot y", "Pose Rot z"};

static void setup_pose_param_limits(survive_optimizer *mpfit_ctx, double *parameter,
									struct mp_par_struct *pose_param_info) {
	for (int i = 0; i < 7; i++) {
		pose_param_info[i].limited[0] = pose_param_info[i].limited[1] = 1;

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

		if (use_jacobian_function != 0 && mpfit_ctx->reprojectModel->reprojectFullJacObjPose) {
			if (use_jacobian_function < 0) {
				mpfit_ctx->parameters_info[i].side = 1;
				mpfit_ctx->parameters_info[i].deriv_debug = 1;
				mpfit_ctx->parameters_info[i].deriv_abstol = .0001;
				mpfit_ctx->parameters_info[i].deriv_reltol = .0001;
			} else {
				mpfit_ctx->parameters_info[i].side = 3;
			}
		}
	}
}

static char *lh_parameter_names[] = {"LH0 x",	 "LH0 y",	 "LH0 z",		"LH0 Rot w", "LH0 Rot x",
									 "LH0 Rot y", "LH0 Rot z", "LH1 x",		"LH1 y",	 "LH1 z",
									 "LH1 Rot w", "LH1 Rot x", "LH1 Rot y", "LH1 Rot z"

};

void survive_optimizer_setup_camera(survive_optimizer *mpfit_ctx, int8_t lh, const SurvivePose *pose, bool isFixed) {
	SurvivePose *cameras = survive_optimizer_get_camera(mpfit_ctx);
	int start = survive_optimizer_get_camera_index(mpfit_ctx) + lh * 7;

	if (pose && !quatiszero(pose->Rot)) {
		InvertPose(&cameras[lh], pose);
	} else {
		cameras[lh] = (SurvivePose){ 0 };
	}

	setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + start, mpfit_ctx->parameters_info + start);

	for (int i = start; i < start + 7; i++) {
		mpfit_ctx->parameters_info[i].fixed = isFixed || pose == 0;
		mpfit_ctx->parameters_info[i].parname = lh_parameter_names[i - start];
	}
}

void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed) {
	for (int lh = 0; lh < mpfit_ctx->cameraLength; lh++) {
		if (ctx->bsd[lh].PositionSet)
			survive_optimizer_setup_camera(mpfit_ctx, lh, &ctx->bsd[lh].Pose, isFixed);
		else {
			survive_optimizer_setup_camera(mpfit_ctx, lh, 0, isFixed);
		}
	}
}

int survive_optimizer_get_parameters_count(const survive_optimizer *ctx) {
	return ctx->cameraLength * 7 + ctx->poseLength * 7 + ctx->ptsLength * 3 +
		   ctx->fcalLength * sizeof(BaseStationCal) / sizeof(double);
}

double *survive_optimizer_get_sensors(survive_optimizer *ctx) {
	if (ctx->ptsLength == 0)
		return ctx->so->sensor_locations;

	return &ctx->parameters[survive_optimizer_get_sensors_index(ctx)];
}

int survive_optimizer_get_sensors_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_calibration_index(ctx) + ctx->fcalLength * sizeof(BaseStationCal) / sizeof(double);
}

BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh) {
	if (ctx->fcalLength <= lh)
		return ctx->so->ctx->bsd[lh].fcal;

	return (BaseStationCal *)(&ctx->parameters[survive_optimizer_get_calibration_index(ctx)]);
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

static int mpfunc(int m, int n, double *p, double *deviates, double **derivs, void *private) {
	survive_optimizer *mpfunc_ctx = private;
	SurviveContext *ctx = mpfunc_ctx->so->ctx;

	const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
	mpfunc_ctx->parameters = p;

	SurvivePose *cameras = survive_optimizer_get_camera(mpfunc_ctx);

	int start = survive_optimizer_get_camera_index(mpfunc_ctx);
	for (int i = 0; i < mpfunc_ctx->cameraLength; i++) {
		if (!mpfunc_ctx->parameters_info[start + 7 * i].fixed) {
			quatnormalize(cameras[i].Rot, cameras[i].Rot);
		}
	}
	const double *sensor_points = survive_optimizer_get_sensors(mpfunc_ctx);

	int pose_idx = -1;
	SurvivePose *pose = 0;
	SurvivePose obj2lh[NUM_GEN2_LIGHTHOUSES] = {0};

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
		const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
		const SurvivePose *world2lh = &cameras[lh];
		const FLT *pt = &sensor_points[meas->sensor_idx * 3];

		if (pose_idx != meas->object) {
			pose_idx = meas->object;
			pose = &survive_optimizer_get_pose(mpfunc_ctx)[meas->object];

			// SV_INFO("Before\t" SurvivePose_format, SURVIVE_POSE_EXPAND(*pose));
			quatnormalize(pose->Rot, pose->Rot);
			// SV_INFO("After\t" SurvivePose_format, SURVIVE_POSE_EXPAND(*pose));

			for (int lh = 0; lh < mpfunc_ctx->so->ctx->activeLighthouses; lh++) {
				ApplyPoseToPose(&obj2lh[lh], &cameras[lh], pose);
			}
		}

		// If the next two measurements are joined; handle the full pair. This lets us just calculate
		// sensorPtInLH once
		const bool nextIsPair =
			i + 1 < m && meas[0].axis == 0 && meas[1].axis == 1 && meas[0].sensor_idx == meas[1].sensor_idx;

		LinmathPoint3d sensorPtInLH;
		ApplyPoseToPoint(sensorPtInLH, &obj2lh[lh], pt);

		if (nextIsPair) {
			FLT out[2];

			reprojectModel->reprojectXY(cal, sensorPtInLH, out);

			deviates[i] = (out[meas[0].axis] - meas[0].value) / meas[0].variance;
			deviates[i + 1] = (out[meas[1].axis] - meas[1].value) / meas[1].variance;

			assert(!isnan(deviates[i]));
			assert(!isnan(deviates[i + 1]));
		} else {
			FLT out = reprojectModel->reprojectAxisFn[meas->axis](cal, sensorPtInLH);
			deviates[i] = (out - meas->value) / meas->variance;

			assert(!isnan(deviates[i]));
		}

		if (derivs) {
			if (nextIsPair) {
				FLT out[7 * 2] = { 0 };
				reprojectModel->reprojectFullJacObjPose(out, pose, pt, world2lh, cal);

				for (int j = 0; j < 7; j++) {
					if (derivs[j]) {
						derivs[j][i] = out[j];
						derivs[j][i + 1] = out[j + 7];
					}
					assert(!isnan(out[j]));
					assert(!isnan(out[j + 7]));
				}
			} else {
				FLT out[7] = { 0 };
				reprojectModel->reprojectAxisJacobFn[meas->axis](out, pose, pt, world2lh, cal);
				for (int j = 0; j < 7; j++) {
					if (derivs[j]) {
						derivs[j][i] = out[j];
					}
					assert(!isnan(out[j]));
				}
			}
		}

		// Skip the next point -- we handled it already
		if (nextIsPair)
			i++;
	}

	return 0;
}

int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result) {
	SurviveContext *ctx = optimizer->so->ctx;
	// SV_INFO("Run start");

#ifndef NDEBUG
	for (int i = 0; i < survive_optimizer_get_parameters_count(optimizer); i++) {
		if ((optimizer->parameters_info[i].limited[0] &&
			 optimizer->parameters[i] < optimizer->parameters_info[i].limits[0]) ||
			(optimizer->parameters_info[i].limited[1] &&
			 optimizer->parameters[i] > optimizer->parameters_info[i].limits[1])) {
			SurviveContext *ctx = optimizer->so->ctx;
			SV_GENERAL_ERROR("Parameter %s is invalid. %f <= %f <= %f should be true",
							 optimizer->parameters_info[i].parname, optimizer->parameters_info[i].limits[0],
							 optimizer->parameters[i], optimizer->parameters_info[i].limits[1])
		}
	}
#endif

	return mpfit(mpfunc, optimizer->measurementsCnt, survive_optimizer_get_parameters_count(optimizer),
				 optimizer->parameters, optimizer->parameters_info, 0, optimizer, result);
}

void survive_optimizer_set_reproject_model(survive_optimizer *optimizer,
										   const survive_reproject_model_t *reprojectModel) {
	optimizer->reprojectModel = reprojectModel;
}
