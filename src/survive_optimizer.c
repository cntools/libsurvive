#include <assert.h>
#include <math.h>
#include <survive_optimizer.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>
#include <stdint.h>

#ifndef NOZLIB
#include <zlib.h>
#endif

#include "generated/survive_reproject.aux.generated.h"
//#include "generated/survive_imu.generated.h"
#include "survive_optimizer.h"

#include "generated/survive_imu.generated.h"
#include "mpfit/mpfit.h"
#include "survive_default_devices.h"

#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#include <sv_matrix.h>

#endif

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

STRUCT_CONFIG_SECTION(survive_optimizer_settings)
    STRUCT_CONFIG_ITEM("mpfit-quat-model", "Model mpfit as quaternion", 0, t->use_quat_model)
	STRUCT_CONFIG_ITEM("mpfit-no-pair-calc", "Don't process as pairs", 0, t->disallow_pair_calc)
	STRUCT_CONFIG_ITEM("mpfit-optimize-scale", "Treat scale as mutable", 0, t->optimize_scale)
	END_STRUCT_CONFIG_SECTION(survive_optimizer_settings)

	static char *object_parameter_names[] = {"Pose x",	   "Pose y",	 "Pose z",	  "Pose Rot w",
											 "Pose Rot x", "Pose Rot y", "Pose Rot z"};
	static char *vel_parameter_names[] = {"Vel x", "Vel y", "Vel z", "Vel Rot x", "Vel Rot y", "Vel Rot z"};

	static void setup_pose_param_limits(survive_optimizer *mpfit_ctx, FLT *parameter,
										struct mp_par_struct *pose_param_info) {
		for (int i = 0; i < 7; i++) {
			pose_param_info[i].limited[0] = pose_param_info[i].limited[1] = (i >= 3 ? false : true);

			pose_param_info[i].limits[0] = -(i >= 3 ? 1.0001 : 20.);
			pose_param_info[i].limits[1] = -pose_param_info[i].limits[0];
		}
}
void survive_optimizer_setup_pose_n(survive_optimizer *mpfit_ctx, const SurvivePose *pose, size_t n, bool isFixed,
									int use_jacobian_function) {
	if (pose)
		survive_optimizer_get_pose(mpfit_ctx)[n] = *pose;
	else
		survive_optimizer_get_pose(mpfit_ctx)[n] = (SurvivePose){.Rot = {1.}};

	setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + n * 7, mpfit_ctx->parameters_info + n * 7);

	for (int i = n * 7; i < 7 * (n + 1); i++) {
		mpfit_ctx->parameters_info[i].fixed = isFixed;
		mpfit_ctx->parameters_info[i].parname = object_parameter_names[i % 7];

		if (use_jacobian_function != 0) {
			assert(mpfit_ctx->reprojectModel->reprojectAxisAngleFullJacObjPose);
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

	if (!mpfit_ctx->disableVelocity) {
		int v_idx = survive_optimizer_get_velocity_index(mpfit_ctx);
		survive_optimizer_get_velocity(mpfit_ctx)[n] = (SurviveVelocity){};
		for (int i = 0; i < 6; i++) {
			mpfit_ctx->parameters_info[i + v_idx].fixed = true;
			mpfit_ctx->parameters_info[i + v_idx].parname = vel_parameter_names[i % 6];
			mpfit_ctx->parameters_info[i + v_idx].side = 0;
		}
	}

	if (mpfit_ctx->settings->optimize_scale) {
		int s_idx = survive_optimizer_get_sensor_scale_index(mpfit_ctx);
		for (int i = 0; i < mpfit_ctx->poseLength; i++) {
			mpfit_ctx->parameters[s_idx + i] = mpfit_ctx->sos[n]->sensor_scale;
			struct mp_par_struct *pinfo = &mpfit_ctx->parameters_info[i + s_idx];
			pinfo->fixed = false;
			pinfo->parname = "scale";

			if (use_jacobian_function != 0) {
				if (use_jacobian_function < 0) {
					pinfo->side = 2;
					pinfo->deriv_debug = 1;
					pinfo->deriv_abstol = .0001;
					pinfo->deriv_reltol = .0001;
				} else {
					pinfo->side = 3;
				}
			}

			pinfo->limited[0] = pinfo->limited[1] = true;
			pinfo->limits[0] = 1 - .1;
			pinfo->limits[1] = 1 + .1;
		}
	}
}
void survive_optimizer_fix_camera(survive_optimizer *mpfit_ctx, int cam_idx) {
	int start = survive_optimizer_get_camera_index(mpfit_ctx) + cam_idx * 7;
	for (int i = start; i < start + 7; i++) {
		mpfit_ctx->parameters[i] = 0;
		mpfit_ctx->parameters_info[i].fixed = 1;
	}
}
void survive_optimizer_setup_pose(survive_optimizer *mpfit_ctx, const SurvivePose *poses, bool isFixed,
								  int use_jacobian_function) {

	for (int i = 0; i < mpfit_ctx->poseLength; i++) {
		survive_optimizer_setup_pose_n(mpfit_ctx, poses ? &poses[i] : 0, i, isFixed, use_jacobian_function);
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
		if (!quatiszero(ctx->bsd[lh].Pose.Rot))
			survive_optimizer_setup_camera(mpfit_ctx, lh, &ctx->bsd[lh].Pose, isFixed, use_jacobian_function);
		else {
			SurvivePose id = {.Rot = {1}};
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
	assert(ctx->poseLength < 20);
	int rtn = ctx->cameraLength * 7 + ctx->poseLength * 7 + ctx->ptsLength * 3 +
			  2 * ctx->cameraLength * sizeof(BaseStationCal) / sizeof(FLT);
	if (!ctx->disableVelocity)
		rtn += ctx->poseLength * 6;
	if (ctx->settings->optimize_scale) {
		rtn += ctx->poseLength;
	}
	return rtn;
}
int survive_optimizer_get_free_parameters_count(const survive_optimizer *ctx) {
	int rtn = 0;
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		rtn += ctx->parameters_info[i].fixed == 0;
	}
	return rtn;
}
FLT *survive_optimizer_get_sensors(survive_optimizer *ctx, size_t idx) {
	if (ctx->ptsLength == 0)
		return ctx->sos[idx]->sensor_locations;

	return &ctx->parameters[survive_optimizer_get_sensors_index(ctx)];
}

int survive_optimizer_get_sensors_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_calibration_index(ctx) + 2 * ctx->cameraLength * sizeof(BaseStationCal) / sizeof(FLT);
}

BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh) {
	if (ctx->cameraLength <= lh)
		return ctx->sos[0]->ctx->bsd[lh].fcal;

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

SURVIVE_EXPORT int survive_optimizer_get_velocity_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_calibration_index(ctx) + ctx->cameraLength * 2 * sizeof(BaseStationCal) / sizeof(FLT);
}

SURVIVE_EXPORT int survive_optimizer_get_sensor_scale_index(const survive_optimizer *ctx) {
	int idx = survive_optimizer_get_velocity_index(ctx);
	return idx + (ctx->disableVelocity ? 0 : (6 * ctx->poseLength));
}

SURVIVE_EXPORT void survive_optimizer_disable_sensor_scale(survive_optimizer *ctx) {
	if (ctx->settings->optimize_scale) {
		int idx = survive_optimizer_get_sensor_scale_index(ctx);
		for (int i = idx; i < ctx->poseLength + idx; i++) {
			ctx->parameters_info[i].fixed = true;
		}
	}
}

int survive_optimizer_get_camera_index(const survive_optimizer *ctx) { return ctx->poseLength * 7; }

SurvivePose *survive_optimizer_get_pose(survive_optimizer *ctx) {
	if (ctx->poseLength)
		return (SurvivePose *)ctx->parameters;
	return &ctx->initialPose;
}

SurviveVelocity *survive_optimizer_get_velocity(survive_optimizer *ctx) {
	if (!ctx->disableVelocity)
		return (SurviveVelocity *)&ctx->parameters[survive_optimizer_get_velocity_index(ctx)];
	return 0;
}

static inline FLT fix_infinity(FLT d) {
	// assert(isfinite(*d));
	assert(!isnan(d));
	if (!isfinite(d)) {
		return linmath_enforce_range(d, -1e3, 1e3);
	}
	return d;
}

typedef union LinmathDualPose {
    LinmathPose quatPose;
    LinmathAxisAnglePose axisAnglePose;
} LinmathDualPose;

static void ApplyDualPoseToPose(const survive_optimizer *mpfit_ctx, LinmathDualPose *out, const LinmathDualPose *p1, const LinmathDualPose *p2) {
    if(mpfit_ctx->settings->use_quat_model) {
        ApplyPoseToPose(&out->quatPose, &p1->quatPose, &p2->quatPose);
        quatnormalize(out->quatPose.Rot, out->quatPose.Rot);
    } else {
        ApplyAxisAnglePoseToPose(&out->axisAnglePose, &p1->axisAnglePose, &p2->axisAnglePose);
    }
}

static void ApplyDualPoseToPoint(const survive_optimizer *mpfit_ctx, LinmathVec3d out, const LinmathDualPose *p1, const LinmathVec3d pt) {
    if(mpfit_ctx->settings->use_quat_model) {
        ApplyPoseToPoint(out, &p1->quatPose, pt);
    } else {
        ApplyAxisAnglePoseToPoint(out, &p1->axisAnglePose, pt);
    }
}


static inline void run_pair_measurement(survive_optimizer *mpfunc_ctx, size_t meas_idx,
										const survive_optimizer_measurement *meas, const SvMat *ang_vel_jacb,
										const LinmathDualPose *obj2world, const LinmathDualPose *obj2lh,
										const LinmathDualPose *world2lh,
										const FLT* pt,
										FLT *deviates, FLT **derivs) {
	const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
	const int lh = meas->light.lh;
	const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);

	LinmathPoint3d sensorPtInLH;
    ApplyDualPoseToPoint(mpfunc_ctx, sensorPtInLH, obj2lh, pt);

	FLT out[2];
	reprojectModel->reprojectXY(cal, sensorPtInLH, out);
	assert(meas[0].light.axis == 0);
	assert(meas[1].light.axis == 1);
#ifndef NDEBUG
	if (reprojectModel->reprojectAxisFn[0]) {
		/*FLT check[] = {reprojectModel->reprojectAxisangleFullXyFn[0](obj2lh, pt, world2lh, cal),
					   reprojectModel->reprojectAxisangleFullXyFn[1](obj2lh, pt, world2lh, cal + 1)};
					   */
		FLT check[] = {reprojectModel->reprojectAxisFn[0](cal, sensorPtInLH),
					   reprojectModel->reprojectAxisFn[1](cal, sensorPtInLH)};
		for (int i = 0; i < 2; i++)
			assert(fabs(check[i] - out[i]) < 1e-5);
	}
#endif

	for (int i = 0; i < 2; i++) {
		deviates[i] = fix_infinity((out[i] - meas[i].light.value) / meas[i].variance);
	}

	if (derivs) {
        int pose_size = mpfunc_ctx->settings->use_quat_model ? 7 : 6;

		int jac_offset_lh = (lh + mpfunc_ctx->poseLength) * 7;
		int jac_offset_obj = meas->light.object * 7;

		LinmathDualPose safe_pose = *obj2world, safe_world2lh = *world2lh;
        if(!mpfunc_ctx->settings->use_quat_model) {
            if (magnitude3d(safe_pose.axisAnglePose.AxisAngleRot) == 0)
                safe_pose.axisAnglePose.AxisAngleRot[0] = 1e-10;
            if (magnitude3d(safe_world2lh.axisAnglePose.AxisAngleRot) == 0)
                safe_world2lh.axisAnglePose.AxisAngleRot[0] = 1e-10;
        }

		// d deviate / d Pose(t-)
		// d Pose(t-) / d Pose(t)
		if (derivs[jac_offset_obj]) {
			FLT jout[7 * 2] = {0};
            if(mpfunc_ctx->settings->use_quat_model) {
                reprojectModel->reprojectFullJacObjPose(jout, &obj2world->quatPose, pt, &world2lh->quatPose, cal);
            } else {
                reprojectModel->reprojectAxisAngleFullJacObjPose(jout, &obj2world->axisAnglePose, pt, &world2lh->axisAnglePose, cal);
            }
			for (int i = 0; i < 2; i++) {
				FLT tmp[6];
				copy3d(tmp, jout + 3 + i * pose_size);
				for (int j = 0; j < 3; j++) {
					jout[3 + j + i * pose_size] = dotnd_strided(tmp, ang_vel_jacb->data + j * 1, 3, 1, 3);
				}
			}

			for (int j = 0; j < pose_size; j++) {
				assert(derivs[jac_offset_obj + j] && "all 7 parameters should be the same for jacobian calculation");
				for (int k = 0; k < 2; k++) {
					if (isnan(jout[j + k * pose_size])) {
						jout[j + k * pose_size] = 0;
					}
				}
				derivs[jac_offset_obj + j][meas_idx] = fix_infinity(jout[j] / meas[0].variance);
				derivs[jac_offset_obj + j][meas_idx + 1] = fix_infinity(jout[j + pose_size] / meas[1].variance);
			}
		}

		// d deviate / d LH
		if (derivs[jac_offset_lh]) {
			FLT out[7 * 2] = {0};
            if(mpfunc_ctx->settings->use_quat_model) {
                reprojectModel->reprojectFullJacLhPose(out, &obj2world->quatPose, pt, &world2lh->quatPose, cal);
            } else {
                reprojectModel->reprojectAxisAngleFullJacLhPose(out, &obj2world->axisAnglePose, pt, &world2lh->axisAnglePose, cal);
            }
			for (int j = 0; j < pose_size; j++) {
				assert(derivs[jac_offset_lh + j] && "all 7 parameters should be the same for jacobian calculation");
				derivs[jac_offset_lh + j][meas_idx] = fix_infinity(out[j] / meas[0].variance);
				derivs[jac_offset_lh + j][meas_idx + 1] = fix_infinity(out[j + pose_size] / meas[1].variance);
			}
		}
	}
}
static void run_single_measurement(survive_optimizer *mpfunc_ctx, size_t meas_idx,
								   const survive_optimizer_measurement *meas, const SvMat *ang_vel_jacb,
								   const LinmathDualPose *obj2world, const LinmathDualPose *obj2lh,
								   const LinmathDualPose *world2lh,
                                   const FLT *pt,
								   FLT *deviates, FLT **derivs) {
	const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
	SurviveContext *ctx = mpfunc_ctx->sos[0]->ctx;
	const int lh = meas->light.lh;

	const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
    int pose_size = mpfunc_ctx->settings->use_quat_model ? 7 : 6;

	LinmathPoint3d sensorPtInLH;
    ApplyDualPoseToPoint(mpfunc_ctx, sensorPtInLH, obj2lh, pt);

	FLT out = reprojectModel->reprojectAxisFn[meas->light.axis](cal, sensorPtInLH);
	deviates[0] = fix_infinity((out - meas->light.value) / meas->variance);

	if (derivs) {
        int pose_size = mpfunc_ctx->settings->use_quat_model ? 7 : 6;
		int jac_offset_lh = (lh + mpfunc_ctx->poseLength) * 7;
		int jac_offset_obj = meas->light.object * 7;

		FLT out[7] = {0};
		// d Deviate / d Pose[t - 1] * d Pose[t - 1] / d Pose[t]
		if (derivs[jac_offset_obj]) {
		    if(mpfunc_ctx->settings->use_quat_model) {
                reprojectModel->reprojectAxisJacobFn[meas->light.axis](out, &obj2world->quatPose, pt, &world2lh->quatPose,
                                                                                cal + meas->light.axis);
		    } else {
                reprojectModel->reprojectAxisAngleAxisJacobFn[meas->light.axis](out, &obj2world->axisAnglePose, pt, &world2lh->axisAnglePose,
                                                                                cal + meas->light.axis);
            }

            int ang_size = mpfunc_ctx->settings->use_quat_model ? 4 : 3;
			FLT tmp[4];
			copynd(tmp, out + 3, ang_size);
			for (int j = 0; j < ang_size; j++) {
				out[3 + j] = dotnd_strided(tmp, ang_vel_jacb->data + j * 1, ang_size, 1, ang_size);
			}

			for (int j = 0; j < pose_size; j++) {
				assert(derivs[jac_offset_obj + j]);
				derivs[jac_offset_obj + j][meas_idx] = isfinite(out[j]) ? out[j] / meas->variance : 0;
			}
		}

		if (derivs[jac_offset_lh]) {
            if(mpfunc_ctx->settings->use_quat_model) {
                reprojectModel->reprojectAxisJacobLhPoseFn[meas->light.axis](out, &obj2world->quatPose, pt, &world2lh->quatPose,
                                                                                      cal + meas->light.axis);
            } else {
                reprojectModel->reprojectAxisAngleAxisJacobLhPoseFn[meas->light.axis](out, &obj2world->axisAnglePose, pt, &world2lh->axisAnglePose,
                                                                                      cal + meas->light.axis);
            }
			for (int j = 0; j < pose_size; j++) {
				assert(derivs[jac_offset_lh + j]);
				derivs[jac_offset_lh + j][meas_idx] = isfinite(out[j]) ? out[j] / meas->variance : 0;
			}
		}
	}
}

static void filter_measurements(survive_optimizer *optimizer, FLT *deviates) {
	struct SurviveObject *so = optimizer->sos[0];
	SurviveContext *ctx = optimizer->sos[0] ? optimizer->sos[0]->ctx : 0;

	FLT lh_deviates[NUM_GEN2_LIGHTHOUSES] = {0};
	size_t lh_meas_cnt[NUM_GEN2_LIGHTHOUSES] = {0};

	FLT avg_dev = 0, lh_avg_dev = 0;
	optimizer->stats.total_meas_cnt += optimizer->measurementsCnt;

	size_t valid_meas = 0;
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		SV_DATA_LOG("mpfit_meas_val[%d][%d][%d]", &meas->light.value, 1, meas->light.sensor_idx, meas->light.lh,
					meas->light.axis);
		SV_DATA_LOG("mpfit_meas_var[%d][%d][%d]", &meas->variance, 1, meas->light.sensor_idx, meas->light.lh,
					meas->light.axis);
		SV_DATA_LOG("mpfit_meas_err[%d][%d][%d]", deviates, 1, meas->light.sensor_idx, meas->light.lh,
					meas->light.axis);

		avg_dev += fabs(deviates[i]);
		valid_meas++;
	}

	avg_dev = avg_dev / (FLT)valid_meas;
	SV_DATA_LOG("opt_avg_deviates", &avg_dev, 1);
	if (avg_dev < .01) {
		avg_dev = .01;
	}
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		FLT P = linmath_norm_pdf(deviates[i], 0, avg_dev);
		FLT chauvenet_criterion = P * optimizer->measurementsCnt;
		if (chauvenet_criterion < .5 && false) {
			meas->invalid = true;
			optimizer->stats.dropped_meas_cnt++;

			SV_VERBOSE(105, "Ignoring noisy data at lh %d sensor %d axis %d val %f (%7.7f/%7.7f) %7.7f %7.7f",
					   meas->light.lh, meas->light.sensor_idx, meas->light.axis, meas->light.value, fabs(deviates[i]),
					   avg_dev, P, chauvenet_criterion);

			deviates[i] = 0.;
		} else {
			SV_VERBOSE(1000, "Data at lh %d sensor %d axis %d val %f (%7.7f/%7.7f)", meas->light.lh,
					   meas->light.sensor_idx, meas->light.axis, meas->light.value, fabs(deviates[i]), avg_dev);
		}
	}

	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		if (meas->meas_type == survive_optimizer_measurement_type_light && meas->invalid == false) {
			lh_deviates[meas->light.lh] += fabs(deviates[i]);
			lh_meas_cnt[meas->light.lh]++;
		}
	}

	size_t obs_lhs = 0;

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (lh_meas_cnt[i]) {
			lh_deviates[i] = lh_deviates[i] / (FLT)lh_meas_cnt[i];
			SV_DATA_LOG("opt_lh_deviate[%d]", &lh_deviates[i], 1, i);
			lh_avg_dev += lh_deviates[i];
			obs_lhs++;

			optimizer->stats.total_lh_cnt++;
		}
	}

	FLT unbias_dev = lh_avg_dev / (obs_lhs - 1.);
	lh_avg_dev = lh_avg_dev / (FLT)obs_lhs;

	if (lh_avg_dev < .01) {
		lh_avg_dev = 0.01;
	}
	SV_DATA_LOG("opt_lh_avg_deviates", &lh_avg_dev, 1);

	if (obs_lhs > 2) {
		for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
			if (lh_meas_cnt[i] == 0)
				continue;

			FLT corrected_dev = unbias_dev - lh_deviates[i] / (obs_lhs - 1.);
			SV_DATA_LOG("opt_lh_corrected_dev[%d]", &corrected_dev, 1, i);
			FLT P = linmath_norm_pdf(lh_deviates[i], 0, lh_avg_dev);
			FLT chauvenet_criterion = P * obs_lhs;

			if (lh_deviates[i] > 100 * corrected_dev) {
				SV_VERBOSE(100, "Data from LH %d seems suspect for %s (%f/%10.10f -- %f)", i,
						   optimizer->sos[0]->codename, lh_deviates[i], corrected_dev, lh_deviates[i] / corrected_dev);
				lh_meas_cnt[i] = 0;
				optimizer->stats.dropped_lh_cnt++;
			} else if (lh_deviates[i] > 0.) {
				SV_VERBOSE(500, "Data from LH %d seems OK for %s (%f/%f -- %f)", i, optimizer->sos[0]->codename,
						   lh_deviates[i], lh_avg_dev, lh_deviates[i] / corrected_dev);
			}
		}
	}

	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		if (meas->meas_type == survive_optimizer_measurement_type_light && lh_meas_cnt[meas->light.lh] == 0) {
			meas->invalid = true;
		}
	}

	optimizer->needsFiltering = false;
}

static int survive_optimizer_get_meas_size(const survive_optimizer *ctx) {
	int rtn = 0;
	for (int i = 0; i < ctx->measurementsCnt; i++) {
		assert(ctx->measurements[i].size != 0);
		rtn += ctx->measurements[i].size;
	}
	return rtn;
}

static int mpfunc(int m, int n, FLT *p, FLT *deviates, FLT **derivs, void *private) {
	survive_optimizer *mpfunc_ctx = private;

	assert(survive_optimizer_get_meas_size(mpfunc_ctx) == m);
	assert(survive_optimizer_get_parameters_count(mpfunc_ctx) == n);

	mpfunc_ctx->parameters = p;

	LinmathDualPose *cameras = (LinmathDualPose*)survive_optimizer_get_camera(mpfunc_ctx);

	int start = survive_optimizer_get_camera_index(mpfunc_ctx);

	int pose_idx = -1;
	FLT calced_timecode = -1;
	LinmathDualPose obj2world = {0};
    LinmathDualPose obj2lh[NUM_GEN2_LIGHTHOUSES] = {0};

    int ang_size = mpfunc_ctx->settings->use_quat_model ? 4 : 3;
	SV_CREATE_STACK_MAT(ang_velocity_jac, ang_size, ang_size);
	sv_set_diag_val(&ang_velocity_jac, 1);

	int meas_count = m;
	int meas_idx = 0;

	for (int mea_block_idx = 0; mea_block_idx < mpfunc_ctx->measurementsCnt; mea_block_idx++) {
		survive_optimizer_measurement *meas = &mpfunc_ctx->measurements[mea_block_idx];

		if (meas->invalid) {
			// mpfit zero initializes everything; so just let deviates/derivs stay 0
			meas_idx += meas->size;
			continue;
		}

		switch (meas->meas_type) {
		case survive_optimizer_measurement_type_light: {
			// If the next two measurements are joined; handle the full pair. This lets us just calculate
			// sensorPtInLH once
			const bool nextIsPair =
				mea_block_idx + 1 < mpfunc_ctx->measurementsCnt && mpfunc_ctx->disableVelocity == true &&
				meas[1].meas_type == survive_optimizer_measurement_type_light && meas[0].light.axis == 0 &&
				meas[1].light.axis == 1 && meas[0].light.sensor_idx == meas[1].light.sensor_idx && !meas[1].invalid &&
				!mpfunc_ctx->settings->disallow_pair_calc;

			const int lh = meas->light.lh;
			const FLT *sensor_points = survive_optimizer_get_sensors(mpfunc_ctx, meas->light.object);

			LinmathDualPose *world2lh = &cameras[lh];
			const FLT *ptp = &sensor_points[meas->light.sensor_idx * 3];
			LinmathVec3d pt;
			copy3d(pt, ptp);

			// d a / d s = d a / d xyz * d xyz / d s
			int scale_idx = -1;
			LinmathVec3d xyzjac_scale = {};
			bool needsScaleJac = false;
			if (mpfunc_ctx->settings->optimize_scale) {
				scale_idx = survive_optimizer_get_sensor_scale_index(mpfunc_ctx) + meas->light.object;
				FLT scale = p[scale_idx];

				SurvivePose imu2trackref = mpfunc_ctx->sos[meas->light.object]->imu2trackref;
				needsScaleJac = derivs && scale_idx >= 0 && derivs[scale_idx];
				if (needsScaleJac) {
					gen_scale_sensor_pt_jac_scale(xyzjac_scale, pt, &imu2trackref, scale);
				}
				gen_scale_sensor_pt(pt, pt, &imu2trackref, scale);
			}

			if (calced_timecode != meas->time && !mpfunc_ctx->disableVelocity) {
				pose_idx = -1;
			}

			if (pose_idx != meas->light.object) {
				calced_timecode = meas->time;
				pose_idx = meas->light.object;
				assert(pose_idx < mpfunc_ctx->poseLength);
				obj2world = *(LinmathDualPose*)(&survive_optimizer_get_pose(mpfunc_ctx)[meas->light.object]);

				if (mpfunc_ctx->disableVelocity == false) {
					LinmathDualPose dPose = obj2world;
					FLT diff = mpfunc_ctx->timecode - meas->time;
					SurviveVelocity *v = survive_optimizer_get_velocity(mpfunc_ctx);

                    if(mpfunc_ctx->settings->use_quat_model) {
                        addscalednd(dPose.quatPose.Pos, obj2world.quatPose.Pos, v->Pos, -diff, 3);
                        survive_apply_ang_velocity(dPose.quatPose.Rot, v->AxisAngleRot, -diff,
                                                   obj2world.quatPose.Rot);
                        gen_apply_ang_velocity_jac_q(ang_velocity_jac.data, v->AxisAngleRot, -diff,
                                                                  obj2world.quatPose.Rot);
					} else {
                        addscalednd(dPose.axisAnglePose.Pos, obj2world.axisAnglePose.Pos, v->Pos, -diff, 3);
                        survive_apply_ang_velocity_aa(dPose.axisAnglePose.AxisAngleRot, v->AxisAngleRot, -diff,
                                                      obj2world.axisAnglePose.AxisAngleRot);
                        gen_apply_ang_velocity_aa_jac_axis_angle2(ang_velocity_jac.data, v->AxisAngleRot, -diff,
                                                                  obj2world.axisAnglePose.AxisAngleRot);
                    }
					obj2world = dPose;
				}

				if(mpfunc_ctx->settings->use_quat_model) {
                    quatnormalize(obj2world.quatPose.Rot, obj2world.quatPose.Rot);
				}

				int lh_count = mpfunc_ctx->cameraLength > 0 ? mpfunc_ctx->cameraLength
															: mpfunc_ctx->sos[pose_idx]->ctx->activeLighthouses;
				for (int lh = 0; lh < lh_count; lh++) {
                    ApplyDualPoseToPose(mpfunc_ctx, &obj2lh[lh], &cameras[lh], &obj2world);
				}
			}

			const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
			if (needsScaleJac) {
				for (int meas_idx_jac = 0; meas_idx_jac < (1 + nextIsPair); meas_idx_jac++) {
					LinmathVec3d ptJac = {};
					int axis = meas[meas_idx_jac].light.axis;
					const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
					if (mpfunc_ctx->settings->use_quat_model) {
						reprojectModel->reprojectAxisJacobSensorPt[axis](ptJac, &obj2world.quatPose, pt,
																		 &world2lh->quatPose, cal + axis);
					} else {
						reprojectModel->reprojectAxisAngleAxisJacobSensorPt[axis](ptJac, &obj2world.axisAnglePose, pt,
																				  &world2lh->axisAnglePose, cal + axis);
					}
					scale3d(ptJac, ptJac, 1. / meas[meas_idx_jac].variance);
					derivs[scale_idx][meas_idx + meas_idx_jac] = dot3d(xyzjac_scale, ptJac);
				}
			}

			if (nextIsPair) {
				run_pair_measurement(mpfunc_ctx, meas_idx, meas, &ang_velocity_jac, &obj2world, &obj2lh[lh], world2lh,
									 pt,
									 deviates + meas_idx, derivs);
				meas_idx++;
				mea_block_idx++;
			} else {
				run_single_measurement(mpfunc_ctx, meas_idx, meas, &ang_velocity_jac, &obj2world, &obj2lh[lh], world2lh,
                                       pt,
									   deviates + meas_idx, derivs);
			}

			break;
		}
		case survive_optimizer_measurement_type_camera_accel: {
			LinmathPoint3d up = {0};

			FLT deriv[4] = {0}, error = 0;
			size_t deriv_idx = 0;

			size_t lh = meas->camera_acc.camera;
			normalize3d(up, survive_optimizer_cam_up_vector(mpfunc_ctx, lh));

            deriv_idx = survive_optimizer_get_camera_index(mpfunc_ctx) + lh * 7 + 3;

			LinmathDualPose *world2lh = (LinmathDualPose *)&cameras[lh];
            if(mpfunc_ctx->settings->use_quat_model) {
                if (isfinite(up[0])) {
                    error = gen_world2lh_up_err(world2lh->quatPose.Rot, up);
                    gen_world2lh_up_err_jac_q1(deriv, world2lh->quatPose.Rot, up);
                }
			} else {
                if (isfinite(up[0])) {
                    error = gen_world2lh_aa_up_err(world2lh->axisAnglePose.AxisAngleRot, up);
                    gen_world2lh_aa_up_err_jac_axis_angle(deriv, world2lh->axisAnglePose.AxisAngleRot, up);
                }
            }

			deviates[meas_idx] = error / meas->variance;
			for (int i = 0; i < ang_size && derivs && derivs[deriv_idx + i]; i++) {
				derivs[deriv_idx + i][meas_idx] = fix_infinity(deriv[i] / meas->variance);
			}
			break;
		}
		case survive_optimizer_measurement_type_object_accel: {
			int obj = meas->pose_acc.object;
			LinmathPoint3d up = {0};
            copy3d(up, survive_optimizer_obj_up_vector(mpfunc_ctx, obj));

			LinmathAxisAngle rot = {0};
			FLT deriv[4] = {0}, error = 0;

			LinmathDualPose *obj2world = (LinmathDualPose  *)(&survive_optimizer_get_pose(mpfunc_ctx)[obj]);
            if(mpfunc_ctx->settings->use_quat_model) {
                error = gen_obj2world_up_err(obj2world->quatPose.Rot, up);
                gen_obj2world_up_err_jac_q1(deriv, obj2world->quatPose.Rot, up);
            } else {
                error = gen_obj2world_aa_up_err(obj2world->axisAnglePose.AxisAngleRot, up);
                gen_obj2world_aa_up_err_jac_axis_angle(deriv, obj2world->axisAnglePose.AxisAngleRot, up);
            }

            int deriv_idx = obj * 7 + 3;
			deviates[meas_idx] = error / meas->variance;
			for (int i = 0; i < ang_size && derivs && derivs[deriv_idx + i]; i++) {
				derivs[deriv_idx + i][meas_idx] = fix_infinity(deriv[i] / meas->variance);
			}
			break;
		}
		case survive_optimizer_measurement_type_object_pose:
			assert(false);
			/*
			if (mpfunc_ctx->current_bias > 0) {
				light_meas -= 7;
				FLT *pp = (FLT *)mpfunc_ctx->initialPose.Pos;
				for (int i = 0; i < 7; i++) {
					deviates[i + light_meas] = (p[i] - pp[i]) * mpfunc_ctx->current_bias;
					if (derivs && derivs[i]) {
						derivs[i][i + light_meas] = mpfunc_ctx->current_bias;
					}
				}
			}
			 */
		}

		meas_idx += meas->size;
	}

	if (mpfunc_ctx->needsFiltering) {
		assert(derivs == 0);
		filter_measurements(mpfunc_ctx, deviates);
	}

	if (mpfunc_ctx->iteration_cb) {
		mpfunc_ctx->iteration_cb(mpfunc_ctx, m, n, p, deviates, derivs);
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

static inline bool sane_covariance(const SvMat *P) {
#ifndef NDEBUG
	for (int i = 0; i < P->rows; i++) {
		if (svMatrixGet(P, i, i) < 0)
			return false;
	}
#ifdef USE_EIGEN
	return svDet(P) > -1e-10;
#endif
#endif
	return true;
}

int survive_optimizer_nonfixed_index(survive_optimizer *ctx, int idx) {
	if (ctx->parameters_info[idx].fixed)
		return -1;

	int rtn = 0;
	for (int i = 0; i < idx; i++) {
		if (!ctx->parameters_info[i].fixed)
			rtn++;
	}
	return rtn;
}

int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result, struct SvMat *R) {
	SurviveContext *ctx = optimizer->sos[0] ? optimizer->sos[0]->ctx : 0;

	mp_config *cfg = optimizer->cfg;
	if (cfg == 0)
		cfg = survive_optimizer_get_cfg(ctx);

	SurvivePose *poses = survive_optimizer_get_pose(optimizer);

	if(!optimizer->settings->use_quat_model) {
        for (int i = 0; i < optimizer->poseLength + optimizer->cameraLength; i++) {
            quattoaxisanglemag(poses[i].Rot, poses[i].Rot);
            poses[i].Rot[3] = 0; // NAN;
            optimizer->parameters_info[i * 7 + 6].fixed = true;
        }
    }

#ifndef NDEBUG
	for (int i = 0; i < survive_optimizer_get_parameters_count(optimizer); i++) {
		if ((optimizer->parameters_info[i].limited[0] &&
			 optimizer->parameters[i] < optimizer->parameters_info[i].limits[0]) ||
			(optimizer->parameters_info[i].limited[1] &&
			 optimizer->parameters[i] > optimizer->parameters_info[i].limits[1]) ||
			isnan(optimizer->parameters[i])) {
			survive_optimizer_serialize(optimizer, "debug.opt");
			SurviveContext *ctx = optimizer->sos[0]->ctx;
			SV_GENERAL_ERROR("Parameter %s(%d) is invalid. %f <= %f <= %f should be true",
							 optimizer->parameters_info[i].parname, i, optimizer->parameters_info[i].limits[0],
							 optimizer->parameters[i], optimizer->parameters_info[i].limits[1])
		}
	}
#endif

	// MPFit runs on temporary storage; so parameters is manipulated in mpfunc. Save it and restore it here.
	FLT *params = optimizer->parameters;
	optimizer->needsFiltering = !optimizer->nofilter;
	size_t meas_count = survive_optimizer_get_meas_size(optimizer);

	int nfree = survive_optimizer_get_free_parameters_count(optimizer);
	FLT *covar = R ? R->data : 0;
	SV_CREATE_STACK_MAT(R_aa, nfree * (covar ? 1 : 0), nfree * (covar ? 1 : 0));
	result->covar_free = covar ? R_aa.data : 0;

	int rtn = mpfit(mpfunc, meas_count, survive_optimizer_get_parameters_count(optimizer), optimizer->parameters,
					optimizer->parameters_info, cfg, optimizer, result);
	optimizer->parameters = params;
	assert(sane_covariance(&R_aa));
	int totalPoseCount = optimizer->poseLength + optimizer->cameraLength;
    int pose_size = optimizer->settings->use_quat_model ? 7 : 6;
    int totalFreePoseCount = nfree / pose_size;
	if (covar) {
		SvMat R_q = svMat(R->rows, R->cols, covar);
		if (optimizer->settings->optimize_scale) {
			int idx = survive_optimizer_get_sensor_scale_index(optimizer);
			int free_idx = survive_optimizer_nonfixed_index(optimizer, idx);
			if (free_idx >= 0) {
				for (int z = 0; z < 1; z++) {
					FLT scale_cov = svMatrixGet(&R_aa, free_idx + z, free_idx + z);
					if (scale_cov > 0) {
						FLT scale = params[idx + z];

						FLT y = scale - optimizer->sos[z]->sensor_scale;
						FLT k = optimizer->sos[z]->sensor_scale_var / (optimizer->sos[z]->sensor_scale_var + scale_cov);
						optimizer->sos[z]->sensor_scale += k * y;
						optimizer->sos[z]->sensor_scale_var *= (1 - k);
						// printf("SENSOR_SCALE %s %e %f %e %f %+f %f\n", optimizer->sos[z]->codename, scale_cov, scale,
						//       optimizer->sos[z]->sensor_scale_var, optimizer->sos[z]->sensor_scale, y, k);
					}
				}
			}
		}
		if(optimizer->settings->use_quat_model) {
            svCopy(&R_aa, &R_q, 0);
		} else {
			SV_CREATE_STACK_MAT(G, R->rows, R_aa.rows);
			SV_CREATE_STACK_MAT(Gp, 4, 3);
            assert(R->rows == R->cols);
            //assert(R->rows == totalFreePoseCount * 7);

            sv_set_diag_val(&G, 1);

            for (int z = 0; z < R->rows / 7; z++) {
                gen_axisangle2quat_jac_axis_angle(Gp.data, ((LinmathAxisAnglePose *) &poses[z])->AxisAngleRot);
                for (int i = 0; i < 4; i++) {
                    for (int j = 0; j < 3; j++) {
                        svMatrixSet(&G, i + z * 7 + 3, j + z * 6 + 3, svMatrixGet(&Gp, i, j));
                    }
                }
            }

            gemm_ABAt_add_scaled(&R_q, &G, &R_aa, 0, 1,
                                 result->bestnorm / (optimizer->measurementsCnt - totalFreePoseCount * pose_size), 0);
            assert(sane_covariance(&R_q));
        }
	}

    if(!optimizer->settings->use_quat_model) {
        for (int i = 0; i < totalPoseCount; i++) {
            quatfromaxisangle(poses[i].Rot, poses[i].Rot, norm3d(poses[i].Rot));
        }
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
	if(f == 0)
	  return;

	fprintf(f, "object       %s\n", opt->sos[0]->codename);
	fprintf(f, "currentBias  %+0.16f\n", opt->current_bias);
	fprintf(f, "initialPose " SurvivePose_format "\n", SURVIVE_POSE_EXPAND(opt->initialPose));
	fprintf(f, "model        %d\n", opt->reprojectModel != &survive_reproject_gen1_model);
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
		fprintf(f, "\t%d", meas->light.lh);
		fprintf(f, " %d", meas->light.axis);
		fprintf(f, " %2d", meas->light.sensor_idx);
		fprintf(f, " %d", meas->light.object);
		fprintf(f, " %+0.16f", meas->light.value);
		fprintf(f, " %+0.16f\n", meas->variance);
	}

	fclose(f);
}

survive_optimizer *survive_optimizer_load(const char *fn) {
	survive_optimizer *opt = calloc(sizeof(survive_optimizer), 1);

	FILE *f = fopen(fn, "r");
	if (f == 0)
		return 0;
	int read_count = 0;

#ifndef LINE_MAX
#define LINE_MAX 2048
#endif
	char buffer[LINE_MAX] = { 0 };
	char device_name[LINE_MAX] = {0};
	opt->poseLength = 1;
	read_count = fscanf(f, "object       %s\n", device_name);
	read_count = fscanf(f, "currentBias  " FLT_sformat "\n", &opt->current_bias);
	read_count = fscanf(f, "initialPose " SurvivePose_sformat "\n", SURVIVE_POSE_SCAN_EXPAND(opt->initialPose));
	int model = 0;
	read_count = fscanf(f, "model        %d\n", &model);
	opt->reprojectModel = model == 0 ? &survive_reproject_gen1_model : &survive_reproject_gen2_model;
	read_count = fscanf(f, "poseLength   %d\n", &opt->poseLength);
	read_count = fscanf(f, "cameraLength %d\n", &opt->cameraLength);
	read_count = fscanf(f, "ptsLength    %d\n", &opt->ptsLength);

	int param_count;
	read_count = fscanf(f, "parameters   %d\n", &param_count);
	char *success = fgets(buffer, LINE_MAX, f); // fscanf(f, "\t#<name>: <fixed> <value> <min> <max> <use_jacobian>\n");
	assert(success);

	(void)read_count;
	assert(param_count == survive_optimizer_get_parameters_count(opt));

	SURVIVE_OPTIMIZER_SETUP_HEAP_BUFFERS(*opt, 0);

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
		read_count = fscanf(f, "\t%hhu", &meas->light.lh);
		read_count = fscanf(f, " %hhu", &meas->light.axis);
		read_count = fscanf(f, " %hhu", &meas->light.sensor_idx);
		read_count = fscanf(f, " %d", &meas->light.object);
		read_count = fscanf(f, " " FLT_sformat, &meas->light.value);
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
			size_t read_size = fread(ct0conf, 1, len, fp);
			if (read_size != len) {
				fprintf(stderr, "Could not read full full config file %s\n", filename);
			}
			survive_default_config_process(so, ct0conf, len);
			fclose(fp);
		}
	}
	opt->sos[0] = so;

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
	size_t sensor_cnt = 32;
	return par_count * (sizeof(FLT) +				  // parameters
						sizeof(struct mp_par_struct)) // parameters_info
		   + ctx->poseLength * sizeof(survive_optimizer_measurement) * 2 * sensor_cnt *
				 NUM_GEN2_LIGHTHOUSES; // measurements
}

SURVIVE_EXPORT void survive_optimizer_setup_buffers(survive_optimizer *ctx, void *parameter_buffer,
													void *parameter_info_buffer, void *measurements_buffer,
													void *so_buffer) {
	size_t par_count = survive_optimizer_get_parameters_count(ctx);
	size_t meas_count = survive_optimizer_get_max_measurements_count(ctx);

	ctx->parameters = (FLT *)parameter_buffer;
	for (int i = 0; i < par_count; i++)
		ctx->parameters[i] = NAN;
	size_t par_offset = par_count * sizeof(FLT);
	ctx->parameters_info = (struct mp_par_struct *)(parameter_info_buffer);

	size_t par_info_offset = par_offset + sizeof(struct mp_par_struct) * par_count;

	ctx->sos = so_buffer;

	ctx->measurements = (survive_optimizer_measurement *)(measurements_buffer);
	size_t measurementAllocSize = meas_count * sizeof(survive_optimizer_measurement);
	memset(ctx->measurements, 0, measurementAllocSize);
	// ctx->obj_up_vectors = (LinmathPoint3d *)((uint8_t *)measurements_buffer + measurementAllocSize);
	// ctx->cam_up_vectors = ctx->obj_up_vectors + ctx->poseLength;
	memset(ctx->parameters_info, 0, sizeof(mp_par) * par_count);
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		ctx->parameters_info[i].fixed = 1;
	}

	if (ctx->current_bias != 0) {
		for (int i = 0; i < ctx->poseLength; i++) {
			survive_optimizer_measurement *meas =
				survive_optimizer_emplace_meas(ctx, survive_optimizer_measurement_type_object_pose);
			meas->pose.object = i;
			meas->pose.pose = Pose2AAPose(&ctx->sos[i]->OutPoseIMU);
			meas->variance = 1. / ctx->current_bias;
		}
	}

	if (ctx->objectUpVectorVariance > 0) {
		for (int i = 0; i < ctx->poseLength; i++) {
		    if(ctx->sos[i]) {
                FLT n = norm3d(ctx->sos[i]->activations.accel);
                if (isfinite(n) && fabs(1 - n) < .01) {
                    survive_optimizer_measurement *meas =
                            survive_optimizer_emplace_meas(ctx, survive_optimizer_measurement_type_object_accel);
                    meas->pose_acc.object = i;
                    normalize3d(meas->pose_acc.acc, ctx->sos[i]->activations.accel);
                    meas->variance = ctx->objectUpVectorVariance;
                }
            }
		}
	}
}

SURVIVE_EXPORT void *survive_optimizer_realloc(void *old_ptr, size_t size) { return realloc(old_ptr, size); }

int survive_optimizer_get_max_measurements_count(const survive_optimizer *ctx) {
	int sensor_cnt = SENSORS_PER_OBJECT;
	assert(ctx->poseLength > 0 && ctx->poseLength < 20);
	return ctx->poseLength * 2 * sensor_cnt * NUM_GEN2_LIGHTHOUSES + (ctx->current_bias == 0 ? 0 : ctx->poseLength) +
		   (ctx->poseLength + ctx->cameraLength);
}

int meas_size(enum survive_optimizer_measurement_type type) {
	switch (type) {
	case survive_optimizer_measurement_type_light:
		return 1;
	case survive_optimizer_measurement_type_camera_accel:
	case survive_optimizer_measurement_type_object_accel:
		return 3;
	case survive_optimizer_measurement_type_object_pose:
		return 6;
	default:
		assert(false);
	}
	return 0;
}

survive_optimizer_measurement *survive_optimizer_emplace_meas(survive_optimizer *ctx,
															  enum survive_optimizer_measurement_type type) {
	assert(survive_optimizer_get_max_measurements_count(ctx) > ctx->measurementsCnt);
	survive_optimizer_measurement *rtn = &ctx->measurements[ctx->measurementsCnt++];
	rtn->meas_type = type;
	rtn->size = meas_size(type);
	return rtn;
}

void survive_optimizer_pop_meas(survive_optimizer *ctx, int cnt) {
	assert(ctx->measurementsCnt >= cnt);
	ctx->measurementsCnt -= cnt;
}

SURVIVE_EXPORT FLT *survive_optimizer_obj_up_vector(survive_optimizer *ctx, int n) {
	for (int i = 0; i < ctx->measurementsCnt; i++) {
		if (ctx->measurements[i].meas_type == survive_optimizer_measurement_type_object_accel &&
			ctx->measurements[i].pose_acc.object == n) {
			return ctx->measurements[i].pose_acc.acc;
		}
	}
	return 0;
}
SURVIVE_EXPORT FLT *survive_optimizer_cam_up_vector(survive_optimizer *ctx, int n) {
	for (int i = 0; i < ctx->measurementsCnt; i++) {
		if (ctx->measurements[i].meas_type == survive_optimizer_measurement_type_camera_accel &&
			ctx->measurements[i].camera_acc.camera == n) {
			return ctx->measurements[i].pose_acc.acc;
		}
	}

	return 0;
}

void survive_optimizer_set_cam_up_vector(survive_optimizer *ctx, int i, FLT variance, const FLT *up) {
	FLT n = norm3d(up);
	if (!isfinite(n) || n == 0)
		return;

	FLT *storage = survive_optimizer_cam_up_vector(ctx, i);
	if (storage == 0 && variance > 0) {
		survive_optimizer_measurement *meas =
			survive_optimizer_emplace_meas(ctx, survive_optimizer_measurement_type_camera_accel);
		meas->camera_acc.camera = i;
		normalize3d(meas->camera_acc.acc, up);
		meas->variance = variance;
		storage = meas->camera_acc.acc;
	}
	if (storage) {
		normalize3d(storage, up);
	}
}

void survive_optimizer_set_obj_up_vector(survive_optimizer *ctx, int i, FLT variance, const FLT *up) {
	FLT *storage = survive_optimizer_obj_up_vector(ctx, i);
	if (storage == 0 && variance > 0) {
		survive_optimizer_measurement *meas =
			survive_optimizer_emplace_meas(ctx, survive_optimizer_measurement_type_object_accel);
		meas->pose_acc.object = i;
		normalize3d(meas->pose_acc.acc, up);
		meas->variance = variance;
		storage = meas->pose_acc.acc;
	}
	if (storage) {
		normalize3d(storage, up);
	}
}
