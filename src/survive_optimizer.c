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

#include "mpfit/mpfit.h"
#include "survive_default_devices.h"
#include "survive_kalman_tracker.h"
#include "survive_recording.h"

#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <cnmatrix/cn_matrix.h>
#include <malloc.h>

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
	STRUCT_CONFIG_ITEM("mpfit-disable-filter", "Model mpfit as quaternion", 0, t->disable_filter)
	STRUCT_CONFIG_ITEM("mpfit-quat-model", "Model mpfit as quaternion", 0, t->use_quat_model)
	STRUCT_CONFIG_ITEM("mpfit-lh-scale-correction", "", 0, t->lh_scale_correction)
	STRUCT_CONFIG_ITEM("mpfit-lh-offset-correction", "", 0, t->lh_offset_correction)
	STRUCT_CONFIG_ITEM("mpfit-no-pair-calc", "Don't process as pairs", 0, t->disallow_pair_calc)
	STRUCT_CONFIG_ITEM("mpfit-optimize-scale-threshold", "Treat scale as mutable", -1, t->optimize_scale_threshold)
	STRUCT_CONFIG_ITEM("mpfit-current-pos-bias", "", -1, t->current_pos_bias)
	STRUCT_CONFIG_ITEM("mpfit-current-rot-bias", "", -1, t->current_rot_bias)
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

int survive_optimizer_get_start_index(const survive_optimizer *ctx, enum survive_optimizer_parameter_type type) {
	int rtn = 0;
	for (int i = 0; i < ctx->parameterBlockCnt; i++) {
		if (ctx->parameters_info[i].param_type == type)
			return rtn;
		rtn += ctx->parameters_info[i].size;
	}
	return -1;
}
int survive_optimizer_get_block_index(const survive_optimizer *ctx, enum survive_optimizer_parameter_type type) {
	for (int i = 0; i < ctx->parameterBlockCnt; i++) {
		if (ctx->parameters_info[i].param_type == type)
			return i;
	}
	return -1;
}

survive_optimizer_parameter *survive_optimizer_get_start_parameter_info(const survive_optimizer *ctx,
																		enum survive_optimizer_parameter_type type) {
	int index = survive_optimizer_get_block_index(ctx, type);
	if (index == -1)
		return 0;
	return &ctx->parameters_info[index];
}
int get_lighthouse_correction_idx_for(survive_optimizer *optimizer, int obj, int lh, int axis) {
	survive_optimizer_parameter * lh_correction = survive_optimizer_get_start_parameter_info(optimizer, survive_optimizer_parameter_object_lighthouse_correction);
	if(!lh_correction) return -1;
	assert(obj == 0);
	return lh_correction->p_idx + lh * SURVIVE_CORRECTION_PARAMS + axis;
}
FLT get_lighthouse_correction_for(survive_optimizer *optimizer, int obj, int lh, int axis) {
	assert(axis < SURVIVE_CORRECTION_PARAMS);
	int idx = get_lighthouse_correction_idx_for(optimizer, obj, lh, axis);
	if(idx < 0) return 0;
	return optimizer->parameters[idx];
}

void survive_optimizer_setup_pose_n(survive_optimizer *mpfit_ctx, const SurvivePose *pose, size_t n, bool isFixed,
									int use_jacobian_function) {
	if (pose)
		survive_optimizer_get_pose(mpfit_ctx)[n] = *pose;
	else
		survive_optimizer_get_pose(mpfit_ctx)[n] = (SurvivePose){.Rot = {1.}};

	setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + n * 7, mpfit_ctx->mp_parameters_info + n * 7);

	for (size_t i = n * 7; i < 7 * (n + 1); i++) {
		mpfit_ctx->mp_parameters_info[i].fixed = isFixed;
		mpfit_ctx->mp_parameters_info[i].parname = object_parameter_names[i % 7];

		if (use_jacobian_function != 0) {
			assert(mpfit_ctx->reprojectModel->reprojectAxisAngleFullJacObjPose);
			if (use_jacobian_function < 0) {
				mpfit_ctx->mp_parameters_info[i].side = 2;
				mpfit_ctx->mp_parameters_info[i].deriv_debug = 1;
				mpfit_ctx->mp_parameters_info[i].deriv_abstol = .01;
				mpfit_ctx->mp_parameters_info[i].deriv_reltol = .01;
				mpfit_ctx->mp_parameters_info[i].step = 1e-3;
			} else {
				mpfit_ctx->mp_parameters_info[i].side = 3;
			}
		}
	}

	if (!mpfit_ctx->disableVelocity) {
		int v_idx = survive_optimizer_get_velocity_index(mpfit_ctx);
		survive_optimizer_get_velocity(mpfit_ctx)[n] = (SurviveVelocity){0};
		for (int i = 0; i < 6; i++) {
			mpfit_ctx->mp_parameters_info[i + v_idx].fixed = true;
			mpfit_ctx->mp_parameters_info[i + v_idx].parname = vel_parameter_names[i % 6];
			mpfit_ctx->mp_parameters_info[i + v_idx].side = 0;
		}
	}
	survive_optimizer_parameter * lh_correction = survive_optimizer_get_start_parameter_info(mpfit_ctx, survive_optimizer_parameter_object_lighthouse_correction);
	if(lh_correction) {
		for (int i = 0; i < mpfit_ctx->poseLength; i++) {
			FLT* lh_obj_params = lh_correction[i].p;
			SurviveObject * so = mpfit_ctx->sos[i];
			if (so == 0)
				break;

			struct mp_par_struct* info = lh_correction[i].pi;
			for(int lh = 0;lh < NUM_GEN2_LIGHTHOUSES;lh++) {
				for (int axis = 0; axis < SURVIVE_CORRECTION_PARAMS; axis++) {
					struct mp_par_struct* p_info = &info[lh*SURVIVE_CORRECTION_PARAMS+axis];
					p_info->fixed = true;
					lh_obj_params[lh * SURVIVE_CORRECTION_PARAMS + axis] = so->lh_correction[lh][axis];
					p_info->step = 1e-5;

					if (use_jacobian_function != 0 && axis != 2) {
						if (use_jacobian_function < 0) {
							p_info->side = 2;
							p_info->deriv_debug = 1;
							p_info->deriv_abstol = 1e-4;
							p_info->deriv_reltol = 1e-4;
							//p_info->step = 1e-1;
						} else {
							p_info->side = 3;
						}
					}

				}
			}
		}
	}

	if (mpfit_ctx->settings->optimize_scale_threshold >= 0) {
		int s_idx = survive_optimizer_get_sensor_scale_index(mpfit_ctx);
		for (int i = 0; i < mpfit_ctx->poseLength; i++) {
			mpfit_ctx->parameters[s_idx + i] = mpfit_ctx->sos[n]->sensor_scale;
			struct mp_par_struct *pinfo = &mpfit_ctx->mp_parameters_info[i + s_idx];
			pinfo->fixed = mpfit_ctx->settings->optimize_scale_threshold > mpfit_ctx->sos[n]->sensor_scale_var;
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

			survive_optimizer_measurement * meas = survive_optimizer_emplace_meas(mpfit_ctx, survive_optimizer_measurement_type_parameters_bias);
			meas->variance = mpfit_ctx->sos[n]->sensor_scale_var;
			meas->parameter_bias.parameter_index = s_idx;
			meas->parameter_bias.expected_value = mpfit_ctx->sos[n]->sensor_scale;
		}
	}
}
SURVIVE_EXPORT void survive_optimizer_remove_data_for_lh(survive_optimizer *optimizer, int cam_idx) {
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		if (meas->meas_type == survive_optimizer_measurement_type_light && meas->light.lh == cam_idx) {
			meas->invalid = true;
		}
	}
}

SURVIVE_EXPORT void survive_optimizer_fix_cam_pos(survive_optimizer *mpfit_ctx, int lh_idx) {
	survive_optimizer_measurement *meas =
		survive_optimizer_emplace_meas(mpfit_ctx, survive_optimizer_measurement_type_camera_position);
	meas->variance = 1e-7;
	meas->camera_pos.camera = lh_idx;
	SurvivePose *cam = &survive_optimizer_get_camera(mpfit_ctx)[lh_idx];
	SurvivePose p = InvertPoseRtn(cam);
	copy3d(meas->camera_pos.pos, p.Pos);
}
void survive_optimizer_fix_cam_yaw(survive_optimizer *mpfit_ctx, int lh_idx) {
	survive_optimizer_measurement *meas =
		survive_optimizer_emplace_meas(mpfit_ctx, survive_optimizer_measurement_type_fixed_rotation);
	meas->variance = 1e-7;
	meas->fixed_rotation.obj = mpfit_ctx->poseLength + lh_idx;
	LinmathPoint3d dir = {1, 0, 0};

	LinmathPoint3d pdir;
	LinmathQuat q;
	quatgetconjugate(q, survive_optimizer_get_camera(mpfit_ctx)[lh_idx].Rot);
	quatrotatevector(pdir, q, dir);

	// Plane that goes through (0, 0, 0), (0, 0, 1), and pdir
	copy3d(meas->fixed_rotation.match_vec, dir);
	meas->fixed_rotation.conjugate = true;
	meas->fixed_rotation.plane[0] = -pdir[1];
	meas->fixed_rotation.plane[1] = pdir[0];
	meas->fixed_rotation.plane[2] = 0;
}
void survive_optimizer_fix_obj_yaw(survive_optimizer *mpfit_ctx, int obj_idx) {
	survive_optimizer_measurement *meas =
		survive_optimizer_emplace_meas(mpfit_ctx, survive_optimizer_measurement_type_fixed_rotation);
	meas->variance = 1e-7;
	meas->fixed_rotation.obj = obj_idx;
	LinmathPoint3d dir = {1, 0, 0};

	LinmathPoint3d pdir;
	quatrotatevector(pdir, survive_optimizer_get_pose(mpfit_ctx)[obj_idx].Rot, dir);
	// Plane that goes through (0, 0, 0), (0, 0, 1), and pdir
	copy3d(meas->fixed_rotation.match_vec, dir);
	meas->fixed_rotation.plane[0] = -pdir[1];
	meas->fixed_rotation.plane[1] = pdir[0];
	meas->fixed_rotation.plane[2] = 0;
}
void survive_optimizer_fix_camera(survive_optimizer *mpfit_ctx, int cam_idx) {
	int start = survive_optimizer_get_camera_index(mpfit_ctx) + cam_idx * 7;
	for (int i = start; i < start + 7; i++) {
		mpfit_ctx->parameters[i] = 0;
		mpfit_ctx->mp_parameters_info[i].fixed = 1;
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

	setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + start, mpfit_ctx->mp_parameters_info + start);

	for (int i = start; i < start + 7; i++) {
		mpfit_ctx->mp_parameters_info[i].fixed = (isFixed || poseIsInvalid);
		mpfit_ctx->mp_parameters_info[i].parname = lh_parameter_names[i - start];

		if (use_jacobian_function != 0 &&
			mpfit_ctx->reprojectModel
				->reprojectAxisAngleFullJacLhPose /*mpfit_ctx->reprojectModel->reprojectFullJacLhPose*/) {
			if (use_jacobian_function < 0) {
				mpfit_ctx->mp_parameters_info[i].side = 2;
				mpfit_ctx->mp_parameters_info[i].deriv_debug = 1;
				mpfit_ctx->mp_parameters_info[i].deriv_abstol = .0001;
				mpfit_ctx->mp_parameters_info[i].deriv_reltol = .0001;
				mpfit_ctx->mp_parameters_info[i].step = .0001;
			} else {
				mpfit_ctx->mp_parameters_info[i].side = 3;
			}
		}
	}
}

void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed,
									 int use_jacobian_function, bool useTruePosition) {
	for (int lh = 0; lh < mpfit_ctx->cameraLength; lh++) {
		const SurvivePose *cam_pos =
			useTruePosition ? survive_get_lighthouse_true_position(ctx, lh) : survive_get_lighthouse_position(ctx, lh);
		if (!quatiszero(cam_pos->Rot))
			survive_optimizer_setup_camera(mpfit_ctx, lh, cam_pos, isFixed, use_jacobian_function);
		else {
			SurvivePose id = {.Rot = {1}};
			survive_optimizer_setup_camera(mpfit_ctx, lh, &id, isFixed, use_jacobian_function);
		}
	}

	survive_optimizer_parameter *bsdParams =
		survive_optimizer_get_start_parameter_info(mpfit_ctx, survive_optimizer_parameter_camera_parameters);
	if (bsdParams) {
		assert(bsdParams->elem_size == mpfit_ctx->cameraLength);
		for (int lh = 0; lh < mpfit_ctx->cameraLength; lh++) {
			BaseStationCal *fcal = (BaseStationCal *)&bsdParams->p[lh * sizeof(BaseStationCal) * 2];
			for (int axis = 0; axis < 2; axis++)
				fcal[axis] = *survive_basestation_cal(ctx, lh, axis);
		}
	}

	size_t start = survive_optimizer_get_calibration_index(mpfit_ctx);
	for (int i = start; i < start + 2 * sizeof(BaseStationCal) / sizeof(FLT) * mpfit_ctx->cameraLength; i++) {
		mpfit_ctx->mp_parameters_info[i].parname = "Fcal parameter";
		mpfit_ctx->mp_parameters_info[i].fixed = true;
	}
}

int survive_optimizer_get_max_parameters_count(const survive_optimizer *ctx) {
	assert(ctx->poseLength < 20);
	int rtn = ctx->cameraLength * 7 + ctx->poseLength * 7 + ctx->ptsLength * 3 +
			  2 * ctx->cameraLength * sizeof(BaseStationCal) / sizeof(FLT);
	if (!ctx->disableVelocity)
		rtn += ctx->poseLength * 6;
	if (ctx->settings->optimize_scale_threshold >= 0) {
		rtn += ctx->poseLength;
	}
	rtn += ctx->poseLength * NUM_GEN2_LIGHTHOUSES * SURVIVE_CORRECTION_PARAMS;
	return rtn;
}
int survive_optimizer_get_parameters_count(const survive_optimizer *ctx) { return ctx->parametersCnt; }
int survive_optimizer_get_free_parameters_count(const survive_optimizer *ctx) {
	int rtn = 0;
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		rtn += ctx->mp_parameters_info[i].fixed == 0;
	}
	return rtn;
}

FLT *survive_optimizer_get_sensors(survive_optimizer *ctx, size_t idx) {
	if (ctx->ptsLength == 0)
		return ctx->sos[idx]->sensor_locations;

	return &ctx->parameters[survive_optimizer_get_sensors_index(ctx)];
}

int survive_optimizer_get_sensors_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_start_index(ctx, survive_optimizer_parameter_obj_points);
}

BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh) {
	int idx = survive_optimizer_get_calibration_index(ctx);
	if (idx < 0)
		return survive_basestation_cal(ctx->sos[0]->ctx, lh, 0);

	BaseStationCal *base = (BaseStationCal *)(&ctx->parameters[idx]);
	return &base[2 * lh];
}

int survive_optimizer_get_calibration_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_start_index(ctx, survive_optimizer_parameter_camera_parameters);
}

SurvivePose *survive_optimizer_get_camera(survive_optimizer *ctx) {
	assert(ctx->cameraLength > 0);
	return (SurvivePose *)&ctx->parameters[survive_optimizer_get_camera_index(ctx)];
}

SURVIVE_EXPORT int survive_optimizer_get_velocity_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_start_index(ctx, survive_optimizer_parameter_object_velocity);
}

SURVIVE_EXPORT int survive_optimizer_get_sensor_scale_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_start_index(ctx, survive_optimizer_parameter_object_scale);
}

SURVIVE_EXPORT void survive_optimizer_disable_sensor_scale(survive_optimizer *ctx) {
	if (ctx->settings->optimize_scale_threshold >= 0) {
		int idx = survive_optimizer_get_sensor_scale_index(ctx);
		for (int i = idx; i < ctx->poseLength + idx; i++) {
			ctx->mp_parameters_info[i].fixed = true;
		}
	}
}

int survive_optimizer_get_camera_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_start_index(ctx, survive_optimizer_parameter_camera);
}

SurvivePose *survive_optimizer_get_pose(survive_optimizer *ctx) {
	if (ctx->poseLength)
		return (SurvivePose *)ctx->parameters;
	return 0;
}

SurviveVelocity *survive_optimizer_get_velocity(survive_optimizer *ctx) {
	if (!ctx->disableVelocity)
		return (SurviveVelocity *)&ctx->parameters[survive_optimizer_get_velocity_index(ctx)];
	return 0;
}

static inline FLT fix_infinity(FLT d) {
	// assert(isfinite(*d));
	// assert(!isnan(d));
	if (isnan(d)) {
		return 0;
	}
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
										const survive_optimizer_measurement *meas, const CnMat *ang_vel_jacb,
										const LinmathDualPose *obj2world, const LinmathDualPose *obj2lh,
										const LinmathDualPose *world2lh, const FLT *pt, FLT *deviates, FLT **derivs) {
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
		FLT correction = get_lighthouse_correction_for(mpfunc_ctx, meas->light.object, meas->light.lh, i);

	    FLT error =  fix_infinity(out[i] - meas[i].light.value - correction);
		deviates[i] = error / meas[i].variance;
        mpfunc_ctx->stats.sensor_error += error * error;
        mpfunc_ctx->stats.sensor_error_cnt++;
	}

	if (derivs) {
        int pose_size = mpfunc_ctx->settings->use_quat_model ? 7 : 6;

		int jac_offset_lh = (lh + mpfunc_ctx->poseLength) * 7;
		int jac_offset_obj = meas->light.object * 7;

		bool needsJacLH = false, needsJacObj = false;
		for(int i = 0;i < 7;i++) {
			needsJacLH |= derivs[jac_offset_lh+i] != 0;
			needsJacObj |= derivs[jac_offset_obj+i] != 0;
		}

		for(int i = 0;i < 2;i++) {
			int p_idx = get_lighthouse_correction_idx_for(mpfunc_ctx, meas->light.object, meas->light.lh, i);
			if (p_idx > -1 && derivs[p_idx]) {
				derivs[p_idx][meas_idx + i] = -1. / meas->variance;
			}
		}

		LinmathDualPose safe_pose = *obj2world, safe_world2lh = *world2lh;
        if(!mpfunc_ctx->settings->use_quat_model) {
            if (magnitude3d(safe_pose.axisAnglePose.AxisAngleRot) == 0)
                safe_pose.axisAnglePose.AxisAngleRot[0] = 1e-10;
            if (magnitude3d(safe_world2lh.axisAnglePose.AxisAngleRot) == 0)
                safe_world2lh.axisAnglePose.AxisAngleRot[0] = 1e-10;
        }

		// d deviate / d Pose(t-)
		// d Pose(t-) / d Pose(t)
		if (needsJacObj) {
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
				for (int k = 0; k < 2; k++) {
					if (isnan(jout[j + k * pose_size])) {
						jout[j + k * pose_size] = 0;
					}
				}
				if(derivs[jac_offset_obj + j]) {
					derivs[jac_offset_obj + j][meas_idx] = fix_infinity(jout[j] / meas[0].variance);
					derivs[jac_offset_obj + j][meas_idx + 1] = fix_infinity(jout[j + pose_size] / meas[1].variance);
				}
			}
		}

		// d deviate / d LH
		if (needsJacLH) {
			FLT out[7 * 2] = {0};
            if(mpfunc_ctx->settings->use_quat_model) {
                reprojectModel->reprojectFullJacLhPose(out, &obj2world->quatPose, pt, &world2lh->quatPose, cal);
            } else {
                reprojectModel->reprojectAxisAngleFullJacLhPose(out, &obj2world->axisAnglePose, pt, &world2lh->axisAnglePose, cal);
            }
			for (int j = 0; j < pose_size; j++) {
				if(derivs[jac_offset_lh + j]) {
					derivs[jac_offset_lh + j][meas_idx] = fix_infinity(out[j] / meas[0].variance);
					derivs[jac_offset_lh + j][meas_idx + 1] = fix_infinity(out[j + pose_size] / meas[1].variance);
				}
			}
		}
	}
}
static void run_single_measurement(survive_optimizer *mpfunc_ctx, size_t meas_idx,
								   const survive_optimizer_measurement *meas, const CnMat *ang_vel_jacb,
								   const LinmathDualPose *obj2world, const LinmathDualPose *obj2lh,
								   const LinmathDualPose *world2lh, const FLT *pt, FLT *deviates, FLT **derivs) {
	const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
	const int lh = meas->light.lh;

	const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
    int pose_size = mpfunc_ctx->settings->use_quat_model ? 7 : 6;

	LinmathPoint3d sensorPtInLH;
    ApplyDualPoseToPoint(mpfunc_ctx, sensorPtInLH, obj2lh, pt);

	FLT out = reprojectModel->reprojectAxisFn[meas->light.axis](cal, sensorPtInLH);
	FLT correction = get_lighthouse_correction_for(mpfunc_ctx, meas->light.object, meas->light.lh, meas->light.axis);
	//SurviveObject * so = mpfunc_ctx->sos[meas->light.object];
	//assert(so->lh_correction[meas->light.lh][meas->light.axis] == correction);
	FLT error = fix_infinity(out - meas->light.value - correction);
	deviates[0] = error / meas->variance;
    mpfunc_ctx->stats.sensor_error += error * error;
    mpfunc_ctx->stats.sensor_error_cnt++;

	if (derivs) {
        int pose_size = mpfunc_ctx->settings->use_quat_model ? 7 : 6;
		int jac_offset_lh = (lh + mpfunc_ctx->poseLength) * 7;
		int jac_offset_obj = meas->light.object * 7;

		bool needsJacLH = false, needsJacObj = false;
		for(int i = 0;i < 7;i++) {
			needsJacLH |= derivs[jac_offset_lh+i] != 0;
			needsJacObj |= derivs[jac_offset_obj+i] != 0;
		}

		int p_idx = get_lighthouse_correction_idx_for(mpfunc_ctx, meas->light.object, meas->light.lh, meas->light.axis);
		if(p_idx > -1 && derivs[p_idx]) {
			derivs[p_idx][meas_idx] = -1. / meas->variance;
		}

		FLT out[7] = {0};
		// d Deviate / d Pose[t - 1] * d Pose[t - 1] / d Pose[t]
		if (needsJacObj) {
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
				if(derivs[jac_offset_obj + j]) {
					derivs[jac_offset_obj + j][meas_idx] = fix_infinity(out[j] / meas->variance);
				}
			}
		}

		if (needsJacLH) {
            if(mpfunc_ctx->settings->use_quat_model) {
                reprojectModel->reprojectAxisJacobLhPoseFn[meas->light.axis](out, &obj2world->quatPose, pt, &world2lh->quatPose,
                                                                                      cal + meas->light.axis);
            } else {
                reprojectModel->reprojectAxisAngleAxisJacobLhPoseFn[meas->light.axis](out, &obj2world->axisAnglePose, pt, &world2lh->axisAnglePose,
                                                                                      cal + meas->light.axis);
            }
			for (int j = 0; j < pose_size; j++) {
				if(derivs[jac_offset_lh + j]) {
					assert(derivs[jac_offset_lh + j]);
					derivs[jac_offset_lh + j][meas_idx] = fix_infinity(out[j] / meas->variance);
				}
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
		if (meas->meas_type == survive_optimizer_measurement_type_light) {
			SV_DATA_LOG("mpfit_meas_val[%d][%d][%d]", &meas->light.value, 1, meas->light.sensor_idx, meas->light.lh,
						meas->light.axis);
			SV_DATA_LOG("mpfit_meas_var[%d][%d][%d]", &meas->variance, 1, meas->light.sensor_idx, meas->light.lh,
						meas->light.axis);
			SV_DATA_LOG("mpfit_meas_err[%d][%d][%d]", deviates, 1, meas->light.sensor_idx, meas->light.lh,
						meas->light.axis);

			avg_dev += fabs(deviates[i] * meas->variance);
			valid_meas++;
		}
	}

	avg_dev = avg_dev / (FLT)valid_meas;
	SV_DATA_LOG("opt_avg_deviates", &avg_dev, 1);
	if (avg_dev < .01) {
		avg_dev = .01;
	}
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		if (meas->meas_type == survive_optimizer_measurement_type_light) {

			FLT chauvenet_criterion =
				linmath_chauvenet_criterion(deviates[i] * meas->variance, 0, avg_dev, optimizer->measurementsCnt);
			if (chauvenet_criterion < .5 && false) {
				meas->invalid = true;
				optimizer->stats.dropped_meas_cnt++;

				SV_VERBOSE(105,
						   "Ignoring noisy data at lh %d sensor %d axis %2d val %f (err: %7.7f/dev: %7.7f/cnt: %d) "
						   "chauv: %7.7f",
						   meas->light.lh, meas->light.sensor_idx, meas->light.axis, meas->light.value,
						   fabs(deviates[i] * meas->variance), avg_dev, (int)optimizer->measurementsCnt,
						   chauvenet_criterion);

				deviates[i] = 0.;
			} else {
				SV_VERBOSE(1000, "Data at lh %d sensor %d axis %d val %f (%7.7f/%7.7f)", meas->light.lh,
						   meas->light.sensor_idx, meas->light.axis, meas->light.value, fabs(deviates[i]), avg_dev);
			}
		}
	}

	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &optimizer->measurements[i];
		if (meas->meas_type == survive_optimizer_measurement_type_light && meas->invalid == false) {
			lh_deviates[meas->light.lh] += fabs(deviates[i] * meas->variance);
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
			FLT chauvenet_criterion = linmath_chauvenet_criterion(lh_deviates[i], 0, lh_avg_dev, obs_lhs);

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
		assert(ctx->measurements[i].size > 0);
		rtn += ctx->measurements[i].size;
	}
	return rtn;
}

static void handle_accel_meas(survive_optimizer *mpfunc_ctx, survive_optimizer_measurement *meas, int meas_idx,
							  FLT *deviates, FLT **derivs) {
	int ang_size = mpfunc_ctx->settings->use_quat_model ? 4 : 3;

	FLT deriv[4 * 3] = {0};
	size_t deriv_idx = 0;

	LinmathPoint3d world_up = {0, 0, 1};

	LinmathDualPose *pose = 0;
	FLT *expected = 0;
	FLT *tx_input = 0;
	if (meas->meas_type == survive_optimizer_measurement_type_object_accel) {
		int obj = meas->pose_acc.object;
		pose = (LinmathDualPose *)(&survive_optimizer_get_pose(mpfunc_ctx)[obj]);
		deriv_idx = obj * 7 + 3;

		tx_input = meas->pose_acc.acc;
		expected = world_up;
	} else {
		size_t lh = meas->camera_acc.camera;
		pose = &((LinmathDualPose *)survive_optimizer_get_camera(mpfunc_ctx))[lh];
		deriv_idx = survive_optimizer_get_camera_index(mpfunc_ctx) + lh * 7 + 3;

		tx_input = world_up;
		expected = meas->camera_acc.acc;
	}

	LinmathPoint3d predicted = {0};
	if (mpfunc_ctx->settings->use_quat_model) {
		assert(false);
	} else {
		gen_axisanglerotatevector(predicted, pose->axisAnglePose.AxisAngleRot, tx_input);
		gen_axisanglerotatevector_jac_axis_angle(deriv, pose->axisAnglePose.AxisAngleRot, tx_input);
	}

	FLT error = 0;
	for (int i = 0; i < 3; i++) {
		FLT e = predicted[i] - expected[i];
		deviates[meas_idx + i] = e / meas->variance;
		error += e;

		for (int j = 0; j < ang_size && derivs; j++) {
			if (derivs[deriv_idx + j]) {
				derivs[deriv_idx + j][meas_idx + i] = fix_infinity(deriv[i * ang_size + j] / meas->variance);
			}
		}
	}

	mpfunc_ctx->stats.object_up_error += error * error;
	mpfunc_ctx->stats.object_up_error_cnt++;
}

static int mpfunc(int m, int n, FLT *p, FLT *deviates, FLT **derivs, void *private) {
	survive_optimizer *mpfunc_ctx = private;

	assert(survive_optimizer_get_meas_size(mpfunc_ctx) == m);
	assert(survive_optimizer_get_parameters_count(mpfunc_ctx) == n);

    mpfunc_ctx->stats.sensor_error = 0; mpfunc_ctx->stats.sensor_error_cnt = 0;
    mpfunc_ctx->stats.object_up_error = 0; mpfunc_ctx->stats.object_up_error_cnt = 0;
    mpfunc_ctx->stats.params_error = 0; mpfunc_ctx->stats.params_error_cnt = 0;

    mpfunc_ctx->parameters = p;

	LinmathDualPose *cameras = (LinmathDualPose*)survive_optimizer_get_camera(mpfunc_ctx);

	int start = survive_optimizer_get_camera_index(mpfunc_ctx);

	int pose_idx = -1;
	FLT calced_timecode = -1;
	LinmathDualPose obj2world = {0};
    LinmathDualPose obj2lh[NUM_GEN2_LIGHTHOUSES] = {0};

	int ang_size = mpfunc_ctx->settings->use_quat_model ? 4 : 3;
	int pose_size = ang_size + 3;
	CN_CREATE_STACK_MAT(ang_velocity_jac, ang_size, ang_size);
	cn_set_diag_val(&ang_velocity_jac, 1);

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
		case survive_optimizer_measurement_type_parameters_bias: {
			int parameter_index = meas->parameter_bias.parameter_index;
			FLT expected_value = meas->parameter_bias.expected_value;
			FLT param_deviation = (p[parameter_index] - expected_value) / (meas->variance + 1e-10);
			deviates[meas_idx] = param_deviation;
			if (derivs && derivs[parameter_index]) {
				derivs[parameter_index][meas_idx] = 1. / (meas->variance + 1e-10);
			}

			mpfunc_ctx->stats.params_error_cnt++;
			mpfunc_ctx->stats.params_error += deviates[meas_idx] * deviates[meas_idx];
			break;
		}
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
			LinmathVec3d xyzjac_scale = {0};
			bool needsScaleJac = false;
			if (mpfunc_ctx->settings->optimize_scale_threshold >= 0  || mpfunc_ctx->settings->lh_scale_correction > 0) {
				scale_idx = survive_optimizer_get_sensor_scale_index(mpfunc_ctx) + meas->light.object;
				FLT scale = (1 + get_lighthouse_correction_for(mpfunc_ctx, meas->light.object, meas->light.lh, 2));
				if(scale_idx >= 0) {
					scale *= p[scale_idx];
				}
				SurvivePose imu2trackref = mpfunc_ctx->sos[meas->light.object]->imu2trackref;
				needsScaleJac = derivs && scale_idx >= 0 && derivs[scale_idx];
				if (needsScaleJac) {
					gen_scale_sensor_pt_jac_scale(xyzjac_scale, pt, &imu2trackref, scale);
				}
				if (scale != 1) {
					gen_scale_sensor_pt(pt, pt, &imu2trackref, scale);
				}
			}

			bool needsNewObj2World = pose_idx != meas->light.object;
			if (calced_timecode != meas->time && !mpfunc_ctx->disableVelocity) {
				needsNewObj2World = true;
			}

			if (needsNewObj2World) {
				calced_timecode = meas->time;
				pose_idx = meas->light.object;
				assert(pose_idx < mpfunc_ctx->poseLength);
				obj2world = *(LinmathDualPose *)(&survive_optimizer_get_pose(mpfunc_ctx)[meas->light.object]);

				if (mpfunc_ctx->disableVelocity == false) {
					LinmathDualPose dPose = obj2world;
					FLT diff = mpfunc_ctx->timecode - meas->time;
					SurviveVelocity *v = survive_optimizer_get_velocity(mpfunc_ctx);

					if (mpfunc_ctx->settings->use_quat_model) {
						addscalednd(dPose.quatPose.Pos, obj2world.quatPose.Pos, v->Pos, -diff, 3);
						survive_apply_ang_velocity(dPose.quatPose.Rot, v->AxisAngleRot, -diff, obj2world.quatPose.Rot);
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

				if (mpfunc_ctx->settings->use_quat_model) {
					quatnormalize(obj2world.quatPose.Rot, obj2world.quatPose.Rot);
				}

				int lh_count = mpfunc_ctx->cameraLength > 0 ? mpfunc_ctx->cameraLength
															: mpfunc_ctx->sos[pose_idx]->ctx->activeLighthouses;

				// Precalc for all known lighthouses. We don't do this for velocity case since the velocity changes
				// obj2world constantly
				if (mpfunc_ctx->disableVelocity) {
					for (int lh = 0; lh < lh_count; lh++) {
						ApplyDualPoseToPose(mpfunc_ctx, &obj2lh[lh], &cameras[lh], &obj2world);
					}
				}
			}

			if (!mpfunc_ctx->disableVelocity) {
				ApplyDualPoseToPose(mpfunc_ctx, &obj2lh[lh], &cameras[lh], &obj2world);
			}

			const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
			if (needsScaleJac) {
				for (int meas_idx_jac = 0; meas_idx_jac < (1 + nextIsPair); meas_idx_jac++) {
					LinmathVec3d ptJac = {0};
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
					derivs[scale_idx][meas_idx + meas_idx_jac] = fix_infinity(dot3d(xyzjac_scale, ptJac));
				}
			}

			if (nextIsPair) {
				run_pair_measurement(mpfunc_ctx, meas_idx, meas, &ang_velocity_jac, &obj2world, &obj2lh[lh], world2lh,
									 pt, deviates + meas_idx, derivs);
				meas_idx++;
				mea_block_idx++;
			} else {
				run_single_measurement(mpfunc_ctx, meas_idx, meas, &ang_velocity_jac, &obj2world, &obj2lh[lh], world2lh,
									   pt, deviates + meas_idx, derivs);
			}

			break;
		}
		case survive_optimizer_measurement_type_camera_accel: {
			handle_accel_meas(mpfunc_ctx, meas, meas_idx, deviates, derivs);
			break;
		}
		case survive_optimizer_measurement_type_camera_position: {
			LinmathVec3d pos; copy3d(pos, meas->camera_pos.pos);
			LinmathVec3d predicted;

			int lh = meas->camera_pos.camera;
			FLT deriv[7 * 3] = { 0 };
			LinmathDualPose *world2lh = (LinmathDualPose *)&cameras[lh];
			gen_apply_axisangle_pose_to_pt(predicted, &world2lh->axisAnglePose, pos);
			gen_apply_axisangle_pose_to_pt_jac_obj_p_axisangle(deriv, &world2lh->axisAnglePose, pos);
			int deriv_idx = survive_optimizer_get_camera_index(mpfunc_ctx) + lh * 7;

			for(int i = 0;i < 3;i++) {
				deviates[meas_idx + i] = predicted[i] / meas->variance;
				for (int j = 0; j < pose_size && derivs; j++) {
					if(derivs[deriv_idx + j]) {
						//printf("%7.7f %d %d\n", predicted[i], meas_idx + i, deriv_idx + j);
						derivs[deriv_idx + j][meas_idx + i] = fix_infinity(deriv[i * pose_size + j] / meas->variance);
					}
				}
			}
		}
			break;
		case survive_optimizer_measurement_type_fixed_rotation: {
			int obj = meas->fixed_rotation.obj;

			LinmathPoint3d predicted = {0, 0, 1};
			FLT deriv[4 * 3] = {0};

			LinmathDualPose *obj2world = (LinmathDualPose *)(&survive_optimizer_get_pose(mpfunc_ctx)[obj]);
			if (mpfunc_ctx->settings->use_quat_model) {
				assert(false);
			} else {
				LinmathVec3d r;
				scale3d(r, obj2world->axisAnglePose.AxisAngleRot, meas->fixed_rotation.conjugate ? -1 : 1);

				gen_axisanglerotatevector(predicted, r, meas->fixed_rotation.match_vec);
				gen_axisanglerotatevector_jac_axis_angle(deriv, r, meas->fixed_rotation.match_vec);
				scalend(deriv, deriv, meas->fixed_rotation.conjugate ? -1 : 1, 9);
			}
			const FLT *pl = meas->fixed_rotation.plane;
			FLT error = dot3d(predicted, pl);
			deviates[meas_idx] = error / meas->variance;

			int deriv_idx = obj * 7 + 3;
			for (int j = 0; j < ang_size && derivs; j++) {
				if (derivs[deriv_idx + j]) {
					derivs[deriv_idx + j][meas_idx] = fix_infinity(
						(pl[0] * deriv[j + 0] + pl[1] * deriv[j + 1 * ang_size] + pl[2] * deriv[j + 2 * ang_size]) /
						meas->variance);
				}
			}

			break;
		}
		case survive_optimizer_measurement_type_object_accel: {
			handle_accel_meas(mpfunc_ctx, meas, meas_idx, deviates, derivs);
			break;
		}

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
/*
	CnMat d = cnVec(m, deviates);
	cn_print_mat(&d);
	CnMat ps = cnVec(n, p);
	cn_print_mat(&ps);
	if(derivs) {
		CN_CREATE_STACK_MAT(J, m, n);
		for (int i = 0; i < m; i++) {
			for (int j = 0; j < n; j++) {
				if (derivs && derivs[j])
					cnMatrixSet(&J, i, j, derivs[j][i]);
				else
					cnMatrixSet(&J, i, j, NAN);
			}
		}
		cn_print_mat(&J);
	}*/

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

		cachedCtx = ctx;
	}
	cachedCfg.iterproc = 0;
	return &cachedCfg;
}

mp_config precise_cfg = {0};
SURVIVE_EXPORT mp_config *survive_optimizer_precise_config() { return &precise_cfg; }

#ifndef NDEBUG
static inline bool sane_covariance(const CnMat *P) {
#ifndef NDEBUG
	for (int i = 0; i < P->rows; i++) {
		if (cnMatrixGet(P, i, i) < 0)
			return false;
	}
#ifdef USE_EIGEN
	return cnDet(P) > -1e-10;
#endif
#endif
	return true;
}
#endif

int survive_optimizer_nonfixed_index(survive_optimizer *ctx, int idx) {
	if (idx < 0 || ctx->mp_parameters_info[idx].fixed)
		return -1;

	int rtn = 0;
	for (int i = 0; i < idx; i++) {
		if (!ctx->mp_parameters_info[i].fixed)
			rtn++;
	}
	return rtn;
}

int meas_cnt_for_obj_lh_axis(survive_optimizer *optimizer, int obj, int lh, int axis) {
	int rtn = 0;
	for(int i = 0;i < optimizer->measurementsCnt;i++) {
		survive_optimizer_measurement * meas = &optimizer->measurements[i];
		if(meas->meas_type == survive_optimizer_measurement_type_light) {
			rtn += meas->light.lh == lh && meas->light.object == obj && meas->light.axis == axis;
		}
	}
	return rtn;
}
static inline int get_axis_count(const size_t *meas_for_lhs_axis, int min_to_count) {
	int num_axis = 0;
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES * 2; i++) {
		if (meas_for_lhs_axis[i] > min_to_count)
			num_axis++;
	}
	return num_axis;
}

int get_meas_for_lhs_axis(survive_optimizer *optimizer, int obj, size_t* meas_for_lhs_axis) {
	size_t mea_cnt = 0;
	for(int i = 0;i < optimizer->measurementsCnt;i++) {
		survive_optimizer_measurement * meas = &optimizer->measurements[i];
		if(meas->invalid == false && meas->meas_type == survive_optimizer_measurement_type_light) {
			mea_cnt++;
			meas_for_lhs_axis[meas->light.lh * 2 + meas->light.axis]++;
		}
	}
	return mea_cnt;
}
void survive_optimizer_remove_invalid_meas(survive_optimizer *optimizer) {
	for (int i = 0; i < optimizer->measurementsCnt; i++) {
		if (optimizer->measurements[i].invalid) {
			optimizer->measurements[i] = optimizer->measurements[optimizer->measurementsCnt - 1];
			optimizer->measurementsCnt--;
			i--;
		}
	}
}
SURVIVE_EXPORT void survive_optimizer_covariance_expand(survive_optimizer *optimizer, const struct CnMat *R_free,
														struct CnMat *R) {
	cn_set_zero(R);
	int idx = 0;
	int *ifree = alloca(sizeof(int) * R_free->rows);
	assert(R_free->rows == survive_optimizer_get_free_parameters_count(optimizer));
	for (int i = 0; i < optimizer->parametersCnt; i++) {
		if (!optimizer->mp_parameters_info[i].fixed) {
			ifree[idx++] = i;
		}
	}
	for (int i = 0; i < R_free->rows; i++) {
		for (int j = 0; j < R_free->rows; j++) {
			cnMatrixSet(R, ifree[i], ifree[j], cnMatrixGet(R_free, i, j));
		}
	}
}
int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result, struct CnMat *R) {
	SurviveContext *ctx = optimizer->sos[0] ? optimizer->sos[0]->ctx : 0;

	mp_config *cfg = optimizer->cfg;
	if (cfg == 0)
		cfg = survive_optimizer_get_cfg(ctx);

	SurvivePose *poses = survive_optimizer_get_pose(optimizer);

	int nonfixed_quat_cnt = 0;
	int *quat_idxs = alloca(sizeof(int) * (optimizer->poseLength + optimizer->cameraLength));
	int *quat_free_idxs = alloca(sizeof(int) * (optimizer->poseLength + optimizer->cameraLength));
	//int *quat_idxs = alloca(sizeof(int) * (optimizer->poseLength + optimizer->cameraLength));
	int fixed_idxs = 0;
	if(!optimizer->settings->use_quat_model) {
		int nfree_til = 0;
        for (int i = 0; i < optimizer->poseLength + optimizer->cameraLength; i++) {
			for(int j = 0;j < 3;j++)
				nfree_til += !optimizer->mp_parameters_info[i * 7 + j].fixed;
            quattoaxisanglemag(poses[i].Rot, poses[i].Rot);
            poses[i].Rot[3] = 0; // NAN;
			if(optimizer->mp_parameters_info[i * 7 + 6].fixed == false) {
				quat_free_idxs[nonfixed_quat_cnt] = nfree_til;
				quat_idxs[nonfixed_quat_cnt++] = i * 7 + 3;
				nfree_til+=4;
			}
			optimizer->mp_parameters_info[i * 7 + 6].fixed = true;
		}
    }

#ifndef NDEBUG
	for (int i = 0; i < survive_optimizer_get_parameters_count(optimizer); i++) {
		SurviveContext *ctx = (optimizer->sos == 0 || optimizer->sos[0] == 0) ? 0 : optimizer->sos[0]->ctx;
		//SV_VERBOSE(100, "Parameter %s(%d) is %d",
		//				 optimizer->mp_parameters_info[i].parname, i,
		//				 optimizer->mp_parameters_info[i].fixed)

		if ((optimizer->mp_parameters_info[i].limited[0] &&
			 optimizer->parameters[i] < optimizer->mp_parameters_info[i].limits[0]) ||
			(optimizer->mp_parameters_info[i].limited[1] &&
			 optimizer->parameters[i] > optimizer->mp_parameters_info[i].limits[1]) ||
			isnan(optimizer->parameters[i])) {

			SV_GENERAL_ERROR("Parameter %s(%d) is invalid. %f <= %f <= %f should be true",
							 optimizer->mp_parameters_info[i].parname, i,
							 optimizer->mp_parameters_info[i].limits[0], optimizer->parameters[i],
							 optimizer->mp_parameters_info[i].limits[1])
			survive_optimizer_serialize(optimizer, "debug.opt");

		}
	}
#endif

	// MPFit runs on temporary storage; so parameters is manipulated in mpfunc. Save it and restore it here.
	FLT *params = optimizer->parameters;
	optimizer->needsFiltering = !optimizer->nofilter && !optimizer->settings->disable_filter;
	FLT* deviates = alloca(survive_optimizer_get_meas_size(optimizer) * sizeof(FLT));
	mpfunc(survive_optimizer_get_meas_size(optimizer), survive_optimizer_get_parameters_count(optimizer), params, deviates, 0, optimizer);

	survive_optimizer_parameter * lh_correction = survive_optimizer_get_start_parameter_info(optimizer, survive_optimizer_parameter_object_lighthouse_correction);
	if(lh_correction && (optimizer->settings->lh_scale_correction > 0 || optimizer->settings->lh_offset_correction > 0)) {
		for (int i = 0; i < optimizer->poseLength; i++) {
			size_t meas_for_lhs_axis[NUM_GEN2_LIGHTHOUSES * 2] = { 0 };
			size_t valid_meas = get_meas_for_lhs_axis(optimizer, i, meas_for_lhs_axis);

			if(valid_meas < 10 || get_axis_count(meas_for_lhs_axis, 3) <= 3)
				continue;

			SurviveObject *so = optimizer->sos[i];

			for (int lh = 0; lh < optimizer->cameraLength; lh++) {
				for (int axis = 0; axis < 3; axis++) {
					int idx = get_lighthouse_correction_idx_for(optimizer, i, lh, axis);
					if(idx > -1 && axis == 2 || meas_for_lhs_axis[lh * SURVIVE_CORRECTION_PARAMS + i] >= 6) {
						optimizer->mp_parameters_info[idx].fixed = false;

						survive_optimizer_measurement *meas = survive_optimizer_emplace_meas(
							optimizer, survive_optimizer_measurement_type_parameters_bias);
						meas->variance = so->lh_correction_variance[lh][axis];
						meas->parameter_bias.parameter_index = idx;
						meas->parameter_bias.expected_value = 0;
					}
					SV_VERBOSE(110, "%s %d %d %d %f (%d, %d) %s %d", survive_colorize_codename(so), i, lh, axis, get_lighthouse_correction_for(optimizer, i, lh, axis),
							   (int)meas_for_lhs_axis[lh * 2 + i], get_axis_count(meas_for_lhs_axis, 0),
							   optimizer->mp_parameters_info[idx].parname, optimizer->mp_parameters_info[idx].fixed);
				}
			}
		}
	}

	survive_optimizer_remove_invalid_meas(optimizer);

	size_t meas_count = survive_optimizer_get_meas_size(optimizer);

	int nfree = survive_optimizer_get_free_parameters_count(optimizer);
	FLT *covar = R ? R->data : 0;
	int covar_size = nfree;
	CN_CREATE_STACK_MAT(R_aa, covar_size * (covar ? 1 : 0), covar_size * (covar ? 1 : 0));
	result->covar_free = covar ? R_aa.data : 0;

	int param_cnt = survive_optimizer_get_parameters_count(optimizer);
	//CN_CREATE_STACK_VEC(PS, survive_optimizer_get_parameters_count(optimizer));
	//result->xerror = PS.data;
	//CN_CREATE_STACK_VEC(MS, survive_optimizer_get_meas_size(optimizer));
	//result->resid = MS.data;
	//CN_CREATE_STACK_MAT(J, nfree, meas_count);
	//result->jac = J.data;

	int rtn = mpfit(mpfunc, meas_count, survive_optimizer_get_parameters_count(optimizer), optimizer->parameters,
					optimizer->mp_parameters_info, cfg, optimizer, result);
	optimizer->parameters = params;

	FLT rchisqr = linmath_max(1, result->bestnorm / nfree);
	if (ctx)
		survive_recording_write_matrix(ctx->recptr, optimizer->sos[0], 10, "full_cov", &R_aa);
	assert(sane_covariance(&R_aa));
	int totalPoseCount = optimizer->poseLength + optimizer->cameraLength;
    int pose_size = optimizer->settings->use_quat_model ? 7 : 6;

    int totalFreePoseCount = nfree / pose_size;
	if (covar) {
		*R = cnMat(R_aa.rows + nonfixed_quat_cnt, R_aa.rows + nonfixed_quat_cnt, R->data);

		CnMat R_q = cnMat(R->rows, R->cols, covar);
		if (optimizer->settings->optimize_scale_threshold >= 0) {
			int idx = survive_optimizer_get_sensor_scale_index(optimizer);
			int free_idx = survive_optimizer_nonfixed_index(optimizer, idx);
			if (free_idx >= 0) {
				for (int z = 0; z < 1; z++) {
					FLT scale_cov = cnMatrixGet(&R_aa, free_idx + z, free_idx + z);
					if (scale_cov > 0) {
						FLT scale = params[idx + z];

						FLT y = scale - optimizer->sos[z]->sensor_scale;
						FLT k = optimizer->sos[z]->sensor_scale_var / (optimizer->sos[z]->sensor_scale_var + scale_cov);
						optimizer->sos[z]->sensor_scale += k * y;
						optimizer->sos[z]->sensor_scale_var *= (1 - k);
					}
				}
			}
		}
		if(optimizer->settings->use_quat_model) {
			cnCopy(&R_aa, &R_q, 0);
		} else {
			int idx = survive_optimizer_nonfixed_index(optimizer, survive_optimizer_get_start_index(optimizer, survive_optimizer_parameter_object_lighthouse_correction));
			CN_CREATE_STACK_MAT(G, R->rows, R_aa.rows);
			CN_CREATE_STACK_MAT(G2, R->rows, R_aa.rows);
			CN_CREATE_STACK_MAT(Gp, 4, 3);
			assert(R->rows == R->cols);

			int cidx = 0;
			for (int z = 0; z < nonfixed_quat_cnt;z++) {
				int idx = quat_free_idxs[z];
				for(;cidx < idx;cidx++)
					cnMatrixSet(&G, cidx, cidx - z, 1);

				gen_axisangle2quat_jac_axis_angle(Gp.data, params + idx);
				for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 3; j++) {
						cnMatrixSet(&G, i + cidx, j + cidx - z, cnMatrixGet(&Gp, i, j));
					}
				}
				cidx += 4;
			}
			for(;cidx < R->rows;cidx++) cnMatrixSet(&G, cidx, cidx - nonfixed_quat_cnt, 1);

			gemm_ABAt_add_scaled(&R_q, &G, &R_aa, 0, 1, 1, 0);

			assert(sane_covariance(&R_q));
        }

		// https://lmfit.github.io/lmfit-py/fitting.html#uncertainties-in-variable-parameters-and-their-correlations

		if(!optimizer->dontScaleCov) {
			cn_multiply_scalar(&R_q, &R_q, rchisqr);
		}
	}
	bool wasSuccess = result->status > 0;

	if(lh_correction != 0 && wasSuccess) {
		for (int i = 0; i < optimizer->poseLength; i++) {
			SurviveObject *so = optimizer->sos[i];
			struct mp_par_struct *info = lh_correction[i].pi;
			bool changed = false;

			for (int lh = 0; lh < optimizer->cameraLength; lh++) {
				for (int axis = 0; axis < SURVIVE_CORRECTION_PARAMS; axis++) {

					int cov_idx = survive_optimizer_nonfixed_index(optimizer, get_lighthouse_correction_idx_for(optimizer, i, lh, axis));

					if(!info[lh * SURVIVE_CORRECTION_PARAMS + axis].fixed) {
						changed = true;
						FLT v = rchisqr * cnMatrixGet(&R_aa, cov_idx, cov_idx);

						FLT y = get_lighthouse_correction_for(optimizer, i, lh, axis) - so->lh_correction[lh][axis];
						FLT k = so->lh_correction_variance[lh][axis] / (so->lh_correction_variance[lh][axis] + v);
						so->lh_correction[lh][axis] += k * y;
						so->lh_correction_variance[lh][axis] *= (1 - k);
						so->lh_correction_variance[lh][axis] += (axis == 2 ? optimizer->settings->lh_scale_correction : optimizer->settings->lh_offset_correction);
						SV_VERBOSE(110, "%s %d %d %d %+7.16f", survive_colorize_codename(so), i, lh, axis, get_lighthouse_correction_for(optimizer, i, lh, axis));
					}
				}
			}
			if(changed) {
				CnMat v = cnMat(2, SURVIVE_CORRECTION_PARAMS, (FLT *)so->lh_correction);
				survive_recording_write_matrix(ctx->recptr, so, 5, "lhc", &v);
			}
		}
	}

    if(!optimizer->settings->use_quat_model) {
        for (int i = 0; i < totalPoseCount; i++) {
            quatfromaxisangle(poses[i].Rot, poses[i].Rot, norm3d(poses[i].Rot));
			optimizer->mp_parameters_info[i * 7 + 6].fixed = optimizer->mp_parameters_info[i * 7 + 5].fixed;
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

	if(opt->sos && opt->sos[0])
		fprintf(f, "object       %s\n", opt->sos[0]->codename);

	fprintf(f, "model        %d\n", opt->reprojectModel != &survive_reproject_gen1_model);
	fprintf(f, "poseLength   %d\n", opt->poseLength);
	fprintf(f, "cameraLength %d\n", opt->cameraLength);
	fprintf(f, "ptsLength    %d\n", opt->ptsLength);

	fprintf(f, "\n");
	fprintf(f, "parameters   %d\n", survive_optimizer_get_parameters_count(opt));
	fprintf(f, "#	          <name>:        <idx>      <fixed>             <value>            <min>            <max> "
			   "<use_jacobian>\n");
	for (int i = 0; i < survive_optimizer_get_parameters_count(opt); i++) {
		struct mp_par_struct *info = &opt->mp_parameters_info[i];
		fprintf(f, "\t%16s:", opt->mp_parameters_info[i].parname);
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
		struct mp_par_struct *info = &opt->mp_parameters_info[i];
		read_count = fscanf(f, "\t");

		char *b = calloc(128, 1);
		opt->mp_parameters_info[i].parname = b;
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
		if (!ctx->mp_parameters_info[i].fixed)
			rtn++;
	}
	return rtn;
}
SURVIVE_EXPORT void survive_optimizer_get_nonfixed(const survive_optimizer *ctx, FLT *params) {
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		if (!ctx->mp_parameters_info[i].fixed)
			*(params++) = ctx->parameters[i];
	}
}

SURVIVE_EXPORT void survive_optimizer_set_nonfixed(survive_optimizer *ctx, FLT *params) {
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		if (!ctx->mp_parameters_info[i].fixed)
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

SURVIVE_EXPORT void survive_optimizer_setup_buffers(survive_optimizer *ctx, FLT *parameter_buffer,
													survive_optimizer_parameter *parameter_info_buffer,
													struct mp_par_struct *mp_parameter_info_buffer,
													void *measurements_buffer, void *so_buffer) {
	size_t par_count = survive_optimizer_get_max_parameters_count(ctx);
	size_t meas_count = survive_optimizer_get_max_measurements_count(ctx);

	ctx->parameters = (FLT *)parameter_buffer;

	for (int i = 0; i < par_count; i++) {
		ctx->parameters[i] = NAN;
	}
	ctx->mp_parameters_info = mp_parameter_info_buffer;
	ctx->parameters_info = parameter_info_buffer;
	memset(ctx->parameters_info, 0, par_count * sizeof(survive_optimizer_parameter));

	ctx->sos = so_buffer;

	ctx->measurements = (survive_optimizer_measurement *)(measurements_buffer);
	size_t measurementAllocSize = meas_count * sizeof(survive_optimizer_measurement);
	memset(ctx->measurements, 0, measurementAllocSize);
	// ctx->obj_up_vectors = (LinmathPoint3d *)((uint8_t *)measurements_buffer + measurementAllocSize);
	// ctx->cam_up_vectors = ctx->obj_up_vectors + ctx->poseLength;
	memset(ctx->mp_parameters_info, 0, sizeof(mp_par) * par_count);
	for (int i = 0; i < survive_optimizer_get_parameters_count(ctx); i++) {
		ctx->mp_parameters_info[i].fixed = 1;
	}

	survive_optimizer_emplace_params(ctx, survive_optimizer_parameter_object_pose, ctx->poseLength);
	survive_optimizer_emplace_params(ctx, survive_optimizer_parameter_camera, ctx->cameraLength);
	if (!ctx->disableVelocity) {
		survive_optimizer_emplace_params(ctx, survive_optimizer_parameter_object_velocity, ctx->poseLength);
	}
	if (ctx->settings->optimize_scale_threshold >= 0) {
		survive_optimizer_emplace_params(ctx, survive_optimizer_parameter_object_scale, ctx->poseLength);
	}

	if(true) {
		//survive_optimizer_emplace_params(ctx, survive_optimizer_parameter_object_lighthouse_correction, ctx->poseLength);
	}

	if (ctx->settings->current_pos_bias > 0) {
		for (int i = 0; i < ctx->poseLength; i++) {
			if (!quatiszero(ctx->sos[i]->OutPoseIMU.Rot)) {

				LinmathDualPose pose = { .quatPose = ctx->sos[i]->OutPoseIMU };
				CnMat x = cnVec(7,  (FLT*)&pose);
				CN_CREATE_STACK_VEC(P_q, 7);
				CN_CREATE_STACK_VEC(P_aa, 6);
				cnkalman_extrapolate_state(ctx->timecode, &ctx->sos[i]->tracker->model, &x, &P_q);

				if (!ctx->settings->use_quat_model) {
					survive_covariance_pose2poseAA(&P_aa, &pose.quatPose, &P_q);
					quattoaxisanglemag(pose.axisAnglePose.AxisAngleRot, pose.quatPose.Rot);
				}

				assert(i == 0);
				for(int z = 0; z < (ctx->settings->use_quat_model ? 7 : 6); z++) {
					survive_optimizer_measurement *meas =
						survive_optimizer_emplace_meas(ctx, survive_optimizer_measurement_type_parameters_bias);
					meas->parameter_bias.expected_value = ((FLT*)&pose.quatPose.Pos)[z];
					meas->parameter_bias.parameter_index = z;
					meas->variance = sqrt(_P_q[z]) * ctx->settings->current_pos_bias;
				}
			}
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
                    survive_kalman_tracker_correct_imu(ctx->sos[i]->tracker, meas->pose_acc.acc, ctx->sos[i]->activations.accel);
                    normalize3d(meas->pose_acc.acc, meas->pose_acc.acc);
                    meas->variance = ctx->objectUpVectorVariance;
                }
            }
		}
	}
}

SURVIVE_EXPORT void *survive_optimizer_realloc(void *old_ptr, size_t size) { return realloc(old_ptr, size); }

int survive_optimizer_get_max_measurements_count(const survive_optimizer *ctx) {
	int sensor_cnt = SENSORS_PER_OBJECT;
	assert(ctx->poseLength > 0);
	return ctx->poseLength * 2 * sensor_cnt * NUM_GEN2_LIGHTHOUSES +
		   (ctx->settings->current_pos_bias <= 0 ? 0 : ctx->poseLength) + (ctx->poseLength + ctx->cameraLength) +
		   survive_optimizer_get_max_parameters_count(ctx)
		;
}

int meas_size(survive_optimizer *ctx, enum survive_optimizer_measurement_type type) {
	switch (type) {
	case survive_optimizer_measurement_type_light:
		return 1;
	case survive_optimizer_measurement_type_fixed_rotation:
		return 1;
	case survive_optimizer_measurement_type_camera_accel:
	case survive_optimizer_measurement_type_object_accel:
		return 3;
	case survive_optimizer_measurement_type_camera_position:
		return 3;
	case survive_optimizer_measurement_type_parameters_bias:
		return 1;
	default:
		assert(false);
	}
	return 0;
}

int params_size(survive_optimizer *ctx, enum survive_optimizer_parameter_type type) {
	switch (type) {
	case survive_optimizer_parameter_object_pose:
		return 7;
	case survive_optimizer_parameter_object_velocity:
		return 6;
	case survive_optimizer_parameter_camera:
		return 7;
	case survive_optimizer_parameter_camera_parameters:
		return sizeof(BaseStationCal) * 2 / sizeof(FLT);
	case survive_optimizer_parameter_obj_points:
		return 3;
	case survive_optimizer_parameter_object_scale:
		return 1;
	case survive_optimizer_parameter_object_lighthouse_correction:
		return NUM_GEN2_LIGHTHOUSES * SURVIVE_CORRECTION_PARAMS;
	default:
		assert(false);
	}
	return 0;
}
static inline const char* params_name(enum survive_optimizer_parameter_type type) {
	switch(type) {
	default: return "Unknown";
	case survive_optimizer_parameter_none: return "None";
	case survive_optimizer_parameter_object_pose: return "Obj Pose";
	case survive_optimizer_parameter_object_velocity: return "Velocity";
	case survive_optimizer_parameter_object_scale: return "Scale";
	case survive_optimizer_parameter_camera: return "Camera";
	case survive_optimizer_parameter_camera_parameters: return "Camera cal";
	case survive_optimizer_parameter_obj_points: return "Points";
	case survive_optimizer_parameter_object_lighthouse_correction: return "LH correction";
	}
}
SURVIVE_EXPORT survive_optimizer_parameter *
survive_optimizer_emplace_params(survive_optimizer *ctx, enum survive_optimizer_parameter_type type, int n) {
	assert(survive_optimizer_get_max_parameters_count(ctx) > ctx->parameterBlockCnt);
	assert(survive_optimizer_get_start_index(ctx, type) == -1);
	survive_optimizer_parameter *rtn = &ctx->parameters_info[ctx->parameterBlockCnt++];
	rtn->param_type = type;
	rtn->size = params_size(ctx, type) * n;
	rtn->elem_size = n;
	rtn->pi = &ctx->mp_parameters_info[ctx->parametersCnt];
	for(int i = 0;i < rtn->size;i++) {
		rtn->pi[i].fixed = true;
		rtn->pi[i].parname = params_name(type);
	}
	rtn->p_idx = ctx->parametersCnt;
	rtn->p = &ctx->parameters[ctx->parametersCnt];
	ctx->parametersCnt += rtn->size;

	return rtn;
}
survive_optimizer_measurement *survive_optimizer_emplace_meas(survive_optimizer *ctx,
															  enum survive_optimizer_measurement_type type) {
	assert(survive_optimizer_get_max_measurements_count(ctx) > ctx->measurementsCnt);
	survive_optimizer_measurement *rtn = &ctx->measurements[ctx->measurementsCnt++];
	rtn->meas_type = type;
	rtn->size = meas_size(ctx, type);
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

	if (variance > 0) {
		survive_optimizer_measurement *meas =
			survive_optimizer_emplace_meas(ctx, survive_optimizer_measurement_type_camera_accel);
		meas->camera_acc.camera = i;
		normalize3d(meas->camera_acc.acc, up);
		assert(isfinite(meas->camera_acc.acc[0]));
		meas->variance = variance;
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
