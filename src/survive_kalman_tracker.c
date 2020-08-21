#include "survive_kalman_tracker.h"
#include "linmath.h"
#include "math.h"
#include "survive_internal.h"
#include "survive_kalman.h"
#include "survive_kalman_tracker.h"
#include <assert.h>
#include <malloc.h>
#include <memory.h>
#include <minimal_opencv.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#include "generated/survive_imu.generated.h"

#define SURVIVE_MODEL_STATE_CNT (sizeof(SurviveKalmanModel) / sizeof(FLT))

static FLT survive_kalman_tracker_position_var2(SurviveKalmanTracker *tracker, FLT *var_diag) {
	FLT _var_diag[SURVIVE_MODEL_STATE_CNT];
	if (var_diag == 0)
		var_diag = _var_diag;

	for (int i = 0; i < SURVIVE_MODEL_STATE_CNT; i++) {
		var_diag[i] = fabs(tracker->model.P[SURVIVE_MODEL_STATE_CNT * i + i]);
	}

	return normnd2(var_diag, 7);
}

static void normalize_model(SurviveKalmanTracker *pTracker) {
	quatnormalize(pTracker->state.Pose.Rot, pTracker->state.Pose.Rot);
}

static inline void mat_eye_diag(CvMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			CV_FLT_PTR(m)[j * m->cols + i] = i == j ? (v ? v[i] : 1.) : 0.;
		}
	}
}

static inline void arr_eye_diag(FLT *m, int rows, int cols, const FLT *v) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			(m)[j * cols + i] = i == j ? (v ? v[i] : 1.) : 0.;
		}
	}
}

struct map_light_data_ctx {
	PoserDataLight *pdl;
	SurviveKalmanTracker *tracker;
};

/**
 * This function reuses the reproject functions to estimate what it thinks the lightcap angle should be based on x_t,
 * and uses that measurement to compare from the actual observed angle. These functions have jacobian functions that
 * correspond to them; see @survive_reproject.c and @survive_reproject_gen2.c
 */
static bool map_light_data(void *user, const struct CvMat *Z, const struct CvMat *x_t, struct CvMat *y,
						   struct CvMat *H_k) {
	struct map_light_data_ctx *cbctx = (struct map_light_data_ctx *)user;
	const struct PoserDataLight *pdl = cbctx->pdl;
	const SurviveKalmanTracker *tracker = cbctx->tracker;

	SurviveObject *so = tracker->so;
	struct SurviveContext *ctx = tracker->so->ctx;
	const survive_reproject_model_t *mdl =
		tracker->so->ctx->lh_version == 0 ? &survive_reproject_model : &survive_reproject_gen2_model;

	int axis = 0;
	switch (pdl->hdr.pt) {
	case POSERDATA_LIGHT:
		axis = (((PoserDataLightGen1 *)pdl)->acode & 1);
		break;
	case POSERDATA_LIGHT_GEN2:
		axis = ((PoserDataLightGen2 *)pdl)->plane;
		break;
	default:
		assert(0);
	}

	survive_reproject_full_xy_fn_t project_fn = mdl->reprojectAxisFullFn[axis];
	survive_reproject_axis_jacob_fn_t project_jacob_fn = mdl->reprojectAxisJacobFn[axis];
	assert(ctx->bsd[pdl->lh].PositionSet);

	const SurvivePose world2lh = InvertPoseRtn(&ctx->bsd[pdl->lh].Pose);
	const SurvivePose obj2world = *(SurvivePose *)CV_FLT_PTR(x_t);

	const FLT *ptInObj = &so->sensor_locations[pdl->sensor_id * 3];
	FLT h_x = project_fn(&obj2world, ptInObj, &world2lh, &ctx->bsd[pdl->lh].fcal[axis]);
	CV_FLT_PTR(y)[0] = CV_FLT_PTR(Z)[0] - h_x;

	memset(CV_FLT_PTR(H_k), 0, sizeof(FLT) * H_k->cols * H_k->rows);

	project_jacob_fn(CV_FLT_PTR(H_k), &obj2world, ptInObj, &world2lh, &ctx->bsd[pdl->lh].fcal[axis]);
	for (int i = 0; i < 7; i++) {
		if (!isfinite(CV_FLT_PTR(H_k)[i])) {
			return false;
		}
	}

	return true;
}

void survive_kalman_tracker_integrate_light(SurviveKalmanTracker *tracker, PoserDataLight *data) {
	SurviveContext *ctx = tracker->so->ctx;

	if (tracker->use_raw_obs) {
		return;
	}

	// A single light cap measurement has an infinite amount of solutions along a plane; so it only helps if we are
	// already in a good place
	if (tracker->light_threshold_var > 0 &&
		survive_kalman_tracker_position_var2(tracker, 0) > tracker->light_threshold_var) {
		return;
	}

	if (tracker->light_required_obs > tracker->stats.obs_count) {
		return;
	}

	if (!ctx->bsd[data->lh].PositionSet) {
		return;
	}

	FLT time = data->hdr.timecode / (FLT)tracker->so->timebase_hz;
	FLT delta = time - tracker->model.t;
	tracker->last_light_time = time;

	if (tracker->light_var >= 0) {
		CvMat Z = cvMat(1, 1, CV_FLT, &data->angle);
		struct map_light_data_ctx cbctx = {
			.tracker = tracker,
			.pdl = data,
		};

		FLT rtn = survive_kalman_predict_update_state_extended(time, &tracker->model, &Z, &tracker->light_var,
															   map_light_data, &cbctx, tracker->adaptive_lightcap);

		tracker->stats.lightcap_total_error += rtn;

		tracker->light_residuals_all *= .9;
		tracker->light_residuals_all += .1 * rtn;

		tracker->light_residuals[data->lh] *= .9;
		tracker->light_residuals[data->lh] += .1 * rtn;

		tracker->stats.lightcap_error_by_lh[data->lh] += rtn;
		tracker->stats.lightcap_count_by_lh[data->lh]++;

		if (tracker->light_residuals[data->lh] > .1) {
			// SV_WARN("Light residual for lh%d is too high -- %f", data->lh, tracker->light_residuals[data->lh]);
			survive_lighthouse_adjust_confidence(ctx, data->lh, -.1);
		}
		tracker->stats.lightcap_count++;

		normalize_model(tracker);
		// survive_kalman_tracker_report_state(&data->hdr, tracker);
	}
	SV_VERBOSE(400, "Resultant state %f (%f) (lightcap %2d) (error %e, %e)  " Point16_format, time, delta, data->lh,
			   tracker->light_residuals[data->lh], tracker->light_residuals_all,
			   LINMATH_VEC16_EXPAND(tracker->model.state));
}

struct map_imu_data_ctx {
	bool use_gyro, use_accel;
	SurviveKalmanTracker *tracker;
};

/**
 * The prediction for IMU given x_t is:
 *
 * [Position, Rotation, Velocity, Ang_Velocity, Acc, Gyro_Bias] = x_t
 *
 * acc_predict  = Rotation^-1 * (Acc/9.80665 + [0, 0, 1])
 * gyro_predict = Rotation^-1 * Ang_Velocity + Gyro_Bias
 *
 * The actual code for this is generated from tools/generate_math_functions/imu_functions.py. It isn't done in
 * C natively to allow for the jacobian code to be generated using symengine
 */
static bool map_imu_data(void *user, const struct CvMat *Z, const struct CvMat *x_t, struct CvMat *y,
						 struct CvMat *H_k) {
	struct map_imu_data_ctx *fn_ctx = user;
	FLT h_x[6];

	for (int i = 0; i < H_k->rows * H_k->cols; i++)
		CV_FLT_PTR(H_k)[i] = NAN;

	SurviveContext *ctx = fn_ctx->tracker->so->ctx;

	SV_VERBOSE(400, "X     " Point16_format, LINMATH_VEC16_EXPAND(CV_FLT_PTR(x_t)))
	SV_VERBOSE(400, "Z     " Point6_format, LINMATH_VEC6_EXPAND(CV_FLT_PTR(Z)))

	SurviveKalmanModel *s = (SurviveKalmanModel *)CV_FLT_PTR(x_t);
	gen_imu_predict(h_x, s);
	assert(H_k->rows * H_k->cols == SURVIVE_MODEL_STATE_CNT * 6);
	gen_imu_predict_jac_kalman_model(CV_FLT_PTR(H_k), s);

	SV_VERBOSE(400, "h_x   " Point6_format, LINMATH_VEC6_EXPAND(h_x))
	subnd(CV_FLT_PTR(y), CV_FLT_PTR(Z), h_x, Z->rows);
	SV_VERBOSE(400, "y     " Point6_format, LINMATH_VEC6_EXPAND(CV_FLT_PTR(y)))
	return true;
}

void survive_kalman_tracker_integrate_imu(SurviveKalmanTracker *tracker, PoserDataIMU *data) {
	SurviveContext *ctx = tracker->so->ctx;

	if (tracker->use_raw_obs) {
		return;
	}

	// Wait til observation is in before reading IMU; gets rid of bad IMU data at the start
	if (tracker->model.t == 0) {
		return;
	}

	if (tracker->stats.obs_count < 16) {
		return;
	}

	FLT time = data->hdr.timecode / (FLT)tracker->so->timebase_hz;
	FLT time_diff = time - tracker->model.t;

	if (time_diff < -.01) {
		// SV_WARN("Processing imu data from the past %fs", time - tracker->rot.t);
		tracker->stats.late_imu_dropped++;
		return;
	}

	if (time_diff > 0.5) {
		SV_WARN("%s is probably dropping IMU packets; %f time reported between %" PRIu64, tracker->so->codename,
				time_diff, data->hdr.timecode);
	}

	FLT rotation_variance[] = {1e5, 1e5, 1e5, 1e5, 1e5, 1e5};

	if (time - tracker->last_light_time > .1) {
		FLT _H[] = {
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		};
		CvMat H = cvMat(9, tracker->model.state_cnt, SURVIVE_CV_F, _H);
		FLT v = 1e-5;
		FLT R[] = {
			1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5,
		};
		CREATE_STACK_MAT(Z, 9, 1);
		memset(_Z, 0, sizeof(FLT) * 9);

		tracker->stats.imu_total_error += survive_kalman_predict_update_state(time, &tracker->model, &Z, &H, R, false);
	}

	struct map_imu_data_ctx fn_ctx = {.tracker = tracker};
	if (tracker->acc_var >= 0) {
		fn_ctx.use_accel = true;
		for (int i = 0; i < 3; i++)
			rotation_variance[i] = tracker->acc_var;
	}

	if (tracker->gyro_var >= 0) {
		fn_ctx.use_gyro = true;
		for (int i = 0; i < 3; i++)
			rotation_variance[3 + i] = tracker->gyro_var;
	}

	if (fn_ctx.use_gyro || fn_ctx.use_accel) {
		FLT *R = rotation_variance;
		int rows = 6;
		int offset = 0;
		CvMat Z = cvMat(rows, 1, CV_FLT, data->accel + offset);

		SV_VERBOSE(400, "Integrating IMU " Point6_format " with cov " Point6_format,
				   LINMATH_VEC6_EXPAND((FLT *)&data->accel[0]), LINMATH_VEC6_EXPAND(R));

		FLT err = survive_kalman_predict_update_state_extended(time, &tracker->model, &Z, R, map_imu_data, &fn_ctx,
															   tracker->adaptive_imu);

		tracker->stats.imu_total_error += err;
		tracker->imu_residuals *= .9;
		tracker->imu_residuals += .1 * err;

		tracker->stats.imu_count++;
		SV_VERBOSE(400, "Resultant state %f (imu %e) " Point19_format, time, tracker->imu_residuals,
				   LINMATH_VEC19_EXPAND(tracker->model.state));
		normalize_model(tracker);
	}

	survive_kalman_tracker_report_state(&data->hdr, tracker);
}

void survive_kalman_tracker_predict(const SurviveKalmanTracker *tracker, FLT t, SurvivePose *out) {
	// if (tracker->model.info.P[0] > 100 || tracker->model.info.P[0] > 100 || tracker->model.t == 0)
	//	return;

	if (tracker->model.t == 0)
		return;

	survive_kalman_predict_state(t, &tracker->model, 0, 7, out->Pos);
	quatnormalize(out->Rot, out->Rot);

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(300, "Predict pose %f %f " SurvivePose_format, t, t - tracker->model.t, SURVIVE_POSE_EXPAND(*out))
}

static void model_q_fn(void *user, FLT t, const CvMat *x, FLT *q_out) {
	SurviveKalmanTracker *tracker = (SurviveKalmanTracker *)user;
	SurviveKalmanModel *state = (SurviveKalmanModel *)CV_FLT_PTR(x);
	/*
	 * Due to the rotational terms in the model, the process noise covariance is complicated. It mixes a XYZ third order
	 * positional model with a second order rotational model with tuning parameters
	 */

	FLT t2 = t * t;
	FLT t3 = t * t * t;
	FLT t4 = t2 * t2;
	FLT t5 = t3 * t2;

	/* ================== Positional ============================== */
	// Estimation with Applications to Tracking and Navigation: Theory Algorithms and Software Ch 6
	// http://wiki.dmdevelopment.ru/wiki/Download/Books/Digitalimageprocessing/%D0%9D%D0%BE%D0%B2%D0%B0%D1%8F%20%D0%BF%D0%BE%D0%B4%D0%B1%D0%BE%D1%80%D0%BA%D0%B0%20%D0%BA%D0%BD%D0%B8%D0%B3%20%D0%BF%D0%BE%20%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9%20%D0%BE%D0%B1%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D0%BA%D0%B5%20%D1%81%D0%B8%D0%B3%D0%BD%D0%B0%D0%BB%D0%BE%D0%B2/Estimation%20with%20Applications%20to%20Tracking%20and%20Navigation/booktext@id89013302placeboie.pdf

	// We mix three order models here based on tuning variables.
	FLT Q_acc[] = {t5 / 20., t4 / 8., t3 / 6., t3 / 3., t2 / 2., t};

	FLT Q_vel[] = {t3 / 3., t2 / 2., t};

	FLT q_p = tracker->process_weight_acc;
	FLT p_p = q_p * Q_acc[0] + tracker->process_weight_vel * Q_vel[0] + tracker->process_weight_pos * t;
	FLT p_v = q_p * Q_acc[1] + tracker->process_weight_vel * Q_vel[1];
	FLT p_a = q_p * Q_acc[2];
	FLT v_v = q_p * Q_acc[3] + tracker->process_weight_vel * Q_vel[2];
	FLT v_a = q_p * Q_acc[4];
	FLT a_a = q_p * Q_acc[5];


	/* ================== Rotational ==============================
	 * 	https://www.ucalgary.ca/engo_webdocs/GL/96.20096.JSchleppe.pdf
	 *      !!! NOTE: This document uses x,y,z,w quaternions !!!
	  This is a rework using the same methodology. Some helper output functions are in the tools/generate_math_functions
	  code.
	 */
	FLT s_w = tracker->process_weight_ang_velocity;
	FLT s_f = s_w / 12. * t3;
	FLT s_s = s_w / 4. * t2;
	FLT qw = state->Pose.Rot[0], qx = state->Pose.Rot[1], qy = state->Pose.Rot[2], qz = state->Pose.Rot[3];
	FLT qws = qw * qw, qxs = qx * qx, qys = qy * qy, qzs = qz * qz;
	FLT qs = qws + qxs + qys + qzs;

	FLT rv = tracker->process_weight_rotation * t;

	/* The gyro bias is expected to change, but slowly through time */
	FLT gb = 1e-10 * t;


	// This is the best way I could think to write the final block matrix...
	// clang-format off
	FLT Q[] = {
	//	      x        y        z                 qw                 qx                 qy                 qz         vx       vy       vz          avx      avy      avz       ax       ay       az        bx, by, bz,
		    p_p,       0,       0,                 0,                 0,                 0,                 0,       p_v,       0,       0,           0,       0,       0,     p_a,       0,       0,        0,  0,  0, // x
		      0,     p_p,       0,                 0,                 0,                 0,                 0,         0,     p_v,       0,           0,       0,       0,       0,     p_a,       0,        0,  0,  0, // y
		      0,       0,     p_p,                 0,                 0,                 0,                 0,         0,       0,     p_v,           0,       0,       0,       0,       0,     p_a,        0,  0,  0, // z

		      0,       0,       0,   rv+s_f*(qs-qws),      s_f*(-qw*qx),      s_f*(-qw*qy),      s_f*(-qw*qz),         0,       0,       0,     -s_s*qx, -s_s*qy, -s_s*qz,       0,       0,       0,        0,  0,  0, // qw
		      0,       0,       0,      s_f*(-qw*qx),   rv+s_f*(qs-qxs),      s_f*(-qx*qy),      s_f*(-qx*qz),         0,       0,       0,      s_s*qw, -s_s*qz,  s_s*qy,       0,       0,       0,        0,  0,  0, // qx
		      0,       0,       0,      s_f*(-qw*qy),      s_f*(-qx*qy),   rv+s_f*(qs-qys),      s_f*(-qy*qz),         0,       0,       0,      s_s*qz,  s_s*qw, -s_s*qx,       0,       0,       0,        0,  0,  0, // qy
		      0,       0,       0,      s_f*(-qw*qz),      s_f*(-qx*qz),      s_f*(-qy*qz),   rv+s_f*(qs-qzs),         0,       0,       0,     -s_s*qy,  s_s*qx,  s_s*qw,       0,       0,       0,        0,  0,  0, // qz


		      p_v,     0,       0,                 0,                 0,                 0,                 0,       v_v,       0,       0,           0,       0,       0,     v_a,       0,       0,        0,  0,  0, // vx
		      0,     p_v,       0,                 0,                 0,                 0,                 0,         0,     v_v,       0,           0,       0,       0,       0,     v_a,       0,        0,  0,  0, // vy
		      0,       0,     p_v,                 0,                 0,                 0,                 0,         0,       0,     v_v,           0,       0,       0,       0,       0,     v_a,        0,  0,  0, // vz

		      0,       0,       0,           -s_s*qx,            s_s*qw,            s_s*qz,           -s_s*qy,         0,       0,       0,     s_w * t,       0,       0,       0,       0,       0,        0,  0,  0, // avx
		      0,       0,       0,           -s_s*qy,           -s_s*qz,            s_s*qw,            s_s*qx,         0,       0,       0,           0, s_w * t,       0,       0,       0,       0,        0,  0,  0, // avy
		      0,       0,       0,           -s_s*qz,            s_s*qy,           -s_s*qx,            s_s*qw,         0,       0,       0,           0,       0, s_w * t,       0,       0,       0,        0,  0,  0, // avz


		    p_a,       0,       0,                 0,                 0,                 0,                 0,       v_a,       0,       0,           0,       0,       0,     a_a,       0,       0,        0,  0,  0, // ax
		      0,     p_a,       0,                 0,                 0,                 0,                 0,         0,     v_a,       0,           0,       0,       0,       0,     a_a,       0,        0,  0,  0, // ay
		      0,       0,     p_a,                 0,                 0,                 0,                 0,         0,       0,     v_a,           0,       0,       0,       0,       0,     a_a,        0,  0,  0, // az

		      0,       0,       0,                 0,                 0,                 0,                 0,         0,       0,       0,           0,       0,       0,       0,       0,       0,       gb,  0,  0 , // bx
			  0,       0,       0,                 0,                 0,                 0,                 0,         0,       0,       0,           0,       0,       0,       0,       0,       0,        0, gb,  0 , // by
			  0,       0,       0,                 0,                 0,                 0,                 0,         0,       0,       0,           0,       0,       0,       0,       0,       0,        0,  0, gb , // bz

	};
	assert(sizeof(Q) == sizeof(FLT) * SURVIVE_MODEL_STATE_CNT * SURVIVE_MODEL_STATE_CNT);
	for(int i = 0;i < SURVIVE_MODEL_STATE_CNT;i++) {
		for(int j = 0;j < i;j++) {
			assert(Q[j + i * SURVIVE_MODEL_STATE_CNT] == Q[i + j * SURVIVE_MODEL_STATE_CNT]);
		}
	}
	// clang-format on
	memcpy(q_out, Q, sizeof(FLT) * SURVIVE_MODEL_STATE_CNT * SURVIVE_MODEL_STATE_CNT);
}

/**
 * The prediction model and associated F matrix use generated code to simplifiy the jacobian. This might not be strictly
 * necessary but allows for quicker development.
 */
static void model_predict(FLT t, const survive_kalman_state_t *k, const CvMat *f_in, CvMat *f_out) {
	assert(t > 0);
	const SurviveKalmanModel *s = (const SurviveKalmanModel *)k->state;
	gen_kalman_model_predict(CV_FLT_PTR(f_out), t, s);
}
static void model_predict_jac(FLT t, FLT *f_out, const struct CvMat *x0) {
	const SurviveKalmanModel *s = (const SurviveKalmanModel *)CV_FLT_PTR(x0);
	if (t == 0) {
		arr_eye_diag(f_out, SURVIVE_MODEL_STATE_CNT, SURVIVE_MODEL_STATE_CNT, 0);
	} else {
		gen_kalman_model_predict_jac_kalman_model(f_out, t, s);
	}
}

FLT survive_imu_integrate_pose(SurviveKalmanTracker *tracker, FLT time, const SurvivePose *pose, const FLT *R) {
	// clang-format off
	FLT _H[] = {
		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	};
	assert(sizeof(_H) == (SURVIVE_MODEL_STATE_CNT * sizeof(FLT) * 7));
	// clang-format on
	CvMat H = cvMat(7, tracker->model.state_cnt, SURVIVE_CV_F, _H);
	CvMat Zp = cvMat(7, 1, SURVIVE_CV_F, (void *)pose->Pos);
	FLT rtn = 0;

	rtn = survive_kalman_predict_update_state(time, &tracker->model, &Zp, &H, R ? R : tracker->Obs_R, R == 0);

	SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(400, "Resultant state %f (pose) " Point16_format, time, LINMATH_VEC16_EXPAND(tracker->model.state));
	return rtn;
}

void survive_kalman_tracker_integrate_observation(PoserData *pd, SurviveKalmanTracker *tracker, const SurvivePose *pose,
												  const FLT *oR) {
	if (tracker->use_raw_obs) {
		SurviveObject *so = tracker->so;

		so->ctx->imuposeproc(so, pd->timecode, pose);
		return;
	}

	survive_long_timecode timecode = pd->timecode;

	struct SurviveContext *ctx = tracker->so->ctx;
	FLT time = timecode / (FLT)tracker->so->timebase_hz;
	if (tracker->model.t == 0) {
		tracker->model.t = time;
	}

	if (time - tracker->model.t < 0) {
		if (time - tracker->model.t > -.1) {
			// time = tracker->model.t;
		} else {
			// SV_WARN("Processing light data from the past %fs", time - tracker->model.t );
			tracker->stats.late_light_dropped++;
			return;
		}
	}
	tracker->last_light_time = time;

	if (tracker->obs_pos_var >= 0 && tracker->obs_rot_var >= 0) {
		FLT R[] = {tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_rot_var,
				   tracker->obs_rot_var, tracker->obs_rot_var, tracker->obs_rot_var};
		if (oR) {
			addnd(R, R, oR, 7);
		}
		tracker->stats.obs_total_error +=
			survive_imu_integrate_pose(tracker, time, pose, tracker->adaptive_obs ? 0 : R);
		tracker->stats.obs_count++;

		survive_kalman_tracker_report_state(pd, tracker);
	}
}

STATIC_CONFIG_ITEM(KALMAN_USE_ADAPTIVE_IMU, "use-adaptive-imu", 'i', "Use adaptive kalman for IMU", 0)
STATIC_CONFIG_ITEM(KALMAN_USE_ADAPTIVE_LIGHTCAP, "use-adaptive-lightcap", 'i', "Use adaptive kalman for Lightcap", 0)
STATIC_CONFIG_ITEM(KALMAN_USE_ADAPTIVE_OBS, "use-adaptive-obs", 'i', "Use adaptive kalman for observations", 0)

STATIC_CONFIG_ITEM(PROCESS_WEIGHT_ACC, "process-weight-acc", 'f', "Acc variance per second", 10.)

STATIC_CONFIG_ITEM(PROCESS_WEIGHT_ANGULAR_VELOCITY, "process-weight-ang-vel", 'f', "Angular velocity variance per second", 1.)
STATIC_CONFIG_ITEM(PROCESS_WEIGHT_VEL, "process-weight-vel", 'f', "Velocity variance per second", 0.)

STATIC_CONFIG_ITEM(PROCESS_WEIGHT_POS, "process-weight-pos", 'f', "Position variance per second", 0.)
STATIC_CONFIG_ITEM(PROCESS_WEIGHT_ROTATION, "process-weight-rot", 'f', "Rotation variance per second", 0.)

STATIC_CONFIG_ITEM(KALMAN_REPORT_IGNORE_START, "report-ignore-start", 'i', "Number of reports to ignore at startup", 0)
STATIC_CONFIG_ITEM(KALMAN_REPORT_IGNORE_THRESHOLD, "report-ignore-threshold", 'f',
				   "Minimum variance to report pose from the kalman filter", 1.)
STATIC_CONFIG_ITEM(KALMAN_LIGHTCAP_IGNORE_THRESHOLD, "light-ignore-threshold", 'f',
				   "Minimum variance to allow light data into the kalman filter", 1.)
STATIC_CONFIG_ITEM(KALMAN_LIGHTCAP_REQUIRED_OBS, "light-required-obs", 'i',
				   "Minimum observations to allow light data into the kalman filter", 10)

STATIC_CONFIG_ITEM(LIGHT_VARIANCE, "light-variance", 'f', "Variance of light sensor readings", 1e-6)
STATIC_CONFIG_ITEM(OBS_POS_VARIANCE, "obs-pos-variance", 'f', "Variance of position integration from light capture",
				   .02)
STATIC_CONFIG_ITEM(OBS_ROT_VARIANCE, "obs-rot-variance", 'f', "Variance of rotation integration from light capture",
				   .01)

STATIC_CONFIG_ITEM(IMU_ACC_VARIANCE, "imu-acc-variance", 'f', "Variance of accelerometer", 5e-5)
STATIC_CONFIG_ITEM(IMU_GYRO_VARIANCE, "imu-gyro-variance", 'f', "Variance of gyroscope", 1e-2)

STATIC_CONFIG_ITEM(USE_IMU, "use-imu", 'i', "Use the IMU as part of the pose solver", 1)
STATIC_CONFIG_ITEM(USE_KALMAN, "use-kalman", 'i', "Apply kalman filter as part of the pose solver", 1)

typedef void (*survive_attach_detach_fn)(SurviveContext *ctx, const char *tag, FLT *var);

static void survive_kalman_tracker_config(SurviveKalmanTracker *tracker, survive_attach_detach_fn fn) {
	fn(tracker->so->ctx, KALMAN_LIGHTCAP_IGNORE_THRESHOLD_TAG, &tracker->light_threshold_var);
	fn(tracker->so->ctx, KALMAN_REPORT_IGNORE_THRESHOLD_TAG, &tracker->report_threshold_var);

	fn(tracker->so->ctx, IMU_ACC_VARIANCE_TAG, &tracker->acc_var);
	fn(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->gyro_var);

	fn(tracker->so->ctx, OBS_POS_VARIANCE_TAG, &tracker->obs_pos_var);
	fn(tracker->so->ctx, OBS_ROT_VARIANCE_TAG, &tracker->obs_rot_var);
	fn(tracker->so->ctx, LIGHT_VARIANCE_TAG, &tracker->light_var);

	fn(tracker->so->ctx, PROCESS_WEIGHT_ACC_TAG, &tracker->process_weight_acc);
	fn(tracker->so->ctx, PROCESS_WEIGHT_VEL_TAG, &tracker->process_weight_vel);
	fn(tracker->so->ctx, PROCESS_WEIGHT_POS_TAG, &tracker->process_weight_pos);

	fn(tracker->so->ctx, PROCESS_WEIGHT_ANGULAR_VELOCITY_TAG, &tracker->process_weight_ang_velocity);
	fn(tracker->so->ctx, PROCESS_WEIGHT_ROTATION_TAG, &tracker->process_weight_rotation);
}

void survive_kalman_tracker_reinit(SurviveKalmanTracker *tracker) {

	tracker->report_ignore_start_cnt = 0;
	tracker->last_light_time = 0;
	tracker->light_residuals_all = 0;
	survive_kalman_state_reset(&tracker->model);

	memset(&tracker->state, 0, sizeof(tracker->state));
	tracker->state.Pose.Rot[0] = 1;

	size_t state_cnt = sizeof(SurviveKalmanModel) / sizeof(FLT);
	memset(tracker->model.P, 0, state_cnt * state_cnt * sizeof(FLT));

	for (int i = 0; i < 7; i++) {
		tracker->model.P[i * SURVIVE_MODEL_STATE_CNT + i] = 1e3;
	}
	for (int i = 16; i < 19; i++) {
		tracker->model.P[i * SURVIVE_MODEL_STATE_CNT + i] = 1;
	}

	FLT Rrs = tracker->obs_rot_var;
	FLT Rps = tracker->obs_pos_var;
	FLT Rr[] = {Rrs, Rrs, Rrs, Rrs, Rps, Rps, Rps};
	arr_eye_diag(tracker->Obs_R, 7, 7, Rr);

	FLT Rimu[] = {tracker->acc_var,	 tracker->acc_var,	tracker->acc_var,
				  tracker->gyro_var, tracker->gyro_var, tracker->gyro_var};
	arr_eye_diag(tracker->IMU_R, 6, 6, Rimu);
}

void survive_kalman_tracker_init(SurviveKalmanTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));

	tracker->so = so;

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(110, "Initializing Filter:");
	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.

	survive_kalman_tracker_config(tracker, survive_attach_configf);

	survive_attach_configi(tracker->so->ctx, KALMAN_REPORT_IGNORE_START_TAG, &tracker->report_ignore_start);
	survive_attach_configi(tracker->so->ctx, KALMAN_LIGHTCAP_REQUIRED_OBS_TAG, &tracker->light_required_obs);
	survive_attach_configi(tracker->so->ctx, KALMAN_USE_ADAPTIVE_IMU_TAG, &tracker->adaptive_imu);
	survive_attach_configi(tracker->so->ctx, KALMAN_USE_ADAPTIVE_LIGHTCAP_TAG, &tracker->adaptive_lightcap);
	survive_attach_configi(tracker->so->ctx, KALMAN_USE_ADAPTIVE_OBS_TAG, &tracker->adaptive_obs);

	bool use_imu = (bool)survive_configi(ctx, "use-imu", SC_GET, 1);
	if (!use_imu) {
		tracker->gyro_var = tracker->acc_var = -1;
	}

	bool use_kalman = (bool)survive_configi(ctx, "use-kalman", SC_GET, 1);
	tracker->use_raw_obs = !use_kalman;

	survive_kalman_set_logging_level(ctx->log_level);
	size_t state_cnt = sizeof(SurviveKalmanModel) / sizeof(FLT);
	survive_kalman_state_init(&tracker->model, state_cnt, model_predict_jac, model_q_fn, tracker,
							  (FLT *)&tracker->state);
	tracker->model.Predict_fn = model_predict;

	survive_kalman_tracker_reinit(tracker);

	SV_VERBOSE(10, "\t%s: %f", IMU_ACC_VARIANCE_TAG, tracker->acc_var);
	SV_VERBOSE(10, "\t%s: %f", IMU_GYRO_VARIANCE_TAG, tracker->gyro_var);
	SV_VERBOSE(10, "\t%s: %f", KALMAN_LIGHTCAP_IGNORE_THRESHOLD_TAG, tracker->light_threshold_var);
	SV_VERBOSE(10, "\t%s: %f", KALMAN_REPORT_IGNORE_THRESHOLD_TAG, tracker->report_threshold_var);
}

SurviveVelocity survive_kalman_tracker_velocity(const SurviveKalmanTracker *tracker) {
	SurviveVelocity rtn = {0};
	survive_kalman_predict_state(0, &tracker->model, 7, 13, rtn.Pos);
	return rtn;
}

void survive_kalman_tracker_free(SurviveKalmanTracker *tracker) {
	SurviveContext *ctx = tracker->so->ctx;

	SV_VERBOSE(5, "IMU %s tracker statistics:", tracker->so->codename);
	SV_VERBOSE(5, "\t%-32s %u", "late imu", tracker->stats.late_imu_dropped);
	SV_VERBOSE(5, "\t%-32s %u", "late light", tracker->stats.late_light_dropped);

	SV_VERBOSE(5, "\t%-32s %u of %u (%2.2f%%)", "Dropped poses", (unsigned)tracker->stats.dropped_poses,
			   (unsigned)(tracker->stats.reported_poses + tracker->stats.dropped_poses),
			   100. * tracker->stats.dropped_poses /
				   (FLT)(tracker->stats.reported_poses + tracker->stats.dropped_poses))

	FLT var[SURVIVE_MODEL_STATE_CNT] = {0};
	scalend(var, tracker->stats.reported_var, 1. / tracker->stats.reported_poses, SURVIVE_MODEL_STATE_CNT);
	SV_VERBOSE(5, "\t%-32s " Point19_format, "Mean reported variance", LINMATH_VEC19_EXPAND(var));
	scalend(var, tracker->stats.dropped_var, 1. / tracker->stats.reported_poses, SURVIVE_MODEL_STATE_CNT);
	SV_VERBOSE(5, "\t%-32s " Point19_format, "Mean dropped variance", LINMATH_VEC19_EXPAND(var));

	SV_VERBOSE(5, "\t%-32s %e (%7u integrations)", "Obs error",
			   tracker->stats.obs_total_error / (FLT)tracker->stats.obs_count, (unsigned)tracker->stats.obs_count);
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations)", "Lightcap error",
			   tracker->stats.lightcap_total_error / (FLT)tracker->stats.lightcap_count,
			   (unsigned)tracker->stats.lightcap_count);
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations)", "IMU error",
			   tracker->stats.imu_total_error / (FLT)tracker->stats.imu_count, (unsigned)tracker->stats.imu_count);
	SV_VERBOSE(5, " ");
	SV_VERBOSE(5, "\t%-32s " Point3_format, "gyro bias", LINMATH_VEC3_EXPAND(tracker->state.GyroBias));
	SV_VERBOSE(5, "\t%-32s " FLT_format, "Lightcap R", tracker->light_var);
	for (int i = 0; i < 6; i++) {
		SV_VERBOSE(5, "\t%-32s " Point6_format, i == 0 ? "Gyro R" : "", LINMATH_VEC6_EXPAND(tracker->IMU_R + 6 * i));
	}
	for (int i = 0; i < 7; i++) {
		SV_VERBOSE(5, "\t%-32s " Point7_format, i == 0 ? "Observation R" : "",
				   LINMATH_VEC7_EXPAND(tracker->Obs_R + 7 * i));
	}

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (tracker->stats.lightcap_count_by_lh[i]) {
			SV_VERBOSE(5, "\tLighthouse %d", i);
			SV_VERBOSE(5, "\t\t%-32s %e", "Avg error",
					   tracker->stats.lightcap_error_by_lh[i] / tracker->stats.lightcap_count_by_lh[i]);
			SV_VERBOSE(5, "\t\t%-32s %u", "Count", (unsigned)tracker->stats.lightcap_count_by_lh[i]);
			SV_VERBOSE(5, "\t\t%-32s %e", "Current error", tracker->light_residuals[i]);
		}
	}

	SV_VERBOSE(5, " ");

	survive_kalman_state_free(&tracker->model);

	survive_detach_config(tracker->so->ctx, KALMAN_REPORT_IGNORE_START_TAG, &tracker->report_ignore_start);
	survive_detach_config(tracker->so->ctx, KALMAN_USE_ADAPTIVE_IMU_TAG, &tracker->adaptive_imu);
	survive_detach_config(tracker->so->ctx, KALMAN_USE_ADAPTIVE_LIGHTCAP_TAG, &tracker->adaptive_lightcap);
	survive_detach_config(tracker->so->ctx, KALMAN_USE_ADAPTIVE_OBS_TAG, &tracker->adaptive_obs);
	survive_detach_config(tracker->so->ctx, KALMAN_LIGHTCAP_REQUIRED_OBS_TAG, &tracker->light_required_obs);

	survive_kalman_tracker_config(tracker, (survive_attach_detach_fn)survive_detach_config);
}

void survive_kalman_tracker_lost_tracking(SurviveKalmanTracker *tracker) {
	SurviveContext *ctx = tracker->so->ctx;
	SV_WARN("Too many failures; reseting calibration %e (%" PRIu64 " stationary)", tracker->light_residuals_all,
			SurviveSensorActivations_stationary_time(&tracker->so->activations));
	tracker->light_residuals_all = 0;
	{
		survive_kalman_tracker_reinit(tracker);
		memset(&tracker->so->OutPoseIMU, 0, sizeof(SurvivePose));
		memset(&tracker->so->OutPose, 0, sizeof(SurvivePose));
	}

	bool objectsAreValid = false;
	for (int i = 0; i < ctx->objs_ct && !objectsAreValid; i++) {
		objectsAreValid |= !quatiszero(ctx->objs[i]->OutPoseIMU.Rot);
	}

	if (!objectsAreValid) {
		for (int lh = 0; lh < ctx->activeLighthouses; lh++) {
			ctx->bsd[lh].PositionSet = 0;
			SV_WARN("LH%d %f", lh, tracker->light_residuals[lh]);
		}
	}
}

bool survive_kalman_tracker_check_valid(SurviveKalmanTracker *tracker) {
	bool isValid = tracker->light_residuals_all < 5e-2 ||
				   (SurviveSensorActivations_stationary_time(&tracker->so->activations) < 48000000 / 10);
	if (!isValid) {
		survive_kalman_tracker_lost_tracking(tracker);
		return false;
	}
	return true;
}

void survive_kalman_tracker_report_state(PoserData *pd, SurviveKalmanTracker *tracker) {
	SurvivePose pose = {0};

	FLT t = pd->timecode / (FLT)tracker->so->timebase_hz;

	if (t < tracker->model.t) {
		assert(tracker->model.t - t < 1);
		t = tracker->model.t;
	}

	if (!survive_kalman_tracker_check_valid(tracker)) {
		tracker->stats.dropped_poses++;
		return;
	}

	survive_kalman_tracker_predict(tracker, t, &pose);

	FLT var_diag[SURVIVE_MODEL_STATE_CNT];
	if ((tracker->report_threshold_var > 0 &&
		 survive_kalman_tracker_position_var2(tracker, var_diag) >= tracker->report_threshold_var) ||
		tracker->report_ignore_start < tracker->report_ignore_start_cnt) {
		tracker->stats.dropped_poses++;
		addnd(tracker->stats.dropped_var, var_diag, tracker->stats.dropped_var, SURVIVE_MODEL_STATE_CNT);
		tracker->report_ignore_start_cnt++;
		return;
	}
	SurviveContext *ctx = tracker->so->ctx;
	addnd(tracker->stats.reported_var, var_diag, tracker->stats.reported_var, SURVIVE_MODEL_STATE_CNT);

	SV_VERBOSE(400, "Tracker variance %s " Point16_format, tracker->so->codename, LINMATH_VEC16_EXPAND(var_diag));
	SV_VERBOSE(400, "Tracker Bias %s     " Point3_format, tracker->so->codename,
			   LINMATH_VEC3_EXPAND(tracker->state.GyroBias));

	tracker->stats.reported_poses++;
	SV_VERBOSE(400, "Tracker report %s   " SurvivePose_format, tracker->so->codename, SURVIVE_POSE_EXPAND(pose));

	SurviveVelocity velocity = survive_kalman_tracker_velocity(tracker);

	SurviveObject *so = tracker->so;

	/*
	 * static int report_in_imu = -1;
		if (report_in_imu == -1) {
			report_in_imu = survive_configi(so->ctx, REPORT_IN_IMU_TAG, SC_GET, 0);
		}

		SurvivePose head2world;
		so->OutPoseIMU = *imu2world;
		if (!report_in_imu) {
			ApplyPoseToPose(&head2world, imu2world, &so->head2imu);
		} else {
			head2world = *imu2world;
		}

		for (int i = 0; i < 7; i++)
			assert(!isnan(((FLT *)imu2world)[i]));

	 */
	so->ctx->imuposeproc(so, pd->timecode, &pose);
	so->ctx->velocityproc(so, pd->timecode, &velocity);
}
