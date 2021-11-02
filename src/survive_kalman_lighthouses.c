#include "survive_kalman_lighthouses.h"
#include "survive_recording.h"
#include "survive_reproject.h"
#include "survive_reproject_gen2.h"

#include "generated/survive_imu.generated.h"

STRUCT_CONFIG_SECTION(SurviveKalmanLighthouse)
STRUCT_CONFIG_ITEM("lh-light-variance", "", -1e-2, t->light_variance);
STRUCT_CONFIG_ITEM("lh-light-stationary-time", "", .1, t->light_stationary_mintime);
STRUCT_CONFIG_ITEM("lh-light-stationary-maxtime", "", 2, t->light_stationary_maxtime);
END_STRUCT_CONFIG_SECTION(SurviveKalmanLighthouse)

//#define TRACK_IN_WORLD2LH

#ifdef TRACK_IN_WORLD2LH
static SurvivePose survive_kalman_lighthouse_lh2world(SurviveKalmanLighthouse *tracker) {
	return InvertPoseRtn(&tracker->state);
}
static SurvivePose survive_kalman_lighthouse_world2lh(SurviveKalmanLighthouse *tracker) { return tracker->state; }
#else
static SurvivePose survive_kalman_lighthouse_lh2world(SurviveKalmanLighthouse *tracker) { return tracker->state; }
#endif

void survive_kalman_lighthouse_report(SurviveKalmanLighthouse *tracker) {
	quatnormalize(tracker->state.Rot, tracker->state.Rot);
	SurvivePose lighthouse2world = survive_kalman_lighthouse_lh2world(tracker);
	FLT var_diag[3] = {0};
	for (int i = 0; i < 3; i++) {
		var_diag[i] = svMatrixGet(&tracker->model.P, i, i);
	}

	if (tracker->lh != 0) {
		survive_recording_write_to_output(tracker->ctx->recptr, "SPHERE lh_conf_size_%d %f %d " Point3_format "\n",
										  tracker->lh, roundf(100. * norm3d(var_diag)) / 100., 0xFF,
										  LINMATH_VEC3_EXPAND(lighthouse2world.Pos));
	}

	tracker->ctx->bsd[tracker->lh].confidence = 1. / norm3d(var_diag);
	assert(sv_is_finite(&tracker->model.state));

	SURVIVE_INVOKE_HOOK(lighthouse_pose, tracker->ctx, tracker->lh, &lighthouse2world);
}

struct map_light_data_ctx {
	SurviveKalmanLighthouse *tracker;
	SurviveObject *so;

	LightInfo savedLight[32];
	uint32_t savedLight_idx;
	SurvivePose objectPose;
};

static inline int get_axis(const struct PoserDataLight *pdl) {
	switch (pdl->hdr.pt) {
	case POSERDATA_LIGHT:
		return (((PoserDataLightGen1 *)pdl)->acode & 1);
		break;
	case POSERDATA_LIGHT_GEN2:
		return ((PoserDataLightGen2 *)pdl)->plane;
		break;
	default:
		assert(0);
	}
	return 0;
}
static bool map_light_data(void *user, const struct SvMat *Z, const struct SvMat *x_t, struct SvMat *y,
						   struct SvMat *H_k) {
	struct map_light_data_ctx *cbctx = (struct map_light_data_ctx *)user;
	const SurviveKalmanLighthouse *tracker = cbctx->tracker;

	SurviveObject *so = cbctx->so;
	struct SurviveContext *ctx = so->ctx;
	const survive_reproject_model_t *mdl = survive_reproject_model(ctx);
	sv_set_zero(H_k);

	FLT *Y = sv_as_vector(y);

	SV_CREATE_STACK_MAT(inv_jacobian_row_ordered, 7, 7);
#ifdef TRACK_IN_WORLD2LH
	const SurvivePose world2lh = *(SurvivePose *)sv_as_const_vector(x_t);
	sv_eye(&inv_jacobian_row_ordered, 0);
#else
	const SurvivePose lh2world = *(SurvivePose *)sv_as_const_vector(x_t);
	const SurvivePose world2lh = InvertPoseRtn(&lh2world);
	gen_invert_pose_jac_obj_p(inv_jacobian_row_ordered.data, &lh2world);
#endif
	for (int i = 0; i < Z->rows; i++) {
		const LightInfo *info = &cbctx->savedLight[i];
		int axis = info->axis;

		survive_reproject_full_xy_fn_t project_fn = mdl->reprojectAxisFullFn[axis];
		survive_reproject_axis_jacob_lh_pose_fn_t project_jacob_fn = mdl->reprojectAxisJacobLhPoseFn[axis];

		const SurvivePose obj2world = cbctx->objectPose;

		const FLT *ptInObj = &so->sensor_locations[info->sensor_idx * 3];
		FLT h_x = project_fn(&obj2world, ptInObj, &world2lh, &ctx->bsd[info->lh].fcal[axis]);
		Y[i] = sv_as_const_vector(Z)[i] - h_x;
		SV_DATA_LOG("Z_bsd_light[%d][%d][%d]", &info->value, 1, info->lh, info->axis, info->sensor_idx);
		SV_DATA_LOG("h_bsd_light[%d][%d][%d]", &h_x, 1, info->lh, info->axis, info->sensor_idx);
		SV_DATA_LOG("Y_bsd_light[%d][%d][%d]", Y, 1, info->lh, info->axis, info->sensor_idx);

		SV_CREATE_STACK_MAT(jacobian, 1, 7);
		project_jacob_fn(sv_as_vector(&jacobian), &obj2world, ptInObj, &world2lh, &ctx->bsd[info->lh].fcal[axis]);

#ifdef SV_MATRIX_IS_COL_MAJOR
		svGEMM(&jacobian, &inv_jacobian_row_ordered, 1, 0, 0, H_k, SV_GEMM_FLAG_B_T);
		Outstanding;
		Do the slow thing here to copy into H_k;
		yada
#else
		SvMat H_ki = sv_row(H_k, i);
		svGEMM(&jacobian, &inv_jacobian_row_ordered, 1, 0, 0, &H_ki, 0);
#endif
	}
	if (!sv_is_finite(H_k))
		return false;

	return true;
}

void survive_kalman_lighthouse_integrate_light(SurviveKalmanLighthouse *tracker, SurviveObject *so,
											   PoserDataLight *data) {
	bool isSync = data->hdr.pt == POSERDATA_SYNC || data->hdr.pt == POSERDATA_SYNC_GEN2;
	if (isSync || tracker->light_variance < 0 || !tracker->ctx->bsd[tracker->lh].PositionSet)
		return;

	FLT stationary_time = SurviveSensorActivations_stationary_time(&so->activations) / (FLT)so->timebase_hz;
	SurviveContext *ctx = tracker->ctx;
	FLT time = data->hdr.timecode / (FLT)so->timebase_hz;
	if (stationary_time < tracker->light_stationary_mintime)
		return;
	if (stationary_time > tracker->light_stationary_maxtime && tracker->light_stationary_maxtime > 0)
		return;

	if (tracker->light_variance >= 0) {
		SV_CREATE_STACK_MAT(Z, 1, 1);

		svMatrixSet(&Z, 0, 0, data->angle);
		struct map_light_data_ctx cbctx = {
			.tracker = tracker,
			.so = so,
			.savedLight_idx = 1,
			.savedLight = {
				{.lh = data->lh, .axis = get_axis(data), .sensor_idx = data->sensor_id, .value = data->angle}}};

		survive_kalman_tracker_predict(so->tracker, 0, &cbctx.objectPose);
		if (quatiszero(cbctx.objectPose.Rot))
			return;

		FLT so_var[3];
		sv_get_diag(&so->tracker->model.P, so_var, 3);
		FLT v = tracker->light_variance + norm3d(so_var);

		FLT light_vars[32] = {0};
		for (int i = 0; i < 32; i++)
			light_vars[i] = v;
		survive_kalman_update_extended_params_t params = {.Hfn = map_light_data, .user = &cbctx};
		survive_kalman_predict_update_state_extended(time, &tracker->model, &Z, light_vars, &params);
		survive_kalman_lighthouse_report(tracker);
	}
}

static SurvivePose copy_model(const FLT *src, size_t state_size) {
	SurvivePose rtn = {0};
	memcpy(rtn.Pos, src, sizeof(FLT) * state_size);
	return rtn;
}

static void survive_kalman_lighthouse_process_noise(void *user, FLT t, const SvMat *x, struct SvMat *q_out) {
	SurviveKalmanLighthouse *tracker = user;
	size_t state_cnt = x->rows;
	SurvivePose state = copy_model(sv_as_const_vector(x), state_cnt);

	/* ================== Positional ============================== */
	// Estimation with Applications to Tracking and Navigation: Theory Algorithms and Software Ch 6
	// http://wiki.dmdevelopment.ru/wiki/Download/Books/Digitalimageprocessing/%D0%9D%D0%BE%D0%B2%D0%B0%D1%8F%20%D0%BF%D0%BE%D0%B4%D0%B1%D0%BE%D1%80%D0%BA%D0%B0%20%D0%BA%D0%BD%D0%B8%D0%B3%20%D0%BF%D0%BE%20%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9%20%D0%BE%D0%B1%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D0%BA%D0%B5%20%D1%81%D0%B8%D0%B3%D0%BD%D0%B0%D0%BB%D0%BE%D0%B2/Estimation%20with%20Applications%20to%20Tracking%20and%20Navigation/booktext@id89013302placeboie.pdf

	// We mix three order models here based on tuning variables.

	FLT p_p = tracker->process_weight_pos * t;

	FLT s_f = 0;
	FLT rv = tracker->process_weight_rotation * t;

	FLT qw = state.Rot[0], qx = state.Rot[1], qy = state.Rot[2], qz = state.Rot[3];
	FLT qws = qw * qw, qxs = qx * qx, qys = qy * qy, qzs = qz * qz;
	FLT qs = qws + qxs + qys + qzs;

	// clang-format off
	FLT Q[] = {
		p_p,       0,       0,                 0,                 0,                 0,                 0,
		0,     p_p,       0,                 0,                 0,                 0,                 0,
		0,       0,     p_p,                 0,                 0,                 0,                 0,

		0,       0,       0,   rv+s_f*(qs-qws),      s_f*(-qw*qx),      s_f*(-qw*qy),      s_f*(-qw*qz),
		0,       0,       0,      s_f*(-qw*qx),   rv+s_f*(qs-qxs),      s_f*(-qx*qy),      s_f*(-qx*qz),
		0,       0,       0,      s_f*(-qw*qy),      s_f*(-qx*qy),   rv+s_f*(qs-qys),      s_f*(-qy*qz),
		0,       0,       0,      s_f*(-qw*qz),      s_f*(-qx*qz),      s_f*(-qy*qz),   rv+s_f*(qs-qzs),
	};
	// clang-format on

	assert(sizeof(Q) == sizeof(FLT) * 7 * 7);
	for (int i = 0; i < 7; i++) {
		for (int j = 0; j < i; j++) {
			assert(Q[j + i * 7] == Q[i + j * 7]);
		}
	}

	sv_copy_in_row_major(q_out, Q, 7);
}

void survive_kalman_lighthouse_init(SurviveKalmanLighthouse *tracker, SurviveContext *ctx, int lh) {
	memset(tracker, 0, sizeof(*tracker));
	tracker->ctx = ctx;
	tracker->lh = lh;
	SurviveKalmanLighthouse_attach_config(ctx, tracker);

	survive_kalman_state_init(&tracker->model, 7, 0, survive_kalman_lighthouse_process_noise, tracker,
							  (FLT *)&tracker->state);
	tracker->state.Rot[0] = 1;
	for (int i = 0; i < 3; i++)
		svMatrixSet(&tracker->model.P, i, i, 1e5);
	for (int i = 3; i < 7; i++)
		svMatrixSet(&tracker->model.P, i, i, 1e3);
}

SURVIVE_EXPORT void survive_kalman_lighthouse_integrate_observation(SurviveKalmanLighthouse *tracker,
																	const SurvivePose *pose, const FLT *_variance) {
	SV_CREATE_STACK_MAT(H, 7, tracker->model.state_cnt);

	size_t state_cnt = tracker->model.state_cnt;
	sv_set_diag_val(&H, 1);
	FLT variance[7] = {.1, .1, .1, .01, .01, .01, .01};
	if (_variance) {
		memcpy(variance, _variance, sizeof(variance));
	}
	FLT v = normnd2(variance, 7);

	if (v > 0) {
#ifdef TRACK_IN_WORLD2LH
		SurvivePose Z = InvertPoseRtn(pose);
#else
		SurvivePose Z = *pose;
#endif
		SvMat Zp = svMat(7, 1, (void *)&Z);
		survive_kalman_predict_update_state(0, &tracker->model, &Zp, &H, variance, 0);

		if (tracker->lh == 1) {
			// sv_set_constant(&tracker->model.P, 0);
			svMatrixSet(&tracker->model.P, 0, 0, 0);
			svMatrixSet(&tracker->model.P, 1, 1, 0);
		}
	} else {
		tracker->state = *pose;
		sv_set_constant(&tracker->model.P, 1e-10);
	}
	survive_kalman_lighthouse_report(tracker);
}
void survive_kalman_lighthouse_free(SurviveKalmanLighthouse *tracker) {
	SurviveKalmanLighthouse_detach_config(tracker->ctx, tracker);
	survive_kalman_state_free(&tracker->model);
	free(tracker);
}
