#pragma once

#include "math.h"
#include "string.h"
#include "survive.h"
#include "survive_reproject.h"
#include <mpfit/mpfit.h>
#include <os_generic.h>

#ifdef __cplusplus
extern "C" {
#endif

enum survive_optimizer_measurement_type {
	survive_optimizer_measurement_type_none,
	survive_optimizer_measurement_type_parameters_bias,
	survive_optimizer_measurement_type_light,
	survive_optimizer_measurement_type_object_accel,
	survive_optimizer_measurement_type_fixed_rotation,
	survive_optimizer_measurement_type_camera_accel,
	survive_optimizer_measurement_type_camera_position,
};

enum survive_optimizer_parameter_type {
	survive_optimizer_parameter_none,
	survive_optimizer_parameter_object_pose,
	survive_optimizer_parameter_object_velocity,
	survive_optimizer_parameter_object_scale,
	survive_optimizer_parameter_object_lighthouse_correction,
	survive_optimizer_parameter_camera,
	survive_optimizer_parameter_camera_parameters,
	survive_optimizer_parameter_obj_points,
};

typedef struct {
	FLT value;
	uint8_t lh;
	uint8_t sensor_idx;
	uint8_t axis;
	int object;
} survive_optimizer_light_measurement;

typedef struct {
	int object;
	LinmathPose pose;
} survive_optimizer_object_pose_measurement;

typedef struct {
	int object;
	LinmathVec3d acc;
} survive_optimizer_object_acc_measurement;

typedef struct {
	int camera;
	LinmathVec3d acc;
} survive_optimizer_camera_acc_measurement;

typedef struct {
	int obj;
	bool conjugate;
	LinmathVec3d match_vec;
	LinmathVec3d plane;
} survive_optimizer_fixed_rotation_measurement;

typedef struct {
	int camera;
	LinmathVec3d pos;
} survive_optimizer_camera_position_measurement;

typedef struct {
	int parameter_index;
	FLT expected_value;
} survive_optimizer_parameter_bias_measurement;

typedef struct {
	FLT time;
	size_t size;
	bool invalid;
	FLT variance;

	enum survive_optimizer_measurement_type meas_type;

	union {
		survive_optimizer_light_measurement light;
		survive_optimizer_object_acc_measurement pose_acc;
		survive_optimizer_camera_acc_measurement camera_acc;
		survive_optimizer_camera_position_measurement camera_pos;
		survive_optimizer_fixed_rotation_measurement fixed_rotation;
		survive_optimizer_parameter_bias_measurement parameter_bias;
	};
} survive_optimizer_measurement;

typedef struct {
  size_t size;
  size_t elem_size;
  size_t p_idx;
  enum survive_optimizer_parameter_type param_type;
  struct mp_par_struct *pi;
  FLT *p;
} survive_optimizer_parameter;

typedef struct survive_optimizer_settings {
	bool use_quat_model;
	bool disable_filter;
	FLT lh_scale_correction;
	FLT lh_offset_correction;
	bool disallow_pair_calc;
	FLT optimize_scale_threshold;
	FLT current_pos_bias;
	FLT current_rot_bias;
} survive_optimizer_settings;

struct mp_par_struct;
struct mp_result_struct;

typedef struct survive_optimizer {
    const survive_optimizer_settings* settings;

	const survive_reproject_model_t *reprojectModel;

	SurviveObject **sos;
	survive_optimizer_measurement *measurements;
	size_t measurementsCnt, parametersCnt, parameterBlockCnt;

	FLT objectUpVectorVariance;
	FLT timecode;

	struct mp_par_struct *mp_parameters_info;
	survive_optimizer_parameter *parameters_info;
	FLT *parameters;

	bool disableVelocity;
	bool dontScaleCov;

	int poseLength;
	int cameraLength;
	int ptsLength;
	bool nofilter;

	mp_config *cfg;

	bool needsFiltering;

	struct {
		uint32_t total_meas_cnt;
		uint32_t total_lh_cnt;
		uint32_t dropped_meas_cnt;
		uint32_t dropped_lh_cnt;

		FLT object_up_error; int object_up_error_cnt;
        FLT sensor_error; int sensor_error_cnt;
        FLT current_error; int current_error_cnt;
		FLT params_error; int params_error_cnt;
	} stats;

	void *user;
	void (*iteration_cb)(struct survive_optimizer *opt_ctx, int m, int n, FLT *p, FLT *deviates, FLT **derivs);
} survive_optimizer;

#define SURVIVE_OPTIMIZER_SETUP_BUFFERS(ctx, alloc_fn, ...)                                                            \
	{                                                                                                                  \
		size_t par_count = survive_optimizer_get_max_parameters_count(&(ctx));                                         \
		size_t meas_count = survive_optimizer_get_max_measurements_count(&(ctx));                                      \
		FLT *param_buffer = alloc_fn(((ctx).parameters), par_count * sizeof(FLT));                                 \
		mp_par *mp_param_info_buffer =                                                                                 \
			(mp_par *)alloc_fn(((ctx).mp_parameters_info), par_count * sizeof(struct mp_par_struct));                  \
		survive_optimizer_parameter *param_info_buffer = (survive_optimizer_parameter *)alloc_fn(                      \
			((ctx).parameters_info), par_count * sizeof(survive_optimizer_parameter));                                 \
		size_t measurementAllocationSize = meas_count * sizeof(survive_optimizer_measurement);                         \
		size_t upAllocationSize = sizeof(LinmathPoint3d) * ((ctx).poseLength + (ctx).cameraLength);                    \
		void *measurement_buffer = alloc_fn(((ctx).measurements), measurementAllocationSize + upAllocationSize);       \
		void *sos_buffer = alloc_fn((ctx).sos, sizeof(SurviveObject *) * (ctx).poseLength);                            \
		memset(sos_buffer, 0, sizeof(SurviveObject *) * (ctx).poseLength);                                             \
		SurviveObject *sos[] = {__VA_ARGS__};                                                                          \
		memcpy(sos_buffer, sos, sizeof(sos));                                                                          \
		survive_optimizer_setup_buffers(&(ctx), param_buffer, param_info_buffer, mp_param_info_buffer,                 \
										measurement_buffer, sos_buffer);                                               \
	}

#define SURVIVE_OPTIMIZER_ALLOCA(ctx, size) alloca(size)
#define SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(ctx, ...)                                                                \
	SURVIVE_OPTIMIZER_SETUP_BUFFERS((ctx), SURVIVE_OPTIMIZER_ALLOCA, __VA_ARGS__)
#define SURVIVE_OPTIMIZER_SETUP_HEAP_BUFFERS(ctx, ...)                                                                 \
	SURVIVE_OPTIMIZER_SETUP_BUFFERS((ctx), survive_optimizer_realloc, __VA_ARGS__)
#define SURVIVE_OPTIMIZER_CLEANUP_STACK_BUFFERS(ctx)
#define SURVIVE_OPTIMIZER_CLEANUP_HEAP_BUFFERS(ctx)                                                                    \
	{                                                                                                                  \
		free(ctx.parameters);                                                                                          \
		free(ctx.mp_parameters_info);                                                                                  \
		free(ctx.measurements);                                                                                        \
	}

SURVIVE_EXPORT void *survive_optimizer_realloc(void *old_ptr, size_t size);

SURVIVE_EXPORT int survive_optimizer_get_max_measurements_count(const survive_optimizer *ctx);
SURVIVE_EXPORT int survive_optimizer_get_max_parameters_count(const survive_optimizer *ctx);
SURVIVE_EXPORT int survive_optimizer_get_parameters_count(const survive_optimizer *ctx);
SURVIVE_EXPORT int survive_optimizer_get_free_parameters_count(const survive_optimizer *ctx);

SURVIVE_EXPORT size_t survive_optimizer_get_total_buffer_size(const survive_optimizer *ctx);

SURVIVE_EXPORT void survive_optimizer_setup_buffers(survive_optimizer *ctx, FLT *parameter_buffer,
													survive_optimizer_parameter *parameter_info_buffer,
													struct mp_par_struct *mp_parameter_info_buffer,
													void *measurements_buffer, void *so_buffer);

SURVIVE_EXPORT SurvivePose *survive_optimizer_get_pose(survive_optimizer *ctx);
SURVIVE_EXPORT int survive_optimizer_get_velocity_index(const survive_optimizer *ctx);
SURVIVE_EXPORT SurviveVelocity *survive_optimizer_get_velocity(survive_optimizer *ctx);

SURVIVE_EXPORT int survive_optimizer_get_sensor_scale_index(const survive_optimizer *ctx);
SURVIVE_EXPORT void survive_optimizer_disable_sensor_scale(survive_optimizer *ctx);

SURVIVE_EXPORT int survive_optimizer_get_camera_index(const survive_optimizer *ctx);

SURVIVE_EXPORT SurvivePose *survive_optimizer_get_camera(survive_optimizer *ctx);

SURVIVE_EXPORT int survive_optimizer_get_calibration_index(const survive_optimizer *ctx);

SURVIVE_EXPORT BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh);

SURVIVE_EXPORT int survive_optimizer_get_sensors_index(const survive_optimizer *ctx);

SURVIVE_EXPORT FLT *survive_optimizer_get_sensors(survive_optimizer *ctx, size_t idx);

SURVIVE_EXPORT void survive_optimizer_setup_pose_n(survive_optimizer *mpfit_ctx, const SurvivePose *pose, size_t n,
												   bool isFixed, int use_jacobian_function);

SURVIVE_EXPORT void survive_optimizer_fix_camera(survive_optimizer *mpfit_ctx, int cam_idx);
SURVIVE_EXPORT void survive_optimizer_fix_obj_yaw(survive_optimizer *mpfit_ctx, int obj_idx);
SURVIVE_EXPORT void survive_optimizer_fix_cam_yaw(survive_optimizer *mpfit_ctx, int lh_idx);
SURVIVE_EXPORT void survive_optimizer_fix_cam_pos(survive_optimizer *mpfit_ctx, int lh_idx);

SURVIVE_EXPORT void survive_optimizer_remove_data_for_lh(survive_optimizer *mpfit_ctx, int cam_idx);

SURVIVE_EXPORT void survive_optimizer_setup_pose(survive_optimizer *mpfit_ctx, const SurvivePose *pose, bool isFixed,
												 int use_jacobian_function);

SURVIVE_EXPORT void survive_optimizer_setup_camera(survive_optimizer *mpfit_ctx, int8_t lh, const SurvivePose *pose,
												   bool isFixed, int use_jacobian_function);

SURVIVE_EXPORT void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed,
													int use_jacobian_function, bool useTruePosition);

SURVIVE_EXPORT const char *survive_optimizer_error(int status);

SURVIVE_EXPORT int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result,
										 struct CnMat *R);
SURVIVE_EXPORT void survive_optimizer_covariance_expand(survive_optimizer *optimizer, const struct CnMat *R_free,
														struct CnMat *R);

SURVIVE_EXPORT void survive_optimizer_set_reproject_model(survive_optimizer *optimizer,
														  const survive_reproject_model_t *reprojectModel);

SURVIVE_EXPORT void survive_optimizer_serialize(const survive_optimizer *optimizer, const char *fn);

SURVIVE_EXPORT survive_optimizer *survive_optimizer_load(const char *fn);

SURVIVE_EXPORT FLT survive_optimizer_current_norm(const survive_optimizer *optimizer);

SURVIVE_EXPORT mp_config *survive_optimizer_precise_config();

SURVIVE_EXPORT int survive_optimizer_nonfixed_cnt(const survive_optimizer *optimizer);

SURVIVE_EXPORT void survive_optimizer_get_nonfixed(const survive_optimizer *optimizer, FLT *params);
SURVIVE_EXPORT void survive_optimizer_set_nonfixed(survive_optimizer *optimizer, FLT *params);

SURVIVE_EXPORT survive_optimizer_measurement *survive_optimizer_emplace_meas(survive_optimizer *ctx,
																			 enum survive_optimizer_measurement_type);
SURVIVE_EXPORT survive_optimizer_parameter *
survive_optimizer_emplace_params(survive_optimizer *ctx, enum survive_optimizer_parameter_type, int n);

SURVIVE_EXPORT void survive_optimizer_pop_meas(survive_optimizer *ctx, int cnt);

SURVIVE_EXPORT FLT *survive_optimizer_obj_up_vector(survive_optimizer *ctx, int i);
SURVIVE_EXPORT FLT *survive_optimizer_cam_up_vector(survive_optimizer *ctx, int i);
SURVIVE_EXPORT void survive_optimizer_set_cam_up_vector(survive_optimizer *ctx, int i, FLT variance,
														const LinmathVec3d up);
SURVIVE_EXPORT void survive_optimizer_set_obj_up_vector(survive_optimizer *ctx, int i, FLT variance,
														const LinmathVec3d up);
SURVIVE_EXPORT void survive_optimizer_settings_attach_config(SurviveContext* ctx, survive_optimizer_settings* t);
SURVIVE_EXPORT void survive_optimizer_settings_detach_config(SurviveContext* ctx, survive_optimizer_settings* t);

#ifdef __cplusplus
}
#endif
