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

typedef struct {
	FLT value;
	FLT variance;
	uint8_t lh;
	uint8_t sensor_idx;
	uint8_t axis;
	int object;

	bool invalid;
} survive_optimizer_measurement;

struct mp_par_struct;
struct mp_result_struct;

typedef struct survive_optimizer {
	const survive_reproject_model_t *reprojectModel;

	SurviveObject **sos;
	survive_optimizer_measurement *measurements;
	size_t measurementsCnt;
	FLT current_bias;
	SurvivePose initialPose;

	FLT *parameters;
	struct mp_par_struct *parameters_info;

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
	} stats;

	void *user;
	void (*iteration_cb)(struct survive_optimizer *opt_ctx, int m, int n, FLT *p, FLT *deviates, FLT **derivs);
} survive_optimizer;

#define SURVIVE_OPTIMIZER_SETUP_BUFFERS(ctx, alloc_fn, ...)                                                            \
	{                                                                                                                  \
		size_t par_count = survive_optimizer_get_parameters_count(&(ctx));                                             \
		size_t sensor_cnt = 32;                                                                                        \
		void *param_buffer = alloc_fn(((ctx).parameters), par_count * sizeof(FLT));                                    \
		void *param_info_buffer = alloc_fn(((ctx).parameters_info), par_count * sizeof(struct mp_par_struct));         \
		void *measurement_buffer =                                                                                     \
			alloc_fn(((ctx).measurements), (ctx).poseLength * sizeof(survive_optimizer_measurement) * 2 * sensor_cnt * \
											   NUM_GEN2_LIGHTHOUSES);                                                  \
		void *sos_buffer = alloc_fn((ctx).sos, sizeof(SurviveObject *) * (ctx).poseLength);                            \
		survive_optimizer_setup_buffers(&(ctx), param_buffer, param_info_buffer, measurement_buffer, sos_buffer);      \
		SurviveObject *sos[] = {__VA_ARGS__};                                                                          \
		memcpy((ctx).sos, sos, sizeof(sos));                                                                           \
	}

#define SURVIVE_OPTIMIZER_ALLOCA(ctx, size) alloca(size)
#define SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(ctx, ...)                                                                \
	SURVIVE_OPTIMIZER_SETUP_BUFFERS(ctx, SURVIVE_OPTIMIZER_ALLOCA, __VA_ARGS__)
#define SURVIVE_OPTIMIZER_SETUP_HEAP_BUFFERS(ctx, ...)                                                                 \
	SURVIVE_OPTIMIZER_SETUP_BUFFERS(ctx, survive_optimizer_realloc, __VA_ARGS__)
#define SURVIVE_OPTIMIZER_CLEANUP_STACK_BUFFERS(ctx)
#define SURVIVE_OPTIMIZER_CLEANUP_HEAP_BUFFERS(ctx)                                                                    \
	{                                                                                                                  \
		free(ctx.parameters);                                                                                          \
		free(ctx.parameters_info);                                                                                     \
		free(ctx.measurements);                                                                                        \
	}

SURVIVE_EXPORT void *survive_optimizer_realloc(void *old_ptr, size_t size);

SURVIVE_EXPORT int survive_optimizer_get_parameters_count(const survive_optimizer *ctx);

SURVIVE_EXPORT size_t survive_optimizer_get_total_buffer_size(const survive_optimizer *ctx);

SURVIVE_EXPORT void survive_optimizer_setup_buffers(survive_optimizer *ctx, void *parameter_buffer,
													void *parameter_info_buffer, void *measurements_buffer,
													void *so_buffer);

SURVIVE_EXPORT SurvivePose *survive_optimizer_get_pose(survive_optimizer *ctx);

SURVIVE_EXPORT int survive_optimizer_get_camera_index(const survive_optimizer *ctx);

SURVIVE_EXPORT SurvivePose *survive_optimizer_get_camera(survive_optimizer *ctx);

SURVIVE_EXPORT int survive_optimizer_get_calibration_index(const survive_optimizer *ctx);

SURVIVE_EXPORT BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh);

SURVIVE_EXPORT int survive_optimizer_get_sensors_index(const survive_optimizer *ctx);

SURVIVE_EXPORT FLT *survive_optimizer_get_sensors(survive_optimizer *ctx, size_t idx);

SURVIVE_EXPORT void survive_optimizer_setup_pose_n(survive_optimizer *mpfit_ctx, const SurvivePose *pose, size_t n,
												   bool isFixed, int use_jacobian_function);

SURVIVE_EXPORT void survive_optimizer_fix_camera(survive_optimizer *mpfit_ctx, int cam_idx);

SURVIVE_EXPORT void survive_optimizer_setup_pose(survive_optimizer *mpfit_ctx, const SurvivePose *pose, bool isFixed,
												 int use_jacobian_function);

SURVIVE_EXPORT void survive_optimizer_setup_camera(survive_optimizer *mpfit_ctx, int8_t lh, const SurvivePose *pose,
												   bool isFixed, int use_jacobian_function);

SURVIVE_EXPORT void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed,
													int use_jacobian_function);

SURVIVE_EXPORT const char *survive_optimizer_error(int status);

SURVIVE_EXPORT int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result);

SURVIVE_EXPORT void survive_optimizer_set_reproject_model(survive_optimizer *optimizer,
														  const survive_reproject_model_t *reprojectModel);

SURVIVE_EXPORT void survive_optimizer_serialize(const survive_optimizer *optimizer, const char *fn);

SURVIVE_EXPORT survive_optimizer *survive_optimizer_load(const char *fn);

SURVIVE_EXPORT FLT survive_optimizer_current_norm(const survive_optimizer *optimizer);

SURVIVE_EXPORT mp_config *survive_optimizer_precise_config();

SURVIVE_EXPORT int survive_optimizer_nonfixed_cnt(const survive_optimizer *optimizer);

SURVIVE_EXPORT void survive_optimizer_get_nonfixed(const survive_optimizer *optimizer, FLT *params);
SURVIVE_EXPORT void survive_optimizer_set_nonfixed(survive_optimizer *optimizer, FLT *params);

#ifdef __cplusplus
}
#endif
