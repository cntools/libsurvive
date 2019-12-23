#pragma once

#include "math.h"
#include "string.h"
#include "survive.h"
#include "survive_reproject.h"
#include <mpfit/mpfit.h>

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
} survive_optimizer_measurement;

struct mp_par_struct;
struct mp_result_struct;

typedef struct {
	const survive_reproject_model_t *reprojectModel;

	SurviveObject *so;
	survive_optimizer_measurement *measurements;
	size_t measurementsCnt;
	FLT current_bias;
	SurvivePose initialPose;

	double *parameters;
	struct mp_par_struct *parameters_info;

	int poseLength;
	int cameraLength;
	int ptsLength;

	mp_config *cfg;
} survive_optimizer;

#define SURVIVE_OPTIMIZER_SETUP_BUFFERS(ctx, alloc_fn)                                                                 \
	{                                                                                                                  \
		size_t par_count = survive_optimizer_get_parameters_count(&(ctx));                                             \
		(ctx).parameters = (double *)alloc_fn(sizeof(double) * par_count);                                             \
		for (int i = 0; i < par_count; i++)                                                                            \
			(ctx).parameters[i] = NAN;                                                                                 \
		(ctx).parameters_info = (struct mp_par_struct *)alloc_fn(sizeof(struct mp_par_struct) * par_count);            \
		size_t sensor_cnt = (ctx).so ? (ctx).so->sensor_ct : 32;                                                       \
		(ctx).measurements = (survive_optimizer_measurement *)alloc_fn(                                                \
			(ctx).poseLength * sizeof(survive_optimizer_measurement) * 2 * sensor_cnt * NUM_GEN2_LIGHTHOUSES);         \
		memset((ctx).parameters_info, 0, sizeof(mp_par) * par_count);                                                  \
		for (int i = 0; i < survive_optimizer_get_parameters_count(&(ctx)); i++) {                                     \
			(ctx).parameters_info[i].fixed = 1;                                                                        \
		}                                                                                                              \
	}

#define SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(ctx) SURVIVE_OPTIMIZER_SETUP_BUFFERS(ctx, alloca)
#define SURVIVE_OPTIMIZER_SETUP_HEAP_BUFFERS(ctx) SURVIVE_OPTIMIZER_SETUP_BUFFERS(ctx, malloc)
#define SURVIVE_OPTIMIZER_CLEANUP_STACK_BUFFERS(ctx)
#define SURVIVE_OPTIMIZER_CLEANUP_HEAP_BUFFERS(ctx)                                                                    \
	{                                                                                                                  \
		free(ctx.parameters);                                                                                          \
		free(ctx.parameters_info);                                                                                     \
		free(ctx.measurements);                                                                                        \
	}

SURVIVE_EXPORT SurvivePose *survive_optimizer_get_pose(survive_optimizer *ctx);
SURVIVE_EXPORT int survive_optimizer_get_camera_index(const survive_optimizer *ctx);
SURVIVE_EXPORT SurvivePose *survive_optimizer_get_camera(survive_optimizer *ctx);

SURVIVE_EXPORT int survive_optimizer_get_calibration_index(const survive_optimizer *ctx);

SURVIVE_EXPORT BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh);

SURVIVE_EXPORT int survive_optimizer_get_sensors_index(const survive_optimizer *ctx);

SURVIVE_EXPORT double *survive_optimizer_get_sensors(survive_optimizer *ctx);

SURVIVE_EXPORT int survive_optimizer_get_parameters_count(const survive_optimizer *ctx);

SURVIVE_EXPORT void survive_optimizer_setup_pose(survive_optimizer *mpfit_ctx, const SurvivePose *pose, bool isFixed,
								  int use_jacobian_function);

SURVIVE_EXPORT void survive_optimizer_setup_camera(survive_optimizer *mpfit_ctx, int8_t lh, const SurvivePose *pose,
												   bool isFixed);
SURVIVE_EXPORT void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed);

SURVIVE_EXPORT const char *survive_optimizer_error(int status);

SURVIVE_EXPORT int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result);

SURVIVE_EXPORT void survive_optimizer_set_reproject_model(survive_optimizer *optimizer,
														  const survive_reproject_model_t *reprojectModel);
SURVIVE_EXPORT void survive_optimizer_serialize(const survive_optimizer *optimizer, const char *fn);

SURVIVE_EXPORT survive_optimizer *survive_optimizer_load(const char *fn);

SURVIVE_EXPORT FLT survive_optimizer_current_norm(const survive_optimizer *optimizer);
SURVIVE_EXPORT mp_config *survive_optimizer_precise_config();
SURVIVE_EXPORT int survive_optimizer_nonfixed_cnt(const survive_optimizer *optimizer);
SURVIVE_EXPORT void survive_optimizer_get_nonfixed(const survive_optimizer *optimizer, double *params);
SURVIVE_EXPORT void survive_optimizer_set_nonfixed(survive_optimizer *optimizer, double *params);
#ifdef __cplusplus
}
#endif
