#pragma once
#include "survive.h"

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
	SurviveObject *so;
	survive_optimizer_measurement *measurements;
	size_t measurementsCnt;
	FLT current_bias;
	SurvivePose initialPose;

	double *parameters;
	struct mp_par_struct *parameters_info;

	int poseLength;
	int cameraLength;
	int fcalLength;
	int ptsLength;
} survive_optimizer;

#define SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(ctx)                                                                     \
	ctx.parameters = alloca(sizeof(double) * survive_optimizer_get_parameters_count(&ctx));                            \
	ctx.parameters_info = alloca(sizeof(mp_par) * survive_optimizer_get_parameters_count(&ctx));                       \
	ctx.measurements = alloca(sizeof(survive_optimizer_measurement) * 2 * ctx.so->sensor_ct * NUM_GEN1_LIGHTHOUSES);   \
	memset(ctx.parameters_info, 0, sizeof(mp_par) * survive_optimizer_get_parameters_count(&ctx));                     \
	for (int i = 0; i < survive_optimizer_get_parameters_count(&ctx); i++) {                                           \
		ctx.parameters_info[i].fixed = 1;                                                                              \
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
SURVIVE_EXPORT void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed);

SURVIVE_EXPORT int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result);

#ifdef __cplusplus
}
#endif
