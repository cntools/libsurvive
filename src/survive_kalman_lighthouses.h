#pragma once
#include "survive_kalman.h"
#include "survive_kalman_tracker.h"

typedef struct SurviveKalmanLighthouse {
	SurvivePose state;

	survive_kalman_state_t model;
	survive_kalman_meas_model_t lightcap_model;

	SurviveContext *ctx;
	int lh;

	FLT process_weight_pos;
	FLT process_weight_rotation;

	FLT light_variance;
	FLT light_stationary_mintime;
	FLT light_stationary_maxtime;

} SurviveKalmanLighthouse;

SURVIVE_EXPORT void survive_kalman_lighthouse_integrate_light(SurviveKalmanLighthouse *tracker, SurviveObject *so,
															  PoserDataLight *data);
SURVIVE_EXPORT void survive_kalman_lighthouse_init(SurviveKalmanLighthouse *tracker, SurviveContext *ctx, int lh);
SURVIVE_EXPORT void survive_kalman_lighthouse_free(SurviveKalmanLighthouse *tracker);
SURVIVE_EXPORT void survive_kalman_lighthouse_integrate_observation(SurviveKalmanLighthouse *tracker,
																	const SurvivePose *pose, const FLT *variance);
