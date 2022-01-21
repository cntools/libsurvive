#pragma once
#include "survive_kalman_tracker.h"
#include <cnkalman/kalman.h>

typedef struct SurviveKalmanLighthouse {
	SurvivePose state;

	cnkalman_state_t model;
	cnkalman_meas_model_t lightcap_model;

	SurviveContext *ctx;
	int lh;

	FLT process_weight_pos;
	FLT process_weight_rotation;
	int report_covariance_cnt;
	FLT light_variance;
	FLT light_stationary_mintime;
	FLT light_stationary_maxtime;

	bool updating;

	struct {
		int reported_poses;
	} stats;
} SurviveKalmanLighthouse;

SURVIVE_EXPORT void survive_kalman_lighthouse_integrate_light(SurviveKalmanLighthouse *tracker, SurviveObject *so,
															  PoserDataLight *data);
SURVIVE_EXPORT void survive_kalman_lighthouse_init(SurviveKalmanLighthouse *tracker, SurviveContext *ctx, int lh);
SURVIVE_EXPORT void survive_kalman_lighthouse_free(SurviveKalmanLighthouse *tracker);
SURVIVE_EXPORT void survive_kalman_lighthouse_integrate_observation(SurviveKalmanLighthouse *tracker,
																	const SurvivePose *pose, const FLT *variance);
SURVIVE_EXPORT void survive_kalman_lighthouse_update_position(SurviveKalmanLighthouse *tracker,
															  const SurvivePose *pose);