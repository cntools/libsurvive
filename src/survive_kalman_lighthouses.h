#pragma once
#include "survive_kalman_tracker.h"
#include <cnkalman/kalman.h>

typedef struct SurviveKalmanLighthouse {
	SurviveLighthouseKalmanModel state, push_state;
	CnMat push_cov;
	FLT push_cov_data[sizeof(SurviveLighthouseKalmanModel) / sizeof(FLT) * sizeof(SurviveLighthouseKalmanModel) /
					  sizeof(FLT)];

	cnkalman_state_t model, bsd_model;
	cnkalman_meas_model_t imu_model, obs_model;

	FLT base_variance, up_variance;
	BaseStationCal initial_variance;
	SurviveContext *ctx;
	int lh;

	int report_covariance_cnt;

	bool updating;

	FLT initial_pos_var, initial_rot_var;
	SurviveLighthouseKalmanErrorModel variance_per_sec;

	struct {
		int reported_poses;
	} stats;
} SurviveKalmanLighthouse;

SURVIVE_EXPORT void survive_kalman_lighthouse_init(SurviveKalmanLighthouse *tracker, SurviveContext *ctx, int lh);
SURVIVE_EXPORT void survive_kalman_lighthouse_ootx(SurviveKalmanLighthouse *tracker);
SURVIVE_EXPORT void survive_kalman_lighthouse_free(SurviveKalmanLighthouse *tracker);
SURVIVE_EXPORT void survive_kalman_lighthouse_integrate_observation(SurviveKalmanLighthouse *tracker,
																	const SurvivePose *pose, const CnMat *variance);
SURVIVE_EXPORT void survive_kalman_lighthouse_reset(SurviveKalmanLighthouse *tracker);
SURVIVE_EXPORT void survive_kalman_lighthouse_update_position(SurviveKalmanLighthouse *tracker,
															  const SurvivePose *pose);
SURVIVE_EXPORT void survive_kalman_lighthouse_report(SurviveKalmanLighthouse *tracker);