#ifndef _SURVIVE_IMU_H
#define _SURVIVE_IMU_H

#include "poser.h"
#include "survive.h"
#include "survive_kalman.h"
#include "survive_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * The kalman model as it pertains to LH tracking has a state space like so:
 *
 * @SurviveKalmanModel:
 * [ SurvivePose Pose, SurviveVelocity Vel, Point3d Acc, Point3d GyroBias ]
 *
 * for a 19 dimension state space.
 *
 * This model has three types of observations:
 * * IMU data - @survive_kalman_tracker_integrate_imu
 * * Raw light data - @survive_kalman_tracker_integrate_light
 * * Poser data - @survive_kalman_tracker_integrate_observation
 *
 * IMU data and raw light data can drift over time; but the poser data input is
 * assumed to be noisy but not drift in time.
 */
typedef struct SurviveKalmanTracker {
	SurviveObject *so;

	FLT acc_var;
	FLT gyro_var;
	FLT obs_pos_var;
	FLT obs_rot_var;
	FLT light_var;

	FLT last_light_time;

	bool use_raw_obs;
	int adaptive_imu, adaptive_lightcap, adaptive_obs;

	FLT light_threshold_var, report_threshold_var;
	int32_t light_required_obs;
	int32_t report_ignore_start;
	int32_t report_ignore_start_cnt;

	FLT process_weight_acc, process_weight_vel, process_weight_pos;
	FLT process_weight_ang_velocity, process_weight_rotation;

	// Kalman state is layed out as SurviveKalmanModel
	SurviveKalmanModel state;
	survive_kalman_state_t model;

	struct {
		uint32_t late_imu_dropped;
		uint32_t late_light_dropped;

		FLT imu_total_error;
		size_t imu_count;
		FLT lightcap_total_error;
		size_t lightcap_count;

		FLT lightcap_error_by_lh[NUM_GEN2_LIGHTHOUSES];
		size_t lightcap_count_by_lh[NUM_GEN2_LIGHTHOUSES];

		FLT obs_total_error;
		size_t obs_count;

		size_t reported_poses, dropped_poses;
		FLT dropped_var[19];
		FLT reported_var[19];
	} stats;

	FLT imu_residuals;
	FLT light_residuals_all;
	FLT light_residuals[NUM_GEN2_LIGHTHOUSES];

	FLT Obs_R[7 * 7];
	FLT IMU_R[6 * 6];
	FLT Lightcap_R;

} SurviveKalmanTracker;

SURVIVE_EXPORT SurviveVelocity survive_kalman_tracker_velocity(const SurviveKalmanTracker *tracker);
SURVIVE_EXPORT void survive_kalman_tracker_predict(const SurviveKalmanTracker *tracker, FLT time, SurvivePose *out);
SURVIVE_EXPORT void survive_kalman_tracker_init(SurviveKalmanTracker *tracker, SurviveObject *so);
SURVIVE_EXPORT void survive_kalman_tracker_free(SurviveKalmanTracker *tracker);
SURVIVE_EXPORT void survive_kalman_tracker_integrate_imu(SurviveKalmanTracker *tracker, PoserDataIMU *data);
SURVIVE_EXPORT void survive_kalman_tracker_integrate_light(SurviveKalmanTracker *tracker, PoserDataLight *data);

SURVIVE_EXPORT void survive_kalman_tracker_integrate_observation(PoserData *pd, SurviveKalmanTracker *tracker,
																 const SurvivePose *pose, const FLT *variance);
SURVIVE_EXPORT void survive_kalman_tracker_report_state(PoserData *pd, SurviveKalmanTracker *tracker);
SURVIVE_EXPORT void survive_kalman_tracker_lost_tracking(SurviveKalmanTracker *tracker);
#ifdef __cplusplus
};
#endif

#endif
