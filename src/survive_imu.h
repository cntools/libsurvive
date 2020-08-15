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

struct SurviveIMUTracker;

typedef struct SurviveIMUTracker {
	SurviveObject *so;

	FLT acc_var;
	FLT gyro_var;
	FLT light_pos_var;
	FLT light_rot_var;
	FLT light_var;
	FLT vel_var;
	FLT pos_var;

	FLT position_process_weight;
	FLT rotation_process_weight;

	// Kalman state is layed out as SurviveKalmanModel
	SurviveKalmanModel state;
	survive_kalman_state_t model;
	// FLT Q_per_sec[16 * 16];

	PoserDataIMU last_data;

	struct {
		uint32_t late_imu_dropped;
		uint32_t late_light_dropped;

		FLT imu_total_error;
		size_t imu_count;
		FLT lightcap_total_error;
		size_t lightcap_count;
		FLT obs_total_error;
		size_t obs_count;
	} stats;

	FLT Obs_R[7 * 7];
	FLT IMU_R[6 * 6];
	FLT Lightcap_R;

} SurviveIMUTracker;

SURVIVE_EXPORT SurviveVelocity survive_imu_velocity(const SurviveIMUTracker *tracker);
SURVIVE_EXPORT void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, FLT time, SurvivePose *out);
SURVIVE_EXPORT void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so);
SURVIVE_EXPORT void survive_imu_tracker_free(SurviveIMUTracker *tracker);
SURVIVE_EXPORT void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data);
SURVIVE_EXPORT void survive_imu_tracker_integrate_light(SurviveIMUTracker *tracker, PoserDataLight *data);

SURVIVE_EXPORT void survive_imu_integrate_velocity_acceleration(SurviveIMUTracker *tracker, FLT time,
																const FLT *velocity, const FLT *R);
SURVIVE_EXPORT void survive_imu_integrate_velocity_angular_velocity_acceleration(SurviveIMUTracker *tracker, FLT time,
																				 const FLT *vel_aa_acc, const FLT *R);

SURVIVE_EXPORT void survive_imu_tracker_integrate_observation(PoserData *pd, SurviveIMUTracker *tracker,
															  const SurvivePose *pose, const FLT *variance);
SURVIVE_EXPORT void survive_imu_tracker_report_state(PoserData *pd, SurviveIMUTracker *tracker);

#ifdef __cplusplus
};
#endif

#endif
