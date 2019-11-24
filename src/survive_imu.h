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

typedef struct SurvivePoseVariance {
	FLT Pose;
	FLT Rot;
} SurvivePoseVariance;

struct kalman_info_t;

typedef void (*survive_kalman_update_f)(struct SurviveIMUTracker *, survive_timecode timecode,
										struct kalman_info_t *info);

struct kalman_info_t {
	survive_timecode last_update;
	FLT variance;
	FLT variance_per_second;

	survive_kalman_update_f update_fn;
};

struct kalman_info_rotation_t {
	struct kalman_info_t info;
	LinmathQuat v;
};

struct kalman_info_position_t {
	struct kalman_info_t info;
	LinmathVec3d v;
};

struct kalman_info_axis_angle_t {
	struct kalman_info_t info;
	SurviveAngularVelocity v;
};

typedef struct kalman_info_pose_axis_angle_t {
	struct kalman_info_position_t Pos;
	struct kalman_info_axis_angle_t AxisAngleRot;
} kalman_info_pose_axis_angle_t;

typedef struct kalman_info_pose_t {
	struct kalman_info_position_t Pos;
	struct kalman_info_rotation_t Rot;
} kalman_info_pose_t;

typedef struct SurviveIMUTracker {
	SurviveObject *so;

	FLT acc_var;
	FLT gyro_var;

	FLT mahony_variance;

	survive_timecode last_kalman_update;
	survive_timecode imu_kalman_update;
	survive_timecode obs_kalman_update;
	survive_kalman_state_t position;
	FLT pos_Q_per_sec[9];

	survive_kalman_state_t rot;
	FLT rot_Q_per_sec[4];

	PoserDataIMU last_data;

	LinmathVec3d integralFB;

} SurviveIMUTracker;

SURVIVE_EXPORT SurviveVelocity survive_imu_velocity(const SurviveIMUTracker *tracker);
SURVIVE_EXPORT void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_timecode timecode,
												SurvivePose *out);
SURVIVE_EXPORT void survive_imu_tracker_update(SurviveIMUTracker *tracker, survive_timecode timecode, SurvivePose *out);
SURVIVE_EXPORT void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so);
SURVIVE_EXPORT void survive_imu_tracker_free(SurviveIMUTracker *tracker);
SURVIVE_EXPORT void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data);
SURVIVE_EXPORT void survive_imu_tracker_integrate_observation(uint32_t timecode, SurviveIMUTracker *tracker,
															  const SurvivePose *pose, const FLT *variance);
SURVIVE_EXPORT void survive_imu_tracker_integrate_velocity(SurviveIMUTracker *tracker, survive_timecode timecode,
														   const FLT *Rv, const SurviveVelocity *vel);
#ifdef __cplusplus
};
#endif

#endif
