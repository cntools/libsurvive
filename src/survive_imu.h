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

	// Laid out as a 3 x 3
	// | position |
	// | velocity |
	// | accel    |
	survive_kalman_state_t position;
	FLT pos_Q_per_sec[81];

	// Laid out as a 2 x 4
	// | rotation   | <-- quat
	// | angular, 0 | <-- axis / angle with 0 padding
	survive_kalman_state_t rot;
	FLT rot_Q_per_sec[49];

	PoserDataIMU last_data;

	LinmathVec3d integralFB;

	LinmathVec3d world_up_while_still;
	int up_while_still_cnt;
	LinmathQuat imuerror_correction_local;
} SurviveIMUTracker;

SURVIVE_EXPORT SurviveVelocity survive_imu_velocity(const SurviveIMUTracker *tracker);
SURVIVE_EXPORT void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_long_timecode timecode,
												SurvivePose *out);
SURVIVE_EXPORT void survive_imu_tracker_update(SurviveIMUTracker *tracker, survive_long_timecode timecode,
											   SurvivePose *out);
SURVIVE_EXPORT void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so);
SURVIVE_EXPORT void survive_imu_tracker_free(SurviveIMUTracker *tracker);
SURVIVE_EXPORT void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data);
SURVIVE_EXPORT void survive_imu_integrate_rotation(SurviveIMUTracker *tracker, double time, const LinmathQuat rotation,
												   const double *R);

SURVIVE_EXPORT void survive_imu_integrate_velocity_acceleration(SurviveIMUTracker *tracker, double time,
																const FLT *velocity, const double *R);
SURVIVE_EXPORT void survive_imu_integrate_velocity(SurviveIMUTracker *tracker, double time, const LinmathVec3d velocity,
												   const double *R);
SURVIVE_EXPORT void survive_imu_integrate_acceleration(SurviveIMUTracker *tracker, double time,
													   const LinmathVec3d accel, const double *R);
SURVIVE_EXPORT void survive_imu_integrate_rotation_angular_velocity(SurviveIMUTracker *tracker, double time,
																	const FLT *rotation_angular_velocity,
																	const double *R);
SURVIVE_EXPORT void survive_imu_integrate_angular_velocity(SurviveIMUTracker *tracker, double time,
														   const LinmathAxisAngle angular_velocity, const double *R);

SURVIVE_EXPORT void survive_imu_tracker_integrate_observation(survive_long_timecode timecode,
															  SurviveIMUTracker *tracker, const SurvivePose *pose,
															  const FLT *variance);
#ifdef __cplusplus
};
#endif

#endif
