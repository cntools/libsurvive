#ifndef _SURVIVE_IMU_H
#define _SURVIVE_IMU_H

#include "poser.h"
#include "survive.h"
#include "survive_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SurviveIMUTracker_p;

typedef struct SurvivePoseVariance {
	FLT Pose;
	FLT Rot;
} SurvivePoseVariance;

typedef struct {
	SurviveObject *so;
	bool is_initialized;

	FLT updir[3];
	FLT accel_scale_bias;

	SurvivePose current_velocity; // Velocity in world frame
	SurvivePoseVariance Pv;

	PoserDataIMU last_data;

	SurvivePose pose;
	SurvivePoseVariance P;

	SurvivePoseVariance lastP;
	SurvivePose lastGT;
	uint32_t lastGTTime;
	LinmathVec3d integralFB;

} SurviveIMUTracker;

SURVIVE_EXPORT void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_timecode timecode,
												SurvivePose *out);
SURVIVE_EXPORT void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so);
SURVIVE_EXPORT void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data);
SURVIVE_EXPORT void survive_imu_tracker_integrate_observation(uint32_t timecode, SurviveIMUTracker *tracker,
															  const SurvivePose *pose, const FLT *variance);

#ifdef __cplusplus
};
#endif

#endif
