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

typedef struct {
	SurviveObject *so;
	bool is_initialized;

	FLT updir[3];
	FLT accel_scale_bias;

	LinmathVec3d current_velocity;	// Velocity in world frame
	PoserDataIMU last_data;
	SurvivePose pose;

	SurvivePose lastGT;
	uint32_t lastGTTime;

	struct {
		FLT Pose;
		FLT Rot;
	} P;

	LinmathVec3d integralFB;

} SurviveIMUTracker;

SURVIVE_EXPORT void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so);
SURVIVE_EXPORT void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data);
SURVIVE_EXPORT void survive_imu_tracker_integrate_observation(uint32_t timecode, SurviveIMUTracker *tracker, SurvivePose *pose,
											   const FLT *variance);

#ifdef __cplusplus
};
#endif

#endif
