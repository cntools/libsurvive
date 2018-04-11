#ifndef _SURVIVE_IMU_H
#define _SURVIVE_IMU_H

#include "poser.h"
#include "survive_types.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SurviveIMUTracker_p;

typedef struct {
	bool is_initialized;

	FLT updir[3];
	FLT accel_scale_bias;

	LinmathVec3d current_velocity;	// Velocity in world frame
	PoserDataIMU last_data;
	SurvivePose pose;

	SurvivePose lastGT;
	uint32_t lastGTTime;

	FLT P[7]; // estimate variance

	LinmathVec3d integralFB;

} SurviveIMUTracker;

void survive_imu_tracker_set_pose(SurviveIMUTracker *tracker, uint32_t timecode, SurvivePose *pose);

void survive_imu_tracker_integrate(SurviveObject *so, SurviveIMUTracker *tracker, PoserDataIMU *data);
void survive_imu_tracker_integrate_observation(SurviveObject *so, uint32_t timecode, SurviveIMUTracker *tracker,
											   SurvivePose *pose, const FLT *variance);

#ifdef __cplusplus
};
#endif

#endif
