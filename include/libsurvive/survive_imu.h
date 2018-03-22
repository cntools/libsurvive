#ifndef _SURVIVE_IMU_H
#define _SURVIVE_IMU_H

#include "poser.h"
#include "survive_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	FLT updir[3];
	LinmathVec3d current_velocity; // Velocity in world frame
	PoserDataIMU last_data;
	SurvivePose pose;

	SurvivePose lastGT;
	uint32_t lastGTTime;
} SurviveIMUTracker;

void survive_imu_tracker_set_pose(SurviveIMUTracker *tracker, uint32_t timecode, SurvivePose *pose);
void survive_imu_tracker_integrate(SurviveObject *so, SurviveIMUTracker *tracker, PoserDataIMU *data);

#ifdef __cplusplus
};
#endif

#endif
