#include "survive_imu.h"
#include "linmath.h"
#include "survive_internal.h"
#include <survive_imu.h>

void survive_imu_tracker_set_pose(SurviveIMUTracker *tracker, uint32_t timecode, SurvivePose *pose) {
	tracker->pose = *pose;

	for (int i = 0; i < 3; i++)
		tracker->current_velocity[i] =
			(pose->Pos[i] - tracker->lastGT.Pos[i]) / (timecode - tracker->lastGTTime) * 48000000.;

	tracker->lastGTTime = timecode;
	tracker->lastGT = *pose;
}

static const int imu_calibration_iterations = 100;

static void RotateAccel(LinmathVec3d rAcc, const SurvivePose *pose, const LinmathVec3d accel) {
	quatrotatevector(rAcc, pose->Rot, accel);
	LinmathVec3d G = {0, 0, -1};
	add3d(rAcc, rAcc, G);
	scale3d(rAcc, rAcc, 9.8);
}
static SurvivePose iterate_position(const SurvivePose *pose, const LinmathVec3d vel, double time_diff,
									const PoserDataIMU *pIMU) {
	SurvivePose result = *pose;

	FLT acc_mul = time_diff * time_diff / 2;
	LinmathVec3d rAcc = {0};
	RotateAccel(rAcc, pose, pIMU->accel);

	fprintf(stderr, "r %f %f %f %f\n", pIMU->accel[0], pIMU->accel[1], pIMU->accel[2], quatmagnitude(pIMU->accel));
	fprintf(stderr, "i %f %f %f %f\n", rAcc[0], rAcc[1], rAcc[2], quatmagnitude(rAcc));

	scale3d(rAcc, rAcc, acc_mul);

	LinmathVec3d gyro;

	for (int i = 0; i < 3; i++) {
		result.Pos[i] += time_diff * vel[i] + rAcc[i];
		gyro[i] = time_diff / 2 * pIMU->gyro[i];
	}

	LinmathEulerAngle curr, next;
	quattoeuler(curr, pose->Rot);
	add3d(next, curr, gyro);
	quatfromeuler(result.Rot, next);

	return result;
}

static void iterate_velocity(LinmathVec3d result, const SurvivePose *pose, const LinmathVec3d vel, double time_diff,
							 PoserDataIMU *pIMU) {
	scale3d(result, vel, 1.);
	LinmathVec3d rAcc = {0};
	RotateAccel(rAcc, pose, pIMU->accel);
	scale3d(rAcc, rAcc, time_diff);
	add3d(result, result, rAcc);
}

void survive_imu_tracker_integrate(SurviveObject *so, SurviveIMUTracker *tracker, PoserDataIMU *data) {
	if (tracker->last_data.timecode == 0) {
		if (tracker->last_data.datamask == imu_calibration_iterations) {
			tracker->last_data = *data;
			tracker->pose.Rot[0] = 1.;

			const FLT up[3] = {0, 0, 1};
			quatfrom2vectors(tracker->pose.Rot, tracker->updir, up);
			return;
		}

		tracker->last_data.datamask++;

		tracker->updir[0] += data->accel[0] / imu_calibration_iterations;
		tracker->updir[1] += data->accel[1] / imu_calibration_iterations;
		tracker->updir[2] += data->accel[2] / imu_calibration_iterations;
		return;
	}

	for (int i = 0; i < 3; i++) {
		tracker->updir[i] = data->accel[i] * .10 + tracker->updir[i] * .90;
	}

	const FLT up[3] = {0, 0, 1};
	LinmathQuat upRot, wouldbeUp;
	LinmathVec3d rup;
	quatrotatevector(rup, tracker->pose.Rot, up);
	quatfrom2vectors(upRot, rup, data->accel);

	quatrotateabout(wouldbeUp, upRot, tracker->pose.Rot);
	quatslerp(tracker->pose.Rot, tracker->pose.Rot, wouldbeUp, .1);

	FLT pose_up[3] = {0, 0, 1};
	quatrotatevector(pose_up, tracker->pose.Rot, tracker->updir);

	FLT time_diff = (data->timecode - tracker->last_data.timecode) / (FLT)so->timebase_hz;

	SurvivePose t_next = iterate_position(&tracker->pose, tracker->current_velocity, time_diff, data);

	LinmathVec3d v_next;
	iterate_velocity(v_next, &tracker->pose, tracker->current_velocity, time_diff, data);

	tracker->pose = t_next;

	fprintf(stderr, "%f %f %f\n", tracker->current_velocity[0], tracker->current_velocity[1],
			tracker->current_velocity[2]);

	scale3d(tracker->current_velocity, v_next, 1);

	tracker->last_data = *data;

	FLT tmp[3];
	ApplyPoseToPoint(tmp, &tracker->pose, up);

	printf("[%f, %f, %f] [%f, %f, %f]\n", tracker->pose.Pos[0], tracker->pose.Pos[1], tracker->pose.Pos[2], tmp[0],
		   tmp[1], tmp[2]);
}
