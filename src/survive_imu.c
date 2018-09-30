#include "survive_imu.h"
#include "linmath.h"
#include "math.h"
#include "survive_imu.h"
#include "survive_internal.h"
#include <assert.h>
#include <memory.h>

// Mahoney is due to https://hal.archives-ouvertes.fr/hal-00488376/document
// See also http://www.olliw.eu/2013/imu-data-fusing/#chapter41 and
// http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
static void mahony_ahrs(SurviveIMUTracker *tracker, LinmathVec3d _gyro, LinmathVec3d _accel) {
	LinmathVec3d gyro;
	memcpy(gyro, _gyro, 3 * sizeof(FLT));

	LinmathVec3d accel;
	memcpy(accel, _accel, 3 * sizeof(FLT));

	const FLT sample_f = tracker->so->imu_freq;
	const FLT prop_gain = .5;
	const FLT int_gain = 0;

	FLT *q = tracker->pose.Rot;

	FLT mag_accel = magnitude3d(accel);

	if (mag_accel != 0.0) {
		scale3d(accel, accel, 1. / mag_accel);

		// Equiv of q^-1 * G
		LinmathVec3d v = {q[1] * q[3] - q[0] * q[2], q[0] * q[1] + q[2] * q[3], q[0] * q[0] - 0.5 + q[3] * q[3]};

		LinmathVec3d error;
		cross3d(error, accel, v);

		if (int_gain > 0.0f) {
			LinmathVec3d fb_correction;
			scale3d(fb_correction, error, int_gain * 2. / sample_f);
			add3d(tracker->integralFB, tracker->integralFB, fb_correction);
			add3d(gyro, gyro, tracker->integralFB);
		}

		scale3d(error, error, prop_gain * 2.);
		add3d(gyro, gyro, error);
	}

	scale3d(gyro, gyro, 0.5 / sample_f);

	LinmathQuat correction = {
		(-q[1] * gyro[0] - q[2] * gyro[1] - q[3] * gyro[2]), (+q[0] * gyro[0] + q[2] * gyro[2] - q[3] * gyro[1]),
		(+q[0] * gyro[1] - q[1] * gyro[2] + q[3] * gyro[0]), (+q[0] * gyro[2] + q[1] * gyro[1] - q[2] * gyro[0])};

	quatadd(q, q, correction);
	quatnormalize(q, q);
}

static const int imu_calibration_iterations = 100;

static void RotateAccel(LinmathVec3d rAcc, const SurvivePose *pose, const LinmathVec3d accel) {
	quatrotatevector(rAcc, pose->Rot, accel);
	LinmathVec3d G = {0, 0, -1};
	add3d(rAcc, rAcc, G);
	scale3d(rAcc, rAcc, 9.8066);
	FLT m = magnitude3d(rAcc);
}
static void iterate_position(SurviveIMUTracker *tracker, double time_diff, const PoserDataIMU *pIMU, FLT *out) {
	const SurvivePose *pose = &tracker->pose;
	const FLT *vel = tracker->current_velocity.Pos;

	for (int i = 0; i < 3; i++)
		out[i] = pose->Pos[i];

	FLT acc_mul = time_diff * time_diff / 2;

	LinmathVec3d acc;
	scale3d(acc, pIMU->accel, tracker->accel_scale_bias);
	LinmathVec3d rAcc = {0};
	RotateAccel(rAcc, pose, acc);
	scale3d(rAcc, rAcc, acc_mul);

	for (int i = 0; i < 3; i++) {
		out[i] += time_diff * vel[i] + rAcc[i];
	}
}

static void iterate_velocity(LinmathVec3d result, SurviveIMUTracker *tracker, double time_diff, PoserDataIMU *pIMU) {
	const SurvivePose *pose = &tracker->pose;
	const FLT *vel = tracker->current_velocity.Pos;
	scale3d(result, vel, 1.);

	LinmathVec3d acc;
	scale3d(acc, pIMU->accel, tracker->accel_scale_bias);

	LinmathVec3d rAcc = {0};
	RotateAccel(rAcc, pose, acc);
	scale3d(rAcc, rAcc, time_diff);
	add3d(result, result, rAcc);
}

void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data) {
	if (!tracker->is_initialized) {
		if (tracker->last_data.datamask == imu_calibration_iterations) {
			tracker->last_data = *data;

			const FLT up[3] = {0, 0, 1};
			quatfrom2vectors(tracker->pose.Rot, tracker->updir, up);
			tracker->accel_scale_bias = 1. / magnitude3d(tracker->updir);
			tracker->is_initialized = true;

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

	mahony_ahrs(tracker, data->gyro, data->accel);

	FLT time_diff =
		survive_timecode_difference(data->timecode, tracker->last_data.timecode) / (FLT)tracker->so->timebase_hz;

	if (survive_timecode_difference(data->timecode, tracker->lastGTTime) < 3200000 * 3 && false) {
		FLT next[3];
		iterate_position(tracker, time_diff, data, next);

		LinmathVec3d v_next;
		iterate_velocity(v_next, tracker, time_diff, data);

		scale3d(tracker->current_velocity.Pos, v_next, 1);
		scale3d(tracker->pose.Pos, next, 1);
	}

	FLT var_meters = .000001;
	FLT var_quat = .05;

	// Note that this implementation is somewhat truncated. Instead of modeling velocity and velocities
	// covariance with position explicitly, we just square the variance for the position indexes. This
	// gives more or less the same calculation without having to do matrix multiplication.

	/*
	if (tracker->P.Pose < 1e100)
		tracker->P.Pose = tracker->P.Pose * tracker->P.Pose + var_meters;

	if (tracker->P.Rot < 1e100)
		tracker->P.Rot = tracker->P.Rot + var_quat;
*/
	tracker->last_data = *data;
}

void survive_update_variances(SurviveIMUTracker *tracker, uint32_t timecode) {
	if (quatiszero(tracker->lastGT.Rot))
		return;

	FLT time_diff = survive_timecode_difference(timecode, tracker->lastGTTime) / (FLT)tracker->so->timebase_hz;
	assert(time_diff < 1.0);
	FLT var_meters = .1;
	FLT var_quat = .5;

	tracker->Pv.Pose += var_meters * time_diff;
	tracker->Pv.Rot += var_quat * time_diff;

	tracker->P.Pose += tracker->Pv.Pose * time_diff;
	tracker->P.Rot += tracker->Pv.Rot * time_diff;

	survive_imu_tracker_predict(tracker, timecode, &tracker->pose);
}

void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_timecode timecode, SurvivePose *out) {
	if (quatiszero(tracker->lastGT.Rot))
		return;

	*out = tracker->lastGT;

	FLT time_diff = survive_timecode_difference(timecode, tracker->lastGTTime) / (FLT)tracker->so->timebase_hz;
	// assert(time_diff < 1.0);

	for (int i = 0; i < 3; i++)
		out->Pos[i] += tracker->current_velocity.Pos[i] * time_diff;

	LinmathQuat rot_change;
	quatmultiplyrotation(rot_change, tracker->current_velocity.Rot, time_diff);
	quatrotateabout(out->Rot, rot_change, out->Rot);
}

void survive_imu_tracker_integrate_velocity(SurviveIMUTracker *tracker, const SurvivePose *pose, const FLT *Rv) {
	FLT combined_variance[2] = {
		Rv[0] + tracker->Pv.Pose,
		Rv[1] + tracker->Pv.Rot,
	};

	FLT incoming_pose_weight[2] = {
		combined_variance[0] == 0 ? 1. : tracker->Pv.Pose / combined_variance[0],
		combined_variance[1] == 0 ? 1. : tracker->Pv.Rot / combined_variance[1],
	};

	for (int i = 0; i < 3; i++)
		tracker->current_velocity.Pos[i] += incoming_pose_weight[0] * (pose->Pos[i] - tracker->current_velocity.Pos[i]);

	quatslerp(tracker->current_velocity.Rot, tracker->current_velocity.Rot, pose->Rot, incoming_pose_weight[1]);

	tracker->Pv.Pose *= (1. - incoming_pose_weight[0]);
	tracker->Pv.Rot *= (1. - incoming_pose_weight[1]);
}

void survive_imu_tracker_integrate_observation(uint32_t timecode, SurviveIMUTracker *tracker, const SurvivePose *pose,
											   const FLT *R) {
	survive_update_variances(tracker, timecode);

	// Kalman filter assuming:
	// F -> Identity
	// H -> Identity
	// Q / R / P -> Diagonal matrices; just treat them as such. This assumption might need some checking but it
	// makes the # of calculations needed much smaller so we may be willing to tolerate some approximation here

	FLT combined_variance[2] = {
		R[0] + tracker->P.Pose,
		R[1] + tracker->P.Rot,
	};

	// If the tracker pose variance accounts for almost all the combined variance, this will be close to 1. If
	// it accounts for very little, it'll be close to 0.
	FLT incoming_pose_weight[2] = {
		combined_variance[0] == 0 ? 1. : tracker->P.Pose / combined_variance[0],
		combined_variance[1] == 0 ? 1. : tracker->P.Rot / combined_variance[1],
	};

	for (int i = 0; i < 3; i++)
		tracker->pose.Pos[i] += incoming_pose_weight[0] * (pose->Pos[i] - tracker->pose.Pos[i]);

	quatslerp(tracker->pose.Rot, tracker->pose.Rot, pose->Rot, incoming_pose_weight[1]);

	tracker->P.Pose *= (1. - incoming_pose_weight[0]);
	tracker->P.Rot *= (1. - incoming_pose_weight[1]);

	FLT time_diff = survive_timecode_difference(timecode, tracker->lastGTTime) / (FLT)tracker->so->timebase_hz;

	if (!quatiszero(tracker->lastGT.Rot)) {
		SurvivePose velocity;
		quatfind(velocity.Rot, tracker->lastGT.Rot, tracker->pose.Rot);
		quatmultiplyrotation(velocity.Rot, velocity.Rot, 1. / time_diff);

		sub3d(velocity.Pos, tracker->pose.Pos, tracker->lastGT.Pos);
		scale3d(velocity.Pos, velocity.Pos, 1. / time_diff);

		SurvivePoseVariance vp = {.Pose = tracker->P.Pose + tracker->lastP.Pose,
								  .Rot = tracker->P.Rot + tracker->lastP.Rot};
		survive_imu_tracker_integrate_velocity(tracker, &velocity, &vp.Pose);
	}

	tracker->lastP = tracker->P;
	tracker->lastGTTime = timecode;
	tracker->lastGT = tracker->pose;
}

void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));
	tracker->current_velocity.Rot[0] = tracker->pose.Rot[0] = 1.;
	tracker->so = so;

	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.
	tracker->P.Pose = 1000;
	tracker->P.Rot = 1000;

	tracker->Pv = tracker->P;
}
