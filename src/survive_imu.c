#include "survive_imu.h"
#include "linmath.h"
#include "survive_internal.h"
#include <survive_imu.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq 240.0f	  // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f) // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f) // 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//---------------------------------------------------------------------------------------------------
// IMU algorithm update
// From http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
static void MahonyAHRSupdateIMU(SurviveIMUTracker *tracker, float gx, float gy, float gz, float ax, float ay,
								float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	const float twoKp = twoKpDef; // 2 * proportional gain (Kp)
	const float twoKi = twoKiDef; // 2 * integral gain (Ki)

	float q0 = tracker->pose.Rot[0];
	float q1 = tracker->pose.Rot[1];
	float q2 = tracker->pose.Rot[2];
	float q3 = tracker->pose.Rot[3];

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if (twoKi > 0.0f) {
			tracker->integralFBx += twoKi * halfex * (1.0f / sampleFreq); // tracker->integral error scaled by Ki
			tracker->integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			tracker->integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += tracker->integralFBx; // apply tracker->integral feedback
			gy += tracker->integralFBy;
			gz += tracker->integralFBz;
		} else {
			tracker->integralFBx = 0.0f; // prevent tracker->integral windup
			tracker->integralFBy = 0.0f;
			tracker->integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	tracker->pose.Rot[0] = q0;
	tracker->pose.Rot[1] = q1;
	tracker->pose.Rot[2] = q2;
	tracker->pose.Rot[3] = q3;
}

static inline uint32_t tick_difference(uint32_t most_recent, uint32_t least_recent) {
	uint32_t diff = 0;
	if (most_recent > least_recent) {
		diff = most_recent - least_recent;
	} else {
		diff = least_recent - most_recent;
	}

	if (diff > 0xFFFFFFFF / 2)
		return 0x7FFFFFFF / 2 - diff;
	return diff;
}

void survive_imu_tracker_set_pose(SurviveIMUTracker *tracker, uint32_t timecode, SurvivePose *pose) {
	tracker->pose = *pose;

	for (int i = 0; i < 3; i++) {
		tracker->current_velocity[i] = 0;
	}
	//(pose->Pos[i] - tracker->lastGT.Pos[i]) / tick_difference(timecode, tracker->lastGTTime) * 48000000.;

	tracker->integralFBx = tracker->integralFBy = tracker->integralFBz = 0.0;
	tracker->lastGTTime = timecode;
	tracker->lastGT = *pose;
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
	const FLT *vel = tracker->current_velocity;

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
	const FLT *vel = tracker->current_velocity;
	scale3d(result, vel, 1.);

	LinmathVec3d acc;
	scale3d(acc, pIMU->accel, tracker->accel_scale_bias);

	LinmathVec3d rAcc = {0};
	RotateAccel(rAcc, pose, acc);
	scale3d(rAcc, rAcc, time_diff);
	add3d(result, result, rAcc);
}

void survive_imu_tracker_integrate(SurviveObject *so, SurviveIMUTracker *tracker, PoserDataIMU *data) {
	if (!tracker->is_initialized) {
		tracker->pose.Rot[0] = 1.;
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

	float gx = data->gyro[0], gy = data->gyro[1], gz = data->gyro[2];
	float ax = data->accel[0], ay = data->accel[1], az = data->accel[2];

	MahonyAHRSupdateIMU(tracker, gx, gy, gz, ax, ay, az);

	FLT time_diff = tick_difference(data->timecode, tracker->last_data.timecode) / (FLT)so->timebase_hz;

	if (tick_difference(data->timecode, tracker->lastGTTime) < 3200000 * 3) {
		FLT next[3];
		iterate_position(tracker, time_diff, data, next);

		LinmathVec3d v_next;
		iterate_velocity(v_next, tracker, time_diff, data);

		scale3d(tracker->current_velocity, v_next, 1);
		scale3d(tracker->pose.Pos, next, 1);
	}

	FLT var_meters = .000001;
	FLT var_quat = .05;
	const FLT Q[7] = {var_meters, var_meters, var_meters, var_quat, var_quat, var_quat, var_quat};

	// Note that this implementation is somewhat truncated. Instead of modeling velocity and velocities
	// covariance with position explicitly, we just square the variance for the position indexes. This
	// gives more or less the same calculation without having to do matrix multiplication.
	for (int i = 0; i < 3; i++)
		tracker->P[i] = tracker->P[i] * tracker->P[i] + Q[i];
	for (int i = 3; i < 7; i++)
		tracker->P[i] += Q[i];

	tracker->last_data = *data;
}

void survive_imu_tracker_integrate_observation(SurviveObject *so, uint32_t timecode, SurviveIMUTracker *tracker,
											   SurvivePose *pose, const FLT *R) {
	if (!tracker->is_initialized) {
		tracker->pose = *pose;
		return;
	}
		
	// Kalman filter assuming:
	// F -> Identity
	// H -> Identity
	// Q / R / P -> Diagonal matrices; just treat them as such. This assumption might need some checking but it
	// makes the # of calculations needed much smaller so we may be willing to tolerate some approximation here

	FLT *xhat = &tracker->pose.Pos[0];
	FLT *zk = &pose->Pos[0];

	FLT yk[7];
	for (int i = 0; i < 7; i++)
		yk[i] = zk[i] - xhat[i];

	FLT sk[7];
	for (int i = 0; i < 7; i++)
		sk[i] = R[i] + tracker->P[i];

	FLT K[7];
	for (int i = 0; i < 7; i++)
		K[i] = tracker->P[i] / sk[i];

	for (int i = 0; i < 7; i++)
		xhat[i] += K[i] * yk[i];
	for (int i = 0; i < 7; i++)
		tracker->P[i] *= (1. - K[i]);

	FLT time_diff = tick_difference(timecode, tracker->lastGTTime) / (FLT)so->timebase_hz;
	for (int i = 0; i < 3; i++)
		tracker->current_velocity[i] = 0.5 * (tracker->pose.Pos[i] - tracker->lastGT.Pos[i]) / time_diff;

	tracker->lastGTTime = timecode;
	tracker->lastGT = tracker->pose;
}