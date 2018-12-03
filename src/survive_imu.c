#include "survive_imu.h"
#include "linmath.h"
#include "math.h"
#include "survive_imu.h"
#include "survive_internal.h"
#include <assert.h>
#include <memory.h>

//#define SV_VERBOSE(...) SV_INFO(__VA_ARGS__)
#define SV_VERBOSE(...)                                                                                                \
	if (0)                                                                                                             \
	SV_INFO(__VA_ARGS__)

// Mahoney is due to https://hal.archives-ouvertes.fr/hal-00488376/document
// See also http://www.olliw.eu/2013/imu-data-fusing/#chapter41 and
// http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
static void mahony_ahrs(SurviveIMUTracker *tracker, LinmathQuat q, const LinmathVec3d _gyro,
						const LinmathVec3d _accel) {
	LinmathVec3d gyro;
	memcpy(gyro, _gyro, 3 * sizeof(FLT));

	LinmathVec3d accel;
	memcpy(accel, _accel, 3 * sizeof(FLT));

	const FLT sample_f = tracker->so->imu_freq;
	const FLT prop_gain = .5;
	const FLT int_gain = 0;

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

static void RotateAccel(LinmathVec3d rAcc, const LinmathQuat rot, const LinmathVec3d accel) {
	quatrotatevector(rAcc, rot, accel);
	LinmathVec3d G = {0, 0, -1};
	add3d(rAcc, rAcc, G);
	scale3d(rAcc, rAcc, 9.8066);
	FLT m = magnitude3d(rAcc);
}

static inline FLT survive_update_kalman_variance(const SurviveIMUTracker *tracker, struct kalman_info_t *info,
												 survive_timecode timecode, FLT new_variance) {
	FLT time_diff = survive_timecode_difference(timecode, info->last_update) / (FLT)tracker->so->timebase_hz;
	FLT variance = info->variance + info->variance_per_second * time_diff;
	FLT combined_variance = new_variance + variance;
	info->last_update = timecode;

	FLT incoming_weight = combined_variance == 0 ? 1. : variance / combined_variance;

	FLT huh = incoming_weight * variance;
	info->variance = (1. - incoming_weight) * variance;

	return incoming_weight;
}

static inline void survive_update_position(SurviveIMUTracker *tracker, struct kalman_info_position_t *pos,
										   survive_timecode timecode, FLT new_variance,
										   const LinmathVec3d new_position) {
	pos->info.update_fn(tracker, timecode, &pos->info);

	FLT incoming_pose_weight = survive_update_kalman_variance(tracker, &pos->info, timecode, new_variance);
	for (int i = 0; i < 3; i++) {
		pos->v[i] += incoming_pose_weight * (new_position[i] - pos->v[i]);
		assert(!isnan(pos->v[i]));
	}
	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE("PW: %f %f %f", incoming_pose_weight, norm3d(pos->v), norm3d(new_position));
}

static inline void survive_update_rotation(SurviveIMUTracker *tracker, struct kalman_info_rotation_t *rot,
										   survive_timecode timecode, FLT new_variance, const LinmathQuat new_rot) {
	if (quatiszero(rot->v)) {
		quatcopy(rot->v, new_rot);
		rot->info.variance = new_variance;
		rot->info.last_update = timecode;
		return;
	}

	rot->info.update_fn(tracker, timecode, &rot->info);

	FLT incoming_pose_weight = survive_update_kalman_variance(tracker, &rot->info, timecode, new_variance);
	SurviveContext *ctx = tracker->so->ctx;
	// SV_VERBOSE("UR: " Quat_format " --- " Quat_format ", %f", LINMATH_QUAT_EXPAND(rot->v),
	// LINMATH_QUAT_EXPAND(new_rot), incoming_pose_weight);
	quatslerp(rot->v, rot->v, new_rot, incoming_pose_weight);
}

static inline void survive_update_euler_rotation(SurviveIMUTracker *tracker, struct kalman_info_euler_t *rot,
												 survive_timecode timecode, FLT new_variance,
												 const LinmathEulerAngle new_rot) {
	rot->info.update_fn(tracker, timecode, &rot->info);
	survive_update_position(tracker, (struct kalman_info_position_t *)rot, timecode, new_variance, new_rot);
}

static inline void survive_update_pose(SurviveIMUTracker *tracker, struct kalman_info_pose_t *pose,
									   survive_timecode timecode, const FLT *new_variance,
									   const LinmathPose *new_pose) {
	survive_update_position(tracker, &pose->Pos, timecode, new_variance[0], new_pose->Pos);
	survive_update_rotation(tracker, &pose->Rot, timecode, new_variance[1], new_pose->Rot);
}
static inline void survive_update_pose_euler(SurviveIMUTracker *tracker, struct kalman_info_pose_euler_t *pose,
											 survive_timecode timecode, const FLT *new_variance,
											 const struct LinmathEulerPose *new_pose) {
	survive_update_position(tracker, &pose->Pos, timecode, new_variance[0], new_pose->Pos);
	survive_update_euler_rotation(tracker, &pose->EulerRot, timecode, new_variance[1], new_pose->EulerRot);
}

void survive_imu_tracker_integrate_rotation(SurviveIMUTracker *tracker, survive_timecode timecode,
											const LinmathQuat Rot, FLT R) {
	survive_update_rotation(tracker, &tracker->pose.Rot, timecode, R, Rot);
}

void survive_imu_tracker_integrate_angular_velocity(SurviveIMUTracker *tracker, survive_timecode timecode,
													const LinmathQuat Rot, FLT R) {
	survive_update_euler_rotation(tracker, &tracker->velocity.EulerRot, timecode, R, Rot);
}

void survive_imu_tracker_integrate_velocity(SurviveIMUTracker *tracker, survive_timecode timecode, const FLT *Rv,
											const SurviveVelocity *vel) {
	survive_update_pose_euler(tracker, &tracker->velocity, timecode, Rv, vel);
}

static inline void update_pose_pos(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_position_t *info = (struct kalman_info_position_t *)_info;
	_info->variance = survive_imu_tracker_predict_pos(tracker, timecode, info->v);
	_info->last_update = timecode;
}
static inline void update_pose_rot(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_rotation_t *info = (struct kalman_info_rotation_t *)_info;
	_info->variance = survive_imu_tracker_predict_rot(tracker, timecode, info->v);
	_info->last_update = timecode;
}

static inline void update_vel_pos(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_position_t *info = (struct kalman_info_position_t *)_info;
	_info->variance = survive_imu_tracker_predict_velocity_pos(tracker, timecode, info->v);
	_info->last_update = timecode;
}
static inline void update_vel_rot(SurviveIMUTracker *tracker, survive_timecode timecode, struct kalman_info_t *_info) {
	struct kalman_info_rotation_t *info = (struct kalman_info_rotation_t *)_info;
	_info->variance = survive_imu_tracker_predict_velocity_rot(tracker, timecode, info->v);
	_info->last_update = timecode;
}

void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data) {

	if (tracker->last_data.datamask == 0 || tracker->pose.Rot.info.variance < 0 ||
		tracker->pose.Pos.info.variance < 0) {
		tracker->last_data = *data;
		return;
	}

	if (tracker->mahony_variance >= 0) {
		LinmathQuat pose_rot;
		quatcopy(pose_rot, tracker->pose.Rot.v);
		mahony_ahrs(tracker, pose_rot, data->gyro, data->accel);
		survive_imu_tracker_integrate_rotation(tracker, data->timecode, pose_rot,
											   tracker->gyro_var + tracker->mahony_variance);
	}

	SurviveContext *ctx = tracker->so->ctx;

	LinmathQuat gyro_q;
	quatfromeuler(gyro_q, data->gyro);

	LinmathEulerPose new_velocity;
	quatconjugateby(gyro_q, tracker->pose.Rot.v, gyro_q);

	quattoeuler(new_velocity.EulerRot, gyro_q);

	FLT Rv[2] = {tracker->pose.Rot.info.variance + tracker->velocity.Pos.info.variance + tracker->acc_var,
				 tracker->pose.Rot.info.variance + tracker->gyro_var};

	FLT time_diff =
		survive_timecode_difference(data->timecode, tracker->last_data.timecode) / (FLT)tracker->so->timebase_hz;
	if (time_diff > 1.0) {
		SV_WARN("%s is probably dropping IMU packets; %f time reported between", tracker->so->codename, time_diff);
	}

	if (!isinf(Rv[0])) {
		LinmathVec3d acc;
		copy3d(acc, data->accel);

		LinmathVec3d rAcc = {0}, avgAcc;
		RotateAccel(rAcc, tracker->pose.Rot.v, acc);

		add3d(avgAcc, rAcc, tracker->last_acc);
		// scale3d(avgAcc, avgAcc, .5 * time_diff);
		scale3d(avgAcc, rAcc, time_diff);
		add3d(new_velocity.Pos, tracker->velocity.Pos.v, avgAcc);
		copy3d(tracker->last_acc, rAcc);

		SV_VERBOSE("NV: " SurviveVel_format, SURVIVE_VELOCITY_EXPAND(new_velocity));
		survive_imu_tracker_integrate_velocity(tracker, data->timecode, Rv, &new_velocity);
	} else {
		survive_imu_tracker_integrate_angular_velocity(tracker, data->timecode, new_velocity.EulerRot, Rv[1]);
	}

	SV_VERBOSE("RV: " SurviveVel_format, LINMATH_VEC3_EXPAND(tracker->velocity.Pos.v), tracker->velocity.EulerRot.v[0],
			   tracker->velocity.EulerRot.v[1], tracker->velocity.EulerRot.v[2]);
	// SV_VERBOSE("D1:" Quat_format " --- " Quat_format, LINMATH_QUAT_EXPAND(tracker->last_pose.Rot.v),
	// LINMATH_QUAT_EXPAND(tracker->pose.Rot.v));
	// SV_VERBOSE("IMU VAR %f", tracker->pose.Pos.info.variance);

	tracker->last_data = *data;
}

FLT survive_imu_tracker_predict_velocity_pos(const SurviveIMUTracker *tracker, survive_timecode timecode, double *out) {
	FLT time_diff =
		survive_timecode_difference(timecode, tracker->velocity.Pos.info.last_update) / (FLT)tracker->so->timebase_hz;

	copy3d(out, tracker->velocity.Pos.v);
	return tracker->velocity.Pos.info.variance + time_diff * tracker->velocity.Pos.info.variance_per_second;
}

FLT survive_imu_tracker_predict_velocity_rot(const SurviveIMUTracker *tracker, survive_timecode timecode,
											 LinmathEulerAngle out) {
	FLT time_diff = survive_timecode_difference(timecode, tracker->velocity.EulerRot.info.last_update) /
					(FLT)tracker->so->timebase_hz;

	copy3d(out, tracker->velocity.EulerRot.v);
	return tracker->velocity.EulerRot.info.variance + time_diff * tracker->velocity.EulerRot.info.variance_per_second;
}

FLT survive_imu_tracker_predict_pos(const SurviveIMUTracker *tracker, survive_timecode timecode, LinmathVec3d out) {
	if (tracker->pose.Pos.info.variance < 0)
		return tracker->pose.Pos.info.variance;

	FLT pose_time_diff =
		survive_timecode_difference(timecode, tracker->pose.Pos.info.last_update) / (FLT)tracker->so->timebase_hz;
	// assert(pose_time_diff < 1.0);
	pose_time_diff = linmath_min(.5, pose_time_diff);

	LinmathVec3d vel, displacement;
	FLT velocity_variance = survive_imu_tracker_predict_velocity_pos(tracker, timecode, vel);

	if (velocity_variance > 10) {
		copy3d(out, tracker->pose.Pos.v);
		return tracker->pose.Pos.info.variance + pose_time_diff * (tracker->pose.Pos.info.variance_per_second);
	}

	scale3d(displacement, vel, pose_time_diff);
	add3d(out, displacement, tracker->pose.Pos.v);
	assert(norm3d(out) < 1000);
	return tracker->pose.Pos.info.variance +
		   pose_time_diff * (velocity_variance * velocity_variance + tracker->pose.Pos.info.variance_per_second);
}

FLT survive_imu_tracker_predict_rot(const SurviveIMUTracker *tracker, survive_timecode timecode, LinmathQuat out) {
	if (quatiszero(tracker->pose.Rot.v))
		return tracker->pose.Rot.info.variance;

	FLT rot_time_diff =
		survive_timecode_difference(timecode, tracker->pose.Rot.info.last_update) / (FLT)tracker->so->timebase_hz;
	// assert(rot_time_diff < 1.0);
	rot_time_diff = linmath_min(.5, rot_time_diff);

	LinmathEulerAngle vel;
	FLT velocity_variance = survive_imu_tracker_predict_velocity_rot(tracker, timecode, vel);

	if (velocity_variance > 10) {
		quatcopy(out, tracker->pose.Rot.v);
		return tracker->pose.Rot.info.variance + rot_time_diff * (tracker->pose.Rot.info.variance_per_second);
	}

	scale3d(vel, vel, rot_time_diff);
	LinmathQuat rot_change;
	quatfromeuler(rot_change, vel);
	quatrotateabout(out, rot_change, tracker->pose.Rot.v);

	return tracker->pose.Rot.info.variance +
		   rot_time_diff * (velocity_variance + tracker->pose.Rot.info.variance_per_second);
}
void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_timecode timecode, SurvivePose *out) {
	if (tracker->velocity.EulerRot.info.variance > 10 || tracker->velocity.Pos.info.variance > 10) {
		copy3d(out->Pos, tracker->pose.Pos.v);
		quatcopy(out->Rot, tracker->pose.Rot.v);
		return;
	}
	survive_imu_tracker_predict_pos(tracker, timecode, out->Pos);
	survive_imu_tracker_predict_rot(tracker, timecode, out->Rot);
}

void survive_imu_tracker_integrate_observation(uint32_t timecode, SurviveIMUTracker *tracker, const SurvivePose *pose,
											   const FLT *R) {
	// Kalman filter assuming:
	// F -> Identity
	// H -> Identity
	// Q / R / P -> Diagonal matrices; just treat them as such. This assumption might need some checking but it
	// makes the # of calculations needed much smaller so we may be willing to tolerate some approximation here

	survive_update_pose(tracker, &tracker->pose, timecode, R, pose);

	FLT time_diff =
		survive_timecode_difference(timecode, tracker->last_pose.Pos.info.last_update) / (FLT)tracker->so->timebase_hz;

	kalman_info_pose_t vel_pose = {0};

	bool use_obv_only = true;
	if (use_obv_only) {
		vel_pose.Pos.info.last_update = timecode;
		vel_pose.Pos.info.variance = R[0];
		copy3d(vel_pose.Pos.v, pose->Pos);

		vel_pose.Rot.info.variance = R[1];
		quatcopy(vel_pose.Rot.v, pose->Rot);
	} else {
		vel_pose = tracker->pose;
	}

	if (!quatiszero(tracker->last_pose.Rot.v) && time_diff != 0. && tracker->use_obs_velocity) {
		// assert(time_diff < 1.0);
		if (time_diff > 1.0) {
			SurviveContext *ctx = tracker->so->ctx;
			SV_WARN("Detected %f gap between observations for %s", time_diff, tracker->so->codename);
		}

		SurviveVelocity velocity = {0};
		LinmathQuat vDiff;
		quatfind(vDiff, tracker->last_pose.Rot.v, vel_pose.Rot.v);
		struct SurviveContext *ctx = tracker->so->ctx;
		SV_VERBOSE("P: " SurvivePose_format, SURVIVE_POSE_EXPAND(*pose));
		SV_VERBOSE("D:" Quat_format " --- " Quat_format " --- " Quat_format,
				   LINMATH_QUAT_EXPAND(tracker->last_pose.Rot.v), LINMATH_QUAT_EXPAND(vel_pose.Rot.v),
				   LINMATH_QUAT_EXPAND(vDiff));

		// quatfind(vDiff, vel_pose.Rot.v, tracker->last_pose.Rot.v);
		// quatmultiplyrotation(t, vDiff, 1. / time_diff);

		quattoeuler(velocity.EulerRot, vDiff);
		double damp_factor = 1. / 10.;
		scale3d(velocity.EulerRot, velocity.EulerRot, damp_factor / time_diff);

		sub3d(velocity.Pos, vel_pose.Pos.v, tracker->last_pose.Pos.v);
		scale3d(velocity.Pos, velocity.Pos, damp_factor / time_diff);
		SV_VERBOSE("EV: " SurviveVel_format, SURVIVE_VELOCITY_EXPAND(velocity));

		SurvivePoseVariance vp = {
			.Pose = vel_pose.Pos.info.variance + tracker->last_pose.Pos.info.variance + tracker->obs_variance,
			.Rot = vel_pose.Rot.info.variance + tracker->last_pose.Rot.info.variance + tracker->obs_rot_variance};
		survive_imu_tracker_integrate_velocity(tracker, timecode, &vp.Pose, &velocity);
		SV_VERBOSE("rV: " SurviveVel_format, LINMATH_VEC3_EXPAND(tracker->velocity.Pos.v),
				   LINMATH_VEC3_EXPAND(tracker->velocity.EulerRot.v));
	}

	tracker->last_pose = vel_pose;
}

STATIC_CONFIG_ITEM(POSE_POSITION_VARIANCE_SEC, "filter-pose-var-per-sec", 'f', "Position variance per second", 0.05);
STATIC_CONFIG_ITEM(POSE_ROT_VARIANCE_SEC, "filter-pose-rot-var-per-sec", 'f', "Position rotational variance per second",
				   0.05);

STATIC_CONFIG_ITEM(VELOCITY_POSITION_VARIANCE_SEC, "filter-vel-var-per-sec", 'f', "Velocity variance per second", 0.3);
STATIC_CONFIG_ITEM(VELOCITY_ROT_VARIANCE_SEC, "filter-vel-rot-var-per-sec", 'f',
				   "Velocity rotational variance per second", 0.3);

STATIC_CONFIG_ITEM(IMU_ACC_VARIANCE, "imu-acc-variance", 'f', "Variance of accelerometer", 10000.);
STATIC_CONFIG_ITEM(IMU_GYRO_VARIANCE, "imu-gyro-variance", 'f', "Variance of gyroscope", 10000.);
STATIC_CONFIG_ITEM(IMU_MAHONY_VARIANCE, "imu-mahony-variance", 'f', "Variance of mahony filter (negative to disable)",
				   -1.0);

STATIC_CONFIG_ITEM(USE_OBS_VELOCITY, "use-obs-velocity", 'i', "Incorporate observed velocity into filter", 1);
STATIC_CONFIG_ITEM(OBS_VELOCITY_POSITION_VAR, "obs-velocity-var", 'f', "Incorporate observed velocity into filter",
				   .01);
STATIC_CONFIG_ITEM(OBS_VELOCITY_ROTATION_VAR, "obs-velocity-rot-var", 'f', "Incorporate observed velocity into filter",
				   .01);

void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));

	tracker->so = so;

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_INFO("Initializing Filter:");
	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.
	tracker->velocity.Pos.info.variance = 1e-3;
	tracker->velocity.EulerRot.info.variance = 1e-3;
	survive_attach_configf(tracker->so->ctx, VELOCITY_POSITION_VARIANCE_SEC_TAG,
						   &tracker->velocity.Pos.info.variance_per_second);
	survive_attach_configf(tracker->so->ctx, VELOCITY_ROT_VARIANCE_SEC_TAG,
						   &tracker->velocity.EulerRot.info.variance_per_second);

	survive_attach_configf(tracker->so->ctx, OBS_VELOCITY_POSITION_VAR_TAG, &tracker->obs_variance);
	survive_attach_configf(tracker->so->ctx, OBS_VELOCITY_ROTATION_VAR_TAG, &tracker->obs_rot_variance);

	tracker->pose.Pos.info.variance = -1;
	tracker->pose.Rot.info.variance = -1;
	survive_attach_configf(tracker->so->ctx, POSE_POSITION_VARIANCE_SEC_TAG,
						   &tracker->pose.Pos.info.variance_per_second);
	survive_attach_configf(tracker->so->ctx, POSE_ROT_VARIANCE_SEC_TAG, &tracker->pose.Rot.info.variance_per_second);

	tracker->pose.Pos.info.update_fn = update_pose_pos;
	tracker->pose.Rot.info.update_fn = update_pose_rot;

	tracker->velocity.Pos.info.update_fn = update_vel_pos;
	tracker->velocity.EulerRot.info.update_fn = update_vel_rot;

	survive_attach_configf(tracker->so->ctx, IMU_MAHONY_VARIANCE_TAG, &tracker->mahony_variance);
	survive_attach_configi(tracker->so->ctx, USE_OBS_VELOCITY_TAG, &tracker->use_obs_velocity);

	survive_attach_configf(tracker->so->ctx, IMU_ACC_VARIANCE_TAG, &tracker->acc_var);
	survive_attach_configf(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->gyro_var);

	SV_INFO("\t%s: %f", POSE_POSITION_VARIANCE_SEC_TAG, tracker->pose.Pos.info.variance_per_second);
	SV_INFO("\t%s: %f", VELOCITY_POSITION_VARIANCE_SEC_TAG, tracker->velocity.Pos.info.variance_per_second);
	SV_INFO("\t%s: %f", IMU_ACC_VARIANCE_TAG, tracker->acc_var);
	SV_INFO("\t%s: %f", IMU_GYRO_VARIANCE_TAG, tracker->gyro_var);
}

SurviveVelocity survive_imu_velocity(const SurviveIMUTracker *tracker) {
	SurviveVelocity rtn;
	copy3d(rtn.Pos, tracker->velocity.Pos.v);
	copy3d(rtn.EulerRot, tracker->velocity.EulerRot.v);
	return rtn;
}
