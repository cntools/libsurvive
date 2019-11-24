#include "survive_imu.h"
#include "linmath.h"
#include "math.h"
#include "survive_imu.h"
#include "survive_internal.h"
#include "survive_kalman.h"
#include <assert.h>
#include <memory.h>
#include <minimal_opencv.h>
#include <malloc.h>

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
	SurviveContext *ctx = 0;
	// SV_VERBOSE(100, "RotateAccel: %f\t" Point3_format, magnitude3d(rAcc), LINMATH_VEC3_EXPAND(rAcc));
	add3d(rAcc, rAcc, G);
	scale3d(rAcc, rAcc, 9.80665);
}

#ifdef USE_DOUBLE
#define SURVIVE_CV_F CV_64F
#else
#define SURVIVE_CV_F CV_32F
#endif

#define CREATE_STACK_MAT(name, rows, cols)                                                                             \
	FLT *_##name = alloca(rows * cols * sizeof(FLT));                                                                  \
	CvMat name = cvMat(rows, cols, SURVIVE_CV_F, _##name);

void update_rotation_from_rotvel(FLT t, survive_kalman_state_t *k, const CvMat *H, const CvMat *K, const CvMat *x_t0,
								 CvMat *x_t1, const FLT *z) {
	CvMat Z = cvMat(1, k->max_dim_cnt, SURVIVE_CV_F, (void *)(z));

	CREATE_STACK_MAT(y, 1, k->max_dim_cnt);
	// y = Z - H * X_K|k-1
	cvGEMM(H, x_t0, -1, &Z, 1, &y, 0);

	// X_k|k = X_k|k-1 + K * y
	cvGEMM(K, &y, 1, x_t0, 1, x_t1, 0);
}

static void update_rotation_from_rotation(FLT t, survive_kalman_state_t *k, const CvMat *H, const CvMat *K,
										  const CvMat *x_t0, CvMat *x_t1, const FLT *z) {
	FLT k_rot = K->data.db[0];
	FLT k_rot_vel = K->data.db[1];

	FLT p_rot = k->info.P[0];
	FLT p_rot_vel = k->info.P[1];

	FLT f_rot = p_rot / (k_rot + p_rot);
	FLT f_rot_vel = p_rot_vel / (k_rot_vel + p_rot_vel);

	if (quatiszero(x_t0->data.db)) {
		quatcopy(x_t1->data.db, z);
		return;
	}

	LinmathQuat original;
	survive_apply_ang_velocity(original, x_t0->data.db + 4, -t, x_t0->data.db);

	quatslerp(x_t1->data.db, x_t0->data.db, z, K->data.db[0]);

	if (t > .001) {
		SurviveAngularVelocity ang_vel;
		survive_find_ang_velocity(ang_vel, t, original, x_t1->data.db);
		linmath_interpolate(x_t1->data.db + 4, 3, x_t0->data.db + 4, ang_vel, K->data.db[1]);
	}
	// printf("x0      " Point3_format "\n", LINMATH_VEC3_EXPAND(x_t0->data.db + 4));
	// printf("ang_vel " Point3_format "\n", LINMATH_VEC3_EXPAND(ang_vel));
	// printf("x1      " Point3_format "\n", LINMATH_VEC3_EXPAND(x_t1->data.db + 4));
}

void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data) {
	SurviveContext *ctx = tracker->so->ctx;

	// Wait til observation is in before reading IMU; gets rid of bad IMU data at the start
	if (tracker->last_data.datamask == 0) {
		tracker->imu_kalman_update = data->hdr.timecode;
		tracker->last_kalman_update = tracker->obs_kalman_update = data->hdr.timecode;
		return;
	}

	if (tracker->last_data.datamask == 1) {
		tracker->last_data = *data;
		tracker->imu_kalman_update = data->hdr.timecode;
		tracker->last_kalman_update = tracker->obs_kalman_update = data->hdr.timecode;
		return;
	}

	// double n = 1. / norm3d(data->accel);
	// if(n > .999 && n < 1.001)
	// tracker->acc_bias = tracker->acc_bias * .95 + (1 / norm3d(data->accel)) * .05;

	// SV_INFO("%7f %7f", n, tracker->acc_bias);

	FLT time_diff =
		survive_timecode_difference(data->hdr.timecode, tracker->last_kalman_update) / (FLT)tracker->so->timebase_hz;
	FLT time_since_obs =
		survive_timecode_difference(data->hdr.timecode, tracker->obs_kalman_update) / (FLT)tracker->so->timebase_hz;

	// printf("i%u %f\n", data->timecode, time_diff);
	LinmathQuat rot;
	survive_kalman_predict_state(0, &tracker->rot, 0, rot);

	assert(time_diff >= 0);
	if (time_since_obs > .05) {
		return;
	}
	if (time_diff > 0.5) {
		SV_WARN("%s is probably dropping IMU packets; %f time reported between %u %u", tracker->so->codename, time_diff,
				data->hdr.timecode, tracker->imu_kalman_update);
	}

	if (tracker->mahony_variance >= 0) {
		LinmathQuat pose_rot;
		quatcopy(pose_rot, rot);
		mahony_ahrs(tracker, pose_rot, data->gyro, data->accel);

		const FLT Hr[] = {1, 0};
		LinmathAxisAngleMag input;
		quattoaxisanglemag(input, pose_rot);

		survive_kalman_predict_update_state(time_diff, &tracker->rot, input, Hr,
											tracker->rot.info.P[0] + tracker->mahony_variance);
		time_diff = 0;
	}

	FLT Rv[2] = {tracker->rot.info.P[0] + tracker->acc_var, tracker->rot.info.P[0] + tracker->gyro_var};

	LinmathVec3d rAcc = {0};
	RotateAccel(rAcc, rot, data->accel);

	const FLT Hp[] = {0, 0, 1};
	survive_kalman_predict_update_state(time_diff, &tracker->position, rAcc, Hp, Rv[0]);

	const FLT Hr[] = {0, 1};
	FLT rot_vel[4] = { 0 };
	quatrotatevector(rot_vel, rot, data->gyro);
	// survive_kalman_predict_update_state(time_diff, &tracker->rot, rot_vel, Hr, Rv[1]);
	survive_kalman_predict_update_state_extended(time_diff, &tracker->rot, rot_vel, Hr, update_rotation_from_rotvel,
												 Rv[1]);

	tracker->imu_kalman_update = tracker->last_kalman_update = data->hdr.timecode;
}

void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_timecode timecode, SurvivePose *out) {
	if (tracker->position.info.P[0] > 100 || tracker->rot.info.P[0] > 100)
		return;

	FLT t = survive_timecode_difference(timecode, tracker->last_kalman_update) / (FLT)tracker->so->timebase_hz;

	survive_kalman_predict_state(t, &tracker->position, 0, out->Pos);

	// LinmathAxisAngleMag r;
	survive_kalman_predict_state(t, &tracker->rot, 0, out->Rot);
	quatnormalize(out->Rot, out->Rot);
	// quatfromaxisanglemag(out->Rot, r);
}

SURVIVE_EXPORT void survive_imu_tracker_update(SurviveIMUTracker *tracker, survive_timecode timecode,
											   SurvivePose *out) {
	survive_imu_tracker_predict(tracker, timecode, out);
}

/*
static void linear_update(FLT t, survive_kalman_state_t *k, const CvMat* H, const CvMat* K, const CvMat* x_t0, CvMat*
x_t1, const FLT* z) { int state_cnt = k->info.state_cnt; CREATE_STACK_MAT(x, state_cnt, k->dimension_cnt);

	CREATE_STACK_MAT(F, state_cnt, state_cnt);
	k->info.F_fn(t, _F);

	survive_kalman_predict(t, k, x_t0, &x);

	CvMat Z = cvMat(1, k->dimension_cnt, SURVIVE_CV_F, (void *) (z));

	CREATE_STACK_MAT(y, 1, k->dimension_cnt);
	// y = Z - H * X_K|k-1
	cvGEMM(H, &x, -1, &Z, 1, &y, 0);

	// X_k|k = X_k|k-1 + K * y
	cvGEMM(K, &y, 1, &x, 1, x_t1, 0);
}
*/

static void rot_predict(FLT t, const survive_kalman_state_t *k, const CvMat *f_in, CvMat *f_out) {
	(void)k;

	const FLT *rot = f_in->data.db;
	const FLT *vel = f_in->data.db + 4;
	copy3d(f_out->data.db + 4, vel);
	f_out->data.db[7] = 0;

	survive_apply_ang_velocity(f_out->data.db, vel, t, rot);
}

void survive_imu_tracker_integrate_observation(uint32_t timecode, SurviveIMUTracker *tracker, const SurvivePose *pose,
											   const FLT *R) {
	if (tracker->last_data.datamask == 0) {
		tracker->last_data.datamask = 1;
		tracker->imu_kalman_update = timecode;
		tracker->last_kalman_update = tracker->obs_kalman_update = timecode;
	}

	FLT time_diff = survive_timecode_difference(timecode, tracker->last_kalman_update) / (FLT)tracker->so->timebase_hz;
	// assert(time_diff >= 0 && time_diff < 10);

	// FLT H[] = {1., time_diff, time_diff * time_diff / 2.};
	FLT H[] = {1., 0, 0};
	survive_kalman_predict_update_state(time_diff, &tracker->position, pose->Pos, H, R[0]);

	LinmathQuat cur_rot;
	// LinmathAxisAngleMag aa_rot, cur_rot;
	// quattoaxisanglemag(aa_rot, pose->Rot);

	survive_kalman_predict_state(time_diff, &tracker->rot, 0, cur_rot);
	// findnearestaxisanglemag(aa_rot, aa_rot, cur_rot);

	survive_kalman_predict_update_state_extended(time_diff, &tracker->rot, pose->Rot, H, update_rotation_from_rotation,
												 R[1]);

	// findnearestaxisanglemag(tracker->rot.state, tracker->rot.state, 0);

	tracker->last_kalman_update = tracker->obs_kalman_update = timecode;
}

STATIC_CONFIG_ITEM(POSE_POSITION_VARIANCE_SEC, "filter-pose-var-per-sec", 'f', "Position variance per second", 0.001);
STATIC_CONFIG_ITEM(POSE_ROT_VARIANCE_SEC, "filter-pose-rot-var-per-sec", 'f', "Position rotational variance per second",
				   0.01);

STATIC_CONFIG_ITEM(VELOCITY_POSITION_VARIANCE_SEC, "filter-vel-var-per-sec", 'f', "Velocity variance per second", 1.0);
STATIC_CONFIG_ITEM(VELOCITY_ROT_VARIANCE_SEC, "filter-vel-rot-var-per-sec", 'f',
				   "Velocity rotational variance per second", 0.1);

STATIC_CONFIG_ITEM(IMU_ACC_VARIANCE, "imu-acc-variance", 'f', "Variance of accelerometer", 1.0);
STATIC_CONFIG_ITEM(IMU_GYRO_VARIANCE, "imu-gyro-variance", 'f', "Variance of gyroscope", 0.001);
STATIC_CONFIG_ITEM(IMU_MAHONY_VARIANCE, "imu-mahony-variance", 'f', "Variance of mahony filter (negative to disable)",
				   -1.);

void rot_f(FLT t, FLT *F) {
	FLT f[] = {1, t, 0, 1};

	memcpy(F, f, sizeof(FLT) * 4);
}

void pos_f(FLT t, FLT *F) {
	FLT f[] = {1, t, t * t / 2., 0, 1, t, 0, 0, 1};

	memcpy(F, f, sizeof(FLT) * 9);
}

void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));

	tracker->so = so;

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(110, "Initializing Filter:");
	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.

	tracker->pos_Q_per_sec[8] = 1.;

	survive_attach_configf(tracker->so->ctx, VELOCITY_POSITION_VARIANCE_SEC_TAG, &tracker->pos_Q_per_sec[4]);
	survive_attach_configf(tracker->so->ctx, VELOCITY_ROT_VARIANCE_SEC_TAG, &tracker->rot_Q_per_sec[3]);

	survive_attach_configf(tracker->so->ctx, POSE_POSITION_VARIANCE_SEC_TAG, &tracker->pos_Q_per_sec[0]);
	survive_attach_configf(tracker->so->ctx, POSE_ROT_VARIANCE_SEC_TAG, &tracker->rot_Q_per_sec[0]);

	survive_attach_configf(tracker->so->ctx, IMU_MAHONY_VARIANCE_TAG, &tracker->mahony_variance);

	survive_attach_configf(tracker->so->ctx, IMU_ACC_VARIANCE_TAG, &tracker->acc_var);
	survive_attach_configf(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->gyro_var);

	size_t rotational_dims[] = {4, 3};
	size_t position_dims[] = {3, 3, 3};
	survive_kalman_state_init(&tracker->rot, 2, rot_f, tracker->rot_Q_per_sec, 0, rotational_dims, 0);
	tracker->rot.info.Predict_fn = rot_predict;
	// tracker->rot.info.Map_fn = rot_map;

	survive_kalman_state_init(&tracker->position, 3, pos_f, tracker->pos_Q_per_sec, 0, position_dims, 0);

	SV_VERBOSE(110, "\t%s: %f", POSE_POSITION_VARIANCE_SEC_TAG, tracker->pos_Q_per_sec[0]);
	SV_VERBOSE(110, "\t%s: %f", POSE_ROT_VARIANCE_SEC_TAG, tracker->rot_Q_per_sec[0]);
	SV_VERBOSE(110, "\t%s: %f", VELOCITY_POSITION_VARIANCE_SEC_TAG, tracker->pos_Q_per_sec[4]);
	SV_VERBOSE(110, "\t%s: %f", VELOCITY_ROT_VARIANCE_SEC_TAG, tracker->rot_Q_per_sec[3]);
	SV_VERBOSE(110, "\t%s: %f", IMU_ACC_VARIANCE_TAG, tracker->acc_var);
	SV_VERBOSE(110, "\t%s: %f", IMU_GYRO_VARIANCE_TAG, tracker->gyro_var);
	SV_VERBOSE(110, "\t%s: %f", IMU_MAHONY_VARIANCE_TAG, tracker->mahony_variance);
}

SurviveVelocity survive_imu_velocity(const SurviveIMUTracker *tracker) {
	SurviveVelocity rtn = {0};
	survive_kalman_predict_state(0, &tracker->position, 1, rtn.Pos);
	survive_kalman_predict_state(0, &tracker->rot, 1, rtn.AxisAngleRot);
	return rtn;
}

void survive_imu_tracker_integrate_velocity(SurviveIMUTracker *tracker, survive_timecode timecode, const FLT *Rv,
											const SurviveVelocity *vel) {
	const FLT H[] = {0, 1, 0};
	FLT time_diff = survive_timecode_difference(timecode, tracker->last_kalman_update) / (FLT)tracker->so->timebase_hz;

	survive_kalman_predict_update_state(time_diff, &tracker->position, vel->Pos, H, Rv[0]);
	survive_kalman_predict_update_state(time_diff, &tracker->rot, vel->AxisAngleRot, H, Rv[1]);

	tracker->last_kalman_update = tracker->obs_kalman_update = timecode;
}

void survive_imu_tracker_free(SurviveIMUTracker *tracker) {
	survive_kalman_state_free(&tracker->position);
	survive_kalman_state_free(&tracker->rot);

	survive_detach_config(tracker->so->ctx, VELOCITY_POSITION_VARIANCE_SEC_TAG, &tracker->pos_Q_per_sec[4]);
	survive_detach_config(tracker->so->ctx, VELOCITY_ROT_VARIANCE_SEC_TAG, &tracker->rot_Q_per_sec[3]);

	survive_detach_config(tracker->so->ctx, POSE_POSITION_VARIANCE_SEC_TAG, &tracker->pos_Q_per_sec[0]);
	survive_detach_config(tracker->so->ctx, POSE_ROT_VARIANCE_SEC_TAG, &tracker->rot_Q_per_sec[0]);

	survive_detach_config(tracker->so->ctx, IMU_MAHONY_VARIANCE_TAG, &tracker->mahony_variance);

	survive_detach_config(tracker->so->ctx, IMU_ACC_VARIANCE_TAG, &tracker->acc_var);
	survive_detach_config(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->gyro_var);
}
