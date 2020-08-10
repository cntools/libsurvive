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

#include "generated/survive_imu.generated.h"

static inline void mat_eye_diag(CvMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			CV_FLT_PTR(m)[j * m->cols + i] = i == j ? (v ? v[i] : 1.) : 0.;
		}
	}
}

static inline void arr_eye_diag(FLT *m, int rows, int cols, const FLT *v) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			(m)[j * cols + i] = i == j ? v[i] : 0.;
		}
	}
}

static const int imu_calibration_iterations = 100;

static void RotateAccel(LinmathVec3d rAcc, const LinmathQuat rot, const LinmathVec3d accel) {
	quatrotatevector(rAcc, rot, accel);
	LinmathVec3d G = {0, 0, -1};
	add3d(rAcc, rAcc, G);
	scale3d(rAcc, rAcc, 9.80665);
}

static void survive_imu_accel_up_correct(SurviveIMUTracker *tracker, LinmathQuat q, const LinmathPoint3d raw_accel) {
	LinmathVec3d up = {0};
	quatrotatevector(up, q, raw_accel);

	LinmathPoint3d G = {0, 0, 1};
	LinmathPoint3d N;
	cross3d(N, up, G);
	FLT angle = atan2(sqrt(up[0] * up[0] + up[1] * up[1]), up[2]);

	FLT s = angle / 2.;
	scale3d(N, N, cos(s) * sin(s));

	LinmathQuat q_up = {1};
	if (norm3d(N) > 1e-3)
		quatfromaxisanglemag(q_up, N);

	quatrotateabout(q, q_up, q);
}
void survive_imu_tracker_integrate_imu(SurviveIMUTracker *tracker, PoserDataIMU *data) {
	SurviveContext *ctx = tracker->so->ctx;

	// Wait til observation is in before reading IMU; gets rid of bad IMU data at the start
	if (tracker->rot.t == 0 || tracker->position.t == 0) {
		return;
	}
	SV_VERBOSE(200, "%s imu mag %f", tracker->so->codename, norm3d(data->accel));
	FLT time = data->hdr.timecode / (FLT)tracker->so->timebase_hz;
	FLT time_diff = time - tracker->rot.t;

	if (time_diff < -.01) {
		// SV_WARN("Processing imu data from the past %fs", time - tracker->rot.t);
		tracker->stats.late_imu_dropped++;
		return;
	}

	LinmathQuat obj2world = {0};
	survive_kalman_predict_state(time, &tracker->rot, 0, 4, obj2world);
	quatnormalize(obj2world, obj2world);

	if (time_diff > 0.5) {
		SV_WARN("%s is probably dropping IMU packets; %f time reported between %lu", tracker->so->codename, time_diff,
				data->hdr.timecode);
	}

	bool standStill =
		SurviveSensorActivations_stationary_time(&tracker->so->activations) > (tracker->so->timebase_hz / 10.);

	bool hasRotation = false;
	bool hasAngularVel = standStill;
	FLT rotation_state_update[7] = {0};
	FLT rotation_variance[] = {tracker->acc_tilt_var, tracker->acc_tilt_var, tracker->acc_tilt_var,
							   tracker->acc_tilt_var, tracker->gyro_var,	 tracker->gyro_var,
							   tracker->gyro_var};

	LinmathVec3d accel_corrected;
	copy3d(accel_corrected, data->accel);

	if (tracker->acc_tilt_var >= 0) {
		quatcopy(rotation_state_update, obj2world);

		survive_imu_accel_up_correct(tracker, rotation_state_update, data->accel);
		hasRotation = true;
	}

	if (tracker->gyro_var >= 0 && !standStill) {
		quatrotatevector(rotation_state_update + 4, obj2world, data->gyro);
		hasAngularVel = true;
	}

	if (hasAngularVel && hasRotation) {
		SV_VERBOSE(200, "Integrating gyro/mahony " Point7_format " with cov " Point7_format,
				   LINMATH_VEC7_EXPAND(rotation_state_update), LINMATH_VEC7_EXPAND(rotation_variance));
		survive_imu_integrate_rotation_angular_velocity(tracker, time, rotation_state_update, rotation_variance);
	} else if (hasAngularVel) {
		SV_VERBOSE(200, "Integrating gyro " Point3_format " with cov " Point3_format,
				   LINMATH_VEC3_EXPAND(rotation_state_update + 4), LINMATH_VEC3_EXPAND(rotation_variance + 4));
		survive_imu_integrate_angular_velocity(tracker, time, rotation_state_update + 4, rotation_variance + 4);
	} else if (hasRotation) {
		SV_VERBOSE(200, "Integrating mahony " Point4_format " with cov " Point4_format,
				   LINMATH_VEC4_EXPAND(rotation_state_update), LINMATH_VEC4_EXPAND(rotation_variance));
		survive_imu_integrate_rotation(tracker, time, rotation_state_update, rotation_variance);
	}

	survive_kalman_predict_state(time, &tracker->rot, 0, 4, obj2world);
	quatnormalize(obj2world, obj2world);

	if (standStill) {
		FLT zeros[6] = {0.};
		FLT v = 1e-2;
		FLT R[] = {v, v, v, v, v, v};
		SV_VERBOSE(200, "Integrating stand still " Point6_format " with cov " Point6_format, LINMATH_VEC6_EXPAND(zeros),
				   LINMATH_VEC6_EXPAND(R));
		LinmathVec3d rAcc = {0};
		RotateAccel(rAcc, obj2world, accel_corrected);

		survive_imu_integrate_velocity_acceleration(tracker, time, zeros, R);
	} else {
		if (tracker->acc_var >= 0) {
			FLT v = tracker->acc_var;

			LinmathVec3d rAcc = {0};
			RotateAccel(rAcc, obj2world, accel_corrected);
			FLT R[] = {v, v, v};

			SV_VERBOSE(200, "Integrating accelerometer " Point3_format " with cov " Point3_format,
					   LINMATH_VEC3_EXPAND(rAcc), LINMATH_VEC3_EXPAND(R));
			survive_imu_integrate_acceleration(tracker, time, rAcc, R);
		}
	}
}

void survive_imu_tracker_predict(const SurviveIMUTracker *tracker, survive_long_timecode timecode, SurvivePose *out) {
	// if (tracker->position.info.P[0] > 100 || tracker->rot.info.P[0] > 100 || tracker->position.t == 0)
	//	return;

	if (tracker->position.t == 0)
		return;

	FLT t = timecode / (FLT)tracker->so->timebase_hz;

	survive_kalman_predict_state(t, &tracker->position, 0, 3, out->Pos);
	survive_kalman_predict_state(t, &tracker->rot, 0, 4, out->Rot);
	quatnormalize(out->Rot, out->Rot);

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(300, "Predict pose %f %f " SurvivePose_format, t, t - tracker->rot.t, SURVIVE_POSE_EXPAND(*out))
}

SURVIVE_EXPORT void survive_imu_tracker_update(SurviveIMUTracker *tracker, survive_long_timecode timecode,
											   SurvivePose *out) {
	survive_imu_tracker_predict(tracker, timecode, out);
}

static void rot_predict_quat(FLT t, const survive_kalman_state_t *k, const CvMat *f_in, CvMat *f_out) {
	(void)k;

	const FLT *rot = CV_FLT_PTR(f_in);
	const FLT *vel = CV_FLT_PTR(f_in) + 4;
	copy3d(CV_FLT_PTR(f_out) + 4, vel);

	survive_apply_ang_velocity(CV_FLT_PTR(f_out), vel, t, rot);
}

static void pos_predict(FLT t, const survive_kalman_state_t *k, const CvMat *f_in, CvMat *f_out) {
	(void)k;

	const FLT *pos = CV_FLT_PTR(f_in);
	const FLT *vel = CV_FLT_PTR(f_in) + 3;
	const FLT *acc = CV_FLT_PTR(f_in) + 6;

	FLT *out = CV_FLT_PTR(f_out);
	memcpy(out, pos, sizeof(FLT) * 9);

	FLT t2 = t * t / 2.;
	for (int i = 0; i < 3; i++) {
		out[i] += t * vel[i] + t2 * acc[i];
		out[i + 3] += t * acc[i];
	}
}

SURVIVE_EXPORT void survive_imu_integrate_velocity_acceleration(SurviveIMUTracker *tracker, FLT time,
																const FLT *velocity_accel, const FLT *R) {
	// clang-format off
	FLT _H[6 * 9] = {
		0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1,
	};
	// clang-format on
	CvMat H = cvMat(6, tracker->position.info.state_cnt, SURVIVE_CV_F, _H);
	CvMat Zp = cvMat(6, 1, SURVIVE_CV_F, (void *)velocity_accel);
	survive_kalman_predict_update_state(time, &tracker->position, &Zp, &H, R);

	SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(200, "Resultant state " Point9_format, LINMATH_VEC9_EXPAND(tracker->position.state));
}
SURVIVE_EXPORT void survive_imu_integrate_velocity(SurviveIMUTracker *tracker, FLT time, const LinmathVec3d velocity,
												   const FLT *R) {
	// clang-format off
	FLT _H[3 * 9] = {
		0, 0, 0, 1, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0,
	};
	// clang-format on
	CvMat H = cvMat(3, tracker->position.info.state_cnt, SURVIVE_CV_F, _H);
	CvMat Zp = cvMat(3, 1, SURVIVE_CV_F, (void *)velocity);
	survive_kalman_predict_update_state(time, &tracker->position, &Zp, &H, R);
}

SURVIVE_EXPORT void survive_imu_integrate_acceleration(SurviveIMUTracker *tracker, FLT time, const LinmathVec3d accel,
													   const FLT *R) {
	// clang-format off
	FLT _H[3 * 9] = {
		0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1,
	};
	// clang-format on
	CvMat H = cvMat(3, tracker->position.info.state_cnt, SURVIVE_CV_F, _H);
	CvMat Zp = cvMat(3, 1, SURVIVE_CV_F, (void *)accel);
	survive_kalman_predict_update_state(time, &tracker->position, &Zp, &H, R);

	SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(200, "Resultant state " Point9_format, LINMATH_VEC9_EXPAND(tracker->position.state));
}
SURVIVE_EXPORT void survive_imu_integrate_rotation_angular_velocity(SurviveIMUTracker *tracker, FLT time,
																	const FLT *rotation_angular_velocity,
																	const FLT *R) {
	// clang-format off
	FLT _H_rot[7 * 7] = {
		1, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1,
	};
	// clang-format on
	struct SurviveContext *ctx = tracker->so->ctx;

	CvMat H_rot = cvMat(7, tracker->rot.info.state_cnt, SURVIVE_CV_F, _H_rot);
	CvMat Zr = cvMat(7, 1, SURVIVE_CV_F, (void *)rotation_angular_velocity);
	survive_kalman_predict_update_state(time, &tracker->rot, &Zr, &H_rot, R);
	quatnormalize(tracker->rot.state, tracker->rot.state);
}

SURVIVE_EXPORT void survive_imu_integrate_angular_velocity(SurviveIMUTracker *tracker, FLT time,
														   const LinmathAxisAngle angular_velocity, const FLT *R) {
	// clang-format off
	FLT _H_rot[3 * 7] = {
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1,
	};
	// clang-format on
	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(120, "integrate_angular_velocity " Point3_format, LINMATH_VEC3_EXPAND(angular_velocity));

	CvMat H_rot = cvMat(3, tracker->rot.info.state_cnt, SURVIVE_CV_F, _H_rot);
	CvMat Zr = cvMat(3, 1, SURVIVE_CV_F, (void *)angular_velocity);
	survive_kalman_predict_update_state(time, &tracker->rot, &Zr, &H_rot, R);
	quatnormalize(tracker->rot.state, tracker->rot.state);
}

void survive_imu_integrate_rotation(SurviveIMUTracker *tracker, FLT time, const LinmathQuat rotation, const FLT *R) {
	// clang-format off
	FLT _H_rot[4 * 7] = {
		1., 0, 0, 0, 0, 0, 0,
		0, 1., 0, 0, 0, 0, 0,
		0, 0, 1., 0, 0, 0, 0,
		0, 0, 0, 1., 0, 0, 0,
	};
	// clang-format on
	CvMat H_rot = cvMat(4, tracker->rot.info.state_cnt, SURVIVE_CV_F, _H_rot);
	CvMat Zr = cvMat(4, 1, SURVIVE_CV_F, (void *)rotation);
	survive_kalman_predict_update_state(time, &tracker->rot, &Zr, &H_rot, R);
	quatnormalize(tracker->rot.state, tracker->rot.state);
}

void survive_imu_integrate_position(SurviveIMUTracker *tracker, FLT time, const LinmathVec3d position, const FLT *R) {
	// clang-format off
	FLT _H[3 * 9] = {
		1., 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1., 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1., 0, 0, 0, 0, 0, 0,
	};
	// clang-format on
	CvMat H = cvMat(3, tracker->position.info.state_cnt, SURVIVE_CV_F, _H);
	CvMat Zp = cvMat(3, 1, SURVIVE_CV_F, (void *)position);
	survive_kalman_predict_update_state(time, &tracker->position, &Zp, &H, R);
}

void survive_imu_tracker_integrate_observation(survive_long_timecode timecode, SurviveIMUTracker *tracker,
											   const SurvivePose *pose, const FLT *R) {
	if (tracker->last_data.datamask == 0) {
		tracker->last_data.datamask = 1;
	}

	struct SurviveContext *ctx = tracker->so->ctx;
	FLT time = timecode / (FLT)tracker->so->timebase_hz;
	if (tracker->position.t == 0) {
		tracker->rot.t = tracker->position.t = time;
	}

	if (time - tracker->rot.t < 0) {
		if (time - tracker->rot.t > -.1) {
			// time = tracker->rot.t;
		} else {
			// SV_WARN("Processing light data from the past %fs", time - tracker->rot.t );
			tracker->stats.late_light_dropped++;
			return;
		}
	}

	if (tracker->light_pos_var >= 0) {
		FLT Rps = tracker->light_pos_var + R[0];
		FLT Rp[] = {Rps, Rps, Rps};
		SV_VERBOSE(200, "Integrating pose position " Point3_format " with cov " Point3_format,
				   LINMATH_VEC3_EXPAND(pose->Pos), LINMATH_VEC3_EXPAND(Rp));
		survive_imu_integrate_position(tracker, time, pose->Pos, Rp);
	}

	if (tracker->light_rot_var >= 0) {
		FLT Rrs = tracker->light_rot_var + R[1];
		FLT Rr[] = {Rrs, Rrs, Rrs, Rrs};
		SV_VERBOSE(200, "Integrating pose rotation " Point4_format " with cov " Point4_format,
				   LINMATH_VEC4_EXPAND(pose->Rot), LINMATH_VEC4_EXPAND(Rr));
		survive_imu_integrate_rotation(tracker, time, pose->Rot, Rr);
	}
}

STATIC_CONFIG_ITEM(POSE_POSITION_VARIANCE_SEC, "filter-pose-var-per-sec", 'f', "Position variance per second", 5e-2)
STATIC_CONFIG_ITEM(POSE_ROT_VARIANCE_SEC, "filter-pose-rot-var-per-sec", 'f', "Position rotational variance per second",
				   1e-1)

STATIC_CONFIG_ITEM(VELOCITY_POSITION_VARIANCE_SEC, "filter-vel-var-per-sec", 'f', "Velocity variance per second", -1.)
STATIC_CONFIG_ITEM(VELOCITY_ROT_VARIANCE_SEC, "filter-vel-rot-var-per-sec", 'f',
				   "Velocity rotational variance per second", -1.)

STATIC_CONFIG_ITEM(ACCEL_POSITION_VARIANCE_SEC_TAG, "filter-acc-var-per-sec", 'f', "Accel variance per second", -1.)

STATIC_CONFIG_ITEM(LIGHT_POS_VARIANCE, "light-pos-variance", 'f', "Variance of position integration from light capture",
				   .01)
STATIC_CONFIG_ITEM(LIGHT_ROT_VARIANCE, "light-rot-variance", 'f', "Variance of rotation integration from light capture",
				   .01)

STATIC_CONFIG_ITEM(IMU_ACC_VARIANCE, "imu-acc-variance", 'f', "Variance of accelerometer", -1.)
STATIC_CONFIG_ITEM(IMU_ACC_TILT_VARIANCE, "imu-acc-tilt-variance", 'f', "Variance of accelerometer for tilt correction",
				   0.1)
STATIC_CONFIG_ITEM(IMU_GYRO_VARIANCE, "imu-gyro-variance", 'f', "Variance of gyroscope", 1e-1)

static void rot_f_quat(FLT t, FLT *F, const struct CvMat *x) {
	(void)x;

	// assert(fabs(t) < .1 && t >= 0);
	if (fabs(t) > .11)
		t = .11;

	// fprintf(stderr, "F eval: %f " SurvivePose_format "\n", t, SURVIVE_POSE_EXPAND(*(SurvivePose*)x->data.db));
	gen_imu_rot_f_jac_imu_rot(F, t, CV_FLT_PTR(x));

	for (int j = 0; j < 49; j++) {
		assert(!isnan(F[j]));
	}
}

static void pos_f(FLT t, FLT *F, const struct CvMat *x) {
	(void)x;

	// assert(fabs(t) < .1 && t >= 0);
	if (fabs(t) > .25)
		t = .25;

	FLT a = t * t / 2.;
	// clang-format off
	const FLT f[] = {
		1, 0, 0, t, 0, 0, a, 0, 0,
		0, 1, 0, 0, t, 0, 0, a, 0,
		0, 0, 1, 0, 0, t, 0, 0, a,
		0, 0, 0, 1, 0, 0, t, 0, 0,
		0, 0, 0, 0, 1, 0, 0, t, 0,
		0, 0, 0, 0, 0, 1, 0, 0, t,
		0, 0, 0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1
	};
	// clang-format on
	assert(sizeof(f) / sizeof(FLT) == 81);
	memcpy(F, f, sizeof(f));
}

void survive_imu_tracker_init(SurviveIMUTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));

	tracker->so = so;

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(110, "Initializing Filter:");
	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.

	FLT accel_pos_var = survive_configf(ctx, ACCEL_POSITION_VARIANCE_SEC_TAG_TAG, SC_GET, 0.0);

	FLT vel_pos_var = survive_configf(ctx, VELOCITY_POSITION_VARIANCE_SEC_TAG, SC_GET, 0.0);
	FLT vel_rot_var = survive_configf(ctx, VELOCITY_ROT_VARIANCE_SEC_TAG, SC_GET, 0.0);

	FLT pos_var = survive_configf(ctx, POSE_POSITION_VARIANCE_SEC_TAG, SC_GET, 0.0);
	FLT rot_var = survive_configf(ctx, POSE_ROT_VARIANCE_SEC_TAG, SC_GET, 0.0);

	if (vel_rot_var < 0)
		vel_rot_var = rot_var;
	if (vel_pos_var < 0)
		vel_pos_var = pos_var;
	if (accel_pos_var < 0)
		accel_pos_var = vel_pos_var;

	survive_attach_configf(tracker->so->ctx, IMU_ACC_TILT_VARIANCE_TAG, &tracker->acc_tilt_var);
	survive_attach_configf(tracker->so->ctx, IMU_ACC_VARIANCE_TAG, &tracker->acc_var);
	survive_attach_configf(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->gyro_var);
	survive_attach_configf(tracker->so->ctx, LIGHT_POS_VARIANCE_TAG, &tracker->light_pos_var);
	survive_attach_configf(tracker->so->ctx, LIGHT_ROT_VARIANCE_TAG, &tracker->light_rot_var);

	survive_kalman_set_logging_level(ctx->log_level);
	survive_kalman_state_init(&tracker->rot, 7, rot_f_quat, tracker->rot_Q_per_sec, 0, 0);
	tracker->rot.info.Predict_fn = rot_predict_quat;

	// tracker->rot.state[0] = 1.;

	survive_kalman_state_init(&tracker->position, 9, pos_f, tracker->pos_Q_per_sec, 0, 0);
	tracker->position.info.Predict_fn = pos_predict;

	FLT pos_Q_diag[] = {pos_var,	 pos_var,		pos_var,	   vel_pos_var,	 vel_pos_var,
						vel_pos_var, accel_pos_var, accel_pos_var, accel_pos_var};
	FLT rot_Q_diag[] = {rot_var, rot_var, rot_var, rot_var, vel_rot_var, vel_rot_var, vel_rot_var};
	arr_eye_diag(tracker->pos_Q_per_sec, 9, 9, pos_Q_diag);
	arr_eye_diag(tracker->rot_Q_per_sec, 7, 7, rot_Q_diag);

	FLT pos_P_diag[9] = {1e9, 1e9, 1e9};
	FLT rot_P_diag[7] = {1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9};

	tracker->rot.state[0] = 1.;

	survive_kalman_set_P(&tracker->position, pos_P_diag);
	survive_kalman_set_P(&tracker->rot, rot_P_diag);

	SV_VERBOSE(110, "\t%s: %f", POSE_POSITION_VARIANCE_SEC_TAG, tracker->pos_Q_per_sec[0]);
	SV_VERBOSE(110, "\t%s: %f", POSE_ROT_VARIANCE_SEC_TAG, tracker->rot_Q_per_sec[0]);
	SV_VERBOSE(110, "\t%s: %f", VELOCITY_POSITION_VARIANCE_SEC_TAG, tracker->pos_Q_per_sec[4]);
	SV_VERBOSE(110, "\t%s: %f", VELOCITY_ROT_VARIANCE_SEC_TAG, tracker->rot_Q_per_sec[3]);
	SV_VERBOSE(110, "\t%s: %f", IMU_ACC_VARIANCE_TAG, tracker->acc_var);
	SV_VERBOSE(110, "\t%s: %f", IMU_GYRO_VARIANCE_TAG, tracker->gyro_var);
	SV_VERBOSE(110, "\t%s: %f", IMU_ACC_TILT_VARIANCE_TAG, tracker->acc_tilt_var);
}

SurviveVelocity survive_imu_velocity(const SurviveIMUTracker *tracker) {
	SurviveVelocity rtn = {0};
	survive_kalman_predict_state(0, &tracker->position, 3, 6, rtn.Pos);
	survive_kalman_predict_state(0, &tracker->rot, 4, 7, rtn.AxisAngleRot);
	return rtn;
}

void survive_imu_tracker_free(SurviveIMUTracker *tracker) {
	survive_kalman_state_free(&tracker->position);
	survive_kalman_state_free(&tracker->rot);

	survive_detach_config(tracker->so->ctx, IMU_ACC_VARIANCE_TAG, &tracker->acc_var);
	survive_detach_config(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->gyro_var);
	survive_detach_config(tracker->so->ctx, IMU_GYRO_VARIANCE_TAG, &tracker->acc_tilt_var);

	survive_detach_config(tracker->so->ctx, LIGHT_POS_VARIANCE_TAG, &tracker->light_pos_var);
	survive_detach_config(tracker->so->ctx, LIGHT_ROT_VARIANCE_TAG, &tracker->light_rot_var);
}
