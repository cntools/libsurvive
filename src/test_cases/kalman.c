#include "../src/survive_kalman_tracker.h"
#include "test_case.h"
#include <cnkalman/kalman.h>
#include <cnmatrix/cn_matrix.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include "malloc.h"
#endif

static void rot_predict_quat(FLT t, const cnkalman_state_t *k, const CnMat *f_in, CnMat *f_out) {
	(void)k;

	const FLT *rot = CN_FLT_PTR(f_in);
	const FLT *vel = CN_FLT_PTR(f_in) + 4;
	copy3d(CN_FLT_PTR(f_out) + 4, vel);

	survive_apply_ang_velocity(CN_FLT_PTR(f_out), vel, t, rot);
}

static const double two_pi = 2.0 * 3.14159265358979323846;

static double CDF(double x, double sigma) {
	if (sigma == 0)
		return (x) == 0.0 ? 0.5 : 0.;
	return (1. + erf((x) / sigma / sqrt(2.))) / 2.;
}

// https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
static double generateGaussianNoise(double mu, double sigma) {
	static const double epsilon = 0.0000001;

	static double z1;
	static bool generate;
	generate = !generate;

	if (!generate)
		return z1 * sigma + mu;

	double u1, u2;
	do {
		u1 = rand() * (1.0 / RAND_MAX);
		u2 = rand() * (1.0 / RAND_MAX);
	} while (u1 <= epsilon);

	double z0;
	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	return z0 * sigma + mu;
}

static SurvivePose randomMovement(const SurvivePose *start, FLT pvariance, FLT rvariance) {
	SurvivePose rtn = *start;
	for (int i = 0; i < 3; i++)
		rtn.Pos[i] += generateGaussianNoise(0, pvariance);
	for (int i = 0; i < 4; i++)
		rtn.Rot[i] += generateGaussianNoise(0, rvariance);

	return rtn;
}

static SurviveVelocity randomVelocity(FLT pvariance, FLT rvariance) {
	SurviveVelocity rtn = {0};
	for (int i = 0; i < 3; i++)
		rtn.Pos[i] += generateGaussianNoise(0, pvariance);
	for (int i = 0; i < 3; i++)
		rtn.AxisAngleRot[i] += generateGaussianNoise(0, rvariance);

	return rtn;
}

static void survive_apply_velocity(SurvivePose *p_1, FLT t, const SurvivePose *p_0, const SurviveVelocity *vel) {
	LinmathVec3d posDisplacement;
	scale3d(posDisplacement, vel->Pos, t);

	add3d(p_1->Pos, p_0->Pos, posDisplacement);

	survive_apply_ang_velocity(p_1->Rot, vel->AxisAngleRot, t, p_0->Rot);
}

int TestKalmanIntegratePose(FLT pvariance, FLT rot_variance) {
	SurviveKalmanTracker kpose = {0};
	SurviveObject so = {.timebase_hz = 48000000};
	survive_kalman_tracker_init(&kpose, &so);
	SurvivePose pose = LinmathPose_Identity;

	srand(42);

	const FLT mvariance = 0.5;
	const FLT rot_movement = 0.01;

	LinmathVec3d pos_variance = {pvariance, pvariance, pvariance};

	SurviveVelocity vel = randomVelocity(mvariance, rot_movement);
	LinmathQuat qq;
	quatfromaxisanglemag(qq, vel.AxisAngleRot);

	survive_long_timecode time = 0;
	survive_long_timecode ticks_per_second = 4800000;
	for (int i = 0; i < 100; i++) {
		time += ticks_per_second;

		survive_apply_velocity(&pose, ticks_per_second / (FLT)so.timebase_hz, &pose, &vel);

		SurvivePose obs = randomMovement(&pose, 0 * pvariance, 0 * rot_variance);

		FLT variance[2] = {pvariance, rot_variance};
		PoserDataLightGen2 pd = {0};
		pd.common.hdr.timecode = time;
		CN_CREATE_STACK_MAT(R, 2, 2);
		cn_set_diag(&R, variance);
		survive_kalman_tracker_integrate_observation(&pd.common.hdr, &kpose, &obs, &R);

		SurvivePose estimate;
		SurviveVelocity estimate_v = survive_kalman_tracker_velocity(&kpose);
		survive_kalman_tracker_predict(&kpose, time, &estimate);

		printf("Observation " SurvivePose_format "\n", SURVIVE_POSE_EXPAND(obs));
		printf("Actual      " SurvivePose_format " " SurviveVel_format "\n", SURVIVE_POSE_EXPAND(pose),
			   SURVIVE_VELOCITY_EXPAND(vel));
		printf("Estimate    " SurvivePose_format " " SurviveVel_format "\n\n", SURVIVE_POSE_EXPAND(estimate),
			   SURVIVE_VELOCITY_EXPAND(estimate_v));

		double acceptable_cdf = CDF(-3, pvariance);
		for (int i = 0; i < 3; i++) {
			double cdf = fabs(0.5 - CDF(fabs(estimate.Pos[i] - pose.Pos[i] + 10000), pvariance));
			// printf("CDF %f %f\n", cdf, acceptable_cdf);
			ASSERT_GT(cdf, acceptable_cdf);
		}
	}

	survive_kalman_tracker_free(&kpose);

	return 0;
}
#include "string.h"

void pos_f(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, struct CnMat *oF) {
	FLT t = 1;
	FLT f[36] = {
		1, 0, 0, t, 0, 0, 0, 1, 0, 0, t, 0, 0, 0, 1, 0, 0, t, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
	};
	CnMat F = cnMat(6, 6, f);
	cnkalman_linear_update(&F, x0, x1);

	if (oF) {
		cnCopy(&F, oF, 0);
	}
}

void meas_model(struct CnMat *Z, const struct CnMat *x_t, const FLT *s) {
	FLT *_h_x = cn_as_vector(Z);
	const FLT *xt = cn_as_const_vector(x_t);

	LinmathPoint3d d;

	sub3d(d, xt, s);
	// fprintf(stderr, Point3_format "\n", LINMATH_VEC3_EXPAND(d));
	_h_x[0] = atan2(d[0], d[1]);
	_h_x[1] = atan2(d[2], sqrt(d[0] * d[0] + d[1] * d[1]));
	_h_x[2] = norm3d(d);
}

bool map_to_obs(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *yhat, struct CnMat *H_k) {

	const FLT *s = (FLT *)user;
	CN_CREATE_STACK_MAT(h_x_t, 3, 1);
	meas_model(&h_x_t, x_t, s);

	CN_CREATE_STACK_MAT(Id, 3, 3);
	cn_set_diag_val(&Id, 1);
	cnGEMM(&Id, Z, 1., &h_x_t, -1, yhat, 0);

	const FLT *xt = cn_as_const_vector(x_t);
	LinmathPoint3d d;
	sub3d(d, xt, s);

	FLT x = d[0], y = d[1], z = d[2];
	FLT n2 = (x * x + y * y + z * z);
	FLT n = sqrtf(n2);

	if (H_k) {
		cn_set_zero(H_k);
		cnMatrixSet(H_k, 0, 0, y / (x * x + y * y));
		cnMatrixSet(H_k, 0, 1, -x / (x * x + y * y));
		cnMatrixSet(H_k, 0, 2, 0);

		cnMatrixSet(H_k, 1, 0, -x * z / n2 / sqrtf(x * x + y * y));
		cnMatrixSet(H_k, 1, 1, -y * z / n2 / sqrtf(x * x + y * y));
		cnMatrixSet(H_k, 1, 2, 1. / sqrtf(x * x + y * y));

		cnMatrixSet(H_k, 2, 0, x / n);
		cnMatrixSet(H_k, 2, 1, y / n);
		cnMatrixSet(H_k, 2, 2, z / n);
	}
	CN_FREE_STACK_MAT(Id);
	CN_FREE_STACK_MAT(h_x_t);
	return true;
}

// https://www.intechopen.com/books/introduction-and-implementations-of-the-kalman-filter/introduction-to-kalman-filter-and-its-applications
TEST(Kalman, ExampleExtended) {
	// cnkalman_set_logging_level(1000);

	cnkalman_state_t position;
	struct cnkalman_meas_model measModel = {.name = "obs", .Hfn = map_to_obs, .ks = {&position}, .ks_cnt = 1};
	FLT pos_Q_per_sec_fixed[36] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 25,
	};
	CN_CREATE_STACK_MAT(pos_Q_per_sec, 6, 6);
	cn_copy_in_row_major(&pos_Q_per_sec, pos_Q_per_sec_fixed, 6);

	FLT P_init[6] = {1, 1, 1, 0, 0, 0};

	cnkalman_state_init(&position, 6, pos_f, 0, &pos_Q_per_sec, 0);
	CnMat P = position.P;
	cn_set_diag(&P, P_init);

	// CN_CREATE_STACK_MAT(F, 6, 6);
	// pos_f(1, &F, 0);

	FLT _true_state[] = {9, -12, 0, -1, -2, 0};
	CnMat true_state = cnMat(6, 1, _true_state);

	FLT _init_state[] = {10, -10, 0, -1, -2, 0};
	memcpy(CN_FLT_PTR(&position.state), _init_state, sizeof(_true_state));

	CN_FLT_PTR(&position.state)[0] += generateGaussianNoise(0, 1);
	CN_FLT_PTR(&position.state)[1] += generateGaussianNoise(0, 1);

	FLT meas_s[] = {.002, .002, .002};
	CN_CREATE_STACK_MAT(Z, 3, 1);

	for (int i = 1; i < 21; i++) {
		LinmathPoint3d sensor = {20 + 20 * cos(2. * LINMATHPI / 30 * (i)), 20 + 20 * sin(2. * LINMATHPI / 30 * (i)),
								 50};

		meas_model(&Z, &true_state, sensor);

		fprintf(stderr, "z_true " Point3_format "\n", LINMATH_VEC3_EXPAND(_Z));

		for (int j = 0; j < 3; j++) {
			_Z[j] += generateGaussianNoise(0, meas_s[j]);
		}

		FLT _R[] = {.0004, .0004, 1};
		CnMat R = cnVec(3, _R);

		cnkalman_meas_model_predict_update(i, &measModel, sensor, &Z, &R);

		fprintf(stderr, "Guess  " SurviveVel_format "\n",
				SURVIVE_VELOCITY_EXPAND(*(SurviveVelocity *)CN_FLT_PTR(&position.state)));
		fprintf(stderr, "GT     " SurviveVel_format "\n", SURVIVE_VELOCITY_EXPAND(*(SurviveVelocity *)_true_state));
		fprintf(stderr, "Sensor " Point3_format "\n", LINMATH_VEC3_EXPAND(sensor));

		FLT diff[6];
		subnd(diff, cn_as_const_vector(&position.state), _true_state, 6);
		FLT err = normnd(diff, 6);
		fprintf(stderr, "Error %f\n", err);
		assert(err < 1);

		CN_CREATE_STACK_MAT(next_state, 6, 1);
		pos_f(1, 0, &true_state, &next_state, 0);
		memcpy(_true_state, cn_as_vector(&next_state), sizeof(_true_state));
		CN_FREE_STACK_MAT(next_state);
	}

	cnkalman_state_free(&position);
	CN_FREE_STACK_MAT(Z);
	CN_FREE_STACK_MAT(F);
	CN_FREE_STACK_MAT(pos_Q_per_sec);
	return 0;
}

typedef struct KalmanModelSim {
	SurviveKalmanModel sim_state;
	SurviveKalmanModel true_state;
	struct SurviveKalmanTracker_Params p;
	cnkalman_state_t kalman_t;
	cnkalman_meas_model_t imu_model_t;
} KalmanModelSim;

struct SurviveKalmanTracker_Params default_params() {
	return (struct SurviveKalmanTracker_Params){
		.process_weight_acc = 10.,
		.process_weight_rotation = 1.,
		.process_weight_ang_velocity = 1.,
	};
}

static void survive_kalman_tracker_process_noise_bounce(void *user, FLT t, const CnMat *x, struct CnMat *q_out) {
	struct SurviveKalmanTracker_Params *params = (struct SurviveKalmanTracker_Params *)user;
	survive_kalman_tracker_process_noise(params, false, t, x, q_out);
}

void KalmanModelSim_init(KalmanModelSim *model) {
	quatnormalize(model->sim_state.Pose.Rot, model->sim_state.Pose.Rot);
	quatnormalize(model->true_state.Pose.Rot, model->true_state.Pose.Rot);

	cnkalman_state_init(&model->kalman_t, sizeof(model->sim_state) / sizeof(FLT), survive_kalman_tracker_predict_jac,
						survive_kalman_tracker_process_noise_bounce, &model->p, (FLT *)&model->sim_state);

	cnkalman_meas_model_init(&model->kalman_t, "imu", &model->imu_model_t,
							 survive_kalman_tracker_imu_measurement_model);
	SurviveKalmanModel initial_variance = {.Pose = {.Pos = {1e5, 1e5, 1e5}, .Rot = {0, 1e5, 1e5, 1e5}},
										   .Velocity = {.AxisAngleRot = {1e3, 1e3, 1e3}},
										   .Acc = {1e3, 1e3, 1e3}};

	for (int i = 0; i < model->kalman_t.state_cnt; i++) {
		cnMatrixSet(&model->kalman_t.P, i, i, ((FLT *)&initial_variance)[i]);
	}
}
void KalmanModelSim_dtor(KalmanModelSim *model) { cnkalman_state_free(&model->kalman_t); }

#define KALMAN_MODEL_FORMAT "S: " SurvivePose_format " V: " SurviveVel_format " A: " Point3_format " B: " Point3_format
#define KALMAN_MODEL_EXPAND(m)                                                                                         \
	SURVIVE_POSE_EXPAND(m.Pose), SURVIVE_VELOCITY_EXPAND(m.Velocity), LINMATH_VEC3_EXPAND(m.Acc),                      \
		LINMATH_VEC3_EXPAND(m.GyroBias)

TEST(Kalman, InstFlip) {
	KalmanModelSim model = {.sim_state =
								{
									.Pose = {.Rot = {1, .1}},
								},
							.true_state =
								{
									.Pose = {.Pos = {10, 10, 10}, .Rot = {0, 1}},
								},
							.p = default_params()};

	KalmanModelSim_init(&model);

	fprintf(stderr, "Testing instant flip\n");
	FLT _R[] = {1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2};
	CnMat R = cnVec(6, _R);
	// cnkalman_set_logging_level(10000);
	for (int i = 0; i < 100; i++) {
		FLT time = i / 1000.;

		FLT input[6], h_x[6];
		CnMat Z = cnVec(6, input);

		quatnormalize(model.true_state.Pose.Rot, model.true_state.Pose.Rot);
		gen_imu_predict(input, &model.true_state);

		FLT err = cnkalman_meas_model_predict_update(time, &model.imu_model_t, 0, &Z, &R);
		quatnormalize(model.sim_state.Pose.Rot, model.sim_state.Pose.Rot);
		fprintf(stderr, "err %.7f Acc: " Point3_format " Vel: " Point3_format " Rotation: " Quat_format "\n", err,
				LINMATH_VEC3_EXPAND(model.sim_state.Acc), LINMATH_VEC3_EXPAND(model.sim_state.Velocity.Pos),
				LINMATH_QUAT_EXPAND(model.sim_state.Pose.Rot));
	}
	KalmanModelSim_dtor(&model);
	fprintf(stderr, "\n");

	return 0;
}

TEST(Kalman, Flip) {
	KalmanModelSim model = {.sim_state =
								{
									.Pose = {.Rot = {1}},
								},
							.true_state = {.Pose = {.Rot = {1}}, .Velocity = {.AxisAngleRot = {M_PI}}},
							.p = default_params()};

	KalmanModelSim_init(&model);

	FLT _R[] = {1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2};
	CnMat R = cnVec(6, _R);
	fprintf(stderr, "Testing flip\n");
	// cnkalman_set_logging_level(10000);
	for (int i = 0; i < 1000; i++) {
		FLT time = i / 1000.;

		FLT input[6], h_x[6];
		CnMat Z = cnMat(6, 1, input);

		SurviveKalmanModel m;
		gen_kalman_model_predict((FLT *)&m, 1. / 1000., &model.true_state);
		model.true_state = m;

		gen_imu_predict(input, &model.true_state);

		FLT err = cnkalman_meas_model_predict_update(time, &model.imu_model_t, 0, &Z, &R);
		quatnormalize(model.sim_state.Pose.Rot, model.sim_state.Pose.Rot);

		fprintf(stderr, "err %f Velocity: " SurviveVel_format " Pose: " SurvivePose_format "\n", err,
				SURVIVE_VELOCITY_EXPAND(model.sim_state.Velocity), SURVIVE_POSE_EXPAND(model.sim_state.Pose));
		fprintf(stderr, "             Velocity: " SurviveVel_format " Pose: " SurvivePose_format "\n",
				SURVIVE_VELOCITY_EXPAND(model.true_state.Velocity), SURVIVE_POSE_EXPAND(model.true_state.Pose));
	}
	KalmanModelSim_dtor(&model);
	fprintf(stderr, "\n");

	return 0;
}

static void write_state(FILE *f, const SurviveKalmanModel *m) {
	const FLT *vals = (const FLT *)m;
	for (int i = 0; i < sizeof(*m) / sizeof(FLT); i++) {
		fprintf(f, "%7.7f, ", vals[i]);
	}
	fprintf(f, "\n");
}

TEST(Kalman, LiftupSetDown) {
	KalmanModelSim model = {.sim_state =
								{
									.Pose = {.Rot = {1}},
								},
							.true_state =
								{
									.Pose = {.Rot = {1}},
								},
							.p = default_params()};

	KalmanModelSim_init(&model);

	FLT _R[] = {1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2};
	CnMat R = cnVec(6, _R);

	fprintf(stderr, "Testing LiftupSetDown\n");
	FILE *rf = fopen("real.ccn", "w");
	FILE *sf = fopen("simul.ccn", "w");
	// cnkalman_set_logging_level(10000);
	for (int i = 0; i < 1000; i++) {
		FLT time = i / 1000.;

		FLT input[6], h_x[6];
		CnMat Z = cnMat(6, 1, input);

		model.true_state.Acc[2] = (i > 250 && i < 750) ? -.1 : .1;
		SurviveKalmanModel m;
		gen_kalman_model_predict((FLT *)&m, 1. / 1000., &model.true_state);
		model.true_state = m;

		gen_imu_predict(input, &model.true_state);
		FLT err = cnkalman_meas_model_predict_update(time, &model.imu_model_t, 0, &Z, &R);
		quatnormalize(model.sim_state.Pose.Rot, model.sim_state.Pose.Rot);

		fprintf(stderr, "err %f " KALMAN_MODEL_FORMAT "\n", err, KALMAN_MODEL_EXPAND(model.sim_state));
		fprintf(stderr, "        %4d " KALMAN_MODEL_FORMAT "\n", i, KALMAN_MODEL_EXPAND(model.true_state));
		write_state(rf, &model.true_state);
		write_state(sf, &model.sim_state);
	}
	KalmanModelSim_dtor(&model);
	fclose(rf);
	fclose(sf);

	fprintf(stderr, "\n");

	return 0;
}
