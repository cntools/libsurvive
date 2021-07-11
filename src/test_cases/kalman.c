#include "../survive_kalman_tracker.h"
#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sv_matrix.h>

#include "../generated/survive_imu.generated.h"
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include "malloc.h"
#endif

static void rot_predict_quat(FLT t, const survive_kalman_state_t *k, const SvMat *f_in, SvMat *f_out) {
	(void)k;

	const FLT *rot = SV_FLT_PTR(f_in);
	const FLT *vel = SV_FLT_PTR(f_in) + 4;
	copy3d(SV_FLT_PTR(f_out) + 4, vel);

	survive_apply_ang_velocity(SV_FLT_PTR(f_out), vel, t, rot);
}

static void rot_f_quat(FLT t, SvMat *F, const struct SvMat *x) {
	(void)x;

	// assert(fabs(t) < .1 && t >= 0);
	if (fabs(t) > .11)
		t = .11;

	// fprintf(stderr, "F eval: %f " SurvivePose_format "\n", t, SURVIVE_POSE_EXPAND(*(SurvivePose*)x->data.db));
	FLT *f_row = alloca(sizeof(FLT) * F->rows * F->cols);
	gen_imu_rot_f_jac_imu_rot(f_row, t, SV_FLT_PTR(x));
	sv_copy_in_row_major(F, f_row, F->cols);

	for (int j = 0; j < 49; j++) {
		assert(!isnan(f_row[j]));
	}
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
		survive_kalman_tracker_integrate_observation(&pd.common.hdr, &kpose, &obs, variance);

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

static void pos_f(FLT t, SvMat *F, const struct SvMat *x) {
	(void)x;
	t = 1;
	FLT f[36] = {
		1, 0, 0, t, 0, 0, 0, 1, 0, 0, t, 0, 0, 0, 1, 0, 0, t, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
	};
	sv_copy_in_row_major(F, f, F->cols);
}

void meas_model(struct SvMat *Z, const struct SvMat *x_t, const FLT *s) {
	FLT *_h_x = sv_as_vector(Z);
	const FLT *xt = sv_as_const_vector(x_t);

	LinmathPoint3d d;

	sub3d(d, xt, s);
	// fprintf(stderr, Point3_format "\n", LINMATH_VEC3_EXPAND(d));
	_h_x[0] = atan2(d[0], d[1]);
	_h_x[1] = atan2(d[2], sqrt(d[0] * d[0] + d[1] * d[1]));
	_h_x[2] = norm3d(d);
}

bool map_to_obs(void *user, const struct SvMat *Z, const struct SvMat *x_t, struct SvMat *yhat, struct SvMat *H_k) {

	const FLT *s = (FLT *)user;
	SV_CREATE_STACK_MAT(h_x_t, 3, 1);
	meas_model(&h_x_t, x_t, s);

	SV_CREATE_STACK_MAT(Id, 3, 3);
	sv_set_diag_val(&Id, 1);
	svGEMM(&Id, Z, 1., &h_x_t, -1, yhat, 0);

	const FLT *xt = sv_as_const_vector(x_t);
	LinmathPoint3d d;
	sub3d(d, xt, s);

	FLT x = d[0], y = d[1], z = d[2];
	FLT n2 = (x * x + y * y + z * z);
	FLT n = sqrtf(n2);

	sv_set_zero(H_k);
	svMatrixSet(H_k, 0, 0, y / (x * x + y * y));
	svMatrixSet(H_k, 0, 1, -x / (x * x + y * y));
	svMatrixSet(H_k, 0, 2, 0);

	svMatrixSet(H_k, 1, 0, -x * z / n2 / sqrtf(x * x + y * y));
	svMatrixSet(H_k, 1, 1, -y * z / n2 / sqrtf(x * x + y * y));
	svMatrixSet(H_k, 1, 2, 1. / sqrtf(x * x + y * y));

	svMatrixSet(H_k, 2, 0, x / n);
	svMatrixSet(H_k, 2, 1, y / n);
	svMatrixSet(H_k, 2, 2, z / n);

	SV_FREE_STACK_MAT(Id);
	SV_FREE_STACK_MAT(h_x_t);
	return true;
}

// https://www.intechopen.com/books/introduction-and-implementations-of-the-kalman-filter/introduction-to-kalman-filter-and-its-applications
TEST(Kalman, ExampleExtended) {
	// survive_kalman_set_logging_level(1000);

	survive_kalman_state_t position;

	FLT pos_Q_per_sec_fixed[36] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0, 0, 0, 25,
	};
	SV_CREATE_STACK_MAT(pos_Q_per_sec, 6, 6);
	sv_copy_in_row_major(&pos_Q_per_sec, pos_Q_per_sec_fixed, 6);

	FLT P_init[6] = {1, 1, 1, 0, 0, 0};

	survive_kalman_state_init(&position, 6, pos_f, 0, &pos_Q_per_sec, 0);
	SvMat P = position.P;
	sv_set_diag(&P, P_init);

	SV_CREATE_STACK_MAT(F, 6, 6);
	pos_f(1, &F, 0);

	FLT _true_state[] = {9, -12, 0, -1, -2, 0};
	SvMat true_state = svMat(6, 1, _true_state);

	FLT _init_state[] = {10, -10, 0, -1, -2, 0};
	memcpy(SV_FLT_PTR(&position.state), _init_state, sizeof(_true_state));

	SV_FLT_PTR(&position.state)[0] += generateGaussianNoise(0, 1);
	SV_FLT_PTR(&position.state)[1] += generateGaussianNoise(0, 1);

	FLT meas_s[] = {.002, .002, .002};
	SV_CREATE_STACK_MAT(Z, 3, 1);

	for (int i = 1; i < 21; i++) {
		LinmathPoint3d sensor = {20 + 20 * cos(2. * LINMATHPI / 30 * (i)), 20 + 20 * sin(2. * LINMATHPI / 30 * (i)),
								 50};

		meas_model(&Z, &true_state, sensor);

		fprintf(stderr, "z_true " Point3_format "\n", LINMATH_VEC3_EXPAND(_Z));

		for (int j = 0; j < 3; j++) {
			_Z[j] += generateGaussianNoise(0, meas_s[j]);
		}

		FLT R[] = {.0004, .0004, 1};
		survive_kalman_predict_update_state_extended(i, &position, &Z, R, map_to_obs, sensor, 0);

		fprintf(stderr, "Guess  " SurviveVel_format "\n",
				SURVIVE_VELOCITY_EXPAND(*(SurviveVelocity *)SV_FLT_PTR(&position.state)));
		fprintf(stderr, "GT     " SurviveVel_format "\n", SURVIVE_VELOCITY_EXPAND(*(SurviveVelocity *)_true_state));
		fprintf(stderr, "Sensor " Point3_format "\n", LINMATH_VEC3_EXPAND(sensor));

		FLT diff[6];
		subnd(diff, sv_as_const_vector(&position.state), _true_state, 6);
		FLT err = normnd(diff, 6);
		fprintf(stderr, "Error %f\n", err);
		assert(err < 1);

		SV_CREATE_STACK_MAT(next_state, 6, 1);
		// SURVIVE_LOCAL_ONLY void cvGEMM(const SvMat *src1, const SvMat *src2, double alpha, const SvMat *src3, double
		// beta,
		svGEMM(&F, &true_state, 1, 0, 0, &next_state, 0);
		memcpy(_true_state, sv_as_vector(&next_state), sizeof(_true_state));
		SV_FREE_STACK_MAT(next_state);
	}

	survive_kalman_state_free(&position);
	SV_FREE_STACK_MAT(Z);
	SV_FREE_STACK_MAT(F);
	SV_FREE_STACK_MAT(pos_Q_per_sec);
	return 0;
}

TEST(Kalman, AngleQuat) {
	// survive_kalman_set_logging_level(1001);
	survive_kalman_state_t rotation;

	FLT av = 1e0, vv = 1e0;
	// clang-format off
	FLT _pos_Q_per_sec[49] = {
		av, 0, 0, 0, 0, 0, 0,
		0, av, 0, 0, 0, 0, 0,
		0,  0,av, 0, 0, 0, 0,
		0,  0, 0, av,0,	0, 0,
		0, 0, 0, 0, vv, 0, 0,
		0, 0, 0, 0, 0, vv, 0,
		0, 0, 0, 0, 0, 0, vv,
	};
	SvMat pos_Q_per_sec = svMat(7, 7, _pos_Q_per_sec);
	// clang-format on

	survive_kalman_state_init(&rotation, 7, rot_f_quat, 0, &pos_Q_per_sec, 0);
	rotation.Predict_fn = rot_predict_quat;

	FLT P_init[] = {100, 100, 100, 100, 100, 100, 100};
	survive_kalman_set_P(&rotation, P_init);

	SV_CREATE_STACK_MAT(true_state, 7, 1);
	FLT true_state_init[7] = {1, 0, 0, 0, 1, 1, -1};
	memcpy(_true_state, true_state_init, sizeof(true_state_init));
	memcpy(SV_FLT_PTR(&rotation.state), true_state_init, sizeof(true_state_init));

	SV_CREATE_STACK_MAT(Z, 4, 1);

	// clang-format off
	FLT _H[] = {
		1, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0,
	};
	// clang-format on
	SvMat H = svMat_from_row_major(4, 7, _H);

	for (int i = 1; i < 100; i++) {
		FLT t = i * .1;

		FLT rv = .01;
		FLT R[] = {rv, rv, rv, rv};
		memcpy(_Z, _true_state, 4 * sizeof(FLT));
		for (int j = 0; j < 4; j++) {
			_Z[j] += generateGaussianNoise(0, R[j]);
		}
		quatnormalize(_Z, _Z);
		survive_kalman_predict_update_state(t, &rotation, &Z, &H, R, 0);
		quatnormalize(SV_FLT_PTR(&rotation.state), SV_FLT_PTR(&rotation.state));
		fprintf(stderr, "Guess  " SurvivePose_format "\n",
				SURVIVE_POSE_EXPAND(*(SurvivePose *)SV_FLT_PTR(&rotation.state)));
		fprintf(stderr, "GT     " SurvivePose_format "\n", SURVIVE_POSE_EXPAND(*(SurvivePose *)_true_state));

		FLT diff[7];
		subnd(diff, SV_FLT_PTR(&rotation.state), _true_state, 7);
		FLT err = normnd(diff, 7);
		fprintf(stderr, "Error %f\n", err);
		assert(err < 1);

		rot_predict_quat(.1, 0, &true_state, &true_state);
	}
	survive_kalman_state_free(&rotation);
	SV_FREE_STACK_MAT(Z);
	SV_FREE_STACK_MAT(true_state);

	return 0;
}

typedef struct KalmanModelSim {
	SurviveKalmanModel sim_state;
	SurviveKalmanModel true_state;
	struct SurviveKalmanTracker_Params p;
	survive_kalman_state_t kalman_t;
} KalmanModelSim;

struct SurviveKalmanTracker_Params default_params() {
	return (struct SurviveKalmanTracker_Params){
		.process_weight_acc = 10.,
		.process_weight_rotation = 1.,
		.process_weight_ang_velocity = 1.,
	};
}

void KalmanModelSim_init(KalmanModelSim *model) {
	quatnormalize(model->sim_state.Pose.Rot, model->sim_state.Pose.Rot);
	quatnormalize(model->true_state.Pose.Rot, model->true_state.Pose.Rot);

	survive_kalman_state_init(
		&model->kalman_t, sizeof(model->sim_state) / sizeof(FLT), survive_kalman_tracker_predict_jac,
		(kalman_process_noise_fn_t)survive_kalman_tracker_process_noise, &model->p, (FLT *)&model->sim_state);
	model->kalman_t.Predict_fn = survive_kalman_tracker_model_predict;

	SurviveKalmanModel initial_variance = {.Pose = {.Pos = {1e10, 1e10, 1e10}, .Rot = {0, 1e10, 1e10, 1e10}},
										   .Velocity = {.AxisAngleRot = {1e3, 1e3, 1e3}},
										   .Acc = {1e3, 1e3, 1e3}};

	for (int i = 0; i < model->kalman_t.state_cnt; i++) {
		svMatrixSet(&model->kalman_t.P, i, i, ((FLT *)&initial_variance)[i]);
	}
}

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
	FLT R[] = {1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2};
	// survive_kalman_set_logging_level(10000);
	for (int i = 0; i < 100; i++) {
		FLT time = i / 1000.;

		FLT input[6], h_x[6];
		SvMat Z = svMat(6, 1, input);

		quatnormalize(model.true_state.Pose.Rot, model.true_state.Pose.Rot);
		gen_imu_predict(input, &model.true_state);

		FLT err = survive_kalman_predict_update_state_extended(time, &model.kalman_t, &Z, R,
															   survive_kalman_tracker_imu_measurement_model, 0, 0);
		quatnormalize(model.sim_state.Pose.Rot, model.sim_state.Pose.Rot);
		fprintf(stderr, "err %.7f Acc: " Point3_format " Vel: " Point3_format " Rotation: " Quat_format "\n", err,
				LINMATH_VEC3_EXPAND(model.sim_state.Acc), LINMATH_VEC3_EXPAND(model.sim_state.Velocity.Pos),
				LINMATH_QUAT_EXPAND(model.sim_state.Pose.Rot));
	}
	survive_kalman_set_logging_level(0);
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

	FLT R[] = {1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2};

	fprintf(stderr, "Testing flip\n");
	// survive_kalman_set_logging_level(10000);
	for (int i = 0; i < 1000; i++) {
		FLT time = i / 1000.;

		FLT input[6], h_x[6];
		SvMat Z = svMat(6, 1, input);

		SurviveKalmanModel m;
		gen_kalman_model_predict((FLT *)&m, 1. / 1000., &model.true_state);
		model.true_state = m;

		gen_imu_predict(input, &model.true_state);

		FLT err = survive_kalman_predict_update_state_extended(time, &model.kalman_t, &Z, R,
															   survive_kalman_tracker_imu_measurement_model, 0, 0);
		quatnormalize(model.sim_state.Pose.Rot, model.sim_state.Pose.Rot);

		fprintf(stderr, "err %f Velocity: " SurviveVel_format " Pose: " SurvivePose_format "\n", err,
				SURVIVE_VELOCITY_EXPAND(model.sim_state.Velocity), SURVIVE_POSE_EXPAND(model.sim_state.Pose));
		fprintf(stderr, "             Velocity: " SurviveVel_format " Pose: " SurvivePose_format "\n",
				SURVIVE_VELOCITY_EXPAND(model.true_state.Velocity), SURVIVE_POSE_EXPAND(model.true_state.Pose));
	}
	survive_kalman_set_logging_level(0);
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

	FLT R[] = {1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2};

	fprintf(stderr, "Testing LiftupSetDown\n");
	FILE *rf = fopen("real.csv", "w");
	FILE *sf = fopen("simul.csv", "w");
	// survive_kalman_set_logging_level(10000);
	for (int i = 0; i < 1000; i++) {
		FLT time = i / 1000.;

		FLT input[6], h_x[6];
		SvMat Z = svMat(6, 1, input);

		model.true_state.Acc[2] = (i > 250 && i < 750) ? -.1 : .1;
		SurviveKalmanModel m;
		gen_kalman_model_predict((FLT *)&m, 1. / 1000., &model.true_state);
		model.true_state = m;

		gen_imu_predict(input, &model.true_state);

		FLT err = survive_kalman_predict_update_state_extended(time, &model.kalman_t, &Z, R,
															   survive_kalman_tracker_imu_measurement_model, 0, 0);
		quatnormalize(model.sim_state.Pose.Rot, model.sim_state.Pose.Rot);

		fprintf(stderr, "err %f " KALMAN_MODEL_FORMAT "\n", err, KALMAN_MODEL_EXPAND(model.sim_state));
		fprintf(stderr, "        %4d " KALMAN_MODEL_FORMAT "\n", i, KALMAN_MODEL_EXPAND(model.true_state));
		write_state(rf, &model.true_state);
		write_state(sf, &model.sim_state);
	}
	fclose(rf);
	fclose(sf);
	survive_kalman_set_logging_level(0);
	fprintf(stderr, "\n");

	return 0;
}
