#include "../survive_imu.h"
#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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

static int TestKalmanIntegratePose(FLT pvariance, FLT rot_variance) {
	SurviveIMUTracker kpose = { 0 };
	SurviveObject so = {.timebase_hz = 48000000};
	survive_imu_tracker_init(&kpose, &so);
	SurvivePose pose = LinmathPose_Identity;

	srand(42);

	const FLT mvariance = 0.5;
	const FLT rot_movement = 0.01;

	LinmathVec3d pos_variance = {pvariance, pvariance, pvariance};

	SurviveVelocity vel = randomVelocity(mvariance, rot_movement);
	LinmathQuat qq;
	quatfromaxisanglemag(qq, vel.AxisAngleRot);

	survive_timecode time = 0;
	survive_timecode ticks_per_second = 4800000;
	for (int i = 0; i < 100; i++) {
		time += ticks_per_second;

		survive_apply_velocity(&pose, ticks_per_second / (FLT)so.timebase_hz, &pose, &vel);

		SurvivePose obs = randomMovement(&pose, 0 * pvariance, 0 * rot_variance);

		FLT variance[2] = {pvariance, rot_variance};
		survive_imu_tracker_integrate_observation(time, &kpose, &obs, variance);

		SurvivePose estimate;
		SurviveVelocity estimate_v = survive_imu_velocity(&kpose);
		survive_imu_tracker_predict(&kpose, time, &estimate);

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

	survive_imu_tracker_free(&kpose);

	return 0;
}

TEST(Kalman, Exact) {
	const FLT pvariance = 0.0000000001;
	const FLT rot_variance = 0.01;
	return TestKalmanIntegratePose(pvariance, rot_variance);
}

TEST(Kalman, Inexact) {
	const FLT pvariance = 1.;
	const FLT rot_variance = 0.01;
	return TestKalmanIntegratePose(pvariance, rot_variance);
}
