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
};

static int TestKalmanIntegratePose(FLT pvariance, FLT rot_variance) {
	SurviveIMUTracker kpose = {};
	SurviveObject so = {.timebase_hz = 48000000};
	survive_imu_tracker_init(&kpose, &so);
	SurvivePose pose = {};
	srand(42);

	const FLT mvariance = 0.5;
	const FLT rot_movement = 0.01;

	LinmathVec3d pos_variance = {pvariance, pvariance, pvariance};

	survive_timecode time = 0;
	survive_timecode ticks_per_second = 4800000;
	for (int i = 0; i < 10000; i++) {
		time += ticks_per_second;

		pose = randomMovement(&pose, mvariance, rot_movement);
		SurvivePose obs = randomMovement(&pose, pvariance, rot_variance);

		FLT variance[2] = {pvariance, rot_variance};
		survive_imu_tracker_integrate_observation(time, &kpose, &obs, variance);

		SurvivePose estimate = kpose.pose;
		// printf("Observation %f %f %f\n", obs.Pos[0], obs.Pos[1], obs.Pos[2]);
		// printf("Actual      %f %f %f\n", pose.Pos[0], pose.Pos[1], pose.Pos[2]);
		// printf("Estimate    %f %f %f\n\n", estimate.Pos[0], estimate.Pos[1], estimate.Pos[2]);

		double acceptable_cdf = CDF(-3, pvariance);
		for (int i = 0; i < 3; i++) {
			double cdf = fabs(0.5 - CDF(fabs(estimate.Pos[i] - pose.Pos[i] + 10000), pvariance));
			// printf("CDF %f %f\n", cdf, acceptable_cdf);
			ASSERT_GT(cdf, acceptable_cdf);
		}
	}

	return 0;
}

TEST(Kalman, Exact) {
	const FLT pvariance = 0.;
	const FLT rot_variance = 0.1;
	return TestKalmanIntegratePose(pvariance, rot_variance);
}

TEST(Kalman, Inexact) {
	const FLT pvariance = 1.;
	const FLT rot_variance = 0.1;
	return TestKalmanIntegratePose(pvariance, rot_variance);
}
