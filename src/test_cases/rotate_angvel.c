
#include "../src/generated/survive_reproject.aux.generated.h"
#include "test_case.h"
#include <linmath.h>

void RotateAngularVelocity(LinmathQuat out, const LinmathQuat rotation, const LinmathQuat av) {
	quatconjugateby(out, rotation, av);
}

int testAngVelEquivalence(const LinmathQuat avInWorld, const LinmathQuat R) {
	LinmathQuat avInR;
	RotateAngularVelocity(avInR, R, avInWorld);

	FLT t = .1;
	LinmathQuat rInWorld;
	quatmultiplyrotation(rInWorld, avInWorld, t);

	LinmathQuat rInR;
	quatmultiplyrotation(rInR, avInR, t);

	LinmathQuat r1, r2;
	LinmathEulerAngle e1, e2;
	quatrotateabout(r1, rInWorld, R);
	quatrotateabout(r2, R, rInR);
	quattoeuler(e1, r1);
	quattoeuler(e2, r2);
	ASSERT_QUAT_EQ(r1, r2);

	return 0;
}

TEST(RotateAngVel, Basic) {
	LinmathQuat av = {9, 0, 0, 1};
	quatnormalize(av, av);

	LinmathQuat flip_along_x = {0, 1, 0, 0};
	ASSERT_SUCCESS(testAngVelEquivalence(av, flip_along_x));

	LinmathQuat flip_along_z = {0, 0, 0, 1};
	ASSERT_SUCCESS(testAngVelEquivalence(av, flip_along_z));

	LinmathQuat flip_along_xyz = {.5, .5, .5, .5};
	ASSERT_SUCCESS(testAngVelEquivalence(av, flip_along_xyz));

	ASSERT_SUCCESS(testAngVelEquivalence(av, LinmathQuat_Identity));

	ASSERT_SUCCESS(testAngVelEquivalence(av, av));

	return 0;
}

TEST(AngularVelocity, find) {
	LinmathQuat a = {1, 0, 0, 0};
	LinmathQuat b = {0, 1, 0, 0};

	SurviveAngularVelocity c;
	survive_find_ang_velocity(c, 1, a, b);

	LinmathQuat c_quat;
	quatfromaxisanglemag(c_quat, c);

	ASSERT_QUAT_EQ(c_quat, b);

	return 0;
}

TEST(AngularVelocity, apply) {
	LinmathQuat a = {1, 0, 0, 0};
	SurviveAngularVelocity b = {LINMATHPI, LINMATHPI, LINMATHPI};

	LinmathQuat c = {0};
	survive_apply_ang_velocity(c, b, 1, a);

	return 0;
}

TEST(AngularVelocity, apply2) {
	/*
	 *
	 * 06-04 05:55:23.269   753   791 I HMDBridge: Velocity Data:  0.634868 0.149354 0.053309 0.102118 -0.247831
-4.802000 0.019992 06-04 05:55:23.269   753   791 I HMDBridge: Pose Data Orig: 0.546112 0.831827 -0.011106 -0.098503
-0.183637 -0.005066 0.649892 06-04 05:55:23.269   753   791 I HMDBridge: Pose Data Proj: 0.539878 0.831712 0.027367
-0.126641 -0.170945 -0.002080 0.650957

	 */
	LinmathQuat a = {0.546112, 0.831827, -0.011106, -0.098503};
	SurviveAngularVelocity b = {0.102118, -0.247831, -4.802000};

	LinmathQuat c = {0};
	survive_apply_ang_velocity(c, b, 0.019992, a);

	LinmathQuat gt = {0.539878160006538, 0.831133907922443, -0.052257988715913, -0.122544286137262};
	ASSERT_QUAT_EQ(c, gt);

	return 0;
}

TEST(AngularVelocity, aa_apply2) {
	/*
	 *
	 * 06-04 05:55:23.269   753   791 I HMDBridge: Velocity Data:  0.634868 0.149354 0.053309 0.102118 -0.247831
-4.802000 0.019992 06-04 05:55:23.269   753   791 I HMDBridge: Pose Data Orig: 0.546112 0.831827 -0.011106 -0.098503
-0.183637 -0.005066 0.649892 06-04 05:55:23.269   753   791 I HMDBridge: Pose Data Proj: 0.539878 0.831712 0.027367
-0.126641 -0.170945 -0.002080 0.650957

	 */
	LinmathQuat a = {0.546112, 0.831827, -0.011106, -0.098503};

	LinmathAxisAngle aa;
	quattoaxisanglemag(aa, a);

	SurviveAngularVelocity b = {0.102118, -0.247831, -4.802000};

	LinmathAxisAngle c_aa = {0};
	survive_apply_ang_velocity_aa(c_aa, b, 0.019992, aa);

	LinmathQuat c;
	quatfromaxisanglemag(c, c_aa);

	LinmathQuat gt = {0.539878160006538, 0.831133907922443, -0.052257988715913, -0.122544286137262};
	ASSERT_QUAT_EQ(c, gt);

	return 0;
}

TEST(AxisAngle, Compose) {
	LinmathAxisAngle a1 = {.1, .2, .3};
	LinmathAxisAngle a2 = {-.2, .4, 0};
	LinmathAxisAngle c1, c2;

	axisanglerotateabout(c1, a1, a2);
	gen_axisanglecompose(c2, a1, a2);

	ASSERT_DOUBLE_ARRAY_EQ(3, c1, c2);
	return 0;
}

void avg_naive(LinmathQuat out, const LinmathQuat q1, const LinmathQuat q2, FLT t) {
	LinmathQuat qd;
	subnd(qd, q1, q2, 4);

	scalend(qd, qd, t, 4);
	addnd(out, qd, q2, 4);
	quatnormalize(out, out);
}

void avg_error_state(LinmathQuat out, const LinmathQuat q1, const LinmathQuat q2, FLT t) {
	LinmathQuat qd;
	subnd(qd, q1, q2, 4);
	scalend(qd, qd, t, 4);

	scalend(qd, qd, .5, 4);
	qd[0] = 1;
	quatnormalize(qd, qd);

	quatrotateabout(out, qd, q1);
	quatnormalize(out, out);
	for (int i = 0; i < 4; i++)
		assert(isfinite(out[i]));
}
void avg_slerp(LinmathQuat out, const LinmathQuat q1, const LinmathQuat q2, FLT t) { quatslerp(out, q1, q2, t); }

void avg_find(LinmathQuat out, const LinmathQuat q1, const LinmathQuat q2, FLT t) {
	LinmathQuat q;
	quatfind(q, q1, q2);
	q[0] = 0; // 1 - fabs(q[0]);
	scalend(q, q, t, 4);
	// q[0] = 1;
	// quatnormalize(q, q);

	addnd(out, q, q2, 4);
	quatnormalize(out, out);

	// quatrotateabout(out, q, q1);
}

typedef void (*avg_fn)(LinmathQuat out, const LinmathQuat q1, const LinmathQuat q2, FLT t);
avg_fn avgFns[] = {avg_naive, avg_find, avg_error_state};

FLT avg_errors[SURVIVE_ARRAY_SIZE(avgFns)] = {0};
int avg_error_failures[SURVIVE_ARRAY_SIZE(avgFns)] = {0};
int err_cnt = 0;
void test_approx(LinmathQuat q1, LinmathQuat q2, FLT t) {
	LinmathQuat q_s;
	avg_slerp(q_s, q1, q2, t);

	FLT err_q1 = quatdifference(q1, q_s);
	FLT err_q2 = quatdifference(q2, q_s);
	FLT err_q = t * err_q1 + (1 - t) * err_q2;

	for (int i = 0; i < SURVIVE_ARRAY_SIZE(avgFns); i++) {
		LinmathQuat q_;
		avgFns[i](q_, q1, q2, t);
		FLT err = quatdifference(q_, q_s) / err_q;
		avg_errors[i] += err;
		avg_error_failures[i] += err > 1;
	}

	err_cnt++;
}

TEST(Quat, ApproxTest) {
	LinmathQuat q1 = {0, 1, 0, 0};
	LinmathQuat q2 = {0, 1, .1, 0};
	quatnormalize(q2, q2);
	test_approx(q1, q2, 1);

	srand(42);
	for (int i = 0; i < 100000; i++) {
		LinmathQuat q1 = {linmath_rand(-1, 1), linmath_rand(-1, 1), linmath_rand(-1, 1), linmath_rand(-1, 1)};
		quatnormalize(q1, q1);
		FLT dx = 1e-2;
		LinmathQuat t = {1 - linmath_rand(-dx, dx), linmath_rand(-dx, dx), linmath_rand(-dx, dx),
						 linmath_rand(-dx, dx)};
		quatnormalize(t, t);

		LinmathQuat q2;
		quatrotateabout(q2, q1, t);

		test_approx(q1, q2, linmath_rand(0, 1));
	}

	printf("Avg error: ");
	for (int i = 0; i < SURVIVE_ARRAY_SIZE(avgFns); i++) {
		printf("%f %d, ", avg_errors[i] / (FLT)err_cnt, avg_error_failures[i]);
	}
	printf("\n");

	return 0;
}

TEST(Quat, EdgeCases) {
	LinmathQuat qm = {0};

	{
		LinmathQuat q1 = {0, 1, 0, 0};
		LinmathQuat q2 = {0, -1, 0, 0};
		quatfind(qm, q1, q2);

		test_approx(q1, q2, .1);
		ASSERT_QUAT_EQ(qm, LinmathQuat_Identity);
	}
	{
		LinmathQuat q1 = {0.0007963, 0.5773501, 0.5773501, 0.5773501};
		LinmathQuat q2 = {-0.0042037, 0.5773452, 0.5773452, 0.5773452};
		quatfind(qm, q1, q2);
		test_approx(q1, q2, .1);
		// ASSERT_QUAT_EQ(qm, LinmathQuat_Identity);
	}
	{
		LinmathQuat q1 = {-0.9999987, 0.0009195, 0.0009195, 0.0009195};
		LinmathQuat q2 = {-0.9999942, -0.0019672, -0.0019672, -0.0019672};
		quatfind(qm, q1, q2);
		test_approx(q1, q2, .1);
		// ASSERT_QUAT_EQ(qm, LinmathQuat_Identity);
	}
	{
		LinmathQuat q1 = {0.01, +.99};
		LinmathQuat q2 = {0.01, -.99};
		quatfind(qm, q1, q2);
		test_approx(q1, q2, .1);
	}

	{
		LinmathQuat q1 = {-0.9986694, 0.0515698};
		LinmathQuat q2 = {-0.9988286, -0.0483884};
		quatfind(qm, q1, q2);
		test_approx(q1, q2, .5);
	}
	return 0;
}
