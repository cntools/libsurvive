
#include "../generated/survive_imu.generated.h"
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

	LinmathQuat gt = {0.539878, 0.831712, 0.027367, -0.126641};
	fprintf(stderr, "%.15f %.15f %.15f %.15f\n", c[0], c[1], c[2], c[3]);
	fprintf(stderr, "%.15f %.15f %.15f %.15f\n", gt[0], gt[1], gt[2], gt[3]);
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