
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