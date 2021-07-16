#include "survive_reproject.h"
#include "test_case.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <survive_reproject_gen2.h>

typedef struct {
	LinmathPoint3d pt;
	FLT expected[2];
} ExampleAndValue;

TEST(Reproject, ReprojectTestCase) {
	BaseStationCal cal[2] = {
		{0.0228424072265625, tan(-0.00945281982421875), 0.0023136138916015625, 1.810546875, 0.0206146240234375, 0, 0},
		{0.0290985107421875, tan(-0.00785064697265625), 0.0020542144775390625, -1.1767578125, -0.01227569580078125, 0,
		 0}};

	LinmathPoint3d ptInLh = {0.6626900065515442, -1.4600844631038516, -2.2421641136382124};
	SurviveAngleReading ang = {0};
	survive_reproject_xy(cal, ptInLh, ang);

	ASSERT_DOUBLE_EQ(ang[0], 1.2759991100686086 - M_PI_2);
	ASSERT_DOUBLE_EQ(ang[1], 0.97860273679241472 - M_PI_2);

	return 0;
}

TEST(Reproject, ReprojectTest) {
	FLT out1[2], out2[2], out3[2], out4[2];
	BaseStationCal bsd[2] = { 0 };
	LinmathPoint3d ptInLH1 = {0, 0, -2};
	LinmathPoint3d ptInLH2 = {0, -1, -2};
	LinmathPoint3d ptInLH3 = {0, 1, -2};
	LinmathPoint3d ptInLH4 = {1, 0, -2};
	survive_reproject_xy(bsd, ptInLH1, out1);
	survive_reproject_xy(bsd, ptInLH2, out2);
	survive_reproject_xy(bsd, ptInLH3, out3);
	survive_reproject_xy(bsd, ptInLH4, out4);

	return 0;
}

TEST(Reproject, ReprojectFull) {
	BaseStationCal cal[2] = { 0 };
	SurvivePose lh2world = {.Pos = {1, 1, 1}, .Rot = {0, 0, 1, 0}};
	SurvivePose world2lh = InvertPoseRtn(&lh2world);
	SurvivePose obj2world = {.Pos = {5, 5, 5}, .Rot = {0, 1, 0, 0}};
	const LinmathPoint3d objInPt = {1, 1, 1};

	SurviveAngleReading ang;
	survive_reproject_full(cal, &world2lh, &obj2world, objInPt, ang);

	ASSERT_DOUBLE_EQ(ang[0], 1.0303768265243125);
	ASSERT_DOUBLE_EQ(ang[1], 0.78539816339744828);

	const LinmathPoint3d objInWorld = {6, 4, 4};
	survive_reproject_from_pose_with_bcal(cal, &world2lh, objInWorld, ang);
	ASSERT_DOUBLE_EQ(ang[0], 1.0303768265243125);
	ASSERT_DOUBLE_EQ(ang[1], 0.78539816339744828);

	return 0;
}

TEST(Reproject, Extents) {
	FLT out[2];
	BaseStationCal cal[2] = { 0 };

	ExampleAndValue examples[] = {
		{.pt = {0, 1, -1}, .expected = {0, M_PI / 4.}},
		{.pt = {0, -1, -1}, .expected = {0, -M_PI / 4.}},
		{.pt = {1, 0, -1}, .expected = {-M_PI / 4., 0}},
		{.pt = {-1, 0, -1}, .expected = {M_PI / 4., 0}},
		{.pt = {0, 0, -1}, .expected = {0, 0}},
		{.pt = {100000000000, 0, -.0000000000001}, .expected = {-M_PI / 2., 0}},
		{.pt = {-100000000000, 0, -.0000000000001}, .expected = {M_PI / 2., 0}},
		{.pt = {0, 100000000000, -.0000000000001}, .expected = {0, M_PI / 2.}},
		{.pt = {0, -100000000000, -.0000000000001}, .expected = {0, -M_PI / 2.}},
	};

	for (int i = 0; i < sizeof(examples) / sizeof(ExampleAndValue); i++) {
		survive_reproject_xy(cal, examples[i].pt, out);
		ASSERT_DOUBLE_EQ(out[0], examples[i].expected[0]);
		ASSERT_DOUBLE_EQ(out[1], examples[i].expected[1]);
	}

	return 0;
}

TEST(Reproject, Extents_gen2) {
	FLT out[2];
	BaseStationCal cal[2] = {0};

	ExampleAndValue examples[] = {
		{.pt = {0, 0, -1}, .expected = {0, 0}},
		{.pt = {0, 1, -1}, .expected = {-0.615480, 0.615480}},
	};

	for (int i = 0; i < sizeof(examples) / sizeof(ExampleAndValue); i++) {
		survive_reproject_xy_gen2(cal, examples[i].pt, out);
		ASSERT_DOUBLE_EQ(out[0], examples[i].expected[0]);
		ASSERT_DOUBLE_EQ(out[1], examples[i].expected[1]);
	}

	return 0;
}
