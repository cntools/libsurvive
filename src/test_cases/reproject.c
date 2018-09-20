#include "survive_reproject.h"
#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
	LinmathPoint3d pt;
	FLT expected[2];
} ExampleAndValue;

TEST(Reproject, ReprojectFull) {
	//	void survive_reproject_full(const BaseStationCal *bcal, const SurvivePose *lh2world, const SurvivePose
	//*obj2world, 	                            const LinmathVec3d obj_pt, FLT *out) {
	BaseStationCal cal[2] = {};
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
	BaseStationCal cal[2] = {};

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
