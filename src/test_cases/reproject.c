#include "survive_reproject.h"
#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
	LinmathPoint3d pt;
	FLT expected[2];
} ExampleAndValue;

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

	return true;
}
