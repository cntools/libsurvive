#pragma once

#include "survive.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	bool enable[2];
	bool invert[2];
	bool swap;
} survive_calibration_options_config;

typedef struct survive_calibration_config {

	survive_calibration_options_config phase, tilt, curve, gibMag, gibPhase;

	bool gibUseSin;

} survive_calibration_config;

void survive_calibration_options_config_apply(const survive_calibration_options_config *option, const FLT *input,
											  FLT *output);

const survive_calibration_config *survive_calibration_default_config(const SurviveContext *ctx);

size_t survive_calibration_config_max_idx();

survive_calibration_config survive_calibration_config_create_from_idx(size_t v);

size_t survive_calibration_config_index(const survive_calibration_config *config);

void survive_reproject(const SurviveContext *ctx, int lighthouse, FLT *point3d, FLT *out);

void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse, const SurvivePose *pose, FLT *point3d,
								 FLT *out);

// This is given a lighthouse -- in the same system as stored in BaseStationData, and
// a 3d point and finds what the effective 'angle' value for a given lighthouse syste
// would be.
// While this is typically opposite of what we want to do -- we want to find the 3d
// position from a 2D coordinate, this is helpful since the minimization of reprojection
// error is a core mechanism to many types of solvers.
void survive_reproject_from_pose_with_config(const SurviveContext *ctx, const survive_calibration_config *config,
											 int lighthouse, const SurvivePose *pose, const FLT *point3d, FLT *out);

void survive_reproject_from_pose_with_bsd(const BaseStationData *bsd, const survive_calibration_config *config,
										  const SurvivePose *pose, const FLT *point3d, FLT *out);

void survive_reproject_with_config(const SurviveContext *ctx, const survive_calibration_config *config, int lighthouse,
								   const FLT *point3d, FLT *out);

void survive_calibration_config_fprint(FILE *file, const survive_calibration_config *config);

void survive_apply_bsd_calibration_by_flag(SurviveContext *ctx, int lh, enum SurviveCalFlag f, const FLT *in, FLT *out);
void survive_apply_bsd_calibration(SurviveContext *ctx, int lh, const FLT *in, FLT *out);

#ifdef __cplusplus
}
#endif
