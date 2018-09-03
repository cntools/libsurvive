#pragma once

#include "survive.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

FLT survive_reproject_axis_x(const BaseStationCal *bcal, const FLT *t_pt);
FLT survive_reproject_axis_y(const BaseStationCal *bcal, const FLT *t_pt);

void survive_reproject_xy(const BaseStationCal *bcal, const FLT *t_pt, FLT *out);
void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse, const SurvivePose *lh_pose,
								 const FLT *point3d, FLT *out);

void survive_reproject_full_jac_obj_pose(FLT *out, const SurvivePose *obj_pose, const LinmathVec3d obj_pt,
										 const SurvivePose *lh2world, const BaseStationCal *bcal);

void survive_reproject_full_x_jac_obj_pose(FLT *out, const SurvivePose *obj_pose, const LinmathVec3d obj_pt,
										   const SurvivePose *lh2world, const BaseStationCal *bcal);

void survive_reproject_full_y_jac_obj_pose(FLT *out, const SurvivePose *obj_pose, const LinmathVec3d obj_pt,
										   const SurvivePose *lh2world, const BaseStationCal *bcal);

void survive_reproject_full(const BaseStationCal *bcal, const SurvivePose *lh2world, const SurvivePose *obj_pose,
							const LinmathVec3d obj_pt, FLT *out);

// This is given a lighthouse -- in the same system as stored in BaseStationData, and
// a 3d point and finds what the effective 'angle' value for a given lighthouse system
// would be.
//
// While this is typically opposite of what we want to do -- we want to find the 3d
// position from a 2D coordinate, this is helpful since the minimization of reprojection
// error is a core mechanism to many types of solvers.

void survive_reproject_from_pose_with_bcal(const BaseStationCal *bcal, const SurvivePose *lh_pose, const FLT *point3d,
										   FLT *out);

void survive_reproject(const SurviveContext *ctx, int lighthouse, const FLT *point3d, FLT *out);

// This is given input from the light sensors and approximates the idealized version of them
// by incorporating the calibration data from the lighthouse. In theory, it's an approximation
// but in practice in converges pretty quickly and to a good degree of accuracy.
// That said, all things being equal, it is better to compare reprojection to raw incoming
// data if you are looking to minimize that error.
void survive_apply_bsd_calibration(const SurviveContext *ctx, int lh, const FLT *in, FLT *out);

#ifdef __cplusplus
}
#endif
