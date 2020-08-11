#pragma once

#include "survive.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/**
 * The conventions for reprojection in libsurvive:
 *
 * - Lighthouses are oriented sorta unintuitively -- If you have a lighthouse sitting on where the tripod mount is, in
 * front of it -- where it is projecting light -- is -Z. To it's right is +X. Up is +Y.
 * - So for instance -- an object to the right and lower than a lighthouse would have a ptInLh of { 1, -1, -1 }
 * - Physically, lighthouses sweep from their right to left, and from up to down.
 * - All angle measurements are in range of -PI / 2 to PI / 2. To the right / bottom of the lighthouse is -PI/2.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef FLT SurviveAngleReading[2];

typedef FLT (*survive_reproject_axis_fn_t)(const BaseStationCal *, const FLT *pt);
typedef void (*survive_reproject_xy_fn_t)(const BaseStationCal *bcal, LinmathVec3d const ptInLh, FLT *out);

typedef FLT (*survive_reproject_full_xy_fn_t)(const SurvivePose *obj2world, const LinmathVec3d ptInObj,
											  const SurvivePose *world2lh, const BaseStationCal *bcal);

//
typedef void (*survive_reproject_axis_jacob_fn_t)(FLT *out, const SurvivePose *, const LinmathPoint3d,
												  const SurvivePose *, const BaseStationCal *);
typedef void (*survive_reproject_full_jac_obj_pose_fn_t)(FLT *out, const SurvivePose *obj2world,
														 const LinmathVec3d ptInObj, const SurvivePose *world2lh,
														 const BaseStationCal *bcal);

typedef survive_reproject_full_jac_obj_pose_fn_t survive_reproject_full_jac_lh_pose_fn_t;
typedef survive_reproject_axis_jacob_fn_t survive_reproject_axis_jacob_lh_pose_fn_t;

typedef void (*survive_reproject_axisangle_axis_jacob_fn_t)(FLT *out, const LinmathAxisAnglePose *,
															const LinmathPoint3d, const LinmathAxisAnglePose *,
															const BaseStationCal *);
typedef void (*survive_reproject_axisangle_full_jac_obj_pose_fn_t)(FLT *out, const LinmathAxisAnglePose *obj2world,
																   const LinmathVec3d ptInObj,
																   const LinmathAxisAnglePose *world2lh,
																   const BaseStationCal *bcal);

typedef survive_reproject_axisangle_axis_jacob_fn_t survive_reproject_axisangle_full_jac_lh_pose_fn_t;
typedef survive_reproject_axisangle_full_jac_obj_pose_fn_t survive_reproject_axisangle_axis_jacob_lh_pose_fn_t;

typedef struct survive_reproject_model_t {
	survive_reproject_xy_fn_t reprojectXY;
	survive_reproject_axis_fn_t reprojectAxisFn[2];
	survive_reproject_full_xy_fn_t reprojectAxisFullFn[2];

	survive_reproject_full_jac_obj_pose_fn_t reprojectFullJacObjPose;
	survive_reproject_axis_jacob_fn_t reprojectAxisJacobFn[2];

	survive_reproject_full_jac_lh_pose_fn_t reprojectFullJacLhPose;
	survive_reproject_axis_jacob_lh_pose_fn_t reprojectAxisJacobLhPoseFn[2];

	survive_reproject_axisangle_full_jac_obj_pose_fn_t reprojectAxisAngleFullJacObjPose;
	survive_reproject_axisangle_axis_jacob_fn_t reprojectAxisAngleAxisJacobFn[2];

	survive_reproject_axisangle_full_jac_lh_pose_fn_t reprojectAxisAngleFullJacLhPose;
	survive_reproject_axisangle_axis_jacob_lh_pose_fn_t reprojectAxisAngleAxisJacobLhPoseFn[2];
} survive_reproject_model_t;

SURVIVE_IMPORT extern const survive_reproject_model_t survive_reproject_model;

SURVIVE_EXPORT FLT survive_reproject_axis_x(const BaseStationCal *bcal, LinmathVec3d const ptInLh);
SURVIVE_EXPORT FLT survive_reproject_axis_y(const BaseStationCal *bcal, LinmathVec3d const ptInLh);

SURVIVE_EXPORT void survive_reproject_xy(const BaseStationCal *bcal, LinmathVec3d const ptInLh, SurviveAngleReading out);
SURVIVE_EXPORT void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse, const SurvivePose *world2lh,
								 LinmathVec3d const ptInWorld, SurviveAngleReading out);

SURVIVE_EXPORT void survive_reproject_full_jac_obj_pose(SurviveAngleReading out, const SurvivePose *obj2world,
										 const LinmathVec3d ptInObj, const SurvivePose *world2lh,
										 const BaseStationCal *bcal);

SURVIVE_EXPORT void survive_reproject_full_x_jac_obj_pose(SurviveAngleReading out, const SurvivePose *obj2world,
										   const LinmathVec3d ptInObj, const SurvivePose *world2lh,
										   const BaseStationCal *bcal);

SURVIVE_EXPORT void survive_reproject_full_y_jac_obj_pose(SurviveAngleReading out, const SurvivePose *obj2world,
										   const LinmathVec3d ptInObj, const SurvivePose *world2lh,
										   const BaseStationCal *bcal);

SURVIVE_EXPORT void survive_reproject_full(const BaseStationCal *bcal, const SurvivePose *world2lh, const SurvivePose *obj2world,
							const LinmathVec3d ptInObj, SurviveAngleReading out);

// This is given a lighthouse -- in the same system as stored in BaseStationData, and
// a 3d point and finds what the effective 'angle' value for a given lighthouse system
// would be.
//
// While this is typically opposite of what we want to do -- we want to find the 3d
// position from a 2D coordinate, this is helpful since the minimization of reprojection
// error is a core mechanism to many types of solvers.

SURVIVE_EXPORT void survive_reproject_from_pose_with_bcal(const BaseStationCal *bcal, const SurvivePose *world2lh,
										   LinmathVec3d const ptInLh, SurviveAngleReading out);

SURVIVE_EXPORT void survive_reproject(const SurviveContext *ctx, int lighthouse, LinmathVec3d const ptInWorld,
					   SurviveAngleReading out);

// This is given input from the light sensors and approximates the idealized version of them
// by incorporating the calibration data from the lighthouse. In theory, it's an approximation
// but in practice in converges pretty quickly and to a good degree of accuracy.
// That said, all things being equal, it is better to compare reprojection to raw incoming
// data if you are looking to minimize that error.
SURVIVE_EXPORT void survive_apply_bsd_calibration(const SurviveContext *ctx, int lh, const SurviveAngleReading in,
								   SurviveAngleReading out);

#ifdef __cplusplus
}
#endif
