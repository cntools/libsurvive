#pragma once

#include "survive.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "survive_reproject.h"

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

SURVIVE_EXPORT FLT survive_reproject_axis_x_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh);
SURVIVE_EXPORT FLT survive_reproject_axis_y_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh);

SURVIVE_EXPORT void survive_reproject_xy_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh,
											  SurviveAngleReading out);
SURVIVE_EXPORT void survive_reproject_from_pose_gen2(const SurviveContext *ctx, int lighthouse,
													 const SurvivePose *world2lh, LinmathVec3d const ptInWorld,
													 SurviveAngleReading out);

SURVIVE_EXPORT void survive_reproject_gen2(const SurviveContext *ctx, int lighthouse, LinmathVec3d const ptInWorld,
										   SurviveAngleReading out);

  SURVIVE_EXPORT void survive_reproject_full_jac_obj_pose_gen2(SurviveAngleReading out, const SurvivePose *obj2world,
										 const LinmathVec3d ptInObj, const SurvivePose *world2lh,
										 const BaseStationCal *bcal);

  SURVIVE_EXPORT void survive_reproject_full_gen2(const BaseStationCal *bcal, const SurvivePose *world2lh, const SurvivePose *obj2world,
					     const LinmathVec3d ptInObj, SurviveAngleReading out);

  SURVIVE_IMPORT extern const survive_reproject_model_t survive_reproject_gen2_model;
  
#ifdef __cplusplus
}
#endif
