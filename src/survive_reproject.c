#include <assert.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#include "force_O3.h"

#ifdef BUILD_LH1_SUPPORT
#include "generated/survive_reproject.generated.h"
#endif
static inline FLT survive_reproject_axis(const BaseStationCal *bcal, FLT axis_value, FLT other_axis_value, FLT Z,
										 bool invert_axis_value) {
	FLT ang = (FLT)M_PI_2 - (invert_axis_value ? -1.f : 1.f) * (FLT_ATAN2(axis_value, Z));

	const FLT phase = bcal->phase;
	const FLT curve = bcal->curve;
	const FLT tilt = bcal->tilt;
	const FLT gibPhase = bcal->gibpha;
	const FLT gibMag = bcal->gibmag;

	const FLT mag = FLT_SQRT(axis_value * axis_value + Z * Z);

	ang -= phase;
	FLT asin_arg = linmath_enforce_range((tilt)*other_axis_value / mag, -1, 1);
	ang -= FLT_ASIN(asin_arg);
	ang -= FLT_COS(gibPhase + ang) * gibMag;
	ang += curve * FLT_ATAN2(other_axis_value, Z) * FLT_ATAN2(other_axis_value, Z);

	assert(!isnan(ang));
	return ang;
}

static inline FLT survive_reproject_axis_x_inline(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis(&bcal[0], ptInLh[0], ptInLh[1], -ptInLh[2], false) - (FLT)M_PI / 2.f;
}

static inline FLT survive_reproject_axis_y_inline(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis(&bcal[1], ptInLh[1], ptInLh[0], -ptInLh[2], true) - (FLT)M_PI / 2.f;
}

FLT survive_reproject_axis_x(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis_x_inline(bcal, ptInLh);
}

FLT survive_reproject_axis_y(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis_y_inline(bcal, ptInLh);
}

void survive_reproject_xy(const BaseStationCal *bcal, LinmathVec3d const ptInLh, SurviveAngleReading out) {
	out[0] = survive_reproject_axis_x_inline(bcal, ptInLh);
	out[1] = survive_reproject_axis_y_inline(bcal, ptInLh);
}

void survive_reproject_full(const BaseStationCal *bcal, const SurvivePose *world2lh, const SurvivePose *obj2world,
							const LinmathVec3d obj_pt, SurviveAngleReading out) {
	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, obj2world, obj_pt);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, world2lh, world_pt);

	survive_reproject_xy(bcal, t_pt, out);
}
SURVIVE_EXPORT void survive_reproject_full_axisangle(const BaseStationCal *bcal, const LinmathAxisAnglePose *world2lh, const LinmathAxisAnglePose *obj2world,
													 const LinmathVec3d obj_pt, SurviveAngleReading out) {
	LinmathVec3d world_pt;
	ApplyAxisAnglePoseToPoint(world_pt, obj2world, obj_pt);

	LinmathPoint3d t_pt;
	ApplyAxisAnglePoseToPoint(t_pt, world2lh, world_pt);

	survive_reproject_xy(bcal, t_pt, out);
}
void survive_reproject_from_pose_with_bcal(const BaseStationCal *bcal, const SurvivePose *world2lh,
										   LinmathVec3d const ptInWorld, SurviveAngleReading out) {
	LinmathPoint3d ptInLh;
	ApplyPoseToPoint(ptInLh, world2lh, ptInWorld);
	survive_reproject_xy(bcal, ptInLh, out);
}

void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse, const SurvivePose *world2lh,
								 LinmathVec3d const pt, SurviveAngleReading out) {
	survive_reproject_from_pose_with_bcal(ctx->bsd[lighthouse].fcal, world2lh, pt, out);
}

void survive_reproject(const SurviveContext *ctx, int lighthouse, LinmathVec3d const ptInWorld,
					   SurviveAngleReading out) {
	SurvivePose world2lh = InvertPoseRtn(survive_get_lighthouse_position(ctx, lighthouse));
	survive_reproject_from_pose(ctx, lighthouse, &world2lh, ptInWorld, out);
}

void survive_apply_bsd_calibration(const SurviveContext *ctx, int lh, const SurviveAngleReading in,
								   SurviveAngleReading out) {
	const BaseStationCal *cal = ctx->bsd[lh].fcal;
	out[0] = in[0] + cal[0].phase;
	out[1] = in[1] + cal[1].phase;
}

SURVIVE_EXPORT const survive_reproject_model_t* survive_reproject_model(SurviveContext* ctx) {
	return ctx->lh_version == 0 ? &survive_reproject_gen1_model : &survive_reproject_gen2_model;
}

const survive_reproject_model_t SURVIVE_EXPORT survive_reproject_gen1_model = {
#ifdef BUILD_LH1_SUPPORT
	.reprojectAxisFn = {survive_reproject_axis_x, survive_reproject_axis_y},
	.reprojectXY = survive_reproject_xy,
	.reprojectAxisFullFn = {gen_reproject_axis_x, gen_reproject_axis_y},

	.reprojectAxisJacobFn = {gen_reproject_axis_x_jac_obj_p, gen_reproject_axis_y_jac_obj_p},
	.reprojectFullJacObjPose = gen_reproject_jac_obj_p,
	.reprojectFullJacLhPose = gen_reproject_jac_lh_p,
	.reprojectAxisJacobLhPoseFn = {gen_reproject_axis_x_jac_lh_p, gen_reproject_axis_y_jac_lh_p},

	.reprojectAxisangleFullXyFn =
		{
			gen_reproject_axis_x_axis_angle,
			gen_reproject_axis_y_axis_angle,
		},
	.reprojectAxisAngleFullJacObjPose = gen_reproject_jac_obj_p_axis_angle,
	.reprojectAxisAngleAxisJacobFn = {gen_reproject_axis_x_jac_obj_p_axis_angle,
									  gen_reproject_axis_y_jac_obj_p_axis_angle},

	.reprojectAxisAngleFullJacLhPose = gen_reproject_jac_lh_p_axis_angle,
	.reprojectAxisAngleAxisJacobLhPoseFn = {gen_reproject_axis_x_jac_lh_p_axis_angle,
											gen_reproject_axis_y_jac_lh_p_axis_angle},
	.reprojectAxisJacobSensorPt =
		{
			gen_reproject_axis_x_jac_sensor_pt,
			gen_reproject_axis_y_jac_sensor_pt,
		},
	.reprojectAxisAngleAxisJacobSensorPt =
		{
			gen_reproject_axis_x_jac_sensor_pt_axis_angle,
			gen_reproject_axis_y_jac_sensor_pt_axis_angle,
		}
#else
	0
#endif
};
