#include "survive_reproject.h"
#include <assert.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <survive_reproject.h>

#pragma GCC push_options
#pragma GCC optimize("Ofast")

#define USE_SINGLE_POINT

#ifdef USE_SINGLE_POINT
#define FLT_REPROJECT float

#define asin_REPROJECT asinf
#define cos_REPROJECT cosf
#define atan2_REPROJECT atan2f
#define sqrt_REPROJECT sqrtf
#define pow_REPROJECT powf
#else
#define FLT_REPROJECT FLT

#define asin_REPROJECT asin
#define cos_REPROJECT cos
#define atan2_REPROJECT atan2
#define sqrt_REPROJECT sqrt
#define pow_REPROJECT pow
#endif

#include "generated/survive_reproject.generated.h"

static inline FLT survive_reproject_axis(const BaseStationCal *bcal, FLT axis_value, FLT other_axis_value, FLT Z,
										 bool invert_axis_value) {
	FLT_REPROJECT ang = (FLT)M_PI_2 - (invert_axis_value ? -1.f : 1.f) * (FLT_ATAN2(axis_value, Z));

	const FLT_REPROJECT phase = bcal->phase;
	const FLT_REPROJECT curve = bcal->curve;
	const FLT_REPROJECT tilt = bcal->tilt;
	const FLT_REPROJECT gibPhase = bcal->gibpha;
	const FLT_REPROJECT gibMag = bcal->gibmag;

	const FLT_REPROJECT mag = FLT_SQRT(axis_value * axis_value + Z * Z);

	ang -= phase;
	FLT_REPROJECT asin_arg = linmath_enforce_range((tilt)*other_axis_value / mag, -1, 1);
	ang -= asin_REPROJECT(asin_arg);
	ang -= cos_REPROJECT(gibPhase + ang) * gibMag;
	ang += curve * atan2_REPROJECT(other_axis_value, Z) * atan2_REPROJECT(other_axis_value, Z);

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
	SurvivePose world2lh = InvertPoseRtn(&ctx->bsd[lighthouse].Pose);
	survive_reproject_from_pose(ctx, lighthouse, &world2lh, ptInWorld, out);
}

void survive_apply_bsd_calibration(const SurviveContext *ctx, int lh, const FLT *in, SurviveAngleReading out) {
	const BaseStationCal *cal = ctx->bsd[lh].fcal;
	out[0] = in[0] + cal[0].phase;
	out[1] = in[1] + cal[1].phase;
}

#pragma GCC pop_options

const survive_reproject_model_t SURVIVE_EXPORT survive_reproject_model = {
	.reprojectAxisFn = {survive_reproject_axis_x, survive_reproject_axis_y},
	.reprojectAxisJacobFn = {gen_reproject_axis_x_jac_obj_p, gen_reproject_axis_y_jac_obj_p},
	.reprojectXY = survive_reproject_xy,
	.reprojectFullJacObjPose = gen_reproject_jac_obj_p,
	.reprojectFullJacLhPose = gen_reproject_jac_lh_p,
	.reprojectAxisJacobLhPoseFn = {gen_reproject_axis_x_jac_lh_p, gen_reproject_axis_y_jac_lh_p}};
