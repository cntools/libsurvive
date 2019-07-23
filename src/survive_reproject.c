#include "survive_reproject.h"
#include <assert.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <survive_reproject.h>

#pragma GCC push_options
#pragma GCC optimize("Ofast")

#include "survive_reproject.generated.h"

static inline FLT survive_reproject_axis(const BaseStationCal *bcal, FLT axis_value, FLT other_axis_value, FLT Z,
										 bool invert_axis_value) {
	FLT ang = M_PI_2 - (invert_axis_value ? -1 : 1) * (atan2(axis_value, Z));

	const FLT phase = bcal->phase;
	const FLT curve = bcal->curve;
	const FLT tilt = bcal->tilt;
	const FLT gibPhase = bcal->gibpha;
	const FLT gibMag = bcal->gibmag;

    const FLT mag = sqrt(axis_value * axis_value  + Z * Z);

	ang -= phase;
	double asin_arg = linmath_enforce_range((tilt)*other_axis_value / mag, -1, 1);
	ang -= asin(asin_arg);
	ang -= cos(gibPhase + ang) * gibMag;
	ang += curve * atan2(other_axis_value, Z) * atan2(other_axis_value, Z);

	assert(!isnan(ang));
	return ang;
}

static inline FLT survive_reproject_axis_x_inline(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis(&bcal[0], ptInLh[0], ptInLh[1], -ptInLh[2], false) - M_PI / 2.;
}

static inline FLT survive_reproject_axis_y_inline(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis(&bcal[1], ptInLh[1], ptInLh[0], -ptInLh[2], true) - M_PI / 2.;
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

void survive_reproject_full_x_jac_obj_pose(SurviveAngleReading out, const SurvivePose *obj_pose, const double *obj_pt,
										   const SurvivePose *world2lh, const BaseStationCal *bcal) {
	FLT phase_0 = bcal[0].phase;
	FLT tilt_0 = bcal[0].tilt;
	FLT curve_0 = bcal[0].curve;
	FLT gibPhase_0 = bcal[0].gibpha;
	FLT gibMag_0 = bcal[0].gibmag;

	gen_reproject_axis_x_jac_obj_p(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_0, tilt_0, curve_0, gibPhase_0,
								   gibMag_0);
}

void survive_reproject_full_y_jac_obj_pose(SurviveAngleReading out, const SurvivePose *obj_pose, const double *obj_pt,
										   const SurvivePose *world2lh, const BaseStationCal *bcal) {
	FLT phase_1 = bcal[1].phase;
	FLT tilt_1 = bcal[1].tilt;
	FLT curve_1 = bcal[1].curve;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_1 = bcal[1].gibmag;

	gen_reproject_axis_y_jac_obj_p(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_1, tilt_1, curve_1, gibPhase_1,
								   gibMag_1);
}

void survive_reproject_full_jac_obj_pose(SurviveAngleReading out, const SurvivePose *obj_pose,
										 const LinmathVec3d obj_pt, const SurvivePose *world2lh,
										 const BaseStationCal *bcal) {
	FLT phase_0 = bcal[0].phase;
	FLT phase_1 = bcal[1].phase;

	FLT tilt_0 = bcal[0].tilt;
	FLT tilt_1 = bcal[1].tilt;

	FLT curve_0 = bcal[0].curve;
	FLT curve_1 = bcal[1].curve;

	FLT gibPhase_0 = bcal[0].gibpha;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_0 = bcal[0].gibmag;
	FLT gibMag_1 = bcal[1].gibmag;

	gen_reproject_jac_obj_p(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_0, phase_1, tilt_0, tilt_1, curve_0,
							curve_1, gibPhase_0, gibPhase_1, gibMag_0, gibMag_1);
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

const survive_reproject_model_t survive_reproject_model = {
	.reprojectAxisFn = {survive_reproject_axis_x, survive_reproject_axis_y},
	.reprojectAxisJacobFn = {survive_reproject_full_x_jac_obj_pose, survive_reproject_full_y_jac_obj_pose},
	.reprojectXY = survive_reproject_xy,
	.reprojectFullJacObjPose = survive_reproject_full_jac_obj_pose};
