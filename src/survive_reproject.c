#include "survive_reproject.h"
#include "survive_reproject.generated.h"
#include <math.h>
#include <survive_reproject.h>

typedef struct survive_calibration_config {
	FLT phase_scale, tilt_scale, curve_scale, gib_scale;
} survive_calibration_config;

static const survive_calibration_config default_config = {
	.phase_scale = 1., .tilt_scale = 1. / 10., .curve_scale = 1. / 10., .gib_scale = -1. / 10.};

static inline FLT survive_reproject_axis(const BaseStationCal *bcal, FLT axis_value, FLT other_axis_value, FLT Z) {
	FLT y = other_axis_value / Z;
	FLT ang = atan2(axis_value, Z);

	const FLT phase = bcal->phase;
	const FLT curve = bcal->curve;
	const FLT tilt = bcal->tilt;
	const FLT gibPhase = bcal->gibpha;
	const FLT gibMag = bcal->gibmag;

	ang -= default_config.phase_scale * phase;
	ang -= tan(default_config.tilt_scale * tilt) * y;
	ang -= default_config.curve_scale * curve * y * y;
	ang -= default_config.gib_scale * sin(gibPhase + ang) * gibMag;

	return ang;
}

FLT survive_reproject_axis_x(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis(&bcal[0], -ptInLh[0], ptInLh[1], -ptInLh[2]);
}

FLT survive_reproject_axis_y(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis(&bcal[1], ptInLh[1], -ptInLh[0], -ptInLh[2]);
}

void survive_reproject_xy(const BaseStationCal *bcal, LinmathVec3d const ptInLh, SurviveAngleReading out) {
	out[0] = survive_reproject_axis_x(bcal, ptInLh);
	out[1] = survive_reproject_axis_y(bcal, ptInLh);
}

void survive_reproject_full(const BaseStationCal *bcal, const SurvivePose *lh2world, const SurvivePose *obj2world,
							const LinmathVec3d obj_pt, FLT *out) {
	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, obj2world, obj_pt);

	SurvivePose world2lh;
	InvertPose(&world2lh, lh2world);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, &world2lh, world_pt);

	survive_reproject_xy(bcal, t_pt, out);
}

void survive_reproject_full_x_jac_obj_pose(FLT *out, const SurvivePose *obj_pose, const double *obj_pt,
										   const SurvivePose *lh2world, const BaseStationCal *bcal) {
	FLT phase_scale = default_config.phase_scale;
	FLT phase_0 = bcal[0].phase;

	FLT tilt_scale = default_config.tilt_scale;
	FLT tilt_0 = bcal[0].tilt;

	FLT curve_scale = default_config.curve_scale;
	FLT curve_0 = bcal[0].curve;

	FLT gib_scale = default_config.gib_scale;
	FLT gibPhase_0 = bcal[0].gibpha;
	FLT gibMag_0 = bcal[0].gibmag;

	gen_reproject_axis_x_jac_obj_p(out, obj_pose->Pos, obj_pt, lh2world->Pos, phase_scale, phase_0, tilt_scale, tilt_0,
								   curve_scale, curve_0, gib_scale, gibPhase_0, gibMag_0);
}

void survive_reproject_full_y_jac_obj_pose(FLT *out, const SurvivePose *obj_pose, const double *obj_pt,
										   const SurvivePose *lh2world, const BaseStationCal *bcal) {
	FLT phase_scale = default_config.phase_scale;
	FLT phase_1 = bcal[1].phase;

	FLT tilt_scale = default_config.tilt_scale;
	FLT tilt_1 = bcal[1].tilt;

	FLT curve_scale = default_config.curve_scale;
	FLT curve_1 = bcal[1].curve;

	FLT gib_scale = default_config.gib_scale;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_1 = bcal[1].gibmag;

	gen_reproject_axis_x_jac_obj_p(out, obj_pose->Pos, obj_pt, lh2world->Pos, phase_scale, phase_1, tilt_scale, tilt_1,
								   curve_scale, curve_1, gib_scale, gibPhase_1, gibMag_1);
}

void survive_reproject_full_jac_obj_pose(FLT *out, const SurvivePose *obj_pose, const LinmathVec3d obj_pt,
										 const SurvivePose *lh2world, const BaseStationCal *bcal) {
	FLT phase_scale = default_config.phase_scale;
	FLT phase_0 = bcal[0].phase;
	FLT phase_1 = bcal[1].phase;

	FLT tilt_scale = default_config.tilt_scale;
	FLT tilt_0 = bcal[0].tilt;
	FLT tilt_1 = bcal[1].tilt;

	FLT curve_scale = default_config.curve_scale;
	FLT curve_0 = bcal[0].curve;
	FLT curve_1 = bcal[1].curve;

	FLT gib_scale = default_config.gib_scale;
	FLT gibPhase_0 = bcal[0].gibpha;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_0 = bcal[0].gibmag;
	FLT gibMag_1 = bcal[1].gibmag;

	gen_reproject_jac_obj_p(out, obj_pose->Pos, obj_pt, lh2world->Pos, phase_scale, phase_0, phase_1, tilt_scale,
							tilt_0, tilt_1, curve_scale, curve_0, curve_1, gib_scale, gibPhase_0, gibPhase_1, gibMag_0,
							gibMag_1);
}

void survive_reproject_from_pose_with_bcal(const BaseStationCal *bcal, const SurvivePose *pose, LinmathVec3d const pt,
										   FLT *out) {
	LinmathQuat invq;
	quatgetreciprocal(invq, pose->Rot);

	LinmathPoint3d tvec;
	quatrotatevector(tvec, invq, pose->Pos);

	LinmathPoint3d t_pt;
	quatrotatevector(t_pt, invq, pt);
	for (int i = 0; i < 3; i++)
		t_pt[i] = t_pt[i] - tvec[i];

	survive_reproject_xy(bcal, t_pt, out);
}

void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse, const SurvivePose *lh2world,
								 LinmathVec3d const pt, FLT *out) {
	survive_reproject_from_pose_with_bcal(ctx->bsd[lighthouse].fcal, lh2world, pt, out);
}

void survive_reproject(const SurviveContext *ctx, int lighthouse, LinmathVec3d const ptInWorld, FLT *out) {
	survive_reproject_from_pose(ctx, lighthouse, &ctx->bsd[lighthouse].Pose, ptInWorld, out);
}

void survive_apply_bsd_calibration(const SurviveContext *ctx, int lh, const FLT *in, FLT *out) {
	const BaseStationCal *cal = ctx->bsd[lh].fcal;
	out[0] = in[0] + default_config.phase_scale * cal[0].phase;
	out[1] = in[1] + default_config.phase_scale * cal[1].phase;

	FLT phase_scale = default_config.phase_scale;
	FLT tilt_scale = default_config.tilt_scale;
	FLT curve_scale = default_config.curve_scale;
	FLT gib_scale = default_config.gib_scale;
	const int iterations = 4;
	for (int i = 0; i < iterations; i++) {
		FLT last_out[2] = {out[0], out[1]};
		FLT tlast_out[2] = {tan(out[0]), tan(out[1])};

		for (int j = 0; j < 2; j++) {
			int oj = j == 0 ? 1 : 0;
			out[j] = in[j];

			out[j] += phase_scale * cal[j].phase;
			out[j] += tan(tilt_scale * cal[j].tilt) * tlast_out[oj];
			out[j] += (cal[j].curve * curve_scale) * tlast_out[oj] * tlast_out[oj];
			out[j] += sin(cal[j].gibpha + last_out[j]) * cal[j].gibmag * gib_scale;
		}
	}
}
