#include "survive_reproject.h"
#include "survive_reproject.generated.h"
#include <math.h>

typedef struct survive_calibration_config {
	FLT phase_scale, tilt_scale, curve_scale, gib_scale;
} survive_calibration_config;

static const survive_calibration_config default_config = {
	.phase_scale = 1., .tilt_scale = 1. / 10., .curve_scale = 1. / 10., .gib_scale = -1. / 10.};

static void survive_reproject_raw(const BaseStationCal *bcal, const double *t_pt, double *out) {

	FLT x = -t_pt[0] / -t_pt[2];
	FLT y = t_pt[1] / -t_pt[2];
	double xy[] = {x, y};
	double ang[] = {atan(x), atan(y)};

	const FLT *phase = bcal->phase;
	const FLT *curve = bcal->curve;
	const FLT *tilt = bcal->tilt;
	const FLT *gibPhase = bcal->gibpha;
	const FLT *gibMag = bcal->gibmag;

	for (int axis = 0; axis < 2; axis++) {
		int opp_axis = axis == 0 ? 1 : 0;

		out[axis] = ang[axis];

		out[axis] -= default_config.phase_scale * phase[axis];
		out[axis] -= tan(default_config.tilt_scale * tilt[axis]) * xy[opp_axis];
		out[axis] -= default_config.curve_scale * curve[axis] * xy[opp_axis] * xy[opp_axis];
		out[axis] -= default_config.gib_scale * sin(gibPhase[axis] + ang[axis]) * gibMag[axis];
	}
}

void survive_reproject_full_jac_obj_pose(FLT *out, const SurvivePose *obj_pose, const LinmathVec3d obj_pt,
										 const SurvivePose *lh2world, const BaseStationCal *bcal) {
	FLT phase_scale = default_config.phase_scale;
	FLT phase_0 = bcal->phase[0];
	FLT phase_1 = bcal->phase[1];

	FLT tilt_scale = default_config.tilt_scale;
	FLT tilt_0 = bcal->tilt[0];
	FLT tilt_1 = bcal->tilt[1];

	FLT curve_scale = default_config.curve_scale;
	FLT curve_0 = bcal->curve[0];
	FLT curve_1 = bcal->curve[1];

	FLT gib_scale = default_config.gib_scale;
	FLT gibPhase_0 = bcal->gibpha[0];
	FLT gibPhase_1 = bcal->gibpha[1];
	FLT gibMag_0 = bcal->gibmag[0];
	FLT gibMag_1 = bcal->gibmag[1];

	gen_reproject_jac_obj_p(out, obj_pose->Pos, obj_pt, lh2world->Pos, phase_scale, phase_0, phase_1, tilt_scale,
							tilt_0, tilt_1, curve_scale, curve_0, curve_1, gib_scale, gibPhase_0, gibPhase_1, gibMag_0,
							gibMag_1);
}

void survive_reproject_full(const BaseStationCal *bcal, const SurvivePose *lh2world, const SurvivePose *obj_pose,
							const LinmathVec3d obj_pt, FLT *out) {
	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, obj_pose, obj_pt);

	SurvivePose world2lh;
	InvertPose(&world2lh, lh2world);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, &world2lh, world_pt);

	survive_reproject_raw(bcal, t_pt, out);
}

void survive_reproject_from_pose_with_bcal(const BaseStationCal *bcal, const SurvivePose *pose, const FLT *pt,
										   FLT *out) {
	LinmathQuat invq;
	quatgetreciprocal(invq, pose->Rot);

	LinmathPoint3d tvec;
	quatrotatevector(tvec, invq, pose->Pos);

	LinmathPoint3d t_pt;
	quatrotatevector(t_pt, invq, pt);
	for (int i = 0; i < 3; i++)
		t_pt[i] = t_pt[i] - tvec[i];

	survive_reproject_raw(bcal, t_pt, out);
}

void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse, const SurvivePose *pose, const FLT *pt,
								 FLT *out) {
	survive_reproject_from_pose_with_bcal(&ctx->bsd[lighthouse].fcal, pose, pt, out);
}

void survive_reproject(const SurviveContext *ctx, int lighthouse, const FLT *point3d, FLT *out) {
	survive_reproject_from_pose(ctx, lighthouse, &ctx->bsd[lighthouse].Pose, point3d, out);
}

void survive_apply_bsd_calibration(const SurviveContext *ctx, int lh, const FLT *in, FLT *out) {
	const BaseStationCal *cal = &ctx->bsd[lh].fcal;
	out[0] = in[0] + default_config.phase_scale * cal->phase[0];
	out[1] = in[1] + default_config.phase_scale * cal->phase[1];

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

			out[j] += phase_scale * cal->phase[j];
			out[j] += tan(tilt_scale * cal->tilt[j]) * tlast_out[oj];
			out[j] += (cal->curve[j] * curve_scale) * tlast_out[oj] * tlast_out[oj];
			out[j] += sin(cal->gibpha[j] + last_out[j]) * cal->gibmag[j] * gib_scale;
		}
	}
}
