#include "survive_reproject.h"
#include <../redist/linmath.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void survive_reproject_from_pose_with_bsd(const BaseStationData *bsd, const survive_calibration_config *config,
										  const SurvivePose *pose, const FLT *pt, FLT *out) {
	LinmathQuat invq;
	quatgetreciprocal(invq, pose->Rot);

	LinmathPoint3d tvec;
	quatrotatevector(tvec, invq, pose->Pos);

	LinmathPoint3d t_pt;
	quatrotatevector(t_pt, invq, pt);
	for (int i = 0; i < 3; i++)
		t_pt[i] = t_pt[i] - tvec[i];

	FLT x = -t_pt[0] / -t_pt[2];
	FLT y = t_pt[1] / -t_pt[2];
	double xy[] = {x, y};
	double ang[] = {atan(x), atan(y)};

	const FLT *phase = bsd->fcal.phase;
	const FLT *curve = bsd->fcal.curve;
	const FLT *tilt = bsd->fcal.tilt;
	const FLT *gibPhase = bsd->fcal.gibpha;
	const FLT *gibMag = bsd->fcal.gibmag;
	enum SurviveCalFlag f = config->use_flag;

	for (int axis = 0; axis < 2; axis++) {
		int opp_axis = axis == 0 ? 1 : 0;

		out[axis] = ang[axis];

		if (f & SVCal_Phase)
			out[axis] -= config->phase_scale * phase[axis];
		if (f & SVCal_Tilt)
			out[axis] -= tan(config->tilt_scale * tilt[axis]) * xy[opp_axis];
		if (f & SVCal_Curve)
			out[axis] -= config->curve_scale * curve[axis] * xy[opp_axis] * xy[opp_axis];
		if (f & SVCal_Gib)
			out[axis] -= config->gib_scale * sin(gibPhase[axis] + ang[axis]) * gibMag[axis];
	}

}

void survive_apply_bsd_calibration_by_flag(SurviveContext *ctx, int lh, struct survive_calibration_config *config,
										   const FLT *in, FLT *out) {
	const BaseStationCal *cal = &ctx->bsd[lh].fcal;
	out[0] = in[0] + config->phase_scale * cal->phase[0];
	out[1] = in[1] + config->phase_scale * cal->phase[1];

	enum SurviveCalFlag f = config->use_flag;
	FLT phase_scale = config->phase_scale;
	FLT tilt_scale = config->tilt_scale;
	FLT curve_scale = config->curve_scale;
	FLT gib_scale = config->gib_scale;
	const int iterations = 4;
	for (int i = 0; i < iterations; i++) {
		FLT last_out[2] = {out[0], out[1]};
		FLT tlast_out[2] = {tan(out[0]), tan(out[1])};
		bool last_iteration = i == iterations - 1;
		for (int j = 0; j < 2; j++) {
			int oj = j == 0 ? 1 : 0;
			out[j] = in[j];
			if (!last_iteration || (f & SVCal_Phase))
				out[j] += phase_scale * cal->phase[j];
			if (!last_iteration || (f & SVCal_Tilt))
				out[j] += tan(tilt_scale * cal->tilt[j]) * tlast_out[oj];
			if (!last_iteration || (f & SVCal_Curve))
				out[j] += (cal->curve[j] * curve_scale) * tlast_out[oj] * tlast_out[oj];
			if (!last_iteration || (f & SVCal_Gib))
				out[j] += sin(cal->gibpha[j] + last_out[j]) * cal->gibmag[j] * gib_scale;
		}
	}
}

void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse, const SurvivePose *pose, FLT *pt,
								 FLT *out) {
	survive_reproject_from_pose_with_bsd(&ctx->bsd[lighthouse], &ctx->calibration_config, pose, pt, out);
}

void survive_reproject(const SurviveContext *ctx, int lighthouse, FLT *point3d, FLT *out) {
	survive_reproject_from_pose(ctx, lighthouse, &ctx->bsd[lighthouse].Pose, point3d, out);
}

survive_calibration_config survive_calibration_config_ctor() {
	return (survive_calibration_config){.use_flag = SVCal_All,
										.phase_scale = 1.,
										.tilt_scale = 1. / 10.,
										.curve_scale = 1. / 10.,
										.gib_scale = -1. / 10.};
}

void survive_apply_bsd_calibration(SurviveContext *ctx, int lh, const FLT *in, FLT *out) {
	survive_apply_bsd_calibration_by_flag(ctx, lh, &ctx->calibration_config, in, out);
}

void survive_reproject_from_pose_with_config(const SurviveContext *ctx, struct survive_calibration_config *config,
											 int lighthouse, const SurvivePose *pose, FLT *point3d, FLT *out) {
	return survive_reproject_from_pose_with_bsd(&ctx->bsd[lighthouse], config, pose, point3d, out);
}
