#include "survive_reproject.h"
#include <../redist/linmath.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void survive_calibration_options_config_normalize(
	survive_calibration_options_config *option) {
	if (!option->enable[0])
		option->invert[0] = false;
	if (!option->enable[1])
		option->invert[1] = false;
	if (!option->enable[0] && !option->enable[1])
		option->swap = false;
}

void survive_calibration_options_config_apply(
	const survive_calibration_options_config *option, const FLT *input,
	FLT *output) {
	FLT tmp[2]; // In case they try to do in place
	for (int i = 0; i < 2; i++) {
		tmp[i] = option->enable[i] * (option->invert[i] ? -1 : 1) *
				 input[i ^ option->swap];
	}
	for (int i = 0; i < 2; i++) {
		output[i] = tmp[i];
	}
}

survive_calibration_config
survive_calibration_config_create_from_idx(size_t v) {
	survive_calibration_config config;
	memset(&config, 0, sizeof(config));

	bool *_this = (bool *)&config;

	for (size_t i = 0; i < sizeof(config); i++) {
		_this[i] = (bool)(v & 1);
		v = v >> 1;
	}

	survive_calibration_options_config_normalize(&config.phase);
	survive_calibration_options_config_normalize(&config.tilt);
	survive_calibration_options_config_normalize(&config.curve);
	survive_calibration_options_config_normalize(&config.gibMag);

	config.gibPhase.enable[0] = config.gibMag.enable[0];
	config.gibPhase.enable[1] = config.gibMag.enable[1];

	survive_calibration_options_config_normalize(&config.gibPhase);

	if (!config.gibPhase.enable[0] && !config.gibPhase.enable[1])
		config.gibUseSin = false;

	return config;
}

size_t
survive_calibration_config_index(const survive_calibration_config *config) {
	bool *_this = (bool *)config;
	size_t v = 0;
	for (size_t i = 0; i < sizeof(*config); i++) {
		v = (v | _this[sizeof(*config) - i - 1]);
		v = v << 1;
	}
	v = v >> 1;
	return v;
}

static FLT gibf(bool useSin, FLT v) {
	if (useSin)
		return sin(v);
	return cos(v);
}

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

	double ang_x = atan(x);
	double ang_y = atan(y);

	double phase[2];
	survive_calibration_options_config_apply(&config->phase, bsd->fcal.phase, phase);
	double tilt[2];
	survive_calibration_options_config_apply(&config->tilt, bsd->fcal.tilt, tilt);
	double curve[2];
	survive_calibration_options_config_apply(&config->curve, bsd->fcal.curve, curve);
	double gibPhase[2];
	survive_calibration_options_config_apply(&config->gibPhase, bsd->fcal.gibpha, gibPhase);
	double gibMag[2];
	survive_calibration_options_config_apply(&config->gibMag, bsd->fcal.gibmag, gibMag);

	out[0] = ang_x + phase[0] + (tilt[0]) * ang_y + curve[0] * ang_y * ang_y +
			 gibf(config->gibUseSin, gibPhase[0] + ang_x) * gibMag[0];
	out[1] = ang_y + phase[1] + (tilt[1]) * ang_x + curve[1] * ang_x * ang_x +
			 gibf(config->gibUseSin, gibPhase[1] + ang_y) * gibMag[1];
}
void survive_reproject_from_pose_with_config(const SurviveContext *ctx, const survive_calibration_config *config,
											 int lighthouse, const SurvivePose *pose, const FLT *pt, FLT *out) {
	const BaseStationData *bsd = &ctx->bsd[lighthouse];
	survive_reproject_from_pose_with_bsd(bsd, config, pose, pt, out);
}

void survive_reproject_with_config(const SurviveContext *ctx, const survive_calibration_config *config, int lighthouse,
								   const FLT *point3d, FLT *out) {
	survive_reproject_from_pose_with_config(ctx, config, lighthouse, &ctx->bsd[lighthouse].Pose, point3d, out);
}

void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse,
								 const SurvivePose *pose, FLT *pt, FLT *out) {
	survive_reproject_from_pose_with_config(ctx, survive_calibration_default_config(ctx), lighthouse, pose, pt, out);
}

void survive_reproject(const SurviveContext *ctx, int lighthouse, FLT *point3d,
					   FLT *out) {
	survive_reproject_from_pose(ctx, lighthouse, &ctx->bsd[lighthouse].Pose,
								point3d, out);
}

const survive_calibration_config *survive_calibration_default_config(const SurviveContext *_ctx) {
	SurviveContext *ctx = (SurviveContext *)_ctx;
	if (ctx->calibration_config == 0) {
		size_t idx = survive_configi(ctx, "default-cal-conf", SC_GET, 0);
		ctx->calibration_config = malloc(sizeof(survive_calibration_config));
		memset(ctx->calibration_config, 0, sizeof(survive_calibration_config));
		*ctx->calibration_config = survive_calibration_config_create_from_idx(idx);
	}
	return ctx->calibration_config;
}

size_t survive_calibration_config_max_idx() {
	survive_calibration_config cfg;
	memset(&cfg, 0x1, sizeof(survive_calibration_config));
	return survive_calibration_config_index(&cfg);
}

static void survive_calibration_options_config_fprint(FILE *file, const survive_calibration_options_config *self) {
	fprintf(file, "\t");
	if (!self->enable[0] && !self->enable[1]) {
		fprintf(file, "disabled");
		return;
	}

	fprintf(file, "swap: %d\n", self->swap);
	for (int i = 0; i < 2; i++) {
		if (self->enable[i]) {
			fprintf(file, "\tinvert[%d]: %d", i, self->invert[i]);
		} else {
			fprintf(file, "\t%d: disabled", i);
		}
	}
}

void survive_calibration_config_fprint(FILE *file, const survive_calibration_config *self) {
	fprintf(file, "Index: %ld\n", survive_calibration_config_index(self));

	fprintf(file, "Phase: \n");
	survive_calibration_options_config_fprint(file, &self->phase);
	fprintf(file, "\n");

	fprintf(file, "Tilt: \n");
	survive_calibration_options_config_fprint(file, &self->tilt);
	fprintf(file, "\n");

	fprintf(file, "Curve: \n");
	survive_calibration_options_config_fprint(file, &self->curve);
	fprintf(file, "\n");

	fprintf(file, "gibPhase: \n");
	survive_calibration_options_config_fprint(file, &self->gibPhase);
	fprintf(file, "\n");

	fprintf(file, "gibMag: \n");
	survive_calibration_options_config_fprint(file, &self->gibMag);
	fprintf(file, "\n");

	fprintf(file, "gibUseSin: %d\n", self->gibUseSin);
}

void survive_apply_bsd_calibration_by_flag(SurviveContext *ctx, int lh, enum SurviveCalFlag f, const FLT *in,
										   FLT *out) {
	const BaseStationCal *cal = &ctx->bsd[lh].fcal;
	out[0] = in[0] + cal->phase[0];
	out[1] = in[1] + cal->phase[1];

	FLT tilt_scale = 10;
	FLT curve_scale = 10000;
	FLT gib_scale = 10;
	const int iterations = 4;
	for (int i = 0; i < iterations; i++) {
		FLT last_out[2] = {out[0], out[1]};
		bool last_iteration = i == iterations - 1;
		for (int j = 0; j < 2; j++) {
			int oj = j == 0 ? 1 : 0;
			out[j] = in[j];
			if (!last_iteration || (f & SVCal_Phase))
				out[j] += (cal->phase[j]);
			if (!last_iteration || (f & SVCal_Tilt))
				out[j] += tan(cal->tilt[j] / tilt_scale) * last_out[oj];
			if (!last_iteration || (f & SVCal_Curve))
				out[j] += (cal->curve[j] / curve_scale) * last_out[oj] * last_out[oj];
			if (!last_iteration || (f & SVCal_Gib))
				out[j] += cos(cal->gibpha[j] + last_out[j]) * cal->gibmag[1] / gib_scale;
		}
	}
}

void survive_apply_bsd_calibration(SurviveContext *ctx, int lh, const FLT *in, FLT *out) {
	survive_apply_bsd_calibration_by_flag(ctx, lh, ctx->calibration_flag, in, out);
}
