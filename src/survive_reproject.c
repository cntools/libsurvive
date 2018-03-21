#include "survive_reproject.h"
#include <../redist/linmath.h>
#include <math.h>
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

void survive_reproject_from_pose_with_config(
	const SurviveContext *ctx, const survive_calibration_config *config,
	int lighthouse, const SurvivePose *pose, const FLT *pt, FLT *out) {
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

	const BaseStationData *bsd = &ctx->bsd[lighthouse];
	double phase[2];
	survive_calibration_options_config_apply(&config->phase, bsd->fcalphase,
											 phase);
	double tilt[2];
	survive_calibration_options_config_apply(&config->tilt, bsd->fcaltilt,
											 tilt);
	double curve[2];
	survive_calibration_options_config_apply(&config->curve, bsd->fcalcurve,
											 curve);
	double gibPhase[2];
	survive_calibration_options_config_apply(&config->gibPhase, bsd->fcalgibpha,
											 gibPhase);
	double gibMag[2];
	survive_calibration_options_config_apply(&config->gibMag, bsd->fcalgibmag,
											 gibMag);

	out[0] = ang_x + phase[0] + tan(tilt[0]) * y + curve[0] * y * y +
			 gibf(config->gibUseSin, gibPhase[0] + ang_x) * gibMag[0];
	out[1] = ang_y + phase[1] + tan(tilt[1]) * x + curve[1] * x * x +
			 gibf(config->gibUseSin, gibPhase[1] + ang_y) * gibMag[1];
}

void survive_reproject_from_pose(const SurviveContext *ctx, int lighthouse,
								 const SurvivePose *pose, FLT *pt, FLT *out) {
	survive_reproject_from_pose_with_config(
		ctx, survive_calibration_default_config(), lighthouse, pose, pt, out);
}

void survive_reproject(const SurviveContext *ctx, int lighthouse, FLT *point3d,
					   FLT *out) {
	survive_reproject_from_pose(ctx, lighthouse, &ctx->bsd[lighthouse].Pose,
								point3d, out);
}

const survive_calibration_config *survive_calibration_default_config() {
	static survive_calibration_config *def = 0;
	if (def == 0) {
		def = malloc(sizeof(survive_calibration_config));
		memset(def, 0, sizeof(survive_calibration_config));
		*def = survive_calibration_config_create_from_idx(0);
	}
	return def;
}

size_t survive_calibration_config_max_idx() {
	survive_calibration_config cfg;
	memset(&cfg, 0x1, sizeof(survive_calibration_config));
	return survive_calibration_config_index(&cfg);
}
