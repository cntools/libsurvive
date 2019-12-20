#include <assert.h>
#include <math.h>
#include <survive_optimizer.h>
#include <survive_reproject.h>

#ifndef NOZLIB
#include <survive_reproject_gen2.h>
#include <zlib.h>

#endif

#include "survive_optimizer.h"

#include "mpfit/mpfit.h"

static char *object_parameter_names[] = {"Pose x",	 "Pose y",	 "Pose z",	"Pose Rot w",
										 "Pose Rot x", "Pose Rot y", "Pose Rot z"};

static void setup_pose_param_limits(survive_optimizer *mpfit_ctx, double *parameter,
									struct mp_par_struct *pose_param_info) {
	for (int i = 0; i < 7; i++) {
		pose_param_info[i].limited[0] = pose_param_info[i].limited[1] = 1;

		pose_param_info[i].limits[0] = -(i >= 3 ? 1.0001 : 20.);
		pose_param_info[i].limits[1] = -pose_param_info[i].limits[0];
	}
}

void survive_optimizer_setup_pose(survive_optimizer *mpfit_ctx, const SurvivePose *poses, bool isFixed,
								  int use_jacobian_function) {
	for (int i = 0; i < mpfit_ctx->poseLength; i++) {
		if (poses)
			survive_optimizer_get_pose(mpfit_ctx)[i] = poses[i];
		else
			survive_optimizer_get_pose(mpfit_ctx)[i] = (SurvivePose){.Rot = {1.}};

		setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + i * 7, mpfit_ctx->parameters_info + i * 7);
	}

	for (int i = 0; i < 7 * mpfit_ctx->poseLength; i++) {
		mpfit_ctx->parameters_info[i].fixed = isFixed;
		mpfit_ctx->parameters_info[i].parname = object_parameter_names[i % 7];

		if (use_jacobian_function != 0 && mpfit_ctx->reprojectModel->reprojectFullJacObjPose) {
			if (use_jacobian_function < 0) {
				mpfit_ctx->parameters_info[i].side = 1;
				mpfit_ctx->parameters_info[i].deriv_debug = 1;
				mpfit_ctx->parameters_info[i].deriv_abstol = .0001;
				mpfit_ctx->parameters_info[i].deriv_reltol = .0001;
			} else {
				mpfit_ctx->parameters_info[i].side = 3;
			}
		}
	}
}

static char *lh_parameter_names[] = {"LH0 x",	 "LH0 y",	 "LH0 z",		"LH0 Rot w", "LH0 Rot x",
									 "LH0 Rot y", "LH0 Rot z", "LH1 x",		"LH1 y",	 "LH1 z",
									 "LH1 Rot w", "LH1 Rot x", "LH1 Rot y", "LH1 Rot z"

};

void survive_optimizer_setup_camera(survive_optimizer *mpfit_ctx, int8_t lh, const SurvivePose *pose, bool isFixed) {
	SurvivePose *cameras = survive_optimizer_get_camera(mpfit_ctx);
	int start = survive_optimizer_get_camera_index(mpfit_ctx) + lh * 7;

	bool poseIsInvalid = pose == 0;
	if (pose && !quatiszero(pose->Rot)) {
		InvertPose(&cameras[lh], pose);
	} else {
		cameras[lh] = (SurvivePose){ 0 };
		poseIsInvalid = true;
	}

	setup_pose_param_limits(mpfit_ctx, mpfit_ctx->parameters + start, mpfit_ctx->parameters_info + start);

	for (int i = start; i < start + 7; i++) {
		mpfit_ctx->parameters_info[i].fixed = isFixed || poseIsInvalid;
		mpfit_ctx->parameters_info[i].parname = lh_parameter_names[i - start];
	}
}

void survive_optimizer_setup_cameras(survive_optimizer *mpfit_ctx, SurviveContext *ctx, bool isFixed) {
	for (int lh = 0; lh < mpfit_ctx->cameraLength; lh++) {
		if (ctx->bsd[lh].PositionSet)
			survive_optimizer_setup_camera(mpfit_ctx, lh, &ctx->bsd[lh].Pose, isFixed);
		else {
			SurvivePose id = {.Rot[0] = 1.};
			survive_optimizer_setup_camera(mpfit_ctx, lh, &id, isFixed);
		}
	}

    for (int lh = 0; lh < mpfit_ctx->cameraLength; lh++) {
        BaseStationCal *fcal = survive_optimizer_get_calibration(mpfit_ctx, lh);
        for(int axis = 0;axis < 2;axis++)
            fcal[axis] = ctx->bsd[lh].fcal[axis];
    }

    size_t start = survive_optimizer_get_calibration_index(mpfit_ctx);
    for (int i = start;i < start + 2 * sizeof(BaseStationCal) / sizeof(double) * mpfit_ctx->cameraLength;i++) {
        mpfit_ctx->parameters_info[i].parname = "Fcal parameter";
        mpfit_ctx->parameters_info[i].fixed = true;
    }
}

int survive_optimizer_get_parameters_count(const survive_optimizer *ctx) {
	return ctx->cameraLength * 7 + ctx->poseLength * 7 + ctx->ptsLength * 3 +
		   2 * ctx->cameraLength * sizeof(BaseStationCal) / sizeof(double);
}

double *survive_optimizer_get_sensors(survive_optimizer *ctx) {
	if (ctx->ptsLength == 0)
		return ctx->so->sensor_locations;

	return &ctx->parameters[survive_optimizer_get_sensors_index(ctx)];
}

int survive_optimizer_get_sensors_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_calibration_index(ctx) + 2 * ctx->cameraLength * sizeof(BaseStationCal) / sizeof(double);
}

BaseStationCal *survive_optimizer_get_calibration(survive_optimizer *ctx, int lh) {
	if (ctx->cameraLength <= lh)
		return ctx->so->ctx->bsd[lh].fcal;

    BaseStationCal* base = (BaseStationCal *)(&ctx->parameters[survive_optimizer_get_calibration_index(ctx)]);
	return &base[2*lh];
}

int survive_optimizer_get_calibration_index(const survive_optimizer *ctx) {
	return survive_optimizer_get_camera_index(ctx) + ctx->cameraLength * 7;
}

SurvivePose *survive_optimizer_get_camera(survive_optimizer *ctx) {
	assert(ctx->cameraLength > 0);
	return (SurvivePose *)&ctx->parameters[survive_optimizer_get_camera_index(ctx)];
}

int survive_optimizer_get_camera_index(const survive_optimizer *ctx) { return ctx->poseLength * 7; }

SurvivePose *survive_optimizer_get_pose(survive_optimizer *ctx) {
	if (ctx->poseLength)
		return (SurvivePose *)ctx->parameters;
	return &ctx->initialPose;
}

static int mpfunc(int m, int n, double *p, double *deviates, double **derivs, void *private) {
	survive_optimizer *mpfunc_ctx = private;
	SurviveContext *ctx = mpfunc_ctx->so ? mpfunc_ctx->so->ctx : 0;

	const survive_reproject_model_t *reprojectModel = mpfunc_ctx->reprojectModel;
	mpfunc_ctx->parameters = p;

	SurvivePose *cameras = survive_optimizer_get_camera(mpfunc_ctx);

	int start = survive_optimizer_get_camera_index(mpfunc_ctx);
	for (int i = 0; i < mpfunc_ctx->cameraLength; i++) {
		if (!mpfunc_ctx->parameters_info[start + 7 * i].fixed) {
			quatnormalize(cameras[i].Rot, cameras[i].Rot);
		}
	}
	const double *sensor_points = survive_optimizer_get_sensors(mpfunc_ctx);

	int pose_idx = -1;
	SurvivePose *pose = 0;
	SurvivePose obj2lh[NUM_GEN2_LIGHTHOUSES] = {0};

	int meas_count = m;
	if (mpfunc_ctx->current_bias > 0) {
		meas_count -= 7;
		FLT *pp = (FLT *)mpfunc_ctx->initialPose.Pos;
		for (int i = 0; i < 7; i++) {
			deviates[i + meas_count] = (p[i] - pp[i]) * mpfunc_ctx->current_bias;
			if (derivs) {
				derivs[i][i + meas_count] = mpfunc_ctx->current_bias;
			}
		}
	}

	for (int i = 0; i < meas_count; i++) {
		const survive_optimizer_measurement *meas = &mpfunc_ctx->measurements[i];
		const int lh = meas->lh;
		const struct BaseStationCal *cal = survive_optimizer_get_calibration(mpfunc_ctx, lh);
		const SurvivePose *world2lh = &cameras[lh];
		const FLT *pt = &sensor_points[meas->sensor_idx * 3];

		if (pose_idx != meas->object) {
			pose_idx = meas->object;
			assert(pose_idx < mpfunc_ctx->poseLength);
			pose = &survive_optimizer_get_pose(mpfunc_ctx)[meas->object];

			// SV_INFO("Before\t" SurvivePose_format, SURVIVE_POSE_EXPAND(*pose));
			quatnormalize(pose->Rot, pose->Rot);
			// SV_INFO("After\t" SurvivePose_format, SURVIVE_POSE_EXPAND(*pose));

			int lh_count = mpfunc_ctx->cameraLength > 0 ? mpfunc_ctx->cameraLength : mpfunc_ctx->so->ctx->activeLighthouses;
			for (int lh = 0; lh < lh_count; lh++) {
				ApplyPoseToPose(&obj2lh[lh], &cameras[lh], pose);
			}
		}

		// If the next two measurements are joined; handle the full pair. This lets us just calculate
		// sensorPtInLH once
		const bool nextIsPair =
			i + 1 < m && meas[0].axis == 0 && meas[1].axis == 1 && meas[0].sensor_idx == meas[1].sensor_idx;

		LinmathPoint3d sensorPtInLH;
		ApplyPoseToPoint(sensorPtInLH, &obj2lh[lh], pt);

		FLT injected_error = 0;
        if (mpfunc_ctx->ptsLength > 0) {
            //injected_error += norm3d(pt) * .01;
        }


        if (nextIsPair) {
			FLT out[2];

			reprojectModel->reprojectXY(cal, sensorPtInLH, out);

			deviates[i] = (out[meas[0].axis] - meas[0].value) / meas[0].variance + injected_error;
			deviates[i + 1] = (out[meas[1].axis] - meas[1].value) / meas[1].variance + injected_error;

			assert(isfinite(deviates[i]));
			assert(isfinite(deviates[i + 1]));
		} else {
			FLT out = reprojectModel->reprojectAxisFn[meas->axis](cal, sensorPtInLH);
			deviates[i] = (out - meas->value) / meas->variance + injected_error;

			assert(isfinite(deviates[i]));
		}

		if (derivs) {
			if (nextIsPair) {
				FLT out[7 * 2] = { 0 };
				reprojectModel->reprojectFullJacObjPose(out, pose, pt, world2lh, cal);

				for (int j = 0; j < 7; j++) {
					if (derivs[j]) {
						derivs[j][i] = out[j];
						derivs[j][i + 1] = out[j + 7];
					}
					assert(!isnan(out[j]));
					assert(!isnan(out[j + 7]));
				}
			} else {
				FLT out[7] = { 0 };
				reprojectModel->reprojectAxisJacobFn[meas->axis](out, pose, pt, world2lh, cal);
				for (int j = 0; j < 7; j++) {
					if (derivs[j]) {
						derivs[j][i] = out[j];
					}
					assert(!isnan(out[j]));
				}
			}
		}

		// Skip the next point -- we handled it already
		if (nextIsPair)
			i++;
	}

	return 0;
}

const char* survive_optimizer_error(int status) {
#define CASE(x) case x: return #x

    switch(status) {
        CASE(MP_ERR_INPUT);
        CASE(MP_ERR_NAN);
        CASE(MP_ERR_FUNC);
        CASE(MP_ERR_NPOINTS);
        CASE(MP_ERR_NFREE);
        CASE(MP_ERR_MEMORY);
        CASE(MP_ERR_INITBOUNDS);
        CASE(MP_ERR_BOUNDS);
        CASE(MP_ERR_PARAM);
        CASE(MP_ERR_DOF);

        /* Potential success status codes */
        CASE(MP_OK_CHI);
        CASE(MP_OK_PAR);
        CASE(MP_OK_BOTH);
        CASE(MP_OK_DIR);

        default:
            return "Unknown error";
    }
}

int survive_optimizer_run(survive_optimizer *optimizer, struct mp_result_struct *result) {
	SurviveContext *ctx = optimizer->so->ctx;
	// SV_INFO("Run start");

#ifndef NDEBUG
	for (int i = 0; i < survive_optimizer_get_parameters_count(optimizer); i++) {
		if ((optimizer->parameters_info[i].limited[0] &&
			 optimizer->parameters[i] < optimizer->parameters_info[i].limits[0]) ||
			(optimizer->parameters_info[i].limited[1] &&
			 optimizer->parameters[i] > optimizer->parameters_info[i].limits[1]) ||
			 isnan(optimizer->parameters[i])) {
		    survive_optimizer_serialize(optimizer, "debug.opt");
			SurviveContext *ctx = optimizer->so->ctx;
			SV_GENERAL_ERROR("Parameter %s is invalid. %f <= %f <= %f should be true",
							 optimizer->parameters_info[i].parname, optimizer->parameters_info[i].limits[0],
							 optimizer->parameters[i], optimizer->parameters_info[i].limits[1])
		}
	}
#endif

	return mpfit(mpfunc, optimizer->measurementsCnt, survive_optimizer_get_parameters_count(optimizer),
				 optimizer->parameters, optimizer->parameters_info, 0, optimizer, result);
}

void survive_optimizer_set_reproject_model(survive_optimizer *optimizer,
										   const survive_reproject_model_t *reprojectModel) {
	optimizer->reprojectModel = reprojectModel;
}

#ifdef NOZLIB
#define gzFile FILE*
#define gzopen fopen
#define gzprintf fprintf
#define gzclose fclose
#endif

void survive_optimizer_serialize(const survive_optimizer *opt, const char *fn) {
    FILE* f = fopen(fn, "w");

    fprintf(f, "object       %s\n", opt->so->codename);
    fprintf(f, "currentBias  %+0.16f\n", opt->current_bias);
    fprintf(f, "initialPose " SurvivePose_format "\n", SURVIVE_POSE_EXPAND(opt->initialPose));
    fprintf(f, "model        %d\n", opt->reprojectModel != &survive_reproject_model);
    fprintf(f, "poseLength   %d\n", opt->poseLength);
    fprintf(f, "cameraLength %d\n", opt->cameraLength);
    fprintf(f, "ptsLength    %d\n", opt->ptsLength);

    fprintf(f, "\n");
    fprintf(f, "parameters   %d\n", survive_optimizer_get_parameters_count(opt));
    fprintf(f, "\t#<name>: <fixed> <value> <min> <max> <use_jacobian>\n");
    for (int i = 0; i < survive_optimizer_get_parameters_count(opt); i++) {
        struct mp_par_struct *info = &opt->parameters_info[i];
        fprintf(f, "\t%10s:", opt->parameters_info[i].parname);
        fprintf(f, " %d", info->fixed);
        fprintf(f, " %+0.16f", opt->parameters[i]);
        fprintf(f, " %+16.f %+16.f", info->limits[0], info->limits[1]);
        fprintf(f, " %d\n", info->side);
    }

    fprintf(f, "\n");
    fprintf(f, "measurementsCnt %ld\n", opt->measurementsCnt);
    fprintf(f, "\t#<lh> <axis> <sensor_idx> <object_idx> <value> <variance>\n");
    for (int i = 0; i < opt->measurementsCnt; i++) {
        survive_optimizer_measurement *meas = &opt->measurements[i];
        fprintf(f, "\t%d", meas->lh);
        fprintf(f, " %d", meas->axis);
        fprintf(f, " %2d", meas->sensor_idx);
        fprintf(f, " %d", meas->object);
        fprintf(f, " %+0.16f", meas->value);
        fprintf(f, " %+0.16f\n", meas->variance);
    }

    fclose(f);
}

survive_optimizer* survive_optimizer_load(const char *fn) {
    survive_optimizer* opt = calloc(sizeof(survive_optimizer), 1);

    FILE *f = fopen(fn, "r");
    int read_count = 0;

    char buffer[LINE_MAX] = {};
    read_count = fscanf(f, "object       %s\n", buffer);
    read_count = fscanf(f, "currentBias  %lf\n", &opt->current_bias);
    read_count = fscanf(f, "initialPose " SurvivePose_sformat "\n", SURVIVE_POSE_SCAN_EXPAND(opt->initialPose));
    int model = 0;
    read_count = fscanf(f, "model        %d\n", &model);
    opt->reprojectModel = model == 0 ? &survive_reproject_model : &survive_reproject_gen2_model;
    read_count = fscanf(f, "poseLength   %d\n", &opt->poseLength);
    read_count = fscanf(f, "cameraLength %d\n", &opt->cameraLength);
    read_count = fscanf(f, "ptsLength    %d\n", &opt->ptsLength);

    int param_count;
    read_count = fscanf(f, "parameters   %d\n", &param_count);
	char *success = fgets(buffer, LINE_MAX, f); // fscanf(f, "\t#<name>: <fixed> <value> <min> <max> <use_jacobian>\n");
	assert(success);

	(void)read_count;
	assert(read_count == survive_optimizer_get_parameters_count(opt));

	SURVIVE_OPTIMIZER_SETUP_HEAP_BUFFERS(*opt);

    for (int i = 0; i < survive_optimizer_get_parameters_count(opt); i++) {
        struct mp_par_struct *info = &opt->parameters_info[i];
        read_count = fscanf(f, "\t");
        opt->parameters_info[i].parname = calloc(128, 1);
        char* b = opt->parameters_info[i].parname;
        char c = fgetc(f);
        while(c != ':') {
            *(b++) = c;
            c = fgetc(f);
        }
        read_count = fscanf(f, " %d", &info->fixed);
        read_count = fscanf(f, " %lf", &opt->parameters[i]);
        read_count = fscanf(f, " %lf %lf", &info->limits[0], &info->limits[1]);
        read_count = fscanf(f, " %d\n", &info->side);
    }

    read_count = fscanf(f, "\n");
    read_count = fscanf(f, "measurementsCnt %lu\n", &opt->measurementsCnt);
    read_count = fscanf(f, "\t#<lh> <axis> <sensor_idx> <object_idx> <value> <variance>\n");
    for (int i = 0; i < opt->measurementsCnt; i++) {
        survive_optimizer_measurement *meas = &opt->measurements[i];
        read_count = fscanf(f, "\t%hhu", &meas->lh);
        read_count = fscanf(f, " %hhu", &meas->axis);
        read_count = fscanf(f, " %hhu", &meas->sensor_idx);
        read_count = fscanf(f, " %d", &meas->object);
        read_count = fscanf(f, " %lf", &meas->value);
        read_count = fscanf(f, " %lf\n", &meas->variance);
    }

    fclose(f);
    return opt;
}

SURVIVE_EXPORT FLT survive_optimizer_current_norm(const survive_optimizer *opt) {
    int m = opt->measurementsCnt;
    int npar = survive_optimizer_get_parameters_count(opt);
    double* p = opt->parameters;
    double *deviates = alloca(sizeof(double) * m);
    double **derivs = 0;
    mpfunc(m, npar, p, deviates, 0, (void*)opt);

    double fnorm = 0;
    for(size_t i = 0;i < m;i++) {
        fnorm += deviates[i] * deviates[i];
    }
    return fnorm;
}

SURVIVE_EXPORT int survive_optimizer_nonfixed_cnt(const survive_optimizer *ctx) {
    int rtn = 0;
    for(int i = 0;i < survive_optimizer_get_parameters_count(ctx);i++) {
        if(!ctx->parameters_info[i].fixed)
            rtn++;
    }
    return rtn;
}
SURVIVE_EXPORT void survive_optimizer_get_nonfixed(const survive_optimizer *ctx, double* params) {
    int rtn = 0;
    for(int i = 0;i < survive_optimizer_get_parameters_count(ctx);i++) {
        if(!ctx->parameters_info[i].fixed)
            *(params++) = ctx->parameters[i];
    }
}
SURVIVE_EXPORT void survive_optimizer_set_nonfixed(survive_optimizer *ctx, double* params) {
    int rtn = 0;
    for(int i = 0;i < survive_optimizer_get_parameters_count(ctx);i++) {
        if(!ctx->parameters_info[i].fixed)
            ctx->parameters[i] = *(params++);
    }
}
