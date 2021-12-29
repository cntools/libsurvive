#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif

#include "src/survive_default_devices.h"
#include "survive_optimizer.h"
#include <cnmatrix/cn_matrix.h>
#include <survive.h>

static void sv_print_mat(const char *name, const CnMat *M, bool newlines) {
	char term = newlines ? '\n' : ' ';
	if (!M) {
		fprintf(stdout, "null%c", term);
		return;
	}
	fprintf(stdout, "%4s %2d x %2d:%c", name, M->rows, M->cols, term);
	FLT scale = cn_sum(M);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			FLT v = cnMatrixGet(M, i, j);
			if (v == 0)
				fprintf(stdout, "         0,\t");
			else
				fprintf(stdout, "%+5.2e,\t", v);
		}
		if (newlines)
			fprintf(stdout, "\n");
	}
	fprintf(stdout, "\n");
}

int main(int argc, char **argv) {
	survive_optimizer *mpctx = survive_optimizer_load(argv[1]);

	if (mpctx == 0) {
		fprintf(stderr, "%s doesn't' exit or didn't load properly\n", argv[1]);
		return -1;
	}

	struct mp_result_struct result = {0};

	SurvivePose *objects = survive_optimizer_get_pose(mpctx);

	for (int i = 0; i < mpctx->poseLength; i++) {
		printf("Obj %2d: " SurvivePose_format "\n", i, SURVIVE_POSE_EXPAND(objects[i]));
	}

	SurvivePose *camera = survive_optimizer_get_camera(mpctx);
	for (int i = 0; i < mpctx->cameraLength; i++) {
		printf("LH  %2d: " SurvivePose_format "\n", i, SURVIVE_POSE_EXPAND(camera[i]));
	}

	result.resid = alloca(mpctx->measurementsCnt * sizeof(double));
	result.xerror = alloca(survive_optimizer_get_parameters_count(mpctx) * sizeof(double));
	result.covar = alloca(survive_optimizer_get_parameters_count(mpctx) *
						  survive_optimizer_get_parameters_count(mpctx) * sizeof(double));

	int status = survive_optimizer_run(mpctx, &result, 0);

	FLT lh_errors[NUM_GEN2_LIGHTHOUSES] = {0};
	size_t lh_cnt[NUM_GEN2_LIGHTHOUSES] = {0};

	printf("Residuals:\n");
	for (int i = 0; i < mpctx->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &mpctx->measurements[i];
		printf("%4d LH %2d Axis %d Sensor %2d Object %d Valid %d %+3.5f\n", i, meas->light.lh, meas->light.axis,
			   meas->light.sensor_idx, meas->light.object, meas->invalid, result.resid[i]);

		lh_errors[meas->light.lh] += fabs(result.resid[i]);
		lh_cnt[meas->light.lh]++;
	}
	printf("\n");
	printf("LH errors:\n");
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (lh_cnt[i]) {
			printf("\t%d %d %f\n", i, (int)lh_cnt[i], lh_errors[i] / (FLT)lh_cnt[i]);
		}
	}
	printf("\n");

	printf("Parameter errors:\n");
	for (int i = 0; i < survive_optimizer_get_parameters_count(mpctx); i++)
		if (result.xerror[i] != 0)
			printf("%4d %+3.5f\n", i, result.xerror[i]);
	printf("\n");
	printf("Covariances: \n");
	CnMat R = cnMat(survive_optimizer_get_parameters_count(mpctx), survive_optimizer_get_parameters_count(mpctx),
					result.covar);
	sv_print_mat("Covariances", &R, true);

	printf("MPFIT status %f/%f (%d measurements, %d - %s)\n", result.orignorm, result.bestnorm,
		   (int)mpctx->measurementsCnt, status, survive_optimizer_error(status));

	printf("Function Evals: %d\n", result.nfev);
	printf("Iterations:     %d\n", result.niter);
	printf("Params:         %d (Fixed: %d, Free: %d)\n", result.npar, result.npar - result.nfree, result.nfree);
	printf("Pegged params:  %d\n", result.npegged);

	for (int i = 0; i < mpctx->poseLength; i++) {
		printf("Obj %2d: " SurvivePose_format "\n", i, SURVIVE_POSE_EXPAND(objects[i]));
	}

	for (int i = 0; i < mpctx->cameraLength; i++) {
		printf("LH %2d: " SurvivePose_format "\n", i, SURVIVE_POSE_EXPAND(camera[i]));
	}

	return 0;
}
