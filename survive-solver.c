#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif

#include "src/survive_default_devices.h"
#include "survive_optimizer.h"
#include <survive.h>

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

	int status = survive_optimizer_run(mpctx, &result);

	FLT lh_errors[NUM_GEN2_LIGHTHOUSES] = {0};
	size_t lh_cnt[NUM_GEN2_LIGHTHOUSES] = {0};

	printf("Residuals:\n");
	for (int i = 0; i < mpctx->measurementsCnt; i++) {
		survive_optimizer_measurement *meas = &mpctx->measurements[i];
		printf("%4d LH %2d Axis %d Sensor %2d Object %d Valid %d %+3.5f\n", i, meas->lh, meas->axis, meas->sensor_idx,
			   meas->object, meas->invalid, result.resid[i]);

		lh_errors[meas->lh] += fabs(result.resid[i]);
		lh_cnt[meas->lh]++;
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
	for (int i = 0; i < survive_optimizer_get_parameters_count(mpctx); i++) {
		for (int j = 0; j < survive_optimizer_get_parameters_count(mpctx); j++) {
			FLT v = result.covar[i + j * survive_optimizer_get_parameters_count(mpctx)];
			if (v != 0) {
				printf("%3d %3d\t%+3.3f\n", i, j, v);
			}
		}
	}

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
