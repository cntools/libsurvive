#include "src/survive_default_devices.h"
#include "survive_optimizer.h"
#include <survive.h>

int main(int argc, char **argv) {
	survive_optimizer *mpctx = survive_optimizer_load(argv[1]);
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

	printf("Residuals:\n");
	for (int i = 0; i < mpctx->measurementsCnt; i++)
		printf("%4d %+3.5f\n", i, result.resid[i]);

	printf("Parameter errors:\n");
	for (int i = 0; i < survive_optimizer_get_parameters_count(mpctx); i++)
		printf("%4d %+3.5f\n", i, result.xerror[i]);

	for (int i = 0; i < survive_optimizer_get_parameters_count(mpctx); i++) {
		for (int j = 0; j < survive_optimizer_get_parameters_count(mpctx); j++)
			printf("%+3.3f   ", result.covar[i + j * survive_optimizer_get_parameters_count(mpctx)]);
		printf("\n");
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