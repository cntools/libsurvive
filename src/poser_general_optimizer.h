#include <stdlib.h>
#include <survive.h>

typedef struct GeneralOptimizerData {
	int failures_to_reset;
	int failures_to_reset_cntr;
	int successes_to_reset;
	int successes_to_reset_cntr;

	uint32_t failures_since_success;
	FLT max_error, max_cal_error;

	struct {
		int runs;
		int poser_seed_runs;
		int32_t successes;
		int error_failures;
	} stats;

	PoserCB seed_poser;
	SurviveObject *so;

	SurvivePose lastSuccess;
	FLT lastSuccessTime;
} GeneralOptimizerData;

SURVIVE_EXPORT void general_optimizer_data_init(GeneralOptimizerData *d, SurviveObject *so);
SURVIVE_EXPORT void general_optimizer_data_dtor(GeneralOptimizerData *d);

SURVIVE_EXPORT void general_optimizer_data_record_failure(GeneralOptimizerData *d);
SURVIVE_EXPORT bool general_optimizer_data_record_success(GeneralOptimizerData *d, FLT error, const SurvivePose *pose, bool isCal);
SURVIVE_EXPORT void general_optimizer_data_record_imu(GeneralOptimizerData *d, PoserDataIMU *imu);
SURVIVE_EXPORT bool general_optimizer_data_record_current_pose(GeneralOptimizerData *d, PoserDataLight *l,
															   SurvivePose *p);
SURVIVE_EXPORT bool general_optimizer_data_record_current_lhs(GeneralOptimizerData *d, PoserDataLight *l,
															  SurvivePose *lhs);
