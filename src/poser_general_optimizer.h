#include <survive.h>
#include <stdlib.h>

typedef struct GeneralOptimizerData {
	int failures_to_reset;
	int failures_to_reset_cntr;
	int successes_to_reset;
	int successes_to_reset_cntr;

	FLT max_error;

	struct {
		int runs;
		int poser_seed_runs;
		int error_failures;
	} stats;

	PoserCB seed_poser;
	SurviveObject *so;
} GeneralOptimizerData;

void general_optimizer_data_init(GeneralOptimizerData *d, SurviveObject *so);
void general_optimizer_data_dtor(GeneralOptimizerData *d);

void general_optimizer_data_record_failure(GeneralOptimizerData *d);
bool general_optimizer_data_record_success(GeneralOptimizerData *d, FLT error);
void general_optimizer_data_record_imu(GeneralOptimizerData *d, PoserDataIMU *imu);
bool general_optimizer_data_record_current_pose(GeneralOptimizerData *d, PoserData *hdr, size_t len_hdr,
												SurvivePose *p);
