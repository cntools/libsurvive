#include "poser_general_optimizer.h"
#include "string.h"

#include <assert.h>
#include <malloc.h>
#include <stdio.h>

STATIC_CONFIG_ITEM( CONFIG_MAX_ERROR, "max-error", 'f', "Maximum error permitted by poser_general_optimizer", .0001 );
STATIC_CONFIG_ITEM( CONFIG_FAIL_TO_RESET, "failures-to-reset", 'i', "Failures needed before seed poser is re-run", 1 );
STATIC_CONFIG_ITEM( CONFIG_SUC_TO_RESET, "successes-to-reset", 'i', "Reset periodically even if there were no failures", -1 );
STATIC_CONFIG_ITEM( CONFIG_SEED_POSER, "seed-poser", 's', "Poser to be used to seed optimizer", "PoserEPNP" );

void *GetDriver(const char *name);
void general_optimizer_data_init(GeneralOptimizerData *d, SurviveObject *so) {
	memset(d, 0, sizeof(*d));
	d->so = so;

	SurviveContext *ctx = so->ctx;

	survive_attach_configf( ctx, "max-error", &d->max_error );
	survive_attach_configi( ctx, "failures-to-reset", &d->failures_to_reset );
	survive_attach_configi( ctx, "successes-to-reset", &d->successes_to_reset );

	const char *subposer = survive_configs(ctx, "seed-poser", SC_GET, "PoserEPNP");
	d->seed_poser = (PoserCB)GetDriver(subposer);

	SV_INFO("Initializing general optimizer:");
	SV_INFO("\tmax-error: %f", d->max_error);
	SV_INFO("\tsuccesses-to-reset: %d", d->successes_to_reset);
	SV_INFO("\tfailures-to-reset: %d", d->failures_to_reset);
	SV_INFO("\tseed-poser: %s(%p)", subposer, d->seed_poser);
}
void general_optimizer_data_record_failure(GeneralOptimizerData *d) {
	if (d->failures_to_reset_cntr > 0)
		d->failures_to_reset_cntr--;
}
bool general_optimizer_data_record_success(GeneralOptimizerData *d, FLT error) {
	d->stats.runs++;
	if (d->max_error > error) {
		if (d->successes_to_reset_cntr > 0)
			d->successes_to_reset_cntr--;
		d->failures_to_reset_cntr = d->failures_to_reset;
		return true;
	}

	if (d->failures_to_reset_cntr > 0)
		d->failures_to_reset_cntr--;

	return false;
}

typedef struct {
	bool hasInfo;
	SurvivePose pose;
} set_position_t;

static void set_position(SurviveObject *so, uint32_t timecode, SurvivePose *new_pose, void *_user) {
	set_position_t *user = _user;
	assert(user->hasInfo == false);
	user->hasInfo = true;
	user->pose = *new_pose;
}

bool general_optimizer_data_record_current_pose(GeneralOptimizerData *d, PoserData *_hdr, size_t len_hdr,
												SurvivePose *soLocation) {
	*soLocation = d->so->OutPose;
	bool currentPositionValid = quatmagnitude(soLocation->Rot) != 0;
	static bool seed_warning = false;
	if (d->successes_to_reset_cntr == 0 || d->failures_to_reset_cntr == 0 || currentPositionValid == 0) {
		PoserCB driver = d->seed_poser;
		SurviveContext *ctx = d->so->ctx;
		if (driver) {

			PoserData *hdr = alloca(len_hdr);
			memcpy(hdr, _hdr, len_hdr);
			memset(hdr, 0, sizeof(PoserData)); // Clear callback functions
			hdr->pt = _hdr->pt;
			hdr->poseproc = set_position;

			set_position_t locations = {0};
			hdr->userdata = &locations;
			driver(d->so, hdr);
			d->stats.poser_seed_runs++;

			if (locations.hasInfo == false) {
				return false;
			} else if (locations.hasInfo) {
				*soLocation = locations.pose;
			}

			d->successes_to_reset_cntr = d->successes_to_reset;
		} else if (seed_warning == false) {
			seed_warning = true;
			SV_INFO("Not using a seed poser for SBA; results will likely be way off");
		}
	}
	return true;
}

void general_optimizer_data_record_imu(GeneralOptimizerData *d, PoserDataIMU *imu) {
	if (d->seed_poser) {
		d->seed_poser(d->so, &imu->hdr);
	}
}

void general_optimizer_data_dtor(GeneralOptimizerData *d) {
	SurviveContext *ctx = d->so->ctx;
	SV_INFO("\tseed runs %d / %d", d->stats.poser_seed_runs, d->stats.runs);
	SV_INFO("\terror failures %d", d->stats.error_failures);
}
