#include "poser_general_optimizer.h"
#include "string.h"
#include "survive_internal.h"

#include <assert.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <stdio.h>

STATIC_CONFIG_ITEM(CONFIG_MAX_ERROR, "max-error", 'f', "Maximum error permitted by poser_general_optimizer.", .1)
STATIC_CONFIG_ITEM(CONFIG_MAX_CAL_ERROR, "max-cal-error", 'f', "Maximum error permitted by poser_general_optimizer.",
				   0.05)
STATIC_CONFIG_ITEM(CONFIG_FAIL_TO_RESET, "failures-to-reset", 'i', "Failures needed before seed poser is re-run.", -1)
STATIC_CONFIG_ITEM(CONFIG_SUC_TO_RESET, "successes-to-reset", 'i',
				   "Reset periodically even if there were no failures. Set to -1 to disable.", -1)
STATIC_CONFIG_ITEM(CONFIG_SEED_POSER, "seed-poser", 's', "Poser to be used to seed optimizer.", "BaryCentricSVD")

STATIC_CONFIG_ITEM(CONFIG_REQUIRED_MEAS, "required-meas", 'i',
				   "Minimum number of measurements needed to try and solve for position", 8)
STATIC_CONFIG_ITEM(CONFIG_TIME_WINDOW, "time-window", 'i',
				   "The length, in ticks, between sensor inputs to treat them as one snapshot",
				   (int)SurviveSensorActivations_default_tolerance * 2)

void general_optimizer_data_init(GeneralOptimizerData *d, SurviveObject *so) {
	memset(d, 0, sizeof(*d));
	d->so = so;

	SurviveContext *ctx = so->ctx;

	survive_attach_configf( ctx, "max-error", &d->max_error );
	survive_attach_configf( ctx, "max-cal-error", &d->max_cal_error );
	survive_attach_configi( ctx, "failures-to-reset", &d->failures_to_reset );
	survive_attach_configi( ctx, "successes-to-reset", &d->successes_to_reset );

	const char *subposer = survive_configs(ctx, "seed-poser", SC_GET, "BaryCentricSVD");
	d->seed_poser = (PoserCB)GetDriverWithPrefix("Poser", subposer);

	SV_VERBOSE(100, "Initializing general optimizer:");
	SV_VERBOSE(100, "\tmax-error: %f", d->max_error);
	SV_VERBOSE(100, "\tsuccesses-to-reset: %d", d->successes_to_reset);
	SV_VERBOSE(100, "\tfailures-to-reset: %d", d->failures_to_reset);
	SV_VERBOSE(100, "\tseed-poser: %s", subposer);
}
void general_optimizer_data_record_failure(GeneralOptimizerData *d) {
	d->stats.error_failures++;
	d->failures_since_success++;
	if (d->failures_to_reset_cntr > 0)
		d->failures_to_reset_cntr--;
}
bool general_optimizer_data_record_success(GeneralOptimizerData *d, FLT error, const SurvivePose *pose, bool isCal) {
	FLT max_error = isCal ? d->max_cal_error : d->max_error;
	d->stats.runs++;
	if (max_error <= 0 || max_error > error) {
		if (d->successes_to_reset_cntr > 0)
			d->successes_to_reset_cntr--;
		if (pose) {
			d->lastSuccess = *pose;
			d->lastSuccessTime = survive_run_time(d->so->ctx);
		}
		d->failures_to_reset_cntr = d->failures_to_reset;
		d->failures_since_success = 0;
		d->stats.successes++;
		return true;
	}

	general_optimizer_data_record_failure(d);

	return false;
}

typedef struct {
	bool hasInfo;
	SurvivePose pose;
	SurvivePose *new_lh2world;
} set_position_t;

static void set_position(SurviveObject *so, uint32_t timecode, const SurvivePose *new_pose, void *_user) {
	set_position_t *user = _user;
	assert(user->hasInfo == false);
	for (int i = 0; i < 3; i++) {
		if (abs((int)new_pose->Pos[i]) > 20.) {
			SurviveContext *ctx = so->ctx;
			SV_WARN("Set position has invalid pose " SurvivePose_format, SURVIVE_POSE_EXPAND(*new_pose));
			return;
		}
	}
	user->hasInfo = true;
	user->pose = *new_pose;
	quatnormalize(user->pose.Rot, user->pose.Rot);
}

void set_cameras(SurviveObject *so, uint8_t lighthouse, SurvivePose *lighthouse_pose, SurvivePose *object_pose,
				 void *_user) {
	set_position_t *user = _user;
	if (user->new_lh2world) {
		user->new_lh2world[lighthouse] = *lighthouse_pose;
		user->hasInfo = true;
	}
}
bool general_optimizer_data_record_current_lhs(GeneralOptimizerData *d, PoserDataLight *l, SurvivePose *lhs) {
	PoserCB driver = d->seed_poser;
	if (driver) {
		size_t len_hdr = PoserData_size(&l->hdr);
		uint8_t *event = alloca(len_hdr);
		memcpy(event, l, len_hdr);
		assert(len_hdr >= sizeof(PoserDataLight));

		PoserDataLight *pl = (PoserDataLight *)event;
		set_position_t locations = {.new_lh2world = lhs};

		pl->hdr.lighthouseposeproc = set_cameras;
		pl->hdr.poseproc = set_position;
		pl->hdr.userdata = &locations;
		pl->assume_current_pose = true;

		driver(d->so, &pl->hdr);

		d->stats.poser_seed_runs++;

		return locations.hasInfo;
	}
	return false;
}
bool general_optimizer_data_record_current_pose(GeneralOptimizerData *d, PoserDataLight *l, SurvivePose *soLocation) {
	if (d->lastSuccessTime + .1 > survive_run_time(d->so->ctx)) {
		*soLocation = d->lastSuccess;
	} else {
		*soLocation = *survive_object_last_imu2world(d->so);
	}

	bool currentPositionValid = quatmagnitude(soLocation->Rot) != 0;
	SurviveContext *ctx = d->so->ctx;

	static bool seed_warning = false;
	if (d->successes_to_reset_cntr == 0 || d->failures_to_reset_cntr == 0 || currentPositionValid == 0) {
		PoserCB driver = d->seed_poser;
		if (driver) {
			size_t len_hdr = PoserData_size(&l->hdr);
			uint8_t *event = alloca(len_hdr);
			memcpy(event, l, len_hdr);
			assert(len_hdr >= sizeof(PoserDataLight));

			PoserDataLight *pl = (PoserDataLight *)event;
			set_position_t locations = { 0 };

			pl->hdr.lighthouseposeproc = set_cameras;
			pl->hdr.poseproc = set_position;
			pl->hdr.userdata = &locations;
			pl->no_lighthouse_solve = true;

			driver(d->so, &pl->hdr);

			d->stats.poser_seed_runs++;

			if (locations.hasInfo == false) {
				return false;
			} else if (locations.hasInfo) {
				*soLocation = locations.pose;
			}

			d->failures_to_reset_cntr = d->failures_to_reset;
			d->successes_to_reset_cntr = d->successes_to_reset;
		} else if (seed_warning == false) {
			seed_warning = true;
			SV_INFO("Not using a seed poser; results will likely be way off");
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

	survive_detach_config(ctx, "max-error", &d->max_error);
	survive_detach_config(ctx, "max-cal-error", &d->max_cal_error);
	survive_detach_config(ctx, "failures-to-reset", &d->failures_to_reset);
	survive_detach_config(ctx, "successes-to-reset", &d->successes_to_reset);

	if (d->seed_poser) {
		PoserData pd;
		pd.pt = POSERDATA_DISASSOCIATE;

		d->seed_poser(d->so, &pd);
	}
	SV_INFO("\tseed runs         %d / %d", d->stats.poser_seed_runs, d->stats.runs);
	SV_INFO("\terror failures    %d", d->stats.error_failures);
}
