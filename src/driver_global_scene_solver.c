#include "os_generic.h"
#include "survive.h"
#include "survive_recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <survive_optimizer.h>
#include <survive_reproject_gen2.h>

STATIC_CONFIG_ITEM(GSS_ENABLE, "globalscenesolver", 'i', "Enable global scene solver", 1)

#ifndef GSS_NUM_STORED_SCENES
#define GSS_NUM_STORED_SCENES 16
#endif

typedef struct global_scene_solver {
	struct SurviveContext *ctx;

	size_t scenes_cnt;
	struct PoserDataGlobalScene scenes[GSS_NUM_STORED_SCENES];

	size_t last_capture_time_cnt;
	survive_long_timecode *last_capture_time;

	bool needsSolve;
	FLT last_addition;

	sync_process_func prior_sync_fn;
	light_pulse_process_func prior_light_pulse;
} global_scene_solver;

static size_t add_scenes(struct global_scene_solver *gss, SurviveObject *so) {
	size_t rtn = 0;
	SurviveContext *ctx = so->ctx;

	survive_timecode sensor_time_window = SurviveSensorActivations_stationary_time(&so->activations) / 2;

	SurviveSensorActivations *activations = &so->activations;

	struct PoserDataGlobalScene *scene = &gss->scenes[gss->scenes_cnt % GSS_NUM_STORED_SCENES];

	scene->pose = so->OutPoseIMU;

	scene->so = so;
	copy3d(scene->accel, activations->accel);
	scene->meas_cnt = 0;
	scene->meas = SV_REALLOC(scene->meas, 32 * 2 * ctx->activeLighthouses * sizeof(scene->meas[0]));

	for (uint8_t lh = 0; lh < ctx->activeLighthouses; lh++) {
		for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				bool isReadingValid = SurviveSensorActivations_isReadingValid(
					activations, sensor_time_window, activations->last_light, sensor, lh, axis);

				if (isReadingValid) {
					const FLT *a = activations->angles[sensor][lh];

					PoserDataGlobalSceneMeasurement *meas = scene->meas + scene->meas_cnt;

					meas->axis = axis;
					meas->value = a[axis];
					meas->sensor_idx = sensor;
					meas->lh = lh;

					scene->meas_cnt++;
				}
			}
		}
	}

	if (scene->meas_cnt > 10) {
		gss->scenes_cnt++;
		rtn++;
	}

	return rtn;
}

static bool run_optimization(global_scene_solver *gss) {
	PoserDataGlobalScenes pgss = {
		.hdr = {.pt = POSERDATA_GLOBAL_SCENES}, .scenes_cnt = gss->scenes_cnt, .scenes = gss->scenes};
	if (pgss.scenes_cnt > GSS_NUM_STORED_SCENES)
		pgss.scenes_cnt = GSS_NUM_STORED_SCENES;

	return gss->ctx->PoserFn(gss->ctx->objs[0], &gss->ctx->objs[0]->PoserFnData, (PoserData *)&pgss) == 0;
}

static void notify_global_data_available(global_scene_solver *gss, SurviveObject *so) {
	PoserDataGlobalScenes pgss = {.hdr = {.pt = POSERDATA_GLOBAL_SCENES}, .scenes_cnt = 0, .scenes = 0};

	gss->ctx->PoserFn(so, &so->PoserFnData, (PoserData *)&pgss);
}

static void check_for_new_objects(global_scene_solver *gss) {
	SurviveContext *ctx = gss->ctx;
	if (ctx->objs_ct > gss->last_capture_time_cnt) {
		gss->last_capture_time = SV_REALLOC(gss->last_capture_time, ctx->objs_ct * sizeof(survive_long_timecode));
		for (int i = gss->last_capture_time_cnt; i < ctx->objs_ct; i++) {
			gss->last_capture_time[i] = 0;

			notify_global_data_available(gss, ctx->objs[i]);
		}
		gss->last_capture_time_cnt = ctx->objs_ct;
	}
}

static size_t check_object(global_scene_solver *gss, int i, SurviveObject *so) {
	size_t scenes_added = 0;
	SurviveContext *ctx = gss->ctx;

	survive_long_timecode last_event_time = SurviveSensorActivations_last_time(&so->activations);
	survive_long_timecode last_change = so->activations.last_light_change;
	if (so->activations.last_movement != gss->last_capture_time[i] &&
		((last_event_time - last_change) > so->timebase_hz * .1)) {

		size_t new_scenes = add_scenes(gss, so);
		if (new_scenes) {
			gss->last_capture_time[i] = so->activations.last_movement;
		}
		scenes_added += new_scenes;

		if (new_scenes) {
			SV_VERBOSE(10, "Adding scene for %s at %6.4f (%f)", so->codename, survive_run_time(ctx),
					   SurviveSensorActivations_stationary_time(&so->activations) / 48000000.);
		}
	}

	FLT now = survive_run_time(ctx);
	if (scenes_added) {
		gss->needsSolve = true;
		gss->last_addition = now;
	}

	if (gss->needsSolve && (gss->last_addition + 1) < now) {
		gss->needsSolve = false;
		run_optimization(gss);
	}

	return scenes_added;
}

static int DriverRegGlobalSceneSolverPoll(struct SurviveContext *ctx, void *driver) { return 0; }

static int DriverRegGlobalSceneSolverClose(struct SurviveContext *ctx, void *driver) {
	global_scene_solver *gss = (global_scene_solver *)driver;
	free(gss->last_capture_time);
	for (int i = 0; i < GSS_NUM_STORED_SCENES; i++) {
		free(gss->scenes[i].meas);
	}
	free(driver);
	return 0;
}

static int survive_get_so_idx(const SurviveObject *so) {
	for (int i = 0; i < so->ctx->objs_ct; i++) {
		if (so == so->ctx->objs[i]) {
			return i;
		}
	}
	return -1;
}

static void light_pulse_fn(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode, FLT length,
						   uint32_t lh) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(so->ctx, DriverRegGlobalSceneSolverClose);
	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
	gss->prior_light_pulse(so, sensor_id, acode, timecode, length, lh);
}
static void sync_fn(SurviveObject *so, survive_channel channel, survive_timecode timeofsync, bool ootx, bool gen) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(so->ctx, DriverRegGlobalSceneSolverClose);
	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
	gss->prior_sync_fn(so, channel, timeofsync, ootx, gen);
}

global_scene_solver *global_scene_solver_init(global_scene_solver *driver, SurviveContext *ctx) {
	driver->ctx = ctx;
	driver->last_capture_time_cnt = 0;
	driver->last_capture_time = SV_CALLOC(driver->last_capture_time_cnt, sizeof(survive_long_timecode) * 4);

	return driver;
}

int DriverRegGlobalSceneSolver(SurviveContext *ctx) {
	global_scene_solver *driver = SV_NEW(global_scene_solver, ctx);

	driver->prior_sync_fn = survive_install_sync_fn(ctx, sync_fn);
	driver->prior_light_pulse = survive_install_light_pulse_fn(ctx, light_pulse_fn);
	survive_add_driver(ctx, driver, DriverRegGlobalSceneSolverPoll, DriverRegGlobalSceneSolverClose);
	return 0;
}

REGISTER_LINKTIME(DriverRegGlobalSceneSolver)
