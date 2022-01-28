#include "os_generic.h"
#include "survive.h"
#include "survive_recording.h"

#include <stdio.h>
#include <stdlib.h>
#include <survive_optimizer.h>
#include <survive_reproject_gen2.h>

STATIC_CONFIG_ITEM(GSS_ENABLE, "globalscenesolver", 'i', "Enable global scene solver", 0)

#ifndef GSS_NUM_STORED_SCENES
#define GSS_NUM_STORED_SCENES 32
#endif

typedef struct global_scene_solver {
	struct SurviveContext *ctx;

	size_t scenes_cnt;
	struct PoserDataGlobalScene scenes[GSS_NUM_STORED_SCENES];

	size_t last_capture_time_cnt;
	survive_long_timecode *last_capture_time;

	int solve_counts;
	int solve_count_max;

	bool needsSolve;
	FLT last_addition;

	imu_process_func imu_fn;
	sync_process_func prior_sync_fn;
	light_pulse_process_func prior_light_pulse;
	ootx_received_process_func prior_ootx_fn;
} global_scene_solver;

static size_t add_scenes(struct global_scene_solver *gss, SurviveObject *so) {
	size_t rtn = 0;
	SurviveContext *ctx = so->ctx;

	survive_long_timecode sensor_time_window = SurviveSensorActivations_stationary_time(&so->activations) / 2;

	SurviveSensorActivations *activations = &so->activations;

	struct PoserDataGlobalScene *scene = &gss->scenes[gss->scenes_cnt % GSS_NUM_STORED_SCENES];

	scene->pose = so->OutPoseIMU;

	scene->so = so;
	copy3d(scene->accel, activations->accel);
	scene->meas_cnt = 0;
	scene->meas = SV_REALLOC(scene->meas, 32 * 2 * ctx->activeLighthouses * sizeof(scene->meas[0]));

	size_t lh_meas[NUM_GEN2_LIGHTHOUSES] = {0};
	for (uint8_t lh = 0; lh < ctx->activeLighthouses; lh++) {
		for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				bool isReadingValid =
					SurviveSensorActivations_is_reading_valid(activations, sensor_time_window, sensor, lh, axis);

				if (isReadingValid) {
					const FLT *a = activations->angles[sensor][lh];

					PoserDataGlobalSceneMeasurement *meas = scene->meas + scene->meas_cnt;

					meas->axis = axis;
					meas->value = a[axis];
					meas->sensor_idx = sensor;
					meas->lh = lh;
					lh_meas[lh]++;
					scene->meas_cnt++;
				}
			}
		}
	}

	if (scene->meas_cnt > 4) {
		gss->scenes_cnt++;
		rtn++;
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			SV_VERBOSE(100, "Scene %d for lh %d", (int)lh_meas[i], i);
		}
	}

	return rtn;
}

static bool run_optimization(global_scene_solver *gss) {
	if (gss->solve_counts > gss->solve_count_max && gss->solve_count_max > 0)
		return false;

	PoserDataGlobalScenes pgss = {
		.hdr = {.pt = POSERDATA_GLOBAL_SCENES}, .scenes_cnt = gss->scenes_cnt, .scenes = gss->scenes};
	if (pgss.scenes_cnt > GSS_NUM_STORED_SCENES)
		pgss.scenes_cnt = GSS_NUM_STORED_SCENES;
	gss->solve_counts++;
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

static void set_needs_solve(global_scene_solver *gss) {
	FLT now = survive_run_time(gss->ctx);

	for (int lh = 0; lh < gss->ctx->activeLighthouses; lh++) {
		if (!gss->ctx->bsd[lh].OOTXSet)
			return;
	}

	gss->needsSolve = true;
	gss->last_addition = now;
}

static size_t check_object(global_scene_solver *gss, int i, SurviveObject *so) {
	if (gss->solve_counts > gss->solve_count_max && gss->solve_count_max > 0)
		return false;

	size_t scenes_added = 0;
	SurviveContext *ctx = gss->ctx;

	survive_long_timecode last_event_time = SurviveSensorActivations_last_time(&so->activations);
	survive_long_timecode last_change = so->activations.last_light_change;
	survive_long_timecode standstill_time = SurviveSensorActivations_stationary_time(&so->activations);
	survive_long_timecode lockout_time = so->timebase_hz * .1;

	bool activations_changed = so->activations.last_light_change != gss->last_capture_time[i];
	bool spreadout = (last_event_time - gss->last_capture_time[i]) > so->timebase_hz * 3;
	bool light_static = (last_event_time - last_change) > lockout_time;
	bool not_moving = (standstill_time > SurviveSensorActivations_default_tolerance * 8);

	if (activations_changed && spreadout && light_static && not_moving) {
		size_t new_scenes = add_scenes(gss, so);
		if (new_scenes) {
			gss->last_capture_time[i] = so->activations.last_light_change;
			scenes_added += new_scenes;
			SV_VERBOSE(10, "Adding scene (%d) for %s at %6.4f (%f)", (int)gss->scenes_cnt % GSS_NUM_STORED_SCENES,
					   so->codename, survive_run_time(ctx),
					   SurviveSensorActivations_stationary_time(&so->activations) / 48000000.);
		}
	}

	if (scenes_added) {
		set_needs_solve(gss);
	}

	if (gss->needsSolve && (gss->last_addition + 1) < survive_run_time(ctx)) {
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
	gss->prior_light_pulse(so, sensor_id, acode, timecode, length, lh);

	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
}
static void imu_fn(SurviveObject *so, int mask, const FLT *accelgyro, survive_timecode timecode, int id) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(so->ctx, DriverRegGlobalSceneSolverClose);
	gss->imu_fn(so, mask, accelgyro, timecode, id);

	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
}
static void sync_fn(SurviveObject *so, survive_channel channel, survive_timecode timeofsync, bool ootx, bool gen) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(so->ctx, DriverRegGlobalSceneSolverClose);
	gss->prior_sync_fn(so, channel, timeofsync, ootx, gen);

	check_for_new_objects(gss);
	check_object(gss, survive_get_so_idx(so), so);
}

global_scene_solver *global_scene_solver_init(global_scene_solver *driver, SurviveContext *ctx) {
	driver->ctx = ctx;
	driver->last_capture_time_cnt = 0;
	driver->last_capture_time = SV_CALLOC_N(driver->last_capture_time_cnt, sizeof(survive_long_timecode) * 4);

	return driver;
}

static void ootx_recv(struct SurviveContext *ctx, uint8_t bsd_idx) {
	global_scene_solver *gss =
		(global_scene_solver *)survive_get_driver_by_closefn(ctx, DriverRegGlobalSceneSolverClose);

	gss->prior_ootx_fn(ctx, bsd_idx);

	set_needs_solve(gss);
}
int DriverRegGlobalSceneSolver(SurviveContext *ctx) {
	global_scene_solver *driver = SV_NEW(global_scene_solver, ctx);

	int flag = survive_configi(ctx, GSS_ENABLE_TAG, SC_GET, 1);
	driver->solve_count_max = flag > 1 ? flag : -1;

	driver->imu_fn = survive_install_imu_fn(ctx, imu_fn);
	driver->prior_sync_fn = survive_install_sync_fn(ctx, sync_fn);
	driver->prior_light_pulse = survive_install_light_pulse_fn(ctx, light_pulse_fn);
	driver->prior_ootx_fn = survive_install_ootx_received_fn(ctx, ootx_recv);

	survive_add_driver(ctx, driver, DriverRegGlobalSceneSolverPoll, DriverRegGlobalSceneSolverClose);
	return SURVIVE_DRIVER_PASSIVE;
}

REGISTER_LINKTIME(DriverRegGlobalSceneSolver)
