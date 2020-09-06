#include "os_generic.h"
#include "survive.h"

#include "gattlib.h"
#include <os_generic.h>
#include <stdio.h>
#include <stdlib.h>
#include <survive_optimizer.h>
#include <survive_reproject_gen2.h>
#include <sys/queue.h>
#include <sys/socket.h>
#include <unistd.h>

STATIC_CONFIG_ITEM(GSS_ENABLE, "globalscenesolver", 'i', "Enable global scene solver", 0)

#ifndef GSS_NUM_STORED_SCENES
#define GSS_NUM_STORED_SCENES 16
#endif

typedef struct {
	FLT value;
	uint8_t lh;
	uint8_t sensor_idx;
	uint8_t axis;
} gss_measurement;

struct gss_scene {
	struct SurviveObject *so;
	SurvivePose pose;
	LinmathPoint3d accel;
	int8_t lh;

	size_t meas_cnt;
	gss_measurement *meas;
};

typedef struct global_scene_solver {
	struct SurviveContext *ctx;

	size_t scenes_cnt;
	struct gss_scene scenes[GSS_NUM_STORED_SCENES];

	size_t last_capture_time_cnt;
	survive_long_timecode *last_capture_time;

} global_scene_solver;

static size_t add_scenes(struct global_scene_solver *gss, SurviveObject *so) {
	size_t rtn = 0;
	SurviveContext *ctx = so->ctx;

	survive_timecode sensor_time_window = SurviveSensorActivations_stationary_time(&so->activations) / 2;

	SurviveSensorActivations *activations = &so->activations;

	struct gss_scene *scene = &gss->scenes[gss->scenes_cnt % GSS_NUM_STORED_SCENES];

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

					gss_measurement *meas = scene->meas + scene->meas_cnt;

					meas->axis = axis;
					meas->value = a[axis];
					meas->sensor_idx = sensor;
					meas->lh = lh;

					scene->meas_cnt++;
				}
			}
		}
	}

	if (scene->meas_cnt > 6) {
		gss->scenes_cnt++;
		rtn++;
	}

	return rtn;
}

static void run_optimization(global_scene_solver *gss) {
	struct SurviveContext *ctx = gss->ctx;
	size_t meas_cnt = 0;
	size_t scenes_cnt = gss->scenes_cnt;
	if (scenes_cnt > GSS_NUM_STORED_SCENES) {
		scenes_cnt = GSS_NUM_STORED_SCENES;
	}
	for (int i = 0; i < scenes_cnt; i++) {
		meas_cnt += gss->scenes[i].meas_cnt;
	}

	survive_optimizer mpfitctx = {
		.reprojectModel = ctx->lh_version == 1 ? &survive_reproject_gen2_model : &survive_reproject_model,
		.poseLength = scenes_cnt,
		.cameraLength = ctx->activeLighthouses,
		.measurementsCnt = meas_cnt,
		//.iteration_cb = iteration_cb
	};

	if (ctx->bsd[0].PositionSet == false)
		return;

	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx, 0);

	survive_optimizer_setup_cameras(&mpfitctx, ctx, false, true);
	survive_optimizer_measurement *meas = mpfitctx.measurements;
	int start = survive_optimizer_get_camera_index(&mpfitctx);
	for (int i = start; i < start + 7; i++) {
		mpfitctx.parameters_info[i].fixed = true;
	}

	for (int i = 0; i < scenes_cnt; i++) {
		meas_cnt += gss->scenes[i].meas_cnt;
		mpfitctx.sos[i] = gss->scenes[i].so;
		survive_optimizer_setup_pose_n(&mpfitctx, &gss->scenes[i].pose, i, false, true);

		SV_VERBOSE(10, "Scene with pose (%s) " SurvivePose_format, mpfitctx.sos[i]->codename,
				   SURVIVE_POSE_EXPAND(gss->scenes[i].pose));
		for (int j = 0; j < gss->scenes[i].meas_cnt; j++) {
			meas->object = i;
			meas->variance = 1;
			meas->value = gss->scenes[i].meas[j].value;
			meas->lh = gss->scenes[i].meas[j].lh;
			meas->axis = gss->scenes[i].meas[j].axis;
			meas->sensor_idx = gss->scenes[i].meas[j].sensor_idx;
			meas->invalid = false;
			meas++;
		}
	}

	mp_result result = {0};
	survive_release_ctx_lock(ctx);
	int res = survive_optimizer_run(&mpfitctx, &result);
	survive_get_ctx_lock(ctx);
	if (res > 0) {

		SurvivePose *opt_cameras = survive_optimizer_get_camera(&mpfitctx);
		SurvivePose cameras[NUM_GEN2_LIGHTHOUSES] = {0};
		for (int i = 0; i < mpfitctx.poseLength; i++) {
			SurvivePose *p = &survive_optimizer_get_pose(&mpfitctx)[i];
			SV_VERBOSE(10, "Solved scene with pose (%s) " SurvivePose_format, mpfitctx.sos[i]->codename,
					   SURVIVE_POSE_EXPAND(*p));
			memcpy(&gss->scenes[i].pose, p, sizeof(SurvivePose));
		}
		for (int i = 0; i < mpfitctx.cameraLength; i++) {
			if (!quatiszero(opt_cameras[i].Rot)) {
				cameras[i] = InvertPoseRtn(&opt_cameras[i]);

				LinmathPoint3d up = {ctx->bsd[i].accel[0], ctx->bsd[i].accel[1], ctx->bsd[i].accel[2]};
				normalize3d(up, up);
				LinmathPoint3d err;
				quatrotatevector(err, cameras[i].Rot, up);
				SV_INFO("Global solve with %d scenes for %d with error of %f/%10.10f (acc err %5.4f)", (int)scenes_cnt,
						i, result.orignorm, result.bestnorm, 1 + err[2]);
			}
		}
		PoserData_lighthouse_poses_func(0, mpfitctx.sos[0], cameras, ctx->activeLighthouses, 0);
	}
}

static int DriverRegGlobalSceneSolverPoll(struct SurviveContext *ctx, void *driver) {
	global_scene_solver *gss = (global_scene_solver *)driver;
	if (ctx->objs_ct > gss->last_capture_time_cnt) {
		gss->last_capture_time = SV_REALLOC(gss->last_capture_time, ctx->objs_ct * sizeof(survive_long_timecode));
		for (int i = gss->last_capture_time_cnt; i < ctx->objs_ct; i++) {
			gss->last_capture_time[i] = 0;
		}
		gss->last_capture_time_cnt = ctx->objs_ct;
	}

	size_t scenes_added = 0;
	for (int i = 0; i < ctx->objs_ct; i++) {
		SurviveObject *so = ctx->objs[i];
		if (so->activations.last_movement != gss->last_capture_time[i] &&
			SurviveSensorActivations_stationary_time(&so->activations) > so->timebase_hz * 1) {

			gss->last_capture_time[i] = so->activations.last_movement;

			scenes_added += add_scenes(gss, so);
		}
	}

	if (scenes_added && gss->scenes_cnt > 2) {
		run_optimization(gss);
	}
	return 0;
}

static int DriverRegGlobalSceneSolverClose(struct SurviveContext *ctx, void *driver) {
	global_scene_solver *gss = (global_scene_solver *)driver;
	free(gss->last_capture_time);
	for (int i = 0; i < GSS_NUM_STORED_SCENES; i++) {
		free(gss->scenes[i].meas);
	}
	free(driver);
	return 0;
}

global_scene_solver *global_scene_solver_init(global_scene_solver *driver, SurviveContext *ctx) {
	driver->ctx = ctx;
	driver->last_capture_time_cnt = 4;
	driver->last_capture_time = SV_CALLOC(driver->last_capture_time_cnt, sizeof(survive_long_timecode));
	return driver;
}

int DriverRegGlobalSceneSolver(SurviveContext *ctx) {
	global_scene_solver *driver = SV_NEW(global_scene_solver, ctx);

	survive_add_driver(ctx, driver, DriverRegGlobalSceneSolverPoll, DriverRegGlobalSceneSolverClose);
	return 0;
}

REGISTER_LINKTIME(DriverRegGlobalSceneSolver)
