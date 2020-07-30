#include "survive_optimizer.h"

#include <malloc.h>

#include "mpfit/mpfit.h"
#include "poser.h"
#include "survive_imu.h"
#include <survive.h>

#include "assert.h"
#include "linmath.h"
#include "math.h"
#include "poser_general_optimizer.h"
#include "string.h"
#include "survive_async_optimizer.h"
#include "survive_cal.h"
#include "survive_config.h"
#include "survive_reproject.h"
#include "survive_reproject_gen2.h"

#ifndef _WIN32
//#define DEBUG_NAN
#endif
#ifdef DEBUG_NAN
#include <fenv.h>
#endif

STATIC_CONFIG_ITEM(SERIALIZE_SOLVE, "serialize-lh-mpfit", 's', "Serialize MPFIT formulization", 0);
STATIC_CONFIG_ITEM(USE_JACOBIAN_FUNCTION, "use-jacobian-function", 'i',
				   "If set to false, a slower numerical approximation of the jacobian is used", 1);
STATIC_CONFIG_ITEM(USE_IMU, "use-imu", 'i', "Use the IMU as part of the pose solver", 1);
STATIC_CONFIG_ITEM(USE_KALMAN, "use-kalman", 'i', "Apply kalman filter as part of the pose solver", 1);
STATIC_CONFIG_ITEM(SENSOR_VARIANCE_PER_SEC, "sensor-variance-per-sec", 'f',
				   "Variance per second to add to the sensor input -- discounts older data", 0.0);
STATIC_CONFIG_ITEM(SENSOR_VARIANCE, "sensor-variance", 'f', "Base variance for each sensor input", 1.0);
STATIC_CONFIG_ITEM(DISABLE_LIGHTHOUSE, "disable-lighthouse", 'i', "Disable given lighthouse from tracking", -1);
STATIC_CONFIG_ITEM(RUN_EVERY_N_SYNCS, "syncs-per-run", 'i', "Number of sync pulses before running optimizer", 1);
STATIC_CONFIG_ITEM(RUN_POSER_ASYNC, "poser-async", 'i', "Run the poser in it's own thread", 0);

typedef struct MPFITStats {
	int meas_failures;
	int total_iterations;
	int total_fev;
	int total_runs;
	double sum_errors;
	double sum_origerrors;
	int status_cnts[9];
} MPFITStats;

typedef struct MPFITGlobalData {
	size_t instances;
	MPFITStats stats;
} MPFITGlobalData;

static MPFITGlobalData g;

typedef struct MPFITData {
	GeneralOptimizerData opt;

	int disable_lighthouse;
	int sensor_time_window;
	// > 0; use jacobian, 0 don't use, < 0 debug
	int use_jacobian_function_obj;
	int use_jacobian_function_lh;
	int required_meas;
  int syncs_per_run;
  int run_async;
  int syncs_per_run_cnt;

  FLT sensor_variance;
  FLT sensor_variance_per_second;

  SurviveIMUTracker tracker;
  bool useIMU;
  bool useKalman;

  const char *serialize_prefix;
  MPFITStats stats;

  struct survive_async_optimizer *async_optimizer;
} MPFITData;

static size_t remove_lh_from_meas(survive_optimizer_measurement *meas, size_t meas_size, int lh) {
	size_t rtn = meas_size;
	for (int i = 0; i < rtn; i++) {
		while (meas[i].lh == lh && i < rtn) {
			meas[i] = meas[rtn - 1];
			rtn--;
		}
	}
	return rtn;
}

static size_t construct_input_from_scene(const MPFITData *d, size_t timecode, const SurviveSensorActivations *scene,
										 size_t *meas_for_lhs, survive_optimizer_measurement *meas) {
	size_t rtn = 0;
	SurviveObject *so = d->opt.so;
	SurviveContext *ctx = so->ctx;

	bool isStationary = SurviveSensorActivations_stationary_time(scene) > so->timebase_hz;
	survive_timecode sensor_time_window = isStationary ? (so->timebase_hz) : d->sensor_time_window;

	const bool force_pair = false;
	for (uint8_t lh = 0; lh < ctx->activeLighthouses; lh++) {
		if (d->disable_lighthouse == lh) {
			continue;
		}

		if (!ctx->bsd[lh].PositionSet && (!isStationary || !ctx->bsd[lh].OOTXSet)) {
			continue;
		}

		if (!ctx->bsd[lh].PositionSet) {
			SV_VERBOSE(200, "Allowing data from %d", lh);
		}

		bool isCandidate = !ctx->bsd[lh].PositionSet;
		size_t candidate_meas = 10;

		size_t meas_for_lh = 0;
		for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				bool isReadingValue =
					SurviveSensorActivations_isReadingValid(scene, sensor_time_window, timecode, sensor, lh, axis);
				if (force_pair) {
					isReadingValue =
						SurviveSensorActivations_isPairValid(scene, sensor_time_window, timecode, sensor, lh);
				}
				if (isReadingValue) {
					const double *a = scene->angles[sensor][lh];
					meas->object = 0;
					meas->axis = axis;
					meas->value = a[axis];
					meas->sensor_idx = sensor;
					meas->lh = lh;
					survive_timecode diff = survive_timecode_difference(timecode, scene->timecode[sensor][lh][axis]);
					meas->variance =
						d->sensor_variance + diff * d->sensor_variance_per_second / (double)so->timebase_hz;
					// SV_INFO("Adding meas %d %d %d %f", lh, sensor, axis, meas->value);
					meas++;
					rtn++;
					meas_for_lh++;
				}
			}
		}
		if (meas_for_lhs) {
			meas_for_lhs[lh] = meas_for_lh;
		}
		if (isCandidate && meas_for_lh < candidate_meas) {
			meas -= meas_for_lh;
			rtn -= meas_for_lh;
		}
	}
	return rtn;
}

static bool find_cameras_invalid_starting_condition(MPFITData *d, size_t meas_size) {
	if (meas_size < d->required_meas * 2) {
		SurviveContext *ctx = d->opt.so->ctx;
		if (meas_size > 0)
			SV_INFO("Can't solve for cameras with just %u measurements", (unsigned int)meas_size);
		return true;
	}
	return false;
}

static bool invalid_starting_condition(MPFITData *d, size_t meas_size, const size_t *meas_for_lhs) {
	static int failure_count = 500;
	struct SurviveObject *so = d->opt.so;

	size_t meas_size_known_lh = 0;
	if (meas_for_lhs) {
		for (uint8_t lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (so->ctx->bsd[lh].PositionSet)
				meas_size_known_lh += meas_for_lhs[lh];
		}
	} else {
		meas_size_known_lh = meas_size;
	}

	if (meas_size_known_lh < d->required_meas) {
		if (failure_count++ == 500) {
			SurviveContext *ctx = so->ctx;
			SV_INFO("Can't solve for position with just %u measurements", (unsigned int)meas_size_known_lh);
			failure_count = 0;
		}
		if (meas_size_known_lh < d->required_meas) {
			d->stats.meas_failures++;
		}
		return true;
	}
	failure_count = 0;
	return false;
}

static void mpfit_set_cameras(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose, SurvivePose *obj_pose,
							  void *user) {
	survive_optimizer *ctx = (survive_optimizer *)user;
	SurvivePose *cameras = survive_optimizer_get_camera(ctx);
	cameras[lighthouse] = InvertPoseRtn(pose);

	assert(!quatiszero(pose->Rot));
	for (int i = 0; i < 7; i++)
		assert(!isnan(((double *)pose)[i]));

	if (obj_pose && !quatiszero(obj_pose->Rot))
		*survive_optimizer_get_pose(ctx) = *obj_pose;
	else
		*survive_optimizer_get_pose(ctx) = LinmathPose_Identity;
}

static inline void serialize_mpfit(MPFITData *d, survive_optimizer *mpfitctx) {
	if (d->serialize_prefix) {
		static int cnt = 0;
		char path[1024] = {0};
		snprintf(path, 1023, "%s_%s_%d.opt", d->serialize_prefix, d->opt.so->codename, cnt++);
		survive_optimizer_serialize(mpfitctx, path);
	}
}

static int get_lh_count(const size_t *meas_for_lhs) {
	int num_lh = 0;
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (meas_for_lhs[i])
			num_lh++;
	}
	return num_lh;
}

struct async_optimizer_user {
	MPFITData *d;
	PoserDataLight pdl;
	bool canPossiblySolveLHS;
	bool worldEstablished;
	size_t meas_for_lhs[NUM_GEN2_LIGHTHOUSES];
};

static int setup_optimizer(struct async_optimizer_user *user, survive_optimizer *mpfitctx,
						   SurviveSensorActivations *scene) {
	MPFITData *d = user->d;
	PoserDataLight *pdl = &user->pdl;
	size_t *meas_for_lhs = user->meas_for_lhs;

	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	SurvivePose *soLocation = survive_optimizer_get_pose(mpfitctx);
	survive_optimizer_setup_cameras(mpfitctx, so->ctx, true, d->use_jacobian_function_lh);

	bool worldEstablished = false;
	for (int lh = 0; lh < ctx->activeLighthouses && !worldEstablished; lh++)
		worldEstablished |= ctx->bsd[lh].PositionSet;

	survive_optimizer_setup_pose(mpfitctx, 0, !worldEstablished, d->use_jacobian_function_obj);

	if (worldEstablished && !general_optimizer_data_record_current_pose(&d->opt, pdl, soLocation)) {
		return -1;
	}

	if (quatiszero(soLocation->Rot))
		soLocation->Rot[0] = 1;

	size_t meas_size = construct_input_from_scene(d, pdl->hdr.timecode, scene, meas_for_lhs, mpfitctx->measurements);

	if (mpfitctx->current_bias > 0) {
		meas_size += 7;
	}

	if (worldEstablished && invalid_starting_condition(d, meas_size, meas_for_lhs)) {
		return -1;
	}

	bool canPossiblySolveLHS = false;
	bool bestObjForCal = true;
	for (int obj_id = 0; obj_id < so->ctx->objs_ct; obj_id++) {
		if (so->ctx->objs[obj_id]->sensor_ct > so->sensor_ct) {
			bestObjForCal = false;
		}
	}

	if (bestObjForCal) {
		for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (!so->ctx->bsd[lh].OOTXSet) {
				// Wait til this thing gets OOTX, and then solve for as much as we can. Avoids doing
				// two solves in a row because of OOTX timing.
				canPossiblySolveLHS = false;
				break;
			}

			if (!so->ctx->bsd[lh].PositionSet && meas_for_lhs[lh] > 0) {
				canPossiblySolveLHS = true;
			}
		}
	}

	SurvivePose lhs[NUM_GEN2_LIGHTHOUSES] = {0};
	if (canPossiblySolveLHS) {
		if (general_optimizer_data_record_current_lhs(&d->opt, pdl, lhs)) {
			for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
				if (!so->ctx->bsd[lh].PositionSet) {
					assert(!isnan(lhs[lh].Rot[0]));
					if (quatiszero(lhs[lh].Rot) && meas_for_lhs[lh] > 0) {
						SV_WARN("Seed poser failed for %d, not trying to solve LH system", lh);
						canPossiblySolveLHS = false;
						break;
					}
				}
			}
		} else {
			canPossiblySolveLHS = false;
		}
	}

	for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
		if (!so->ctx->bsd[lh].PositionSet) {
			if (canPossiblySolveLHS) {
				if (meas_for_lhs[lh]) {
					SV_INFO("Attempting to solve for %d with %lu meas", lh, meas_for_lhs[lh]);
					survive_optimizer_setup_camera(mpfitctx, lh, &lhs[lh], false, d->use_jacobian_function_lh);
				}
			} else {
				SV_VERBOSE(200, "Removing data for %d", lh);
				meas_size = remove_lh_from_meas(mpfitctx->measurements, meas_size, lh);
				meas_for_lhs[lh] = 0;
			}
		}
	}

	user->worldEstablished = worldEstablished;
	user->canPossiblySolveLHS = canPossiblySolveLHS;

	if (!worldEstablished && !canPossiblySolveLHS) {
		return -1;
	}

	mpfitctx->measurementsCnt = meas_size;

	if (d->useKalman || d->useIMU) {
		survive_imu_tracker_predict(&d->tracker, pdl->hdr.timecode, soLocation);
	}

	mpfitctx->initialPose = *soLocation;

	if (canPossiblySolveLHS) {
		serialize_mpfit(d, mpfitctx);
		mpfitctx->cfg = survive_optimizer_precise_config();
	}

	return 0;
}

static FLT handle_optimizer_results(survive_optimizer *mpfitctx, int res, const mp_result *result,
									struct async_optimizer_user *user_data, SurvivePose *out) {
	FLT rtn = -1;

	size_t meas_size = mpfitctx->measurementsCnt;
	SurvivePose *soLocation = survive_optimizer_get_pose(mpfitctx);
	bool canPossiblySolveLHS = user_data->canPossiblySolveLHS;
	bool worldEstablished = user_data->worldEstablished;
	size_t *meas_for_lhs = user_data->meas_for_lhs;
	MPFITData *d = user_data->d;
	PoserDataLight *pdl = &user_data->pdl;

	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	bool status_failure = res <= 0;
	if (status_failure) {
		SV_WARN("MPFIT status failure %s %f/%f (%d measurements, %d)", so->codename, result->orignorm, result->bestnorm,
				(int)meas_size, res);

		general_optimizer_data_record_failure(&d->opt);
		return -1;
	}

	bool error_failure = !general_optimizer_data_record_success(&d->opt, result->bestnorm);
	if (!status_failure && !error_failure) {
		quatnormalize(soLocation->Rot, soLocation->Rot);

		if (canPossiblySolveLHS) {
			if (!worldEstablished)
				*soLocation = (SurvivePose){0};

			SurvivePose *opt_cameras = survive_optimizer_get_camera(mpfitctx);
			SurvivePose cameras[NUM_GEN2_LIGHTHOUSES] = {0};
			for (int i = 0; i < mpfitctx->cameraLength; i++) {
				if (meas_for_lhs[i] > 0 && !quatiszero(opt_cameras[i].Rot)) {
					cameras[i] = InvertPoseRtn(&opt_cameras[i]);
					SV_INFO("Solved for %d with error of %f/%10.10f", i, result->orignorm, result->bestnorm);
				}
			}
			PoserData_lighthouse_poses_func(&pdl->hdr, so, cameras, ctx->activeLighthouses, soLocation);
		}

		*out = *soLocation;
		rtn = result->bestnorm;

		SV_VERBOSE(500, "MPFIT success %s %f/%10.10f (%d measurements, %d result, %d lighthouses)", so->codename,
				   result->orignorm, result->bestnorm, (int)meas_size, res, get_lh_count(meas_for_lhs));

	} else {
		SV_WARN("MPFIT failure %s %f/%f (%d measurements, %d result, %d lighthouses, %d canSolveLHs)", so->codename,
				result->orignorm, result->bestnorm, (int)meas_size, res, get_lh_count(meas_for_lhs),
				canPossiblySolveLHS);
	}

	d->stats.total_fev += result->nfev;
	d->stats.total_iterations += result->niter;
	d->stats.total_runs++;
	d->stats.sum_errors += result->bestnorm;
	d->stats.sum_origerrors += result->orignorm;
	if (result->status > 0) {
		assert(result->status < 10);
		d->stats.status_cnts[result->status - 1]++;
	}
	return rtn;
}

static void handle_results(MPFITData *d, PoserDataLight *lightData, FLT error, SurvivePose *estimate) {
	SurviveObject *so = d->opt.so;
	if (error > 0) {

		FLT var_meters = d->useKalman ? (FLT)0.01 + error : 0;
		FLT var_quat = d->useKalman ? (FLT)0.01 + error : 0;
		FLT var[2] = {var_meters, var_quat};

		if (d->useKalman) {
			survive_imu_tracker_integrate_observation(lightData->hdr.timecode, &d->tracker, estimate, var);
			SurvivePose predicted = {0};
			survive_imu_tracker_predict(&d->tracker, lightData->hdr.timecode, &predicted);

			// SV_INFO("MPFIT VAR %f", var[0]);

			SurviveVelocity vel = survive_imu_velocity(&d->tracker);
			PoserData_poser_pose_func_with_velocity(&lightData->hdr, so, &predicted, &vel);
		} else {
			PoserData_poser_pose_func(&lightData->hdr, so, estimate);
		}
	}
}

static void async_optimizer_cb(struct survive_async_optimizer_buffer *buffer, int res,
							   struct mp_result_struct *result) {
	SurvivePose out = {0};
	struct async_optimizer_user *user = buffer->user;

	survive_get_ctx_lock(user->d->opt.so->ctx);
	FLT error = handle_optimizer_results(&buffer->optimizer, res, result, user, &out);
	handle_results(user->d, &user->pdl, error, &out);
	survive_release_ctx_lock(user->d->opt.so->ctx);
}

typedef void (*handle_results_fn)(MPFITData *d, PoserDataLight *lightData, FLT error, SurvivePose *estimate);
static void run_mpfit_find_3d_structure_async(MPFITData *d, PoserDataLight *pdl, SurviveSensorActivations *scene,
											  SurvivePose *out) {
	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	survive_async_optimizer_buffer *opt_buff = survive_async_optimizer_alloc_optimizer(d->async_optimizer);

	opt_buff->optimizer.reprojectModel =
		ctx->lh_version == 0 ? &survive_reproject_model : &survive_reproject_gen2_model;
	opt_buff->optimizer.so = so;
	opt_buff->optimizer.poseLength = 1;
	opt_buff->optimizer.cameraLength = so->ctx->activeLighthouses;

	SURVIVE_OPTIMIZER_SETUP_HEAP_BUFFERS(opt_buff->optimizer);

	struct async_optimizer_user *user_data = opt_buff->user;
	if (user_data == 0) {
		user_data = opt_buff->user = SV_NEW(struct async_optimizer_user);
	}

	user_data->d = d;
	user_data->pdl = *pdl;

	int setup_results = setup_optimizer(opt_buff->user, &opt_buff->optimizer, scene);
	if (setup_results < 0) {
		handle_results(d, pdl, -1, out);
		return;
	}

	survive_async_optimizer_run(d->async_optimizer, opt_buff);
	/*
	 */
}

static FLT run_mpfit_find_3d_structure(MPFITData *d, PoserDataLight *pdl, SurviveSensorActivations *scene,
									   SurvivePose *out) {
	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	survive_optimizer mpfitctx = {
		.reprojectModel = ctx->lh_version == 0 ? &survive_reproject_model : &survive_reproject_gen2_model,
		.so = so,
		//.current_bias = 0.01,
		.poseLength = 1,
		.cameraLength = so->ctx->activeLighthouses,
	};

	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx);

	struct async_optimizer_user user_data = {.d = d, .pdl = *pdl};

	int setup_results = setup_optimizer(&user_data, &mpfitctx, scene);
	if (setup_results < 0) {
		return setup_results;
	}

	mp_result result = {0};
	int res = survive_optimizer_run(&mpfitctx, &result);
	return handle_optimizer_results(&mpfitctx, res, &result, &user_data, out);
}

static FLT run_mpfit_find_cameras(MPFITData *d, PoserDataFullScene *pdfs) {
	SurviveObject *so = d->opt.so;

	survive_optimizer mpfitctx = {.so = so,
								  .poseLength = 1,
								  .cameraLength = so->ctx->activeLighthouses,
								  .reprojectModel =
									  so->ctx->lh_version ? &survive_reproject_gen2_model : &survive_reproject_model};

	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx);

	survive_optimizer_setup_cameras(&mpfitctx, so->ctx, false, d->use_jacobian_function_lh);
	survive_optimizer_setup_pose(&mpfitctx, 0, true, false);

	mpfitctx.cfg = survive_optimizer_precise_config();

	SurviveSensorActivations activations;
	PoserDataFullScene2Activations(pdfs, &activations);
	activations.lh_gen = so->ctx->lh_version;
	activations.last_imu = so->timebase_hz * 2;

	size_t meas_for_lhs[NUM_GEN2_LIGHTHOUSES] = {0};
	size_t meas_size = construct_input_from_scene(d, 0, &activations, meas_for_lhs, mpfitctx.measurements);

	if (mpfitctx.current_bias > 0) {
		meas_size += 7;
	}
	mpfitctx.measurementsCnt = meas_size;

	if (find_cameras_invalid_starting_condition(d, meas_size)) {
		return -1;
	}

	SurvivePose *cameras = survive_optimizer_get_camera(&mpfitctx);

	{
		SurviveContext *ctx = so->ctx;
		if (d->opt.seed_poser) {
			PoserData hdr = pdfs->hdr;
			memset(&pdfs->hdr, 0, sizeof(pdfs->hdr)); // Clear callback functions
			pdfs->hdr.pt = hdr.pt;
			pdfs->hdr.lighthouseposeproc = mpfit_set_cameras;
			pdfs->hdr.userdata = &mpfitctx;
			so->PoserFnData = d->opt.seed_poser_data;
			d->opt.seed_poser(so, &pdfs->hdr);
			d->opt.seed_poser_data = so->PoserFnData;
			so->PoserFnData = d;
			pdfs->hdr = hdr;
		} else {
			SV_INFO("Not using a seed poser for MPFIT; results will likely be way off");
			for (int i = 0; i < so->ctx->activeLighthouses; i++) {
				so->ctx->bsd[i].Pose = (SurvivePose){0};
				so->ctx->bsd[i].Pose.Rot[0] = 1.;
			}
		}
	}

	for (int i = 0; i < so->ctx->activeLighthouses; i++) {
		if (meas_for_lhs[i] && quatiszero(cameras[i].Rot)) {
			SurviveContext *ctx = so->ctx;
			SV_WARN("Seed poser did not solve some lighthouses, bailing.");
			return -1;
		}
	}

	mp_result result = {0};

	mpfitctx.initialPose.Rot[0] = 1;

	serialize_mpfit(d, &mpfitctx);
	int res = survive_optimizer_run(&mpfitctx, &result);

	double rtn = -1;
	bool status_failure = res <= 0;
	SurviveContext *ctx = so->ctx;

	if (!status_failure) {
		general_optimizer_data_record_success(&d->opt, result.bestnorm);
		rtn = result.bestnorm;

		SurvivePose lh2worlds[NUM_GEN2_LIGHTHOUSES] = { 0 };
		for (int i = 0; i < so->ctx->activeLighthouses; i++) {
			if (quatmagnitude(cameras[i].Rot) != 0) {
				quatnormalize(cameras[i].Rot, cameras[i].Rot);
				lh2worlds[i] = InvertPoseRtn(&cameras[i]);
				SV_INFO("Solved for %d with error of %f/%f", i, result.orignorm, result.bestnorm);
			}
		}

		PoserData_lighthouse_poses_func(&pdfs->hdr, so, lh2worlds, so->ctx->activeLighthouses, 0);
		SV_INFO("MPFIT success %f %d", result.bestnorm, res);
	} else {
		SV_INFO("MPFIT failure %f %d", result.bestnorm, res);
		// general_optimizer_data_record_failure(&d->opt);
	}

	return rtn;
}

static inline void print_stats(SurviveContext *ctx, MPFITStats *stats) {
	if (stats->total_iterations == 0)
		return;

	SV_INFO("\tmeas failures     %d", stats->meas_failures);
	SV_INFO("\ttotal iterations  %d", stats->total_iterations);
	SV_INFO("\tavg iterations    %f", (double)stats->total_iterations / stats->total_runs);
	SV_INFO("\ttotal fevals      %d", stats->total_fev);
	SV_INFO("\tavg fevals        %f", (double)stats->total_fev / stats->total_runs);
	SV_INFO("\ttotal runs        %d", stats->total_runs);
	SV_INFO("\tavg error         %10.10f", stats->sum_errors / stats->total_runs);
	SV_INFO("\tavg orig error    %10.10f", stats->sum_origerrors / stats->total_runs);
	for (int i = 0; i < sizeof(stats->status_cnts) / sizeof(int); i++) {
		SV_INFO("\tStatus %10s %d", survive_optimizer_error(i + 1), stats->status_cnts[i]);
	}
}

int PoserMPFIT(SurviveObject *so, PoserData *pd) {
	SurviveContext *ctx = so->ctx;
	if (so->PoserFnData == 0) {
		so->PoserFnData = SV_CALLOC(1, sizeof(MPFITData));
		g.instances++;
		MPFITData *d = so->PoserFnData;

		general_optimizer_data_init(&d->opt, so);
		survive_imu_tracker_init(&d->tracker, so);

		d->useIMU = (bool)survive_configi(ctx, "use-imu", SC_GET, 1);
		d->useKalman = (bool)survive_configi(ctx, "use-kalman", SC_GET, 1);
		d->required_meas = survive_configi(ctx, "required-meas", SC_GET, 8);
		d->syncs_per_run = survive_configi(ctx, "syncs-per-run", SC_GET, 1);
		d->run_async = survive_configi(ctx, RUN_POSER_ASYNC_TAG, SC_GET, 0);
		if (d->run_async) {
			d->async_optimizer = survive_async_init(async_optimizer_cb);
		}
		d->sensor_time_window = survive_configi(ctx, "time-window", SC_GET, SurviveSensorActivations_default_tolerance);
		d->use_jacobian_function_obj = survive_configi(ctx, "use-jacobian-function", SC_GET, 1);
		d->use_jacobian_function_lh = survive_configi(ctx, "use-jacobian-function-lh", SC_GET, 1);
		if (d->use_jacobian_function_lh * d->use_jacobian_function_obj < 0) {
			d->use_jacobian_function_lh = d->use_jacobian_function_obj = -1;
			SV_WARN("MPFit doesn't properly support having some analytical parameters and debug parameters. Setting "
					"all to debug.");
		}
		d->serialize_prefix = survive_configs(ctx, "serialize-lh-mpfit", SC_GET, 0);
		survive_attach_configi(ctx, "disable-lighthouse", &d->disable_lighthouse);
		survive_attach_configf(ctx, "sensor-variance-per-sec", &d->sensor_variance_per_second);
		survive_attach_configf(ctx, "sensor-variance", &d->sensor_variance);

#ifdef DEBUG_NAN
		feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

		SV_VERBOSE(110, "Initializing MPFIT:");
		SV_VERBOSE(110, "\trequired-meas: %d", d->required_meas);
		SV_VERBOSE(110, "\ttime-window: %d", d->sensor_time_window);
		SV_VERBOSE(110, "\tsensor-variance: %f", d->sensor_variance);
		SV_VERBOSE(110, "\tsensor-variance-per-sec: %f", d->sensor_variance_per_second);
		SV_VERBOSE(110, "\tuse-imu: %d", d->useIMU);
		SV_VERBOSE(110, "\tuse-kalman: %d", d->useKalman);
		SV_VERBOSE(110, "\tuse-jacobian-function: %d", d->use_jacobian_function_obj);
	}
	MPFITData *d = so->PoserFnData;
	switch (pd->pt) {
	case POSERDATA_FULL_SCENE: {
		SurviveContext *ctx = so->ctx;
		PoserDataFullScene *pdfs = (PoserDataFullScene *)(pd);
		double error = run_mpfit_find_cameras(d, pdfs);
		// std::cerr << "Average reproj error: " << error << std::endl;
		return 0;
	}
	case POSERDATA_SYNC_GEN2:
	case POSERDATA_SYNC: {
		// No poses if calibration is ongoing
		if (ctx->calptr && ctx->calptr->stage < 5)
			return 0;
		SurviveSensorActivations *scene = &so->activations;
		PoserDataLight *lightData = (PoserDataLight *)pd;
		SurvivePose estimate = {0};

		FLT error = -1;
		if (++d->syncs_per_run_cnt >= d->syncs_per_run) {
			d->syncs_per_run_cnt = 0;

			if (d->run_async) {
				run_mpfit_find_3d_structure_async(d, lightData, scene, &estimate);
			} else {
				error = run_mpfit_find_3d_structure(d, lightData, scene, &estimate);
				handle_results(d, lightData, error, &estimate);
			}
		}
		return 0;
	}

	case POSERDATA_DISASSOCIATE: {
		SV_INFO("MPFIT stats for %s:", so->codename);
		if (ctx->log_level > 5) {
			print_stats(ctx, &d->stats);
		}

		g.stats.total_fev += d->stats.total_fev;
		g.stats.total_runs += d->stats.total_runs;
		g.stats.sum_errors += d->stats.sum_errors;
		g.stats.meas_failures += d->stats.meas_failures;
		g.stats.total_iterations += d->stats.total_iterations;
		g.stats.sum_origerrors += d->stats.sum_origerrors;
		for (int i = 0; i < sizeof(d->stats.status_cnts) / sizeof(int); i++) {
			g.stats.status_cnts[i] += d->stats.status_cnts[i];
		}

		g.instances--;
		if (ctx->log_level >= 1) {
			if (g.instances == 0) {
				SV_INFO("MPFIT overall stats:");
				print_stats(ctx, &g.stats);
			}
		}
		general_optimizer_data_dtor(&d->opt);
		survive_imu_tracker_free(&d->tracker);
		survive_detach_config(ctx, "disable-lighthouse", &d->disable_lighthouse);
		survive_detach_config(ctx, "sensor-variance-per-sec", &d->sensor_variance_per_second);
		survive_detach_config(ctx, "sensor-variance", &d->sensor_variance);

		free(d);
		so->PoserFnData = 0;
		return 0;
	}
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;
		if (ctx->calptr && ctx->calptr->stage < 5) {
		} else if (d->useIMU) {
			survive_imu_tracker_integrate_imu(&d->tracker, imu);
			// SV_INFO("diff?%8u", imu->timecode);
			SurvivePose out = { 0 };
			survive_imu_tracker_predict(&d->tracker, imu->hdr.timecode, &out);
			if (!quatiszero(out.Rot)) {
				SurviveVelocity vel = survive_imu_velocity(&d->tracker);
				PoserData_poser_pose_func_with_velocity(pd, so, &out, &vel);
			}
			// SV_INFO("%+.07f %+.07f %+.07f", imu->gyro[0], imu->gyro[1], imu->gyro[2]);
		} else if (d->useKalman) {
			SurvivePose out = { 0 };
			survive_imu_tracker_predict(&d->tracker, imu->hdr.timecode, &out);
			if (!quatiszero(out.Rot)) {
				SurviveVelocity vel = survive_imu_velocity(&d->tracker);
				PoserData_poser_pose_func_with_velocity(pd, so, &out, &vel);
			}
		}

		general_optimizer_data_record_imu(&d->opt, imu);
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserMPFIT);
