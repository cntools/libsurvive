#include "survive_optimizer.h"

#include <malloc.h>

#include "mpfit/mpfit.h"
#include "poser.h"
#include <survive.h>
#include <survive_imu.h>

#include "assert.h"
#include "linmath.h"
#include "math.h"
#include "poser_general_optimizer.h"
#include "string.h"
#include "survive_cal.h"
#include "survive_config.h"
#include "survive_reproject.h"

#ifdef DEBUG_NAN
#include <fenv.h>
#endif

STATIC_CONFIG_ITEM(USE_JACOBIAN_FUNCTION, "use-jacobian-function", 'i',
				   "If set to false, a slower numerical approximation of the jacobian is used", 1);
STATIC_CONFIG_ITEM(USE_IMU, "use-imu", 'i', "Use the IMU as part of the pose solver", 1);
STATIC_CONFIG_ITEM(SENSOR_VARIANCE_PER_SEC, "sensor-variance-per-sec", 'f',
				   "Variance per second to add to the sensor input -- discounts older data", 0.0);
STATIC_CONFIG_ITEM(SENSOR_VARIANCE, "sensor-variance", 'f', "Base variance for each sensor input", 1.0);
STATIC_CONFIG_ITEM(DISABLE_LIGHTHOUSE, "disable-lighthouse", 'i', "Disable given lighthouse from tracking", -1);

typedef struct MPFITData {
	GeneralOptimizerData opt;

	int last_acode;
	int last_lh;

	int disable_lighthouse;
	int sensor_time_window;
	// > 0; use jacobian, 0 don't use, < 0 debug
	int use_jacobian_function;
	int required_meas;

	FLT sensor_variance;
	FLT sensor_variance_per_second;

	SurviveIMUTracker tracker;
	bool useIMU;

	struct {
		int meas_failures;
	} stats;
} MPFITData;

static size_t construct_input_from_scene(const MPFITData *d, size_t timecode, const SurviveSensorActivations *scene,
										 survive_optimizer_measurement *meas) {
	size_t rtn = 0;
	SurviveObject *so = d->opt.so;
	const bool force_pair = false;
	for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
		for (uint8_t lh = 0; lh < 2; lh++) {
			if (d->disable_lighthouse == lh)
				continue;

			for (uint8_t axis = 0; axis < 2; axis++) {
				bool isReadingValue =
					SurviveSensorActivations_isReadingValid(scene, d->sensor_time_window, timecode, sensor, lh, axis);
				if (force_pair) {
					isReadingValue =
						SurviveSensorActivations_isPairValid(scene, d->sensor_time_window, timecode, sensor, lh);
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
					meas++;
					rtn++;
				}
			}
		}
	}
	return rtn;
}

static bool find_cameras_invalid_starting_condition(MPFITData *d, size_t meas_size) {
	if (meas_size < d->required_meas * 2) {
		SurviveContext *ctx = d->opt.so->ctx;
		SV_INFO("Can't solve for cameras with just %u measurements", (unsigned int)meas_size);
		return true;
	}
	return false;
}

static bool invalid_starting_condition(MPFITData *d, size_t meas_size) {
	static int failure_count = 500;
	bool hasAllBSDs = true;
	struct SurviveObject *so = d->opt.so;
	for (int lh = 0; lh < so->ctx->activeLighthouses; lh++)
		hasAllBSDs &= so->ctx->bsd[lh].PositionSet;

	if (!hasAllBSDs || meas_size < d->required_meas) {
		if (hasAllBSDs && failure_count++ == 500) {
			SurviveContext *ctx = so->ctx;
			SV_INFO("Can't solve for position with just %u measurements", (unsigned int)meas_size);
			failure_count = 0;
		}
		if (meas_size < d->required_meas) {
			d->stats.meas_failures++;
		}
		return true;
	}
	failure_count = 0;
	return false;
}

static double run_mpfit_find_3d_structure(MPFITData *d, PoserDataLight *pdl, SurviveSensorActivations *scene,
										  SurvivePose *out) {
	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	survive_optimizer mpfitctx = {
		.so = so,
		//.current_bias = 0.001,
		.poseLength = 1,
		.cameraLength = so->ctx->activeLighthouses,
	};

	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx);

	SurvivePose *soLocation = survive_optimizer_get_pose(&mpfitctx);
	survive_optimizer_setup_cameras(&mpfitctx, so->ctx, true);

	bool updateCameras = false;

	if (updateCameras) {
		int start = survive_optimizer_get_camera_index(&mpfitctx);
		for (int i = start + 7; i < start + 7 * mpfitctx.cameraLength; i++) {
			mpfitctx.parameters_info[i].fixed = false;
		}
	}
	survive_optimizer_setup_pose(&mpfitctx, 0, false, d->use_jacobian_function);

	size_t meas_size = construct_input_from_scene(d, pdl->timecode, scene, mpfitctx.measurements);

	if (mpfitctx.current_bias > 0) {
		meas_size += 7;
	}
	mpfitctx.measurementsCnt = meas_size;

	if (invalid_starting_condition(d, meas_size)) {
		return -1;
	}

	if (!general_optimizer_data_record_current_pose(&d->opt, &pdl->hdr, sizeof(*pdl), soLocation)) {
		return -1;
	}

	mp_result result = {0};

	mpfitctx.initialPose = *soLocation;

	int res = survive_optimizer_run(&mpfitctx, &result);

	double rtn = -1;
	bool status_failure = res <= 0;
	bool error_failure = !general_optimizer_data_record_success(&d->opt, result.bestnorm);
	if (!status_failure && !error_failure) {
		quatnormalize(soLocation->Rot, soLocation->Rot);
		*out = *soLocation;
		rtn = result.bestnorm;

		if (updateCameras) {
			SurvivePose *cameras = survive_optimizer_get_camera(&mpfitctx);
			for (int i = 0; i < mpfitctx.cameraLength; i++) {
				SurvivePose p = InvertPoseRtn(cameras + i);
				so->ctx->lighthouseposeproc(so->ctx, i, &p, soLocation);
			}
		}
	} else {
		SV_INFO("MPFIT failure %f (%d measurements)", result.bestnorm, (int)meas_size);
	}

	return rtn;
}

static void mpfit_set_cameras(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose, SurvivePose *obj_pose,
							  void *user) {
	survive_optimizer *ctx = (survive_optimizer *)user;
	SurvivePose *cameras = survive_optimizer_get_camera(ctx);
	cameras[lighthouse] = InvertPoseRtn(pose);
	if (obj_pose)
		*survive_optimizer_get_pose(ctx) = *obj_pose;
	else
		*survive_optimizer_get_pose(ctx) = LinmathPose_Identity;
}

static double run_mpfit_find_cameras(MPFITData *d, PoserDataFullScene *pdfs) {
	SurviveObject *so = d->opt.so;

	survive_optimizer mpfitctx = {
		.so = so,
		.poseLength = 1,
		.cameraLength = so->ctx->activeLighthouses,
	};

	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx);

	survive_optimizer_setup_cameras(&mpfitctx, so->ctx, false);
	survive_optimizer_setup_pose(&mpfitctx, 0, true, false);

	SurviveSensorActivations activations;
	PoserDataFullScene2Activations(pdfs, &activations);
	size_t meas_size = construct_input_from_scene(d, 0, &activations, mpfitctx.measurements);

	if (mpfitctx.current_bias > 0) {
		meas_size += 7;
	}
	mpfitctx.measurementsCnt = meas_size;

	SurvivePose *cameras = survive_optimizer_get_camera(&mpfitctx);

	if (find_cameras_invalid_starting_condition(d, meas_size)) {
		return -1;
	}

	{
		const char *subposer = survive_configs(so->ctx, "seed-poser", SC_GET, "EPNP");

		PoserCB driver = (PoserCB)GetDriverWithPrefix("Poser", subposer);
		SurviveContext *ctx = so->ctx;
		if (driver) {
			PoserData hdr = pdfs->hdr;
			memset(&pdfs->hdr, 0, sizeof(pdfs->hdr)); // Clear callback functions
			pdfs->hdr.pt = hdr.pt;
			pdfs->hdr.lighthouseposeproc = mpfit_set_cameras;
			pdfs->hdr.userdata = &mpfitctx;
			driver(so, &pdfs->hdr);
			pdfs->hdr = hdr;
		} else {
			SV_INFO("Not using a seed poser for MPFIT; results will likely be way off");
			for (int i = 0; i < 2; i++) {
				so->ctx->bsd[i].Pose = (SurvivePose){0};
				so->ctx->bsd[i].Pose.Rot[0] = 1.;
			}
		}
	}

	mp_result result = {0};

	mpfitctx.initialPose.Rot[0] = 1;

	int res = survive_optimizer_run(&mpfitctx, &result);

	double rtn = -1;
	bool status_failure = res <= 0;
	// bool error_failure = !general_optimizer_data_record_success(&d->opt, result.bestnorm);
	if (!status_failure) {
		rtn = result.bestnorm;

		SurvivePose additionalTx = {0};
		for (int i = 0; i < so->ctx->activeLighthouses; i++) {
			if (quatmagnitude(cameras[i].Rot) != 0) {
				quatnormalize(cameras[i].Rot, cameras[i].Rot);
				SurvivePose lh2world = InvertPoseRtn(&cameras[i]);
				PoserData_lighthouse_pose_func(&pdfs->hdr, so, i, &additionalTx, &lh2world,
											   survive_optimizer_get_pose(&mpfitctx));
			}
		}

	} else {
		SurviveContext *ctx = so->ctx;
		SV_INFO("MPFIT failure %f", result.bestnorm);
	}

	return rtn;
}
int PoserMPFIT(SurviveObject *so, PoserData *pd) {
	SurviveContext *ctx = so->ctx;
	if (so->PoserData == 0) {
		so->PoserData = calloc(1, sizeof(MPFITData));
		MPFITData *d = so->PoserData;

		general_optimizer_data_init(&d->opt, so);
		d->useIMU = (bool)survive_configi(ctx, "use-imu", SC_GET, 1);
		d->required_meas = survive_configi(ctx, "required-meas", SC_GET, 8);

		d->sensor_time_window = survive_configi(ctx, "time-window", SC_GET, SurviveSensorActivations_default_tolerance);
		d->use_jacobian_function = survive_configi(ctx, "use-jacobian-function", SC_GET, 1);
		survive_attach_configi(ctx, "disable-lighthouse", &d->disable_lighthouse);
		survive_attach_configf(ctx, "sensor-variance-per-sec", &d->sensor_variance_per_second);
		survive_attach_configf(ctx, "sensor-variance", &d->sensor_variance);

#ifdef DEBUG_NAN
		feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif

		SV_INFO("Initializing MPFIT:");
		SV_INFO("\trequired-meas: %d", d->required_meas);
		SV_INFO("\ttime-window: %d", d->sensor_time_window);
		SV_INFO("\tsensor-variance: %f", d->sensor_variance);
		SV_INFO("\tsensor-variance-per-sec: %f", d->sensor_variance_per_second);
		SV_INFO("\tuse-imu: %d", d->useIMU);
		SV_INFO("\tuse-jacobian-function: %d", d->use_jacobian_function);
	}
	MPFITData *d = so->PoserData;
	switch (pd->pt) {
	case POSERDATA_FULL_SCENE: {
		SurviveContext *ctx = so->ctx;
		PoserDataFullScene *pdfs = (PoserDataFullScene *)(pd);
		double error = run_mpfit_find_cameras(d, pdfs);
		// std::cerr << "Average reproj error: " << error << std::endl;
		return 0;
	}
	case POSERDATA_LIGHT: {
		// No poses if calibration is ongoing
		if (ctx->calptr && ctx->calptr->stage < 5)
			return 0;
		SurviveSensorActivations *scene = &so->activations;
		PoserDataLight *lightData = (PoserDataLight *)pd;
		SurvivePose estimate;

		// only process sweeps
		FLT error = -1;
		if (d->last_lh != lightData->lh || d->last_acode != lightData->acode) {
			error = run_mpfit_find_3d_structure(d, lightData, scene, &estimate);

			d->last_lh = lightData->lh;
			d->last_acode = lightData->acode;

			if (error > 0) {
				if (d->useIMU) {
					FLT var_meters = 0.5;
					FLT var_quat = error + .05;
					FLT var[7] = {error * var_meters, error * var_meters, error * var_meters, error * var_quat,
								  error * var_quat,   error * var_quat,   error * var_quat};

					survive_imu_tracker_integrate_observation(so, lightData->timecode, &d->tracker, &estimate, var);
					estimate = d->tracker.pose;
				}

				PoserData_poser_pose_func(&lightData->hdr, so, &estimate);
			}
		}
		return 0;
	}

	case POSERDATA_DISASSOCIATE: {
		SV_INFO("MPFIT stats:");
		SV_INFO("\tmeas failures %d", d->stats.meas_failures);
		general_optimizer_data_dtor(&d->opt);
		free(d);
		so->PoserData = 0;
		return 0;
	}
	case POSERDATA_IMU: {

		PoserDataIMU *imu = (PoserDataIMU *)pd;
		if (ctx->calptr && ctx->calptr->stage < 5) {
		} else if (d->useIMU) {
			survive_imu_tracker_integrate(so, &d->tracker, imu);
			PoserData_poser_pose_func(pd, so, &d->tracker.pose);
		}

		general_optimizer_data_record_imu(&d->opt, imu);
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserMPFIT);
