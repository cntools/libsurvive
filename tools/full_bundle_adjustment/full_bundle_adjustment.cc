#include <chrono>
#include <iostream>
#include <libsurvive/survive.h>
#include <libsurvive/survive_optimizer.h>
#include <libsurvive/survive_reproject.h>
#include <map>
#include <math.h>
#include <memory>
#include <mpfit/mpfit.h>
#include <set>
#include <vector>

uint32_t timestamp;
std::vector<survive_optimizer_measurement> measurements;

SurvivePose currentPose = LinmathPose_Identity;
std::vector<SurvivePose> poses;

static size_t construct_input_from_scene(SurviveObject *so, size_t timecode, survive_timecode sensor_time_window,
										 FLT sensor_variance, FLT sensor_variance_per_second) {
	size_t rtn = 0;
	auto scene = &so->activations;
	const bool force_pair = false;
	for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
		for (uint8_t lh = 0; lh < 2; lh++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				bool isReadingValue =
					SurviveSensorActivations_isReadingValid(scene, sensor_time_window, timecode, sensor, lh, axis);
				if (force_pair) {
					isReadingValue =
						SurviveSensorActivations_isPairValid(scene, sensor_time_window, timecode, sensor, lh);
				}
				if (isReadingValue) {
					const double *a = scene->angles[sensor][lh];
					measurements.push_back({});
					auto meas = &measurements.back();
					meas->axis = axis;
					meas->value = a[axis];
					meas->sensor_idx = sensor;
					meas->lh = lh;
					meas->object = poses.size();
					survive_timecode diff = survive_timecode_difference(timecode, scene->timecode[sensor][lh][axis]);
					meas->variance = sensor_variance + diff * sensor_variance_per_second / (double)so->timebase_hz;
					rtn++;
				}
			}
		}
	}
	return rtn;
}

static void capture_data(SurviveObject *so) {
	auto cnt = construct_input_from_scene(so, timestamp, SurviveSensorActivations_default_tolerance, 1, 10);
	if (cnt < 8) {
		while (!measurements.empty() && measurements.back().object == poses.size())
			measurements.pop_back();
	} else {
		poses.push_back(currentPose);
	}
}

int last_lh = -1, last_acode = -1;
survive_timecode last_timecode = -1;
void light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length,
				   uint32_t lighthouse) {
	survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);

	timestamp = timecode;
	if (last_lh != lighthouse || last_acode != acode) {
		if (last_timecode == -1 || survive_timecode_difference(timecode, last_timecode) / (FLT)so->timebase_hz > 1.) {
			capture_data(so);
			last_timecode = timecode;
		}
	}
	last_lh = lighthouse;
	last_acode = acode;
}

void raw_pose_process(SurviveObject *so, uint32_t lighthouse, SurvivePose *pose) {
	survive_default_raw_pose_process(so, lighthouse, pose);
	currentPose = *pose;
}

static double full_bundle_adjustment(SurviveObject *so) {
	struct SurviveContext *ctx = so->ctx;

	survive_optimizer mpfitctx = {};
	mpfitctx.so = so;
	mpfitctx.poseLength = poses.size();
	mpfitctx.cameraLength = so->ctx->activeLighthouses;

	std::vector<double> parameters(survive_optimizer_get_parameters_count(&mpfitctx));
	std::vector<mp_par_struct> parameter_infos(survive_optimizer_get_parameters_count(&mpfitctx));
	mpfitctx.parameters = &parameters.front();
	mpfitctx.measurements = &measurements.front();
	mpfitctx.measurementsCnt = measurements.size();
	mpfitctx.parameters_info = &parameter_infos.front();

	SurvivePose *soLocation = survive_optimizer_get_pose(&mpfitctx);
	survive_optimizer_setup_cameras(&mpfitctx, ctx, true);
	int start = survive_optimizer_get_camera_index(&mpfitctx);
	for (int i = start + 7; i < start + 7 * mpfitctx.cameraLength; i++) {
		mpfitctx.parameters_info[i].fixed = false;
	}

	int use_jacobian_function = 0;
	survive_optimizer_setup_pose(&mpfitctx, &poses.front(), false, use_jacobian_function);

	mp_result result = {0};

	mpfitctx.initialPose = LinmathPose_Identity;

	int res = survive_optimizer_run(&mpfitctx, &result);

	double rtn = -1;
	bool status_failure = res <= 0;

	if (!status_failure) {
		quatnormalize(soLocation->Rot, soLocation->Rot);
		rtn = result.bestnorm;

		SurvivePose *cameras = survive_optimizer_get_camera(&mpfitctx);
		for (int i = 0; i < mpfitctx.cameraLength; i++) {
			SurvivePose p = InvertPoseRtn(cameras + i);
			so->ctx->lighthouseposeproc(so->ctx, i, &p, soLocation);
			SV_INFO("%f %f %f", p.Pos[0], p.Pos[1], p.Pos[2]);
		}
	} else {
		SV_INFO("Optimization failure");
	}
	SV_INFO("Optimization %f/%f (%d measurements) %d", result.orignorm, result.bestnorm, (int)measurements.size(), res);

	return rtn;
}

int main(int argc, char **argv) {
	auto ctx = survive_init(argc, argv);
	if (ctx == nullptr)
		return -1;

	survive_install_light_fn(ctx, light_process);
	survive_install_pose_fn(ctx, raw_pose_process);

	while (survive_poll(ctx) == 0) {
	}

	full_bundle_adjustment(ctx->objs[0]);

	survive_close(ctx);

	return 0;
}
