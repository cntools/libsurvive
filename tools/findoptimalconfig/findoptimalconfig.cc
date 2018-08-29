#include <assert.h>
#include <iostream>
#include <libsurvive/survive.h>
#include <libsurvive/survive_reproject.h>
#include <map>
#include <math.h>
#include <memory>
#include <set>
#include <vector>

#include <sba/sba.h>

struct SBAData {
	int last_acode = -1;
	int last_lh = -1;

	int failures_to_reset = 1;
	int failures_to_reset_cntr = 0;
	int successes_to_reset = 1;
	int successes_to_reset_cntr = 0;

	FLT sensor_variance = 1.;
	FLT sensor_variance_per_second = 0;
	int sensor_time_window = SurviveSensorActivations_default_tolerance;

	int required_meas = 8;
};

struct PlaybackDataInput {
	SurviveObject *so = nullptr;
	SurvivePose position;
	uint32_t timestamp;
	std::vector<char> vmask;
	std::vector<double> meas, cov;
	SurviveSensorActivations activations;
	PlaybackDataInput(SurviveObject *so, const SurvivePose &position)
		: so(so), position(position), activations(so->activations) {
		int32_t sensor_count = so->sensor_ct;
		vmask.resize(sensor_count * NUM_LIGHTHOUSES);
		cov.resize(4 * sensor_count * NUM_LIGHTHOUSES);
		meas.resize(2 * sensor_count * NUM_LIGHTHOUSES);
	}
	void shrink(size_t new_size) {
		cov.resize(4 * new_size);
		meas.resize(2 * new_size);
	}
	~PlaybackDataInput() {}
};

struct PlaybackData {
	SurviveObject *so = nullptr;
	BaseStationData bsd[2];
	std::vector<PlaybackDataInput> inputs;
};

SBAData settings;
static size_t construct_input_from_scene(SurviveObject *so, uint32_t timestamp, char *vmask, double *meas,
										 double *cov) {
	size_t rtn = 0;
	auto scene = &so->activations;
	for (size_t sensor = 0; sensor < so->sensor_ct; sensor++) {
		for (size_t lh = 0; lh < 2; lh++) {
			if (SurviveSensorActivations_isPairValid(scene, settings.sensor_time_window, timestamp, sensor, lh)) {
				double *a = scene->angles[sensor][lh];
				vmask[sensor * NUM_LIGHTHOUSES + lh] = 1;

				if (cov) {
					*(cov++) = settings.sensor_variance +
							   std::abs((double)timestamp - scene->timecode[sensor][lh][0]) *
								   settings.sensor_variance_per_second / (double)so->timebase_hz;
					*(cov++) = 0;
					*(cov++) = 0;
					*(cov++) = settings.sensor_variance +
							   std::abs((double)timestamp - scene->timecode[sensor][lh][1]) *
								   settings.sensor_variance_per_second / (double)so->timebase_hz;
				}
				meas[rtn++] = a[0];
				meas[rtn++] = a[1];
			} else {
				vmask[sensor * NUM_LIGHTHOUSES + lh] = 0;
			}
		}
	}
	return rtn;
}
uint32_t timestamp;

void light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length,
				   uint32_t lighthouse) {
	timestamp = timecode;
	survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);
}

SurvivePose lastPose = {};
void pose_process(SurviveObject *so, uint32_t timecode, SurvivePose *pose) {
	survive_default_raw_pose_process(so, timecode, pose);
	PlaybackData *d = (PlaybackData *)so->ctx->user_ptr;
	d->so = so;
	d->inputs.emplace_back(so, *pose);
	auto &input = d->inputs.back();
	input.timestamp = timestamp;
	int meas = construct_input_from_scene(so, timestamp, &input.vmask.front(), &input.meas.front(), &input.cov.front());
	input.shrink(meas / 2);

	double dist = 0;
	if (d->inputs.empty() == false) {
		dist = dist3d(pose->Pos, lastPose.Pos);
	}
	if (meas / 2 < 8 || dist > .00009)
		d->inputs.pop_back();
	lastPose = *pose;
}

void lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *pose, SurvivePose *obj_pose) {
	survive_default_lighthouse_pose_process(ctx, lighthouse, pose, obj_pose);
	PlaybackData *d = (PlaybackData *)ctx->user_ptr;
	d->bsd[lighthouse] = ctx->bsd[lighthouse];
}

std::map<size_t, std::map<size_t, double>> errors;

typedef struct {
	survive_calibration_config calibration_config;
	SurviveObject *so;
	SurvivePose obj_pose;
} sba_context;

static void str_metric_function(int j, int i, double *bi, double *xij, void *adata) {
	SurvivePose obj = *(SurvivePose *)bi;
	int sensor_idx = j >> 1;
	int lh = j & 1;

	sba_context *ctx = (sba_context *)(adata);
	SurviveObject *so = ctx->so;

	assert(lh < 2);
	assert(sensor_idx < so->sensor_ct);

	quatnormalize(obj.Rot, obj.Rot);
	FLT xyz[3];
	ApplyPoseToPoint(xyz, &obj, &so->sensor_locations[sensor_idx * 3]);

	// std::cerr << "Processing " << sensor_idx << ", " << lh << std::endl;
	SurvivePose *camera = &so->ctx->bsd[lh].Pose;
	survive_reproject_from_pose_with_config(so->ctx, &ctx->calibration_config, lh, camera, xyz, xij);
}

double sba_opt(SurviveContext *ctx, const survive_calibration_config &config, PlaybackDataInput &data) {
	double *covx = 0;
	SurviveObject *so = data.so;

	SurvivePose soLocation = data.position;

	double opts[SBA_OPTSSZ] = {0};
	double info[SBA_INFOSZ] = {0};

	sba_context _ctx = {config, so};

	opts[0] = SBA_INIT_MU;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH; // max_reproj_error * meas.size();
	opts[4] = 0.0;

	int status = sba_str_levmar(1, // Number of 3d points
								0, // Number of 3d points to fix in spot
								NUM_LIGHTHOUSES * so->sensor_ct, &data.vmask.front(),
								soLocation.Pos,		// Reads as the full pose though
								7,					// pnp -- SurvivePose
								&data.meas.front(), // x* -- measurement data
								&data.cov.front(),  // cov data
								2,					// mnp -- 2 points per image
								str_metric_function,
								0,	 // jacobia of metric_func
								&_ctx, // user data
								100,   // Max iterations
								0,	 // verbosity
								opts,  // options
								info); // info

	int meas_size = data.meas.size() / 2;
	if (meas_size == 0)
		return 0;

	{
		SurviveContext *ctx = so->ctx;
		// Docs say info[0] should be divided by meas; I don't buy it really...
		static int cnt = 0;
		if (cnt++ > 1000) {
			SV_INFO("%f original reproj error for %u meas", (info[0] / meas_size * 2), (int)meas_size);
			SV_INFO("%f cur reproj error", (info[1] / meas_size * 2));
			cnt = 0;
		}
	}
	assert(!isinf(info[1]));
	return info[1] / meas_size * 2;
}

struct optimal_cal_ctx {
	std::vector<double> sensors;
	std::vector<int> lighthouses;
	SurviveContext *ctx;
};

static void metric_function(int j, int i, double *aj, double *xij, void *adata) {
	optimal_cal_ctx *ctx = (optimal_cal_ctx *)(adata);

	FLT sensorInWorld[3] = {ctx->sensors[i * 3 + 0], ctx->sensors[i * 3 + 1], ctx->sensors[i * 3 + 2]};
	int lh = ctx->lighthouses[i];
	BaseStationData bsd = ctx->ctx->bsd[lh];
	survive_calibration_config cfg = *(survive_calibration_config *)aj;

	survive_reproject_from_pose_with_bcal(&bsd, &cfg, &ctx->ctx->bsd[lh].Pose, sensorInWorld, xij);
}

double find_optimal_cal(SurviveContext *ctx, PlaybackData &data) {
	optimal_cal_ctx _ctx;
	std::vector<char> vmask;
	std::vector<double> cov, meas;
	_ctx.ctx = ctx;
	for (auto &in : data.inputs) {
		for (size_t sensor = 0; sensor < in.so->sensor_ct; sensor++) {
			FLT p[3];
			ApplyPoseToPoint(p, &in.position, &data.so->sensor_locations[sensor * 3]);
			for (size_t lh = 0; lh < 2; lh++) {
				_ctx.sensors.emplace_back(p[0]);
				_ctx.sensors.emplace_back(p[1]);
				_ctx.sensors.emplace_back(p[2]);
				_ctx.lighthouses.emplace_back(lh);

				auto scene = &in.activations;
				if (SurviveSensorActivations_isPairValid(scene, settings.sensor_time_window, in.timestamp, sensor,
														 lh)) {
					double *a = scene->angles[sensor][lh];
					vmask.emplace_back(1); //[sensor * NUM_LIGHTHOUSES + lh] = 1;

					meas.emplace_back(a[0]);
					meas.emplace_back(a[1]);
				} else {
					vmask.emplace_back(0);
				}
			}
		}
	}

	double *covx = 0;
	SurviveObject *so = data.so;

	double opts[SBA_OPTSSZ] = {0};
	double info[SBA_INFOSZ] = {0};

	survive_calibration_config config = {0};
	config.use_flag = SVCal_All;

	opts[0] = SBA_INIT_MU;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH; // max_reproj_error * meas.size();
	opts[4] = 0.0;

	int status = sba_mot_levmar(data.inputs.size() * so->sensor_ct * NUM_LIGHTHOUSES, // number of 3d points
								1,				   // Number of cameras -- 2 lighthouses
								0,				   // Number of cameras to not modify
								&vmask[0],		   // boolean vis mask
								(double *)&config, // camera parameters
								4,				   // sizeof(BaseStationCal) / sizeof(FLT),
								&meas[0],		   // 2d points for 3d objs
								covx,			   // covariance of measurement. Null sets to identity
								2,				   // 2 points per image
								metric_function,
								0,	 // jacobia of metric_func
								&_ctx, // user data
								50,	// Max iterations
								0,	 // verbosity
								opts,  // options
								info); // info

	if (status > 0) {
	} else {
		assert(false);
	}
	int meas_size = _ctx.sensors.size() / 2;
	if (meas_size == 0)
		return 0;

	{
		SurviveContext *ctx = so->ctx;
		// Docs say info[0] should be divided by meas; I don't buy it really...
		static int cnt = 0;
		if (cnt++ > 1000) {
			SV_INFO("%f original reproj error for %u meas", (info[0] / meas_size * 2), (int)meas_size);
			SV_INFO("%f cur reproj error", (info[1] / meas_size * 2));
			cnt = 0;
		}
	}
	assert(!isinf(info[1]));
	std::cerr << "Used " << meas_size << " measurements" << std::endl;

	return info[1] / meas_size * 2;
}
double find_avg_reproj_error(SurviveContext *ctx, const survive_calibration_config &config, PlaybackDataInput &data) {
	return sba_opt(ctx, config, data);
	/*
	for (size_t sensor = 0; sensor < data.so->sensor_ct; sensor++) {
	  for (size_t lh = 0; lh < 2; lh++) {
		if( *(vmask++) ) {
	  cnt++;
	  FLT pt[3];
	  ApplyPoseToPoint(pt, &data.position, data.so->sensor_locations + sensor * 3);

	  FLT reproj_meas[2];
	  survive_reproject_from_pose_with_config(ctx, &config, lh, &ctx->bsd[lh].Pose, pt, reproj_meas);

	  auto x = reproj_meas[0] - meas[0];
	  auto y = reproj_meas[1] - meas[1];
	  err += cov[0]*x*x + cov[2]*y*y;

	  meas += 2;
	  cov += 4;
		}
	  }
	  }*/
}

double find_avg_reproj_error(SurviveContext *ctx, const survive_calibration_config &config, PlaybackData &data) {
	double err = 0;
	for (auto &in : data.inputs) {
		err += find_avg_reproj_error(ctx, config, in);
	}
	return err / data.inputs.size();
}

int main(int argc, char **argv) {
	std::vector<std::pair<size_t, size_t>> sections = {
		{5, 0}, // phase
				//{ 5, 5 },  // tilt
				//{ 5, 10 }, // curve
				//{ 11, 15 } // gibs + useSin
	};

	for (int i = 1; i < argc; i++) {
		PlaybackData data;

		char const *args[] = {argv[0],
							  "--use-bsd-cal",
							  "0",
							  "--calibrate",
							  "--playback-factor",
							  "0",
							  "--disambiguator",
							  "StateBased",
							  "--defaultposer",
							  "SBA",
							  "--sba-required-meas",
							  "8",
							  "--sba-max-error",
							  ".1",
							  "--playback",
							  argv[i]};

		auto ctx = survive_init(sizeof(args) / sizeof(args[0]), (char *const *)args);
		ctx->user_ptr = &data;

		survive_install_pose_fn(ctx, pose_process);
		survive_install_lighthouse_pose_fn(ctx, lighthouse_process);
		survive_install_light_fn(ctx, light_process);

		while (survive_poll(ctx) == 0) {
		}

		find_optimal_cal(ctx, data);

		survive_close(ctx);
	}

	return 0;
}
