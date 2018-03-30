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
std::ostream &operator<<(std::ostream &o, const survive_calibration_options_config &self) {
	o << "\t";
	if (!self.enable[0] && !self.enable[1]) {
		o << "disabled";
		return o;
	}

	o << "swap: " << self.swap << std::endl;
	for (int i = 0; i < 2; i++) {
		if (self.enable[i]) {
			o << "\tinvert[" << i << "]: " << self.invert[i];
		} else {
			o << "\t" << i << ": disabled";
		}
	}
	return o;
}

std::ostream &operator<<(std::ostream &o, const survive_calibration_config &self) {
	o << "Index: " << survive_calibration_config_index(&self) << std::endl;
	o << "Phase: " << std::endl << self.phase << std::endl;
	o << "Tilt: " << std::endl << self.tilt << std::endl;
	o << "Curve: " << std::endl << self.curve << std::endl;
	o << "gibPhase: " << std::endl << self.gibPhase << std::endl;
	o << "gibMag: " << std::endl << self.gibMag << std::endl;
	o << "gibUseSin: " << self.gibUseSin << std::endl;
	return o;
}

struct SBAData {
	int last_acode = -1;
	int last_lh = -1;

	int failures_to_reset = 1;
	int failures_to_reset_cntr = 0;
	int successes_to_reset = 1;
	int successes_to_reset_cntr = 0;

	FLT sensor_variance = 1.;
	FLT sensor_variance_per_second = 0;
	int sensor_time_window = 1600000;

	int required_meas = 8;
};

struct PlaybackDataInput {
	SurviveObject *so = nullptr;
	SurvivePose position;

	std::vector<char> vmask;
	std::vector<double> meas, cov;
	PlaybackDataInput(SurviveObject *so, const SurvivePose &position) : so(so), position(position) {
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

void raw_pose_process(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	survive_default_raw_pose_process(so, lighthouse, pose);
	PlaybackData *d = (PlaybackData *)so->ctx->user_ptr;
	d->inputs.emplace_back(so, *pose);
	auto &input = d->inputs.back();
	int meas = construct_input_from_scene(so, timestamp, &input.vmask.front(), &input.meas.front(), &input.cov.front());
	input.shrink(meas / 2);
	if (meas / 2 < 12)
		d->inputs.pop_back();
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
	bool currentPositionValid = quatmagnitude(&soLocation.Rot[0]) != 0;

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
								50,	// Max iterations
								0,	 // verbosity
								opts,  // options
								info); // info

	if (status > 0) {
	} else {
		assert(false);
	}
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

double find_avg_reproj_error(SurviveContext *ctx, const survive_calibration_config &config, PlaybackDataInput &data) {
	auto vmask = &data.vmask.front();
	auto cov = &data.cov.front();
	auto meas = &data.meas.front();
	double err = 0;
	size_t cnt = 0;

	err += sba_opt(ctx, config, data);
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
	return err;
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
		{28, 0}, // phase
				 //    { 5, 5 },  // tilt
				 //    { 5, 10 }, // curve
				 //    { 11, 15 } // gibs + useSin
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
							  "12",
							  "--playback",
							  argv[i]};

		auto ctx = survive_init(sizeof(args) / sizeof(args[0]), (char *const *)args);
		ctx->user_ptr = &data;

		survive_install_raw_pose_fn(ctx, raw_pose_process);
		survive_install_lighthouse_pose_fn(ctx, lighthouse_process);
		survive_install_light_fn(ctx, light_process);

		while (survive_poll(ctx) == 0) {
		}

		for (int j = 0; j < sections.size(); j++) {
			auto &range = sections[j];
			for (size_t _i = 0; _i < (1 << range.first); _i++) {
				int i = _i << range.second;
				survive_calibration_config config = survive_calibration_config_create_from_idx(i);
				if (i == survive_calibration_config_index(&config)) {
					double error = find_avg_reproj_error(ctx, config, data);
					errors[j][i] += error;
				}
			}
			std::cerr << "Finished grouping " << j << std::endl;
		}

		survive_close(ctx);
	}

	for (int i = 0; i < errors.size(); i++) {
		std::cout << "Grouping " << i << std::endl;
		auto compFunctor = [](std::pair<size_t, double> elem1, std::pair<size_t, double> elem2) {
			if (elem1.second == elem2.second)
				return elem1.first < elem2.first;
			return elem1.second < elem2.second;
		};

		std::set<std::pair<size_t, double>, typeof(compFunctor)> set(errors[i].begin(), errors[i].end(), compFunctor);

		for (auto err : set) {
			survive_calibration_config config = survive_calibration_config_create_from_idx(err.first);
			if (err.first == survive_calibration_config_index(&config)) {
				double error = err.second;
				std::cout << "Config " << err.first << " " << error << std::endl;
				std::cout << config << std::endl;
			}
		}
	}
	return 0;
}
