#include "survive.h"
#include "survive_playback.h"
#include <assert.h>

static FLT freq_per_channel[NUM_GEN2_LIGHTHOUSES] = {
	50.0521, 50.1567, 50.3673, 50.5796, 50.6864, 50.9014, 51.0096, 51.1182,
	51.2273, 51.6685, 52.2307, 52.6894, 52.9217, 53.2741, 53.7514, 54.1150,
};

SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode,
												 bool ootx, bool gen) {
	struct SurviveContext *ctx = so->ctx;

	assert(channel <= NUM_GEN2_LIGHTHOUSES);
	so->last_sync_time[channel] = timecode;
	SV_INFO("Sync   ch%2d  %d %d %12x", channel, ootx, gen, timecode);
}
SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
												  survive_timecode timecode, bool flag) {
	struct SurviveContext *ctx = so->ctx;

	if (ctx->calptr) {
		// survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length, lh);
	}

	survive_recording_sweep_process(so, sensor_id, timecode, timecode, channel);

	survive_timecode last_sweep = so->last_sync_time[channel];
	assert(channel <= NUM_GEN2_LIGHTHOUSES);

	SV_INFO("Sensor ch%2d %2d %d %12x %6d", channel, sensor_id, flag, timecode, timecode - so->last_sync_time[channel]);

	FLT time_since_sync = (survive_timecode_difference(timecode, last_sweep) / 48000000.);
	FLT time_per_rot = 1. / freq_per_channel[channel];

	FLT angle = time_since_sync / time_per_rot * 2. * LINMATHPI;

	so->ctx->sweep_angleproc(so, channel, sensor_id, timecode, angle);
}

SURVIVE_EXPORT void survive_default_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
														survive_timecode timecode, FLT angle) {
	struct SurviveContext *ctx = so->ctx;
	SV_INFO("Sensor ch%2d %2d %12f", channel, sensor_id, angle);

	PoserDataLightGen2 l = {
		.hdr =
			{
				.pt = POSERDATA_LIGHT_GEN2,
			},
		.sensor_id = sensor_id,
		.timecode = timecode,
		.angle = angle,
		.lh = channel,
	};

	// Simulate the use of only one lighthouse in playback mode.
	if (channel < ctx->activeLighthouses)
		SurviveSensorActivations_add_gen2(&so->activations, &l);

	survive_recording_sweep_angle_process(so, sensor_id, timecode, angle, channel);

	if (ctx->calptr) {
		// survive_cal_angle(so, sensor_id, acode, timecode, length, angle, lh);
	}
	if (so->PoserFn) {
		so->PoserFn(so, (PoserData *)&l);
	}
}

SURVIVE_EXPORT void survive_default_gen2_detected_process(SurviveObject *so) {
	SurviveContext *ctx = so->ctx;
	bool allowExperimental = (bool)survive_configi(ctx, "lhv2-experimental", SC_GET, 0);
	if (!allowExperimental) {
		if (so->ctx->currentError == SURVIVE_OK) {
			SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG,
					 "System detected lighthouse v2 system. Currently, libsurvive does not work with this "
					 "setup. If you want to see debug information for this system, pass in "
					 "'--lhv2-experimental'");
		}
	}

	so->ctx->lh_version = 1;
}