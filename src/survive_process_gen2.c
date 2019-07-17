#include "ootx_decoder.h"
#include "survive.h"
#include "survive_config.h"
#include "survive_internal.h"
#include "survive_playback.h"
#include <assert.h>
#include <math.h>

static FLT freq_per_channel[NUM_GEN2_LIGHTHOUSES] = {
	50.0521, 50.1567, 50.3673, 50.5796, 50.6864, 50.9014, 51.0096, 51.1182,
	51.2273, 51.6685, 52.2307, 52.6894, 52.9217, 53.2741, 53.7514, 54.1150,
};

static void ootx_error_clbk_d(ootx_decoder_context *ct, const char *msg) {
	SurviveContext *ctx = (SurviveContext *)(ct->user);
	int id = ct->user1;
	SV_INFO("(%d) %s", id, msg);
}

static void ootx_packet_clbk_d(ootx_decoder_context *ct, ootx_packet *packet) {
	static uint8_t lighthouses_completed = 0;

	SurviveContext *ctx = (SurviveContext *)(ct->user);
	SurviveCalData *cd = ctx->calptr;
	int id = ct->user1;

	SV_INFO("Got OOTX packet %d %p", id, cd);

	lighthouse_info_v6 v6;
	init_lighthouse_info_v6(&v6, packet->data);

	BaseStationData *b = &ctx->bsd[id];
	// print_lighthouse_info_v6(&v6);

	b->BaseStationID = v6.id;
	b->fcal[0].phase = v6.fcal_0_phase;
	b->fcal[1].phase = v6.fcal_1_phase;
	b->fcal[0].tilt = tan(v6.fcal_0_tilt);
	b->fcal[1].tilt = tan(v6.fcal_1_tilt); // XXX??? Is this right? See https://github.com/cnlohr/libsurvive/issues/18
	b->fcal[0].curve = v6.fcal_0_curve;
	b->fcal[1].curve = v6.fcal_1_curve;
	b->fcal[0].gibpha = v6.fcal_0_gibphase;
	b->fcal[1].gibpha = v6.fcal_1_gibphase;
	b->fcal[0].gibmag = v6.fcal_0_gibmag;
	b->fcal[1].gibmag = v6.fcal_1_gibmag;
	b->accel[0] = v6.accel_dir_x;
	b->accel[1] = v6.accel_dir_y;
	b->accel[2] = v6.accel_dir_z;
	b->mode = v6.mode_current;
	b->OOTXSet = 1;

	config_set_lighthouse(ctx->lh_config, b, id);
	lighthouses_completed++;

	if (lighthouses_completed >= ctx->activeLighthouses) {
		config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));
	}
}

SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode,
												 bool ootx, bool gen) {
	struct SurviveContext *ctx = so->ctx;
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);

	assert(channel <= NUM_GEN2_LIGHTHOUSES);
	so->last_sync_time[bsd_idx] = timecode;

	ootx_decoder_context *decoderContext = ctx->bsd[bsd_idx].ootx_data;
	if (decoderContext == 0) {
		decoderContext = ctx->bsd[bsd_idx].ootx_data = calloc(1, sizeof(ootx_decoder_context));
		ootx_init_decoder_context(decoderContext);
		decoderContext->user = ctx;
		decoderContext->ootx_packet_clbk = ootx_packet_clbk_d;
		decoderContext->ootx_error_clbk = ootx_error_clbk_d;
	}
	ootx_pump_bit(decoderContext, ootx);
	// SV_INFO("Sync   ch%2d  %d %d %12x", channel, ootx, gen, timecode);
}
SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
												  survive_timecode timecode, bool flag) {
	struct SurviveContext *ctx = so->ctx;
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);

	if (ctx->calptr) {
		// survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length, lh);
	}

	survive_recording_sweep_process(so, sensor_id, timecode, timecode, channel);

	survive_timecode last_sweep = so->last_sync_time[bsd_idx];
	assert(channel <= NUM_GEN2_LIGHTHOUSES);

	// SV_INFO("Sensor ch%2d %2d %d %12x %6d", channel, sensor_id, flag, timecode, timecode -
	// so->last_sync_time[bsd_idx]);

	FLT time_since_sync = (survive_timecode_difference(timecode, last_sweep) / 48000000.);
	FLT time_per_rot = 1. / freq_per_channel[channel];

	FLT angle = time_since_sync / time_per_rot * 2. * LINMATHPI;

	so->ctx->sweep_angleproc(so, channel, sensor_id, timecode, angle);
}

SURVIVE_EXPORT void survive_default_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
														survive_timecode timecode, FLT angle) {
	struct SurviveContext *ctx = so->ctx;
	// SV_INFO("Sensor ch%2d %2d %12f", channel, sensor_id, angle);
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);

	PoserDataLightGen2 l = {
		.hdr =
			{
				.pt = POSERDATA_LIGHT_GEN2,
			},
		.sensor_id = sensor_id,
		.timecode = timecode,
		.angle = angle,
		.lh = bsd_idx,
	};

	// Simulate the use of only one lighthouse in playback mode.
	if (bsd_idx < ctx->activeLighthouses)
		SurviveSensorActivations_add_gen2(&so->activations, &l);

	survive_recording_sweep_angle_process(so, channel, sensor_id, timecode, angle);

	if (ctx->calptr) {
		// survive_cal_angle(so, sensor_id, acode, timecode, length, angle, lh);
	}
	if (so->PoserFn) {
		so->PoserFn(so, (PoserData *)&l);
	}

	static int timer = 0;
	if (timer < 100) {
		timer++;
		if (timer == 100) {

			PoserCB configPoser = GetDriver("PoserEPNP");

			PoserDataFullScene pdfs = {.hdr = {.pt = POSERDATA_FULL_SCENE}};
			Activations2PoserDataFullScene(&so->activations, &pdfs);
			so->PoserData = 0;
			ctx->activeLighthouses = 1;
			configPoser(so, &pdfs.hdr);
		}
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