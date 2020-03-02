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
	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	int id = ct->user1;

	if (!ctx->bsd[id].OOTXSet)
		SV_INFO("(%d) %s", ctx->bsd[id].mode != 255 ? ctx->bsd[id].mode : id, msg);
}

static void ootx_packet_clbk_d_gen2(ootx_decoder_context *ct, ootx_packet *packet) {
	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	int id = ct->user1;

	lighthouse_info_v15 v15;
	init_lighthouse_info_v15(&v15, packet->data);

	BaseStationData *b = &ctx->bsd[id];

	bool doSave = b->BaseStationID != v15.id || b->OOTXSet == false;

	if (doSave) {
		SV_INFO("Got OOTX packet %d", ctx->bsd[id].mode);

		b->BaseStationID = v15.id;
		for (int i = 0; i < 2; i++) {
			b->fcal[i].phase = v15.fcal_phase[i];
			b->fcal[i].tilt = v15.fcal_tilt[i];
			b->fcal[i].curve = v15.fcal_curve[i];
			b->fcal[i].gibpha = v15.fcal_gibphase[i];
			b->fcal[i].gibmag = v15.fcal_gibmag[i];
			b->fcal[i].ogeephase = v15.fcal_ogeephase[i];
			b->fcal[i].ogeemag = v15.fcal_ogeemag[i];
		}

		for (int i = 0; i < 3; i++) {
			b->accel[i] = v15.accel_dir[i];
		}

		// Although we know this already....
		b->mode = v15.mode_current & 0x7F;
		b->OOTXSet = 1;

		config_set_lighthouse(ctx->lh_config, b, id);
		config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));
	}
}

static void ootx_packet_cblk_d_gen1(ootx_decoder_context *ct, ootx_packet *packet) {

	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	int id = ct->user1;

	SV_INFO("Got OOTX packet %d", id);

	lighthouse_info_v6 v6;
	init_lighthouse_info_v6(&v6, packet->data);

	BaseStationData *b = &ctx->bsd[id];

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
	config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));
}

void survive_ootx_behavior(SurviveObject *so, int8_t bsd_idx, int8_t lh_version, bool ootx) {
	struct SurviveContext *ctx = so->ctx;
	if (ctx->bsd[bsd_idx].OOTXSet == false) {
		ootx_decoder_context *decoderContext = ctx->bsd[bsd_idx].ootx_data;

		if (decoderContext == 0) {
			if (lh_version == 1) {
				SV_INFO("OOTX not set for LH in channel %d; attaching ootx decoder using device %s",
						ctx->bsd[bsd_idx].mode, so->codename);
			} else {
				SV_INFO("OOTX not set for LH %d; attaching ootx decoder using device %s", bsd_idx, so->codename);
			}
			decoderContext = ctx->bsd[bsd_idx].ootx_data = SV_CALLOC(1, sizeof(ootx_decoder_context));
			ootx_init_decoder_context(decoderContext);
			decoderContext->user1 = bsd_idx;
			decoderContext->user = so;
			decoderContext->ootx_packet_clbk = lh_version ? ootx_packet_clbk_d_gen2 : ootx_packet_cblk_d_gen1;
			decoderContext->ootx_error_clbk = ootx_error_clbk_d;
		}
		if (decoderContext->user == so) {
			ootx_pump_bit(decoderContext, ootx);

			if (ctx->bsd[bsd_idx].OOTXSet) {
				ctx->bsd[bsd_idx].ootx_data = 0;
				ootx_free_decoder_context(decoderContext);
				free(decoderContext);
			}
		}
	}
}

SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode,
												 bool ootx, bool gen) {
	struct SurviveContext *ctx = so->ctx;
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);
	if (bsd_idx == -1) {
		SV_WARN("Invalid channel requested(%d) for %s", channel, so->codename)
		return;
	}

	assert(channel <= NUM_GEN2_LIGHTHOUSES);

	survive_recording_sync_process(so, channel, timecode, ootx, gen);

	so->last_time_between_sync[bsd_idx] = survive_timecode_difference(timecode, so->last_sync_time[bsd_idx]);
	FLT hz = 48000000. / so->last_time_between_sync[bsd_idx];
	SV_VERBOSE(150, "Sync hz %2d: %2.6fhz (err: %0.6fhz) ootx: %d gen: %d", channel, hz,
			   fabs(hz - freq_per_channel[channel]), ootx, gen);

	so->last_sync_time[bsd_idx] = timecode;

	survive_ootx_behavior(so, bsd_idx, ctx->lh_version, ootx);

	PoserDataLightGen2 l = {.common = {
								.hdr =
									{
										.pt = POSERDATA_SYNC_GEN2,
										.timecode = timecode,
									},
								.lh = bsd_idx,
							}};

	if (so->PoserFn) {
		so->PoserFn(so, (PoserData *)&l);
	}
}
SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
												  survive_timecode timecode, bool half_clock_flag) {
	struct SurviveContext *ctx = so->ctx;

	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);
	if (bsd_idx == -1) {
		SV_WARN("Invalid channel requested(%d) for %s", channel, so->codename)
		return;
	}

	survive_notify_gen2(so, "sweep called");

	if (ctx->calptr) {
		// survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length, lh);
	}

	survive_recording_sweep_process(so, channel, sensor_id, timecode, half_clock_flag);

	survive_timecode last_sweep = so->last_sync_time[bsd_idx];
	assert(channel <= NUM_GEN2_LIGHTHOUSES);

	// SV_INFO("Sensor ch%2d %2d %d %12x %6d", channel, sensor_id, flag, timecode, timecode
	// so->last_sync_time[bsd_idx]);

	FLT time_since_sync = (survive_timecode_difference(timecode, last_sweep) / 48000000.);
	// if (half_clock_flag)
	//	time_since_sync += 0.5 / 48000000.;

	FLT hz = 48000000. / so->last_time_between_sync[bsd_idx];
	if (fabs(hz - freq_per_channel[channel]) > 1.0) {
		hz = freq_per_channel[channel];
	}

	FLT time_per_rot = 1. / hz;

	if (time_since_sync > time_per_rot)
		return;

	FLT angle = time_since_sync / time_per_rot * 2. * LINMATHPI;
	FLT angle2 = (time_since_sync + .5 / 48000000.) / time_per_rot * 2. * LINMATHPI;

	// SV_INFO("Sensor ch%2d %2d %12f %12f %d %.16f", channel, sensor_id, angle / M_PI * 180., angle2 / M_PI * 180.,
	// half_clock_flag, time_since_sync);

	int8_t plane = angle > LINMATHPI;
	if (plane)
		angle -= 4 * LINMATHPI / 3.;
	else
		angle -= 2 * LINMATHPI / 3.;

	so->ctx->sweep_angleproc(so, channel, sensor_id, timecode, plane, angle);
}

SURVIVE_EXPORT void survive_default_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
														survive_timecode timecode, int8_t plane, FLT angle) {
	struct SurviveContext *ctx = so->ctx;
	// SV_INFO("Sensor ch%2d %2d %12f", channel, sensor_id, angle / M_PI * 180.);
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);
	if (bsd_idx == -1) {
		SV_WARN("Invalid channel requested(%d) for %s", channel, so->codename)
		return;
	}

	PoserDataLightGen2 l = {.common =
								{
									.hdr =
										{
											.pt = POSERDATA_LIGHT_GEN2,
											.timecode = timecode,
										},
									.sensor_id = sensor_id,
									.angle = angle,
									.lh = bsd_idx,
								},
							.plane = plane};

	// Simulate the use of only one lighthouse in playback mode.
	if (bsd_idx < ctx->activeLighthouses)
		SurviveSensorActivations_add_gen2(&so->activations, &l);

	survive_recording_sweep_angle_process(so, channel, sensor_id, timecode, plane, angle);

	if (so->PoserFn) {
		so->PoserFn(so, (PoserData *)&l);
	}
}

SURVIVE_EXPORT void survive_default_gen_detected_process(SurviveObject *so, int lh_version) {
	SurviveContext *ctx = so->ctx;

	if (ctx->lh_version != -1) {
		static bool seenWarning = false;
		if (seenWarning == false) {
			SV_WARN("Detected both LH gen1 and LH gen2 systems. Mixed mode is only supported for experimentation.");
			seenWarning = true;
		}

		ctx->lh_version = 3;
		return;
	}
	assert(ctx->lh_version == -1);

	SV_INFO("Detected LH gen %d system.", lh_version + 1);
	ctx->lh_version = lh_version;

	if (ctx->lh_version == 0) {
		int fastCalibrate = survive_configi(ctx, "fast-calibrate", SC_GET, 0);
		if (fastCalibrate) {
			SV_INFO("Using fast calibration");
			return;
		}

		int calibrateMandatory = survive_configi(ctx, "force-calibrate", SC_GET, 0);
		int calibrateForbidden = survive_configi(ctx, "disable-calibrate", SC_GET, 1) == 1;
		if (calibrateMandatory && calibrateForbidden) {
			SV_INFO("Contradictory settings --force-calibrate and --disable-calibrate specified. Switching to normal "
					"behavior.");
			calibrateMandatory = calibrateForbidden = 0;
		}

		if (!calibrateForbidden) {
			bool isCalibrated = ctx->activeLighthouses > 0;
			for (int i = 0; i < ctx->activeLighthouses; i++) {
				if (!ctx->bsd[i].PositionSet) {
					SV_INFO("Lighthouse %d position is unset", i);
					isCalibrated = false;
				}
			}

			bool doCalibrate = isCalibrated == false || calibrateMandatory;

			if (!isCalibrated) {
				SV_INFO(
					"Uncalibrated configuration detected. Attaching calibration. Please don't move tracked objects for "
					"the duration of calibration. Pass '--disable-calibrate' to skip calibration");
			} else if (doCalibrate) {
				SV_INFO("Calibration requested. Previous calibration will be overwritten.");
			}

			if (doCalibrate && ctx->objs_ct > 0) {
				ctx->bsd[0].PositionSet = ctx->bsd[1].PositionSet = false;
				survive_cal_install(ctx);
			}
		}
	}
}
