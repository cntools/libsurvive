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
		SV_INFO("(%d) %s", ctx->bsd[id].mode, msg);
}

static void ootx_packet_clbk_d(ootx_decoder_context *ct, ootx_packet *packet) {
	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	SurviveCalData *cd = ctx->calptr;
	int id = ct->user1;

	lighthouse_info_v15 v15;
	init_lighthouse_info_v15(&v15, packet->data);

	BaseStationData *b = &ctx->bsd[id];

	bool doSave = b->BaseStationID != v15.id || b->OOTXSet == false;

	if (doSave) {
		SV_INFO("Got OOTX packet %d %p", ctx->bsd[id].mode, cd);

		b->BaseStationID = v15.id;
		for (int i = 0; i < 2; i++) {
			b->fcal[i].phase = v15.fcal_phase[i];
			b->fcal[i].tilt = tan(v15.fcal_tilt[i]);
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

SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode,
												 bool ootx, bool gen) {
	struct SurviveContext *ctx = so->ctx;
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);

	assert(channel <= NUM_GEN2_LIGHTHOUSES);

	// FLT hz = 48000000. / survive_timecode_difference(timecode, so->last_sync_time[bsd_idx]);
	// SV_INFO("Sync hz %2d: %2.6fhz (err: %0.6fhz)", channel, hz, hz - freq_per_channel[channel]);
	so->last_sync_time[bsd_idx] = timecode;

	if (ctx->bsd[bsd_idx].OOTXSet == false) {
		ootx_decoder_context *decoderContext = ctx->bsd[bsd_idx].ootx_data;

		if (decoderContext == 0) {
			SV_INFO("OOTX not set for LH in channel %d; attaching ootx decoder using device %s", channel, so->codename);

			decoderContext = ctx->bsd[bsd_idx].ootx_data = calloc(1, sizeof(ootx_decoder_context));
			ootx_init_decoder_context(decoderContext);
			decoderContext->user1 = bsd_idx;
			decoderContext->user = so;
			decoderContext->ootx_packet_clbk = ootx_packet_clbk_d;
			decoderContext->ootx_error_clbk = ootx_error_clbk_d;
		}
		if (decoderContext->user == so) {
			ootx_pump_bit(decoderContext, ootx);

			if (ctx->bsd[bsd_idx].OOTXSet) {
				ctx->bsd[bsd_idx].ootx_data = 0;
				ootx_free_decoder_context(decoderContext);
			}
		}
	}

	PoserDataLightGen2 l = {
		.hdr =
			{
				.pt = POSERDATA_SYNC_GEN2,
			},
		.timecode = timecode,
		.lh = bsd_idx,
	};

	if (so->PoserFn) {
		so->PoserFn(so, (PoserData *)&l);
	}
}
SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
												  survive_timecode timecode, bool half_clock_flag) {
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
	if (half_clock_flag)
		time_since_sync += 0.5 / 48000000.;

	FLT time_per_rot = 1. / freq_per_channel[channel];

	if (time_since_sync > time_per_rot)
		return;

	FLT angle = time_since_sync / time_per_rot * 2. * LINMATHPI;

	int8_t plane = angle > LINMATHPI;
	if (plane)
		angle -= 4 * LINMATHPI / 3.;
	else
		angle -= 2 * LINMATHPI / 3.;
	so->ctx->sweep_angleproc(so, channel, sensor_id, timecode, plane, angle);
}

SURVIVE_EXPORT void survive_default_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
														survive_timecode timecode, int8_t plane, FLT angle) {
	survive_notify_gen2(so);

	struct SurviveContext *ctx = so->ctx;
	// SV_INFO("Sensor ch%2d %2d %12f", channel, sensor_id, angle);
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);

	PoserDataLightGen2 l = {.hdr =
								{
									.pt = POSERDATA_LIGHT_GEN2,
								},
							.sensor_id = sensor_id,
							.timecode = timecode,
							.angle = angle,
							.lh = bsd_idx,
							.plane = plane};

	// Simulate the use of only one lighthouse in playback mode.
	if (bsd_idx < ctx->activeLighthouses)
		SurviveSensorActivations_add_gen2(&so->activations, &l);

	survive_recording_sweep_angle_process(so, channel, sensor_id, timecode, plane, angle);

	if (ctx->calptr) {
		// survive_cal_angle(so, sensor_id, acode, timecode, length, angle, lh);
	}
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
		int calibrateMandatory = survive_configi(ctx, "force-calibrate", SC_GET, 0);
		int calibrateForbidden = survive_configi(ctx, "disable-calibrate", SC_GET, 1) == 1;
		if (calibrateMandatory && calibrateForbidden) {
			SV_INFO("Contradictory settings --force-calibrate and --disable-calibrate specified. Switching to normal "
					"behavior.");
			calibrateMandatory = calibrateForbidden = 0;
		}

		if (!calibrateForbidden) {
			bool isCalibrated = true;
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
	} else {
		int calibrateMandatory = survive_configi(ctx, "force-calibrate", SC_GET, 0);
		if (calibrateMandatory) {
			SV_INFO("Force calibrate flag set -- clearing position on all lighthouses");
			for (int i = 0; i < ctx->activeLighthouses; i++) {
				ctx->bsd[i].PositionSet = 0;
			}
		}
	}
}
