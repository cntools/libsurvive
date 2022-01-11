#include "survive.h"

#include "ootx_decoder.h"
#include "survive_kalman_tracker.h"
#include "survive_recording.h"

#define TICKS_PER_ROTATION 400000

void survive_default_light_pulse_process(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode,
										 FLT length, uint32_t lh) {}

void survive_ootx_behavior(SurviveObject *so, int8_t bsd_idx, int8_t lh_version, int ootx);

void survive_default_light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep, uint32_t timecode,
								   uint32_t length, uint32_t lh) {
	lh = survive_get_bsd_idx(so->ctx, lh);

	if (so->ctx->bsd[lh].disable)
		return;

	survive_notify_gen1(so, "Lightcap called");

	SurviveContext *ctx = so->ctx;
	int base_station = lh;

	if (sensor_id == -1 || sensor_id == -2) {
		uint8_t dbit = (acode & 2) >> 1;
		survive_ootx_behavior(so, lh, ctx->lh_version, dbit);
	}

	survive_recording_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lh);

	FLT length_sec = length / (FLT)so->timebase_hz;

	if (sensor_id <= -1) {
		PoserDataLightGen1 l = {
			.common =
				{
					.hdr =
						{
							.pt = POSERDATA_SYNC,
							.timecode = SurviveSensorActivations_long_timecode_light(&so->activations, timecode),
						},
					.sensor_id = sensor_id,
					.angle = 0,
					.lh = lh,
				},
			.acode = acode,
			.length = length,
		};

		if (sensor_id == -3)
			SurviveSensorActivations_add_sync(&so->activations, &l.common);

		ootx_decoder_context *decoderContext = ctx->bsd[lh].ootx_data;
		uint32_t ootxOffset = decoderContext ? decoderContext->offset : 0;
		uint32_t ootxTotalOffset = decoderContext ? decoderContext->total_offset : 0;
		SV_VERBOSE(600, "%s Sync    %8x %3d.%2d.%d %8u %u %d %d / %d", survive_colorize_codename(so),
				   ctx->bsd[lh].BaseStationID, sensor_id, lh, acode & 1, timecode, length, acode & 2 > 0, ootxOffset,
				   ootxTotalOffset);
		survive_kalman_tracker_integrate_light(so->tracker, &l.common);
		SURVIVE_POSER_INVOKE(so, &l);
		SURVIVE_INVOKE_HOOK_SO(light_pulse, so, sensor_id, acode, timecode, length_sec, lh);

		return;
	}

	if (base_station > NUM_GEN1_LIGHTHOUSES)
		return;

	// No loner need sync information past this point.
	if (sensor_id < 0)
		return;

	if (timeinsweep > TICKS_PER_ROTATION) {
		SV_WARN("Disambiguator gave invalid timeinsweep %s %u", so->codename, timeinsweep);
		return;
	}

	FLT angle = timeinsweep * (LINMATHPI / TICKS_PER_ROTATION) - LINMATHPI_2;
	assert(angle >= -LINMATHPI && angle <= LINMATHPI);

	SV_DATA_LOG("angle_sweep[%d][%d][%d]", &angle, 1, sensor_id, lh, acode & 1);
	SV_VERBOSE(600, "%s %s %2d.%2d.%d %8u %f %u %f %u", survive_colorize_codename(so), survive_colorize("SWEEP"),
			   sensor_id, lh, acode & 1, timecode, angle, timeinsweep, angle / M_PI * 180 + 90., length);
	SURVIVE_INVOKE_HOOK_SO(angle, so, sensor_id, acode, timecode, length_sec, angle, lh);
}

void survive_default_lightcap_process(SurviveObject *so, const LightcapElement *le) {
	survive_notify_gen1(so, "Lightcap called");
}

void survive_default_angle_process(SurviveObject *so, int sensor_id, int acode, uint32_t timecode, FLT length,
								   FLT angle, uint32_t lh) {
	survive_notify_gen1(so, "Default angle called");
	SurviveContext *ctx = so->ctx;

	if (ctx->bsd[lh].disable)
		return;

	PoserDataLightGen1 l = {
		.common =
			{
				.hdr =
					{
						.pt = POSERDATA_LIGHT,
						.timecode = SurviveSensorActivations_long_timecode_light(&so->activations, timecode),
					},
				.sensor_id = sensor_id,
				.angle = angle,
				.lh = lh,
			},
		.acode = acode,
		.length = length,
	};
	so->stats.hit_from_lhs[lh]++;
	// Simulate the use of only one lighthouse in playback mode.
	if (lh < ctx->activeLighthouses) {
		if (SurviveSensorActivations_add(&so->activations, &l)) {
			survive_kalman_tracker_integrate_light(so->tracker, &l.common);
			so->stats.accepted_data[lh]++;
			SURVIVE_POSER_INVOKE(so, &l);
		} else {
			so->stats.rejected_data[lh]++;
			SV_DATA_LOG("rejected_light[%d][%d][%d]", &angle, 1, sensor_id, lh, acode & 0x1);
		}
	}

	survive_recording_angle_process(so, sensor_id, acode, timecode, length, angle, lh);
}
