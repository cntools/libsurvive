#include "survive.h"

#include "survive_recording.h"
#include <assert.h>
#include <os_generic.h>
#include <stdio.h>

uint8_t survive_map_sensor_id(SurviveObject *so, uint8_t reported_id) {
	if (so->channel_map) {
		assert(reported_id < 32);
		int ole = reported_id;
		uint8_t mapped_id = so->channel_map[ole];
		if (mapped_id >= so->sensor_ct) {
			SurviveContext *ctx = so->ctx;
			SV_WARN("Invalid sensor %d detected hit (%d)", mapped_id, ole);
			return -1;
		}
		return mapped_id;
	}
	return reported_id;
}

bool handle_lightcap(SurviveObject *so, const LightcapElement *_le) {
	// Gen2 devices can trigger this on startup; but later packets should
	// reliably change to lh_version == 1. If we see 50+ lightcap packets
	// without these gen2 packets we can just call it for gen1.
	if (so->ctx->lh_version == -1) {
		static size_t pulse_count = 0;
		static size_t total_count = 0;
		static size_t large_size_count = 0;

		total_count++;

		if (_le->length >= 0x8000) {
			// if(large_size_count++ > 30)
			//	survive_notify_gen2(so, "Lightcap length >= 0x8000");
		} else if (_le->length >= 3000 && _le->length < 6500) {
			pulse_count++;
			// Only look for the OOTX pulses; otherwise we get false hits and can potentially choose gen1
			// on a gen2 system
			if (pulse_count++ > 30) {
				survive_notify_gen1(so, "OOTX pulses detected");
			}
		}

		if (total_count > 100) {
			survive_notify_gen2(so, "no OOTX pulses detected");
		}
		return true;
	}

	LightcapElement le = *_le;
	survive_recording_lightcap(so, &le);

	le.sensor_id = survive_map_sensor_id(so, le.sensor_id);
	if (le.sensor_id == (uint8_t)-1) {
		return false;
	}
	so->ctx->lightcapproc(so, &le);

	return true;
}
