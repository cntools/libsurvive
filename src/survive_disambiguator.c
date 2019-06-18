#include "survive.h"

#include "survive_playback.h"
#include <assert.h>
#include <os_generic.h>
#include <stdio.h>

//#define LOG_LIGHTDATA

void handle_lightcap(SurviveObject *so, LightcapElement *le) {
	survive_recording_lightcap(so, le);
#ifdef LOG_LIGHTDATA
	static FILE *flog;
	static double start = 0;
	if (!flog) {
		flog = fopen("lightcap.txt", "wb");
		start = OGGetAbsoluteTime();
	}
	fprintf(flog, "%.6f %2d %4d %9d\n", OGGetAbsoluteTime() - start, le->sensor_id, le->length, le->timestamp);
#endif

	if (so->channel_map) {
		assert(le->sensor_id < 32);
		int ole = le->sensor_id;
		le->sensor_id = so->channel_map[ole];
		if (le->sensor_id >= so->sensor_ct) {
		  SurviveContext* ctx = so->ctx;
		  SV_WARN("Invalid sensor %d detected hit (%d)", le->sensor_id, ole);
		  return;
		}
	}
	so->ctx->lightcapfunction(so, le);
}
