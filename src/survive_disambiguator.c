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
		le->sensor_id = so->channel_map[le->sensor_id];
		assert(le->sensor_id != -1);
	}
	so->ctx->lightcapfunction(so, le);
}
