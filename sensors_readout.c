#include <survive.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include "math.h"
#include <assert.h>
#include <os_generic.h>
#include <signal.h>
#include <stdlib.h>

void intHandler(int dummy) {
	if (keepRunning == 0)
		exit(-1);
	keepRunning = 0;
}

#endif
bool needsRedraw = false;

struct sensor_stats {
	double MN, MX;
};

struct sensor_stats stats[32][NUM_GEN2_LIGHTHOUSES][SENSORS_PER_OBJECT][2];

void process_reading(int i, int lh, int sensor, int axis, FLT angle) {
	struct sensor_stats *s = &stats[i][lh][sensor][axis];

	s->MN = fmin(angle, s->MN);
	s->MX = fmax(angle, s->MX);
}

static void redraw(SurviveContext *ctx) {
	system("clear");

	for (int i = 0; i < ctx->objs_ct; i++) {
		SurviveObject *so = ctx->objs[i];

		printf("Object: %s:\n", so->codename);
		for (int lh = 0; lh < ctx->activeLighthouses; lh++) {
			if (ctx->bsd[lh].OOTXSet == false)
				continue;

			printf("\tLH: %d (%d)\n", lh, ctx->bsd[lh].mode);
			for (int sensor = 0; sensor < so->sensor_ct; sensor++) {
				struct sensor_stats *s = &stats[i][lh][sensor][0];
				printf("\t\t%2d: ", sensor);
				for (int axis = 0; axis < 2; axis++) {
					FLT f = so->activations.angles[sensor][lh][axis];
					process_reading(i, lh, sensor, axis, f);
					if (isnan(f)) {
						printf("          ");
					} else {
						printf("%+1.6f ", f);
					}
				}

				printf("%+1.6f %+1.6f %+1.6f %+1.6f \n", s[0].MN, s[0].MX, s[1].MN, s[1].MX);
			}
		}
	}

	needsRedraw = false;
}

void sync_fn(SurviveObject *so, survive_channel channel, survive_timecode timeinsweep, bool ootx, bool gen) {
	if (needsRedraw)
		redraw(so->ctx);
	survive_default_sync_process(so, channel, timeinsweep, ootx, gen);
}

int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);
#endif

	struct sensor_stats *s = &stats[0][0][0][0];
	for (int i = 0; i < 32 * NUM_GEN2_LIGHTHOUSES * SENSORS_PER_OBJECT * 2; i++) {
		s[i].MX = s[i].MN = NAN;
	}

	SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	FLT last_redraw = OGGetAbsoluteTime();
	survive_install_sync_fn(ctx, sync_fn);
	survive_startup(ctx);
	while (keepRunning && survive_poll(ctx) == 0) {
		FLT this_time = OGGetAbsoluteTime();
		if (this_time > last_redraw + .03) {
			needsRedraw = true;
			last_redraw = this_time;
		}
	}

	survive_close(ctx);
	return 0;
}