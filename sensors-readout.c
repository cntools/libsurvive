#include <survive.h>

static volatile int keepRunning = 1;

#include "math.h"
#include <os_generic.h>
#include <stdlib.h>
#ifdef __linux__

#include <assert.h>

#include <ctype.h>
#include <os_generic.h>

#include <signal.h>

void intHandler(int dummy) {
	if (keepRunning == 0)
		exit(-1);
	keepRunning = 0;
}

#endif
bool needsRedraw = false;
bool surviveIsDone = false;
struct sensor_stats {
	double MN, MX;
};
struct sensor_time_stats {
	size_t hit_count;
	double hz;

	size_t hz_count;
	survive_timecode hz_start;
};
struct sensor_stats stats[32][NUM_GEN2_LIGHTHOUSES][SENSORS_PER_OBJECT][2] = {};
struct sensor_time_stats time_stats[32][SENSORS_PER_OBJECT] = {};

void process_reading(int i, int lh, int sensor, int axis, FLT angle) {
	struct sensor_stats *s = &stats[i][lh][sensor][axis];

	s->MN = fmin(angle, s->MN);
	s->MX = fmax(angle, s->MX);
}

void sweep_fn(SurviveObject *so, survive_channel channel, int sensor_id, survive_timecode timecode, int8_t plane,
			  FLT angle) {
	size_t idx = 0;
	for (idx = 0; idx < so->ctx->objs_ct && so->ctx->objs[idx] != so; idx++)
		;

	time_stats[idx][sensor_id].hit_count++;

	double time_since_start =
		survive_timecode_difference(timecode, time_stats[idx][sensor_id].hz_start) / (double)so->timebase_hz;
	struct SurviveContext *ctx = so->ctx;

	time_stats[idx][sensor_id].hz_count++;
	if (time_since_start > 3. || time_stats[idx][sensor_id].hz_start == 0) {
		if (time_stats[idx][sensor_id].hz_start != 0)
			time_stats[idx][sensor_id].hz = time_stats[idx][sensor_id].hz_count / time_since_start;
		time_stats[idx][sensor_id].hz_count = 0;
		time_stats[idx][sensor_id].hz_start = timecode;
	}

	survive_default_sweep_angle_process(so, channel, sensor_id, timecode, plane, angle);
}

const char *column_width = "          ";
static void print_int(int i) { printf("%9d |", i); }
static void print(float f) {
	if (isnan(f)) {
		printf("%s|", column_width);
	} else if (fabs(f) < 10.) {
		printf("%+9.6f |", f);
	} else {
		printf("%+9.4f |", f);
	}
}

static void print_label(const char *l) { printf("%*s|", 10, l); }

int printf_fn(SurviveContext *ctx, const char *fault, ...) { return 0; }

int lh = 0;
bool useRawSensorId = false;
static uint8_t get_raw_sensor_id(SurviveObject *so, uint8_t sensor_id) {
	if (so->channel_map) {
		for (int i = 0; i < 32; i++) {
			if (so->channel_map[i] == sensor_id) {
				return i;
			}
		}
		return -1;
	}
	return sensor_id;
}

static void redraw(SurviveContext *ctx) {
	printf("\033[;H");
	for (int i = 0; i < ctx->objs_ct; i++) {
		SurviveObject *so = ctx->objs[i];

		printf("Object: %s: ", so->codename);

		{
			double v[2] = {0, 0};
			int v_cnt[2] = {0};
			for (int sensor = 0; sensor < so->sensor_ct; sensor++) {
				for (int axis = 0; axis < 2; axis++) {
					FLT f = so->activations.angles[sensor][lh][axis];
					if (!isnan(f)) {
						v_cnt[axis]++;
						v[axis] += f;
					}
				}
			}

			for (int axis = 0; axis < 2; axis++) {
				printf("%1.6f ", v[axis] / (double)v_cnt[axis]);
			}
		}

		printf("\n");

		printf("|\e[4m");
		const char *labels[] = {"ch.sensor", "Hits",	"Hits/sec", "X",	 "Y",		"min X",
								"max X",	 "width X", "min Y",	"max Y", "width Y", 0};
		for (const char **l = labels; *l; l++) {
			print_label(*l);
		}
		printf("\e[0m\n");

		{
			for (int sensor = 0; sensor < so->sensor_ct; sensor++) {
				struct sensor_stats *s = &stats[i][lh][sensor][0];

				bool allNans = true;
				for (int axis = 0; axis < 2 && allNans; axis++) {
					FLT f = so->activations.angles[sensor][lh][axis];
					allNans &= isnan(f);
				}

				if (allNans)
					continue;

				if (sensor % 2 == 0)
					printf("\e[2m");
				if (sensor == so->sensor_ct - 1)
					printf("\e[4m");

				uint8_t displaySensor = useRawSensorId ? get_raw_sensor_id(so, sensor) : sensor;

				printf("| %2d.%02d    |", ctx->bsd[lh].mode, displaySensor);

				print_int(time_stats[i][sensor].hit_count);
				print(time_stats[i][sensor].hz);
				for (int axis = 0; axis < 2; axis++) {
					FLT f = so->activations.angles[sensor][lh][axis];
					process_reading(i, lh, sensor, axis, f);
					print(f);
				}

				for (int axis = 0; axis < 2; axis++) {
					print(s[axis].MN);
					print(s[axis].MX);
					print(s[axis].MX - s[axis].MN);
				}
				printf("\e[0m");
				printf("\n");
			}
			printf("\n");
		}
	}

	needsRedraw = false;
}

void imu_fn(SurviveObject *so, int mode, FLT *accelgyro, survive_timecode timecode, int id) {
	if (needsRedraw)
		redraw(so->ctx);
	survive_default_imu_process(so, mode, accelgyro, timecode, id);
}

void *KBThread(void *user) {
	SurviveContext *ctx = user;

	while (keepRunning) {
		int c = tolower(getchar());
		system("clear");
		if (c == 'l') {
			lh++;
			if (!ctx->bsd[lh].OOTXSet)
				lh = 0;
		} else if (c == 'q') {
			keepRunning = false;
		} else if (c == 'r') {
			useRawSensorId = !useRawSensorId;
		}
		if (surviveIsDone) {
			redraw(ctx);
		} else {
			needsRedraw = true;
		}
	}
	return 0;
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
	survive_install_sweep_angle_fn(ctx, sweep_fn);
	survive_install_printf_fn(ctx, printf_fn);
	survive_install_imu_fn(ctx, imu_fn);
	survive_startup(ctx);

	system("clear");

	og_thread_t kbThread = OGCreateThread(KBThread, ctx);

	while (keepRunning && survive_poll(ctx) == 0) {
		FLT this_time = OGGetAbsoluteTime();
		if (this_time > last_redraw + .03) {
			needsRedraw = true;
			last_redraw = this_time;
		}
	}
	surviveIsDone = true;
	if (keepRunning) {
		printf("Survive done, type 'q <enter>' to exit...\n");
	}

	OGJoinThread(kbThread);

	survive_close(ctx);
	return 0;
}
