#include <survive.h>

static volatile int keepRunning = 1;

#include "math.h"
#include <os_generic.h>
#include <stdlib.h>
#ifdef __linux__
#include <sys/ioctl.h>
#include <unistd.h>

#include <assert.h>
#include <signal.h>

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
const char *column_width = "          ";
static void print(float f) {
	if (isnan(f)) {
		printf("%s|", column_width);
	} else {
		printf("%+1.6f |", f);
	}
}

static void print_label(const char *l) { printf("%*s|", 10, l); }


char* new_str(const char* s) {
    char* rtn = calloc(strlen(s) + 1, sizeof(char));
    strcpy(rtn, s);
    return rtn;
}

char *lines[10] = {};
size_t lines_idx = 0;

int lh = -1;

int window_rows = -1, window_cols = -1;
#define gotoxy(x,y) printf("\033[%d;%dH", (y), (x))
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
		const char *labels[] = {"ch.sensor", "X", "Y", "min X", "max X", "width X", "min Y", "max Y", "width Y", 0};
		for (const char **l = labels; *l; l++) {
			print_label(*l);
		}
		printf("\e[0m\n");

		int lh_start = lh == -1 ? 0 : lh;
		int lh_end = lh == -1 ? NUM_GEN2_LIGHTHOUSES : (lh + 1);

		for(int lh = lh_start;lh < lh_end;lh++)
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

				printf("| %2d.%02d    |", ctx->bsd[lh].mode, sensor);
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
				printf("\33[2K\r\n");
			}
			printf("\33[2K\r\n");
		}
	}

    if(window_cols != -1) {
        gotoxy(0, window_rows - 10 - 1);
        printf("=== Log ===\n");
        for(int i = 0;i < 10;i++) {
            char* line = lines[(lines_idx + i)%10];
            if(line != 0)
                printf("\33[2K\r %s\n", line);
        }
    }

	needsRedraw = false;
}

void light_fn(SurviveObject *so, int sensor_id, int acode, int timeinsweep,
        survive_timecode timecode, survive_timecode length, uint32_t lh) {
    survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lh);
    if (needsRedraw)
        redraw(so->ctx);
}

void imu_fn(SurviveObject *so, int mode, FLT *accelgyro, survive_timecode timecode, int id) {
	if (needsRedraw)
		redraw(so->ctx);
}

void info_fn(SurviveContext *ctx, SurviveLogLevel logLevel, const char *fault) {
    free(lines[lines_idx % 10]);
    lines[lines_idx % 10] = new_str(fault);
    lines_idx++;
    redraw(ctx);
}

void *KBThread(void *user) {
	SurviveContext *ctx = user;

	while (keepRunning) {
		int c = getchar();
		system("clear");
		needsRedraw = true;
		if (c == 10) {
			lh++;
			if (!ctx->bsd[lh].OOTXSet)
				lh = -1;
		}
	}
	return 0;
}
int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);

    struct winsize w;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    window_cols = w.ws_col;
    window_rows = w.ws_row;
#endif

	struct sensor_stats *s = &stats[0][0][0][0];
	for (int i = 0; i < 32 * NUM_GEN2_LIGHTHOUSES * SENSORS_PER_OBJECT * 2; i++) {
		s[i].MX = s[i].MN = NAN;
	}

	SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	FLT last_redraw = OGGetAbsoluteTime();
	survive_install_log_fn(ctx, info_fn);
	survive_install_imu_fn(ctx, imu_fn);
	survive_install_light_fn(ctx, light_fn);
	survive_startup(ctx);

	system("clear");

	OGCreateThread(KBThread, ctx);

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