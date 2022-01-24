#include "src/survive_kalman_tracker.h"
#include <survive.h>

static volatile int keepRunning = 1;
static void redraw(SurviveContext *ctx);

#include "math.h"
#include <os_generic.h>
#include <stdlib.h>
#include <variance.h>

#ifndef _WIN32
#include <sys/ioctl.h>
#include <unistd.h>

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

struct variance_measure imu_variance = {.size = 6};

struct sensor_stats {
	double MN, MX;
	struct variance_measure variance;
};
struct sensor_time_stats {
	size_t hit_count;
	double hz;

	size_t hz_count;
	survive_timecode hz_start;
};
struct sensor_stats stats[32][NUM_GEN2_LIGHTHOUSES][SENSORS_PER_OBJECT][2] = {0};
struct sensor_time_stats time_stats[32][NUM_GEN2_LIGHTHOUSES][SENSORS_PER_OBJECT] = {0};

struct sensor_time_stats imu_time_stats[32];

void process_reading(int i, int lh, int sensor, int axis, FLT angle) {
	struct sensor_stats *s = &stats[i][lh][sensor][axis];

	if (isnan(angle))
		return;

	variance_measure_add(&stats[i][lh][sensor][axis].variance, &angle);
	s->MN = fmin(angle, s->MN);
	s->MX = fmax(angle, s->MX);
}

static void record_data(SurviveObject *so, int sensor_id, survive_timecode timecode, uint32_t lh) {
	size_t idx = 0;
	for (idx = 0; idx < so->ctx->objs_ct && so->ctx->objs[idx] != so; idx++)
		;

	time_stats[idx][lh][sensor_id].hit_count++;

	double time_since_start =
		survive_timecode_difference(timecode, time_stats[idx][lh][sensor_id].hz_start) / (double)so->timebase_hz;
	struct SurviveContext *ctx = so->ctx;

	time_stats[idx][lh][sensor_id].hz_count++;
	if (time_since_start > 3. || time_stats[idx][lh][sensor_id].hz_start == 0) {
		if (time_stats[idx][lh][sensor_id].hz_start != 0)
			time_stats[idx][lh][sensor_id].hz = time_stats[idx][lh][sensor_id].hz_count / time_since_start;
		time_stats[idx][lh][sensor_id].hz_count = 0;
		time_stats[idx][lh][sensor_id].hz_start = timecode;

		variance_measure_reset(&stats[idx][lh][sensor_id]->variance);
	}
}

void angle_fn(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode,
	      FLT length, FLT angle, uint32_t lh) {
	record_data(so, sensor_id, timecode, lh);

	survive_default_angle_process(so, sensor_id, acode, timecode, length, angle, lh);
}

void sweep_fn(SurviveObject *so, survive_channel channel, int sensor_id, survive_timecode timecode, int8_t plane,
			  FLT angle) {
	record_data(so, sensor_id, timecode, survive_get_bsd_idx(so->ctx, channel));
	survive_default_sweep_angle_process(so, channel, sensor_id, timecode, plane, angle);

	if (needsRedraw)
		redraw(so->ctx);
}

const char *column_width = "          ";
static void print_int(int i) { printf("%9d |", i); }

static void print_small(float f) { printf("%+3.2f ", f); }
static void print_small_sci(float f) { printf("%+2.1e ", f); }

static void print(float f) {
	if (isnan(f)) {
		printf("%s|", column_width);
	} else if (fabs(f) > 0 && fabs(f) < 1e-4) {
		printf("%+9.2e |", f);
	} else if (fabs(f) < 10.) {
		printf("%+9.6f |", f);
	} else {
		printf("%+9.4f |", f);
	}
}

static void print_label(const char *l) { printf("%*s|", 10, l); }

int printf_fn(SurviveContext *ctx, const char *fault, ...) { return 0; }

int lh = -1;
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

char *new_str(const char *s) {
	char *rtn = calloc(strlen(s) + 1, sizeof(char));
	strcpy(rtn, s);
	return rtn;
}

char *lines[10] = {0};
size_t lines_idx = 0;

static void draw_model(const SurviveKalmanModel *mdl) {
	printf("Rot: ");
	for (int i = 0; i < 4; i++)
		print_small_sci(mdl->Pose.Rot[i]);
	printf("Acc: ");
	print_small_sci(norm3d(mdl->Acc));
	for (int i = 0; i < 3; i++)
		print_small_sci(mdl->Acc[i]);
	printf("Vel: ");
	for (int i = 0; i < 3; i++)
		print_small_sci(mdl->Velocity.Pos[i]);
	for (int i = 0; i < 3; i++)
		print_small_sci(mdl->Velocity.AxisAngleRot[i]);
	printf("Fix: ");
	for (int i = 0; i < 4; i++)
		print_small_sci(mdl->IMUBias.IMUCorrection[i]);
	for (int i = 0; i < 3; i++)
		print_small_sci(mdl->IMUBias.AccScale[i]);
	printf("\n");
}

int window_rows = -1, window_cols = -1;
#define gotoxy(x, y) printf("\033[%d;%dH", (y), (x))
static void redraw(SurviveContext *ctx) {
	printf("\033[;H");
	for (int i = 0; i < ctx->objs_ct; i++) {
		SurviveObject *so = ctx->objs[i];

		printf("%s (%+5.2fs still): ", so->codename,
			   SurviveSensorActivations_stationary_time(&so->activations) / 48000000.);

		if (lh >= 0) {
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
				printf("%1.6f ", v[axis] / (v_cnt[axis] == 0 ? 1 : (double)v_cnt[axis]));
			}
		}

		FLT calc_imu_var[6];
		variance_measure_calc(&imu_variance, calc_imu_var);

		printf("IMU: %5.1fhz ", imu_time_stats[i].hz);
		print_small(norm3d(so->activations.last_accel) - 1);
		for (int i = 0; i < 3; i++)
			print_small(so->activations.last_accel[i]);
		for (int i = 0; i < 3; i++)
			print_small(so->activations.gyro[i]);
		printf("Var: ");
		for (int i = 0; i < 6; i++)
			print_small_sci(calc_imu_var[i]);
		printf("\n");

		draw_model(&so->tracker->state);
		FLT Pd[sizeof(SurviveKalmanModel) / sizeof(FLT)] = {0};
		cn_get_diag(&so->tracker->model.P, Pd, sizeof(Pd) / sizeof(Pd[0]));
		draw_model((const SurviveKalmanModel *)Pd);

		printf("|\x1B[4m");
		const char *labels[] = {"ch.sensor", "Hits",  "Hits/sec", "X",	   "Y",		  "min X", "max X",
								"width X",	 "var X", "min Y",	  "max Y", "width Y", "var Y", 0};
		for (const char **l = labels; *l; l++) {
			print_label(*l);
		}
		printf("\x1B[0m\n");

		int lh_start = lh == -1 ? 0 : lh;
		int lh_end = lh == -1 ? NUM_GEN2_LIGHTHOUSES : (lh + 1);

		for (int lh = lh_start; lh < lh_end; lh++) {
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
					printf("\x1B[2m");
				if (sensor == so->sensor_ct - 1)
					printf("\x1B[4m");

				uint8_t displaySensor = useRawSensorId ? get_raw_sensor_id(so, sensor) : sensor;

				printf("| %2d.%02d    |", ctx->bsd[lh].mode, displaySensor);

				print_int(time_stats[i][lh][sensor].hit_count);
				print(time_stats[i][lh][sensor].hz);
				for (int axis = 0; axis < 2; axis++) {
					FLT f = so->activations.angles[sensor][lh][axis];
					process_reading(i, lh, sensor, axis, f);
					print(f);
				}

				for (int axis = 0; axis < 2; axis++) {
					print(s[axis].MN);
					print(s[axis].MX);
					print(s[axis].MX - s[axis].MN);

					FLT var;
					variance_measure_calc(&s[axis].variance, &var);
					print(var);
				}

				printf("\x1B[0m");
				printf("\r\n\33[2K");
			}
			printf("\33[2K\r\n");
		}
	}

	if (window_cols != -1 && false) {
		gotoxy(0, window_rows - 10 - 1);
		printf("=== Log ===\n");
		for (int i = 0; i < 10; i++) {
			char *line = lines[(lines_idx + i) % 10];
			if (line != 0)
				printf("\33[2K\r %s\n", line);
		}
	}

	needsRedraw = false;
}

void light_fn(SurviveObject *so, int sensor_id, int acode, int timeinsweep, survive_timecode timecode,
			  survive_timecode length, uint32_t lh) {
	survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lh);
	if (needsRedraw)
		redraw(so->ctx);
}

void imu_fn(SurviveObject *so, int mode, const FLT *accelgyro, survive_timecode timecode, int id) {
	variance_measure_add(&imu_variance, accelgyro);

	size_t idx = 0;
	for (idx = 0; idx < so->ctx->objs_ct && so->ctx->objs[idx] != so; idx++)
		;

	imu_time_stats[idx].hit_count++;

	double time_since_start =
		survive_timecode_difference(timecode, imu_time_stats[idx].hz_start) / (double)so->timebase_hz;
	struct SurviveContext *ctx = so->ctx;

	imu_time_stats[idx].hz_count++;
	if (time_since_start > 3. || imu_time_stats[idx].hz_start == 0) {
		if (imu_time_stats[idx].hz_start != 0)
			imu_time_stats[idx].hz = imu_time_stats[idx].hz_count / time_since_start;
		imu_time_stats[idx].hz_count = 0;
		imu_time_stats[idx].hz_start = timecode;
		variance_measure_reset(&imu_variance);
	}

	survive_default_imu_process(so, mode, accelgyro, timecode, id);
}

void info_fn(SurviveContext *ctx, SurviveLogLevel logLevel, const char *fault) {
	free(lines[lines_idx % 10]);
	lines[lines_idx % 10] = new_str(fault);
	lines_idx++;
	redraw(ctx);
}

static void inc_lh(SurviveContext *ctx) {
	do {
		lh++;
	} while (lh < NUM_GEN2_LIGHTHOUSES && !ctx->bsd[lh].OOTXSet);
	if (lh == NUM_GEN2_LIGHTHOUSES)
		lh = -1;
}

void *KBThread(void *user) {
	SurviveContext *ctx = user;

	while (keepRunning) {
		int c = tolower(getchar());
		int err_clear = system("clear");
		(void)err_clear;

		if (c == 'l') {
			inc_lh(ctx);
		} else if (c == 'q') {
			keepRunning = false;
		} else if (c == 'r') {
			useRawSensorId = !useRawSensorId;
		}

		if (surviveIsDone) {
			redraw(ctx);
		} else {
			needsRedraw = true;
			if (c == 10) {
				inc_lh(ctx);
			}
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
	survive_install_sweep_angle_fn(ctx, sweep_fn);
	survive_install_angle_fn(ctx, angle_fn);
	survive_install_printf_fn(ctx, printf_fn);
	survive_install_log_fn(ctx, info_fn);
	survive_install_imu_fn(ctx, imu_fn);
	survive_install_light_fn(ctx, light_fn);
	survive_startup(ctx);

	int clear_err = system("clear");
	(void)clear_err;

	og_thread_t kbThread = OGCreateThread(KBThread, "kb-thread", ctx);

	while (keepRunning && survive_poll(ctx) == 0) {
		FLT this_time = OGGetAbsoluteTime();
		if (this_time > last_redraw + .03) {
			needsRedraw = true;
			last_redraw = this_time;
			redraw(ctx);		       
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
