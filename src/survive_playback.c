// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.

#include <stdio.h>
#include <stdlib.h>
#include <survive.h>

#include <string.h>
#include <sys/time.h>

#include "survive_config.h"
#include "survive_default_devices.h"

#include "os_generic.h"
#include "stdarg.h"

typedef struct SurviveRecordingData {
	bool alwaysWriteStdOut;
	FILE *output_file;
} SurviveRecordingData;

static double timestamp_in_us() {
	static double start_time_us = 0;
	if (start_time_us == 0.)
		start_time_us = OGGetAbsoluteTime();
	return OGGetAbsoluteTime() - start_time_us;
}

static void write_to_output(SurviveRecordingData *recordingData, const char *format, ...) {
	double ts = timestamp_in_us();

	if (recordingData->output_file) {
		va_list args;
		va_start(args, format);
		fprintf(recordingData->output_file, "%0.6f ", ts);
		vfprintf(recordingData->output_file, format, args);
		va_end(args);
	}

	if (recordingData->alwaysWriteStdOut) {
		va_list args;
		va_start(args, format);
		fprintf(stdout, "%0.6f ", ts);
		vfprintf(stdout, format, args);
		va_end(args);
	}
}
void survive_recording_config_process(SurviveObject *so, char *ct0conf, int len) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	char *buffer = malloc(len);
	memcpy(buffer, ct0conf, len);
	for (int i = 0; i < len; i++)
		if (buffer[i] == '\n')
			buffer[i] = ' ';

	write_to_output(recordingData, "%s CONFIG %.*s\n", so->codename, len, buffer);
}

void survive_recording_lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *lh_pose,
										  SurvivePose *obj) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData, "%d LH_POSE %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", lighthouse,
					lh_pose->Pos[0], lh_pose->Pos[1], lh_pose->Pos[2], lh_pose->Rot[0], lh_pose->Rot[1],
					lh_pose->Rot[2], lh_pose->Rot[3]);
}
void survive_recording_raw_pose_process(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData, "%s POSE %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", so->codename, pose->Pos[0],
					pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void survive_recording_info_process(SurviveContext *ctx, const char *fault) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData, "INFO LOG %s\n", fault);
}

void survive_recording_angle_process(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode, FLT length,
									 FLT angle, uint32_t lh) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData, "%s A %d %d %u %0.6f %0.6f %u\n", so->codename, sensor_id, acode, timecode, length,
					angle, lh);
}

void survive_recording_light_process(struct SurviveObject *so, int sensor_id, int acode, int timeinsweep,
									 uint32_t timecode, uint32_t length, uint32_t lh) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (acode == -1) {
		write_to_output(recordingData, "%s S %d %d %d %u %u %u\n", so->codename, sensor_id, acode, timeinsweep,
						timecode, length, lh);
		return;
	}

	const char *LH_ID = 0;
	const char *LH_Axis = 0;

	switch (acode) {
	case 0:
	case 2:
		LH_ID = "L";
		LH_Axis = "X";
		break;
	case 1:
	case 3:
		LH_ID = "L";
		LH_Axis = "Y";
		break;
	case 4:
	case 6:
		LH_ID = "R";
		LH_Axis = "X";
		break;
	case 5:
	case 7:
		LH_ID = "R";
		LH_Axis = "Y";
		break;
	}
	write_to_output(recordingData, "%s %s %s %d %d %d %u %u %u\n", so->codename, LH_ID, LH_Axis, sensor_id, acode,
					timeinsweep, timecode, length, lh);
}

void survive_recording_imu_process(struct SurviveObject *so, int mask, FLT *accelgyro, uint32_t timecode, int id) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData, "%s I %d %u %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %d\n", so->codename, mask, timecode,
					accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id);
}

struct SurvivePlaybackData {
	SurviveContext *ctx;
	const char *playback_dir;
	FILE *playback_file;
	int lineno;

	FLT time_factor;
	double next_time_us;
};
typedef struct SurvivePlaybackData SurvivePlaybackData;

static int parse_and_run_imu(const char *line, SurvivePlaybackData *driver) {
	char dev[10];
	int timecode = 0;
	FLT accelgyro[6];
	int mask;
	int id;

	int rr = sscanf(line, "%s I %d %d " FLT_format " " FLT_format " " FLT_format " " FLT_format " " FLT_format
						  " " FLT_format "%d",
					dev, &mask, &timecode, &accelgyro[0], &accelgyro[1], &accelgyro[2], &accelgyro[3], &accelgyro[4],
					&accelgyro[5], &id);

	if (rr != 10) {
		fprintf(stderr, "Warning:  On line %d, only %d values read: '%s'\n",
				driver->lineno, rr, line);
		return -1;
	}

	SurviveObject *so = survive_get_so_by_name(driver->ctx, dev);
	if (!so) {
		static bool display_once = false;
		SurviveContext *ctx = driver->ctx;
		if (display_once == false) {
			SV_ERROR("Could not find device named %s from lineno %d\n", dev, driver->lineno);
		}
		display_once = true;
		return -1;
	}

	driver->ctx->imuproc(so, mask, accelgyro, timecode, id);
	return 0;
}

static int parse_and_run_lightcode(const char *line,
								   SurvivePlaybackData *driver) {
	char lhn[10];
	char axn[10];
	char dev[10];
	uint32_t timecode = 0;
	int sensor_id = 0;
	int acode = 0;
	int timeinsweep = 0;
	uint32_t length = 0;
	uint32_t lh = 0;

	int rr = sscanf(line, "%8s %8s %8s %u %d %d %d %u %u\n", dev, lhn, axn, &sensor_id, &acode, &timeinsweep, &timecode,
					&length, &lh);

	if (rr != 9) {
		fprintf(stderr, "Warning:  On line %d, only %d values read: '%s'\n",
				driver->lineno, rr, line);
		return -1;
	}

	SurviveObject *so = survive_get_so_by_name(driver->ctx, dev);
	if (!so) {
		static bool display_once = false;
		SurviveContext *ctx = driver->ctx;
		if (display_once == false) {
			SV_ERROR("Could not find device named %s from lineno %d\n", dev, driver->lineno);
		}
		display_once = true;

		return -1;
	}

	driver->ctx->lightproc(so, sensor_id, acode, timeinsweep, timecode, length, lh);
	return 0;
}

static int playback_poll(struct SurviveContext *ctx, void *_driver) {
	SurvivePlaybackData *driver = _driver;
	FILE *f = driver->playback_file;

	if (f && !feof(f) && !ferror(f)) {
		int i;
		driver->lineno++;
		char *line;

		if (driver->next_time_us == 0) {
			char *buffer;
			size_t n = 0;
			ssize_t r = getdelim(&line, &n, ' ', f);
			if (r <= 0)
				return 0;

			if (sscanf(line, "%lf", &driver->next_time_us) != 1) {
				free(line);
				return 0;
			}
			free(line);
			line = 0;
		}

		if (driver->next_time_us * driver->time_factor > timestamp_in_us())
			return 0;
		driver->next_time_us = 0;

		char *buffer;
		size_t n = 0;
		ssize_t r = getline(&line, &n, f);
		if (r <= 0)
			return 0;

		char dev[10];
		char op[10];
		if (sscanf(line, "%8s %8s", dev, op) < 2)
			return 0;

		if ((op[0] != 'R' && op[0] != 'L' && op[0] != 'I') || op[1] != 0)
			return 0;

		switch (op[0]) {
		case 'L':
		case 'R':
			parse_and_run_lightcode(line, driver);
			break;
		case 'I':
			parse_and_run_imu(line, driver);
			break;
		}

		free(line);
	} else {
		if (f) {
			fclose(driver->playback_file);
		}
		driver->playback_file = 0;
		return -1;
	}

	return 0;
}

static int playback_close(struct SurviveContext *ctx, void *_driver) {
	SurvivePlaybackData *driver = _driver;
	if (driver->playback_file)
		fclose(driver->playback_file);
	driver->playback_file = 0;

	return 0;
}

void survive_install_recording(SurviveContext *ctx) {
	const char *dataout_file = survive_configs(ctx, "record", SC_SETCONFIG, "");
	int record_to_stdout = survive_configi(ctx, "record-stdout", SC_SETCONFIG, 0);

	if (strlen(dataout_file) > 0 || record_to_stdout) {
		ctx->recptr = calloc(1, sizeof(struct SurviveRecordingData));

		ctx->recptr->output_file = fopen(dataout_file, "w");
		if (ctx->recptr->output_file == 0 && !record_to_stdout) {
			SV_INFO("Could not open %s for writing\n", dataout_file);
			free(ctx->recptr);
			ctx->recptr = 0;
			return;
		}
		SV_INFO("Recording to '%s'", dataout_file);
		ctx->recptr->alwaysWriteStdOut = record_to_stdout;
		if (record_to_stdout) {
			SV_INFO("Recording to stdout");
		}
	}
}

int DriverRegPlayback(SurviveContext *ctx) {
	const char *playback_file = survive_configs(ctx, "playback", SC_SETCONFIG, "");

	if (strlen(playback_file) == 0) {
		return 0;
	}

	SurvivePlaybackData *sp = calloc(1, sizeof(SurvivePlaybackData));
	sp->ctx = ctx;
	sp->playback_dir = playback_file;
	sp->time_factor = survive_configf(ctx, "playback-factor", SC_SETCONFIG, 1.f);

	printf("%s\n", playback_file);

	sp->playback_file = fopen(playback_file, "r");
	if (sp->playback_file == 0) {
		fprintf(stderr, "Could not open playback events file %s",
				playback_file);
		return -1;
	}

	SV_INFO("Using playback file '%s' with timefactor of %f", playback_file, sp->time_factor);
	SurviveObject *hmd = survive_create_hmd(ctx, "Playback", sp);
	SurviveObject *wm0 = survive_create_wm0(ctx, "Playback", sp, 0);
	SurviveObject *wm1 = survive_create_wm1(ctx, "Playback", sp, 0);
	SurviveObject *tr0 = survive_create_tr0(ctx, "Playback", sp);
	SurviveObject *ww0 = survive_create_ww0(ctx, "Playback", sp);

	SurviveObject *objs[] = {hmd, wm0, wm1, tr0, ww0, 0};

	FLT time;
	while (!feof(sp->playback_file) && !ferror(sp->playback_file)) {
		char *line = 0;
		size_t n;
		ssize_t r = getline(&line, &n, sp->playback_file);

		if (r <= 0)
			continue;

		char dev[10];
		char command[10];

		if (sscanf(line, "%lf %s %s", &time, dev, command) != 3) {
			break;
		}

		if (strcmp(command, "CONFIG") == 0) {
			char *configStart = line;

			// Skip three spaces
			for (int i = 0; i < 3; i++) {
				while (*(++configStart) != ' ')
					;
			}
			size_t len = strlen(configStart);

			for (SurviveObject **obj = objs; *obj; obj++) {
				if (*obj && strcmp(dev, (*obj)->codename) == 0 && ctx->configfunction(*obj, configStart, len) == 0) {
					SV_INFO("Found %s in playback file...", dev);
					survive_add_object(ctx, *obj);
					*obj = 0;
				}
			}
		}
	}

	for (SurviveObject **obj = objs; *obj; obj++) {
		if (*obj) {
			free(*obj);
		}
	}
	fseek(sp->playback_file, 0, SEEK_SET); // same as rewind(f);

	survive_add_driver(ctx, sp, playback_poll, playback_close, 0);
	return 0;
}

REGISTER_LINKTIME(DriverRegPlayback);
