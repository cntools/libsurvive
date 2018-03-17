// Data recorder mod with GUI showing light positions.

#ifdef __linux__
#include <unistd.h>
#endif

#include <os_generic.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include <time.h>

#ifndef _MSC_VER
#include <sys/time.h>
#endif

#include "redist/os_generic.h"

struct SurviveContext *ctx;

FILE *output_file = 0;

double timestamp_in_us() {
	static double start_time_us = 0;
	if (start_time_us == 0)
		start_time_us = OGGetAbsoluteTime();
	return OGGetAbsoluteTime() - start_time_us;
}

void write_to_output(const char *format, ...) {
	va_list args;
	va_start(args, format);
	fprintf(output_file, "%0.6f ", timestamp_in_us());
	vfprintf(output_file, format, args);

	va_end(args);
}
int my_config_process(SurviveObject *so, char *ct0conf, int len) {
	char *buffer = malloc(len);
	memcpy(buffer, ct0conf, len);
	for (int i = 0; i < len; i++)
		if (buffer[i] == '\n')
			buffer[i] = ' ';

	write_to_output("%s CONFIG ", so->codename);
	fwrite(buffer, 1, len, output_file);
	fwrite("\n", 1, 1, output_file);
	return survive_default_htc_config_process(so, ct0conf, len);
}

void my_lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *pose) {
	survive_default_lighthouse_pose_process(ctx, lighthouse, pose);
	write_to_output("%d LH_POSE %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", lighthouse, pose->Pos[0], pose->Pos[1],
					pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}
void my_raw_pose_process(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	survive_default_raw_pose_process(so, lighthouse, pose);
	write_to_output("%s POSE %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", so->codename, pose->Pos[0], pose->Pos[1],
					pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void my_info_process(SurviveContext *ctx, const char *fault) { write_to_output("INFO LOG %s\n", fault); }
void my_angle_process(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle,
					  uint32_t lh) {
	survive_default_angle_process(so, sensor_id, acode, timecode, length, angle, lh);
	write_to_output("%s A %d %d %u %0.6f %0.6f %u\n", so->codename, sensor_id, acode, timecode, length, angle, lh);
}

void my_light_process(struct SurviveObject *so, int sensor_id, int acode,
					  int timeinsweep, uint32_t timecode, uint32_t length,
					  uint32_t lh) {
	survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode,
								  length, lh);

	if (acode == -1) {
		write_to_output("%s S %d %d %d %u %u %u\n", so->codename, sensor_id, acode, timeinsweep, timecode, length, lh);
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
	write_to_output("%s %s %s %d %d %d %u %u %u\n", so->codename, LH_ID, LH_Axis, sensor_id, acode, timeinsweep,
					timecode, length, lh);
}

void my_imu_process(struct SurviveObject *so, int mask, FLT *accelgyro,
					uint32_t timecode, int id) {
	survive_default_imu_process(so, mask, accelgyro, timecode, id);
	write_to_output("%s I %d %u %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %d\n", so->codename, mask, timecode, accelgyro[0],
					accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id);
}

int main(int argc, char **argv) {
	if (argc > 1) {
		output_file = fopen(argv[1], "w");
		if (output_file == 0) {
			fprintf(stderr, "Could not open %s for writing", argv[1]);
			return -1;
		}
	} else {
		output_file = stdout;
	}

	ctx = survive_init_with_config_cb(0, my_config_process);

	survive_install_light_fn(ctx, my_light_process);
	survive_install_imu_fn(ctx, my_imu_process);
	survive_install_lighthouse_pose_fn(ctx, my_lighthouse_process);
	survive_install_raw_pose_fn(ctx, my_raw_pose_process);
	survive_install_angle_fn(ctx, my_angle_process);
	survive_install_info_fn(ctx, my_info_process);

	survive_cal_install(ctx);

	if (!ctx) {
		fprintf(stderr, "Fatal. Could not start\n");
		exit(1);
	}

	while (survive_poll(ctx) == 0) {
	}

	return 0;
}
