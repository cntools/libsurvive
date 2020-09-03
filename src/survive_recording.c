// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <survive.h>

#include <stdio.h>
#include <stdlib.h>
#include <survive.h>

#include <assert.h>
#include <errno.h>
#include <string.h>

#include <inttypes.h>

#include "survive_recording.h"

#include "survive_config.h"
#include "survive_default_devices.h"

#include "ctype.h"
#include "os_generic.h"
#include "stdarg.h"

#include "survive_gz.h"

STATIC_CONFIG_ITEM(PLAYBACK_RECORD_RAWLIGHT, "record-rawlight", 'i', "Whether or not to output raw light data", 1)
STATIC_CONFIG_ITEM(PLAYBACK_RECORD_IMU, "record-imu", 'i', "Whether or not to output imu data", 1)
STATIC_CONFIG_ITEM(PLAYBACK_RECORD_CAL_IMU, "record-cal-imu", 'i', "Whether or not to output calibrated imu data", 0)
STATIC_CONFIG_ITEM(PLAYBACK_RECORD_ANGLE, "record-angle", 'i', "Whether or not to output angle data", 1)


STATIC_CONFIG_ITEM(RECORD, "record", 's', "File to record to if you wish to make a recording.", "")
STATIC_CONFIG_ITEM(RECORD_STDOUT, "record-stdout", 'i', "Whether or not to dump recording data to stdout", 0)
  
typedef struct SurviveRecordingData {
	SurviveContext *ctx;
	bool alwaysWriteStdOut;
	bool writeRawLight;
        bool writeIMU;
		bool writeCalIMU;
		bool writeAngle;
		gzFile output_file;
} SurviveRecordingData;

static void write_to_output_raw(SurviveRecordingData *recordingData, const char *string, int len) {
	if (recordingData->output_file) {
		gzwrite(recordingData->output_file, string, len);
	}

	if (recordingData->alwaysWriteStdOut) {
		fwrite(string, 1, len, stdout);
	}
}

#ifdef SURVIVE_HEX_FLOATS
#define FLT_PRINTF "%0.6a "
#else
#define FLT_PRINTF "%0.6f "
#endif

static void write_to_output(SurviveRecordingData *recordingData, const char *format, ...) {
	if (!recordingData) {
		return;
	}

	double ts = survive_run_time(recordingData->ctx);

	if (recordingData->output_file) {
		va_list args;
		va_start(args, format);
		gzprintf(recordingData->output_file, FLT_PRINTF, ts);
		gzvprintf(recordingData->output_file, format, args);

		va_end(args);
	}

	if (recordingData->alwaysWriteStdOut) {
		va_list args;
		va_start(args, format);
		fprintf(stdout, FLT_PRINTF, ts);
		vfprintf(stdout, format, args);
		va_end(args);
	}
}
void survive_recording_config_process(SurviveObject *so, char *ct0conf, int len) {
	SurviveRecordingData *recordingData = so->ctx ? so->ctx->recptr : 0;
	if (recordingData == 0 || len < 0)
		return;

	char *buffer = SV_CALLOC(1, len + 1);
	memcpy(buffer, ct0conf, len);
	for (int i = 0; i < len; i++)
		if (buffer[i] == '\n' || buffer[i] == '\r')
			buffer[i] = ' ';

	write_to_output(recordingData, "%s CONFIG ", so->codename);
	write_to_output_raw(recordingData, buffer, len);

	write_to_output_raw(recordingData, "\r\n", 2);

	free(buffer);
}

void survive_recording_lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *lh_pose) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData,
					"%d LH_POSE " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n",
					lighthouse, lh_pose->Pos[0], lh_pose->Pos[1], lh_pose->Pos[2], lh_pose->Rot[0], lh_pose->Rot[1],
					lh_pose->Rot[2], lh_pose->Rot[3]);
}
void survive_recording_velocity_process(SurviveObject *so, uint8_t lighthouse, const SurviveVelocity *pose) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData,
					"%s VELOCITY " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n",
					so->codename, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->AxisAngleRot[0],
					pose->AxisAngleRot[1], pose->AxisAngleRot[2]);
}
void survive_recording_raw_pose_process(SurviveObject *so, uint8_t lighthouse, const SurvivePose *pose) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(
		recordingData, "%s POSE " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n",
		so->codename, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void survive_recording_external_velocity_process(SurviveContext *ctx, const char *name, const SurviveVelocity *pose) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData,
					"%s EXTERNAL_VELOCITY " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n",
					name, pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->AxisAngleRot[0], pose->AxisAngleRot[1],
					pose->AxisAngleRot[2]);
}

void survive_recording_external_pose_process(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(
		recordingData,
		"%s EXTERNAL_POSE " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF "\r\n", name,
		pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void survive_recording_info_process(SurviveContext *ctx, const char *fault) {
	SurviveRecordingData *recordingData = ctx->recptr;
	if (recordingData == 0)
		return;

	write_to_output(recordingData, "INFO LOG %s\r\n", fault);
}

void survive_recording_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode, bool ootx,
									bool gen) {
	SurviveRecordingData *recordingData = so->ctx->recptr;

	const char* dev = so->codename;
	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	write_to_output(recordingData, SYNC_PRINTF, SYNC_PRINTF_ARGS);
}

void survive_recording_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
										   survive_timecode timecode, int8_t plane, FLT angle) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	const char *dev = so->codename;
	write_to_output(recordingData, SWEEP_ANGLE_PRINTF, SWEEP_ANGLE_PRINTF_ARGS);
}

void survive_recording_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
									 survive_timecode timecode, bool flag) {
	SurviveRecordingData *recordingData = so->ctx->recptr;

	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	if (!recordingData->writeAngle)
		return;

	const char *dev = so->codename;
	write_to_output(recordingData, SWEEP_PRINTF, SWEEP_PRINTF_ARGS);
}

void survive_recording_button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
									  const enum SurviveAxis *axisId, const float *axisVals) {
	SurviveRecordingData *recordingData = so->ctx->recptr;

	if (!recordingData) {
		return;
	}

	const char *dev = so->codename;
	write_to_output(recordingData, "%s BUTTON %u %u\r\n", dev, eventType, buttonId);
}
void survive_recording_angle_process(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode, FLT length,
									 FLT angle, uint32_t lh) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (!recordingData || !recordingData->writeAngle) {
		return;
	}

	write_to_output(recordingData, "%s A %d %d %u " FLT_PRINTF FLT_PRINTF "%u\r\n", so->codename, sensor_id, acode,
					timecode, length, angle, lh);
}

void survive_recording_lightcap(SurviveObject *so, LightcapElement *le) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (recordingData->writeRawLight) {
		write_to_output(recordingData, "%s C %d %u %u\r\n", so->codename, le->sensor_id, le->timestamp, le->length);
	}
}

void survive_recording_light_process(struct SurviveObject *so, int sensor_id, int acode, int timeinsweep,
									 uint32_t timecode, uint32_t length, uint32_t lh) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (!recordingData->writeAngle) {
	  return;
	}
	
	if (acode == -1) {
		write_to_output(recordingData, "%s S %d %d %d %u %u %u\r\n", so->codename, sensor_id, acode, timeinsweep,
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

	write_to_output(recordingData, "%s %s %s %d %d %d %u %u %u\r\n", so->codename, LH_ID, LH_Axis, sensor_id, acode,
					timeinsweep, timecode, length, lh);
}

void survive_recording_imu_process(struct SurviveObject *so, int mask, FLT *accelgyro, uint32_t timecode, int id) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (!recordingData->writeCalIMU) {
		return;
	}

	write_to_output(recordingData,
					"%s I %d %u " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF
					" " FLT_PRINTF FLT_PRINTF FLT_PRINTF "%d\r\n",
					so->codename, mask, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4],
					accelgyro[5], accelgyro[6], accelgyro[7], accelgyro[8], id);
}

void survive_recording_raw_imu_process(struct SurviveObject *so, int mask, FLT *accelgyro, uint32_t timecode, int id) {
	SurviveRecordingData *recordingData = so->ctx->recptr;
	if (recordingData == 0)
		return;

	if (!recordingData->writeIMU) {
		return;
	}

	write_to_output(recordingData,
					"%s i %d %u " FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF FLT_PRINTF
					" " FLT_PRINTF FLT_PRINTF FLT_PRINTF "%d\r\n",
					so->codename, mask, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4],
					accelgyro[5], accelgyro[6], accelgyro[7], accelgyro[8], id);
}

void survive_destroy_recording(SurviveContext *ctx) {
	if (ctx->recptr) {
		gzclose(ctx->recptr->output_file);
		free(ctx->recptr);
		ctx->recptr = 0;
	}
}

void survive_record_config(SurviveContext *ctx, const char *tag, uint8_t type, void *user) {
	char buf[128];
	survive_config_as_str(ctx, buf, sizeof(buf), tag, "");
	write_to_output(ctx->recptr, "OPTION %s %c %s\n", tag, type, buf);
}

void survive_install_recording(SurviveContext *ctx) {
	const char *dataout_file = survive_configs(ctx, "record", SC_GET, "");
	int record_to_stdout = survive_configi(ctx, "record-stdout", SC_GET, 0);

	if (strlen(dataout_file) > 0 || record_to_stdout) {
		ctx->recptr = SV_CALLOC(1, sizeof(struct SurviveRecordingData));
		ctx->recptr->ctx = ctx;
		if (strlen(dataout_file) > 0) {
			if (strstr(dataout_file, ".pcap")) {
				int (*usb_driver)(SurviveContext *) = (int (*)(SurviveContext *))GetDriver("DriverRegUSBMon_Record");
				if (usb_driver) {
					usb_driver(ctx);
					return;
				}
				SV_WARN("Playback file %s is a USB packet capture, but the usbmon playback driver does not exist.",
						dataout_file);
				return;
			} else {

				bool useCompression = strncmp(dataout_file + strlen(dataout_file) - 3, ".gz", 3) == 0;

				ctx->recptr->output_file = gzopen(dataout_file, useCompression ? "w" : "wT");
				if (ctx->recptr->output_file == 0) {
					SV_INFO("Could not open %s for writing", dataout_file);
					free(ctx->recptr);
					ctx->recptr = 0;
					return;
				}
				SV_INFO("Recording to '%s' Compression: %d", dataout_file, useCompression);
			}
		}

		ctx->recptr->alwaysWriteStdOut = record_to_stdout;
		if (record_to_stdout) {
			SV_INFO("Recording to stdout");
		}

		ctx->recptr->writeRawLight = survive_configi(ctx, "record-rawlight", SC_GET, 1);
		ctx->recptr->writeIMU = survive_configi(ctx, "record-imu", SC_GET, 1);
		ctx->recptr->writeCalIMU = survive_configi(ctx, "record-cal-imu", SC_GET, 0);
		ctx->recptr->writeAngle = survive_configi(ctx, "record-angle", SC_GET, 1);
	}

	survive_config_iterate(ctx, survive_record_config, ctx->recptr);
}
