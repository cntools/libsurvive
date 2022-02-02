//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_config.h"
#include "survive_default_devices.h"
#include "survive_recording.h"
#include <assert.h>
#include <survive.h>

#include "math.h"
#include "string.h"
#include "survive_kalman_lighthouses.h"
#include "survive_kalman_tracker.h"
#include "survive_str.h"

#include "survive_private.h"

void survive_default_button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
									const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisValues) {
}

STATIC_CONFIG_ITEM(REPORT_IN_IMU, "report-in-imu", 'b', "Debug option to output poses in IMU space.", 0)
STATIC_CONFIG_ITEM(USE_EXTERNAL_LH, "use-external-lighthouse", 'b', "Use external lighthouse if available", 0)
void survive_default_imupose_process(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *imu2world) {
	static int report_in_imu = -1;
	if (report_in_imu == -1) {
		report_in_imu = survive_configi(so->ctx, REPORT_IN_IMU_TAG, SC_GET, 0);
	}

	SurvivePose head2world;
	so->OutPoseIMU = *imu2world;
	if (!report_in_imu) {
		ApplyPoseToPose(&head2world, imu2world, &so->head2imu);
	} else {
		head2world = *imu2world;
	}

	for (int i = 0; i < 7; i++)
		assert(!isnan(((FLT *)imu2world)[i]));

	SurviveContext *ctx = so->ctx;

	head2world.Pos[2] -= ctx->floor_offset;
	SURVIVE_INVOKE_HOOK_SO(pose, so, timecode, &head2world);
}
void survive_default_pose_process(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose) {
	so->OutPose = *pose;
	so->OutPose_timecode = timecode;
	survive_recording_raw_pose_process(so, timecode, pose);
}
void survive_default_velocity_process(SurviveObject *so, survive_long_timecode timecode,
									  const SurviveVelocity *velocity) {
	survive_recording_velocity_process(so, timecode, velocity);
	so->velocity = *velocity;
	so->velocity_timecode = timecode;
}

void survive_default_external_velocity_process(SurviveContext *ctx, const char *name, const SurviveVelocity *vel) {
	for (int i = 0; i < ctx->objs_ct; i++) {
		SurviveObject *so = ctx->objs[i];
		if (strcmp(so->serial_number, name) == 0) {
			SurviveVelocity out = survive_kalman_tracker_velocity(so->tracker);

			FLT diff[] = {dist3d(out.Pos, vel->Pos), dist3d(out.AxisAngleRot, vel->AxisAngleRot)};
			SV_DATA_LOG("external_velocity_diff", diff, 2);
			break;
		}
	}

	survive_recording_external_velocity_process(ctx, name, vel);
}
static inline bool check_str(const char* blacklist, const char* name) {
	const char* s = strstr(blacklist, name);
	if(s) {
		return (s[strlen(name)] == 0 || s[strlen(name)] == ',') &&
			(s == blacklist || s[-1] == ',');
	}
	return false;
}

static inline void calculate_external2world(SurviveContext *ctx) {
	SurvivePose externalLH[NUM_GEN2_LIGHTHOUSES] = { 0 };
	SurvivePose bsds[NUM_GEN2_LIGHTHOUSES] = {0};
	int num_poses = 0;
	for(int i = 0;i < SURVIVE_ARRAY_SIZE(ctx->private_members->ExternalPoses) && ctx->private_members->ExternalPoses->name[0] != 0;i++) {
		for (int j = 0; j < ctx->activeLighthouses; j++) {
			if(!ctx->bsd[j].PositionSet) continue;

			char buf[32] = {0};
			snprintf(buf, 32, "LHB-%08X", ctx->bsd[j].BaseStationID);
			if (strcmp(buf, ctx->private_members->ExternalPoses[i].name) == 0) {
				externalLH[num_poses] = ctx->private_members->ExternalPoses[i].pose;
				bsds[num_poses] = *survive_get_lighthouse_true_position(ctx, j);
				bsds[num_poses].Pos[2] -= ctx->floor_offset;
				num_poses++;
			} else {
				snprintf(buf, 32, "previous_LH%d", ctx->bsd[j].mode);
				if (strcmp(buf, ctx->private_members->ExternalPoses[i].name) == 0) {
					externalLH[num_poses] = ctx->private_members->ExternalPoses[i].pose;
					bsds[num_poses] = *survive_get_lighthouse_true_position(ctx, j);
					bsds[num_poses].Pos[2] -= ctx->floor_offset;
					num_poses++;
				}
			}
		}
	}

	if (num_poses == 0) {
		ctx->private_members->external2world = (SurvivePose) { .Rot = { 1 }};
	} else {
		KabschPoses(&ctx->private_members->external2world, externalLH, bsds, num_poses);
		// Kabsch(&ctx->private_members->external2world, ptsExtLH.data, ptsWorldLH.data, num_pairs);
		SV_VERBOSE(110, "external2world " SurvivePose_format,
				   SURVIVE_POSE_EXPAND(ctx->private_members->external2world));
		survive_recording_write_to_output(ctx->recptr, "EXTERNAL_TO_WORLD " SurvivePose_format "\n", SURVIVE_POSE_EXPAND(ctx->private_members->external2world));
	}
}

const SurvivePose* survive_external_to_world(const SurviveContext *ctx) {
	return &ctx->private_members->external2world;
}

static inline void insert_external_pose(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	bool isLH = strncmp(name, "LHB-", 4) == 0 ||
				strncmp(name, "previous_LH", strlen("previous_LH")) == 0;

	for(int i = 0;i < SURVIVE_ARRAY_SIZE(ctx->private_members->ExternalPoses);i++) {
		if(ctx->private_members->ExternalPoses[i].name[0] == 0) {
			strncpy(ctx->private_members->ExternalPoses[i].name, name, 16);
		}
		if(strncmp(name, ctx->private_members->ExternalPoses[i].name, 16) == 0) {
			ctx->private_members->ExternalPoses[i].pose = *pose;
			break;
		}
	}

	if(isLH) {
		calculate_external2world(ctx);
	}
}

void survive_default_external_pose_process(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	if (strncmp(name, "LHB", 3) == 0) {
		bool useExternal = survive_configb(ctx, USE_EXTERNAL_LH_TAG, SC_GET, 0);
		for (int i = 0; i < ctx->activeLighthouses && useExternal; i++) {
			char buf[32] = {0};
			snprintf(buf, 32, "LHB-%08X", ctx->bsd[i].BaseStationID);
			if (strcmp(buf, name) == 0) {
				SURVIVE_INVOKE_HOOK(raw_lighthouse_pose, ctx, i, pose);
			}
		}
	}
	for (int i = 0; i < ctx->objs_ct; i++) {
		SurviveObject *so = ctx->objs[i];
		if (strcmp(so->serial_number, name) == 0) {
			SurvivePose out = {0};
			survive_kalman_tracker_predict(so->tracker, so->tracker->model.t, &out);
			if (!quatiszero(out.Rot)) {
				SurvivePose head2world;
				ApplyPoseToPose(&head2world, &out, &so->head2imu);

				FLT diff[] = {dist3d(head2world.Pos, pose->Pos), quatdifference(head2world.Rot, pose->Rot)};
				SV_DATA_LOG("external_diff", diff, 2);
			}
			break;
		}
	}

	const char *blacklist = survive_configs(ctx, "blacklist-devs", SC_GET, 0);
	if(check_str(blacklist, name)) {
		return;
	}

	insert_external_pose(ctx, name, pose);

	survive_recording_external_pose_process(ctx, name, pose);
}

void survive_default_ootx_received_process(struct SurviveContext *ctx, uint8_t bsd_idx) {
	config_set_lighthouse(ctx->lh_config, &ctx->bsd[bsd_idx], bsd_idx);
	survive_kalman_lighthouse_ootx(ctx->bsd[bsd_idx].tracker);

	survive_recording_write_to_output(ctx->recptr, "LH_UP %d " Point3_format "\n", ctx->bsd[bsd_idx].mode,
									  LINMATH_VEC3_EXPAND(ctx->bsd[bsd_idx].accel));
	config_save(ctx);
}

void survive_default_raw_lighthouse_pose_process(SurviveContext *ctx, uint8_t lighthouse,
											 const SurvivePose *lighthouse_pose) {
	bool notSet = ctx->bsd[lighthouse].PositionSet == 0;
	if (lighthouse_pose) {
		for (int i = 0; i < 3; i++)
			assert(isfinite(lighthouse_pose->Pos[i]));
		for (int i = 0; i < 4; i++)
			assert(isfinite(lighthouse_pose->Rot[i]));

		if (ctx->bsd[lighthouse].PositionSet) {
			FLT d = dist3d(lighthouse_pose->Pos, ctx->bsd[lighthouse].Pose.Pos);
			if (d < ctx->settings.lh_max_update || d > ctx->settings.lh_max_nudge_distance) {
				ctx->bsd[lighthouse].true_pos = ctx->bsd[lighthouse].Pose = *lighthouse_pose;
			} else {
				ctx->bsd[lighthouse].old_pos = ctx->bsd[lighthouse].Pose;
				ctx->bsd[lighthouse].true_pos = *lighthouse_pose;
				ctx->bsd[lighthouse].old_pos_time = survive_run_time(ctx);
				ctx->bsd[lighthouse].true_pos_time =
					ctx->bsd[lighthouse].old_pos_time + d / ctx->settings.lh_update_velocity;

				SV_VERBOSE(100, "LH %d moving %f over %f seconds (ID: %08x, mode: %2d) " SurvivePose_format, lighthouse,
						   d, d / ctx->settings.lh_update_velocity, (unsigned)ctx->bsd[lighthouse].BaseStationID,
						   ctx->bsd[lighthouse].mode, SURVIVE_POSE_EXPAND(*lighthouse_pose));
			}
		} else {
			ctx->bsd[lighthouse].true_pos = ctx->bsd[lighthouse].Pose = *lighthouse_pose;
			ctx->bsd[lighthouse].PositionSet = 1;
		}
		survive_kalman_lighthouse_update_position(ctx->bsd[lighthouse].tracker, lighthouse_pose);
	} else {
		ctx->bsd[lighthouse].PositionSet = 0;
	}

	config_set_lighthouse(ctx->lh_config, &ctx->bsd[lighthouse], lighthouse);
	config_save(ctx);

	LinmathPoint3d up = {ctx->bsd[lighthouse].accel[0], ctx->bsd[lighthouse].accel[1], ctx->bsd[lighthouse].accel[2]};
	normalize3d(up, up);
	LinmathPoint3d err;
	quatrotatevector(err, lighthouse_pose->Rot, up);

	if (notSet || ctx->log_level >= 100) {
		SV_VERBOSE(10, "Position found for LH %d(ID: %08x, mode: %2d, err: %f) " SurvivePose_format, lighthouse,
				   (unsigned)ctx->bsd[lighthouse].BaseStationID, ctx->bsd[lighthouse].mode, 1 - err[2],
				   SURVIVE_POSE_EXPAND(ctx->bsd[lighthouse].Pose));
		if (ctx->bsd[lighthouse].true_pos_time) {
			SV_VERBOSE(10, "                                                               " SurvivePose_format,
					   SURVIVE_POSE_EXPAND(*lighthouse_pose));
		}
	}

	SurvivePose external_pose = ctx->bsd[lighthouse].Pose;
	external_pose.Pos[2] -= ctx->floor_offset;
	calculate_external2world(ctx);

	SURVIVE_INVOKE_HOOK(lighthouse_pose, ctx, lighthouse, &external_pose);
}

SURVIVE_EXPORT FLT survive_get_floor_offset(const SurviveContext* ctx) { return ctx->floor_offset; }
SURVIVE_EXPORT void survive_set_floor_offset(SurviveContext* ctx, FLT floor_offset_meters) {
	ctx->floor_offset = floor_offset_meters;
	calculate_external2world(ctx);
	survive_configf(ctx, "floor-offset", SC_OVERRIDE | SC_SETCONFIG, floor_offset_meters);
	config_save(ctx);
}

void survive_default_lighthouse_pose_process(SurviveContext *ctx, uint8_t lighthouse,
											 const SurvivePose *lighthouse_pose) {
	survive_recording_lighthouse_process(ctx, lighthouse, lighthouse_pose);
}

STATIC_CONFIG_ITEM(SURVIVE_SERIALIZE_DEV_CONFIG, "serialize-device-config", 'b', "Serialize device config files", 0)

int survive_default_config_process(SurviveObject *so, char *ct0conf, int len) {
	survive_recording_config_process(so, ct0conf, len);
	so->conf = ct0conf;
	so->conf_cnt = len;

	int rtn = survive_load_htc_config_format(so, ct0conf, len);
	if (survive_configi(so->ctx, "serialize-device-config", SC_GET, 0) != 0) {
		for (int i = 0; i < 2; i++) {
			char raw_fname[128];
			sprintf(raw_fname, "%s_config.json", i == 0 ? so->serial_number : so->codename);
			FILE *f = fopen(raw_fname, "w");
			fwrite(ct0conf, len, 1, f);
			fclose(f);

			SurviveContext *ctx = so->ctx;
			SV_INFO("Wrote out %d bytes to %s", len, raw_fname);
		}
	}

	return rtn;
}

SURVIVE_EXPORT char *survive_export_config(SurviveObject *so) {
	cstring str = {0};
	str_append(&str, "{\n");
	str_append(&str, "    \"device_class\": \"generic_tracker\",\n");
	str_append(&str, "    \"imu\": {\n");
	str_append_printf(&str, "        \"acc_bias\": [ %f, %f, %f], \n", LINMATH_VEC3_EXPAND(so->acc_bias));
	str_append_printf(&str, "        \"acc_scale\": [ %f, %f, %f], \n", LINMATH_VEC3_EXPAND(so->acc_scale));
	str_append_printf(&str, "        \"gyro_bias\": [ %f, %f, %f], \n", LINMATH_VEC3_EXPAND(so->gyro_bias));
	str_append_printf(&str, "        \"gyro_scale\": [ %f, %f, %f], \n", LINMATH_VEC3_EXPAND(so->gyro_scale));
	str_append_printf(&str, "        \"position\": [ %f, %f, %f], \n", LINMATH_VEC3_EXPAND(so->imu2trackref.Pos));
	str_append(&str, "    }\n");
	str_append(&str, "    \"lighthouse_config\": {\n");
	str_append(&str, "        \"channelMap\": [\n");
	if (so->channel_map) {
		for (int i = 0; i < so->sensor_ct; i++) {
			str_append_printf(&str, "            %d,\n", so->channel_map[i]);
		}
	} else {
		for (int i = 0; i < so->sensor_ct; i++) {
			str_append_printf(&str, "            %d,\n", i);
		}
	}
	str_append(&str, "        ],\n");
	str_append(&str, "        \"modelNormals\": [\n");
	for (int i = 0; i < so->sensor_ct; i++) {
		str_append_printf(&str, "            [  %f, %f, %f ], \n", LINMATH_VEC3_EXPAND(so->sensor_normals + i * 3));
	}
	str_append(&str, "        ],\n");
	str_append(&str, "        \"modelPoints\": [\n");
	for (int i = 0; i < so->sensor_ct; i++) {
		str_append_printf(&str, "            [ %f, %f, %f ], \n", LINMATH_VEC3_EXPAND(so->sensor_locations + i * 3));
	}
	str_append(&str, "        ]\n");
	str_append(&str, "    }\n");
	str_append(&str, "}\n");

	return str.d;
}

// https://github.com/ValveSoftware/openvr/wiki/ImuSample_t
static void calibrate_acc(SurviveObject *so, FLT *agm) {
	agm[0] -= so->acc_bias[0];
	agm[1] -= so->acc_bias[1];
	agm[2] -= so->acc_bias[2];

	agm[0] *= so->acc_scale[0];
	agm[1] *= so->acc_scale[1];
	agm[2] *= so->acc_scale[2];
}

static void calibrate_gyro(SurviveObject *so, FLT *agm) {
	agm[0] -= so->gyro_bias[0];
	agm[1] -= so->gyro_bias[1];
	agm[2] -= so->gyro_bias[2];

	agm[0] *= so->gyro_scale[0];
	agm[1] *= so->gyro_scale[1];
	agm[2] *= so->gyro_scale[2];
}

void survive_default_raw_imu_process(SurviveObject *so, int mask, const FLT *accelgyromag, uint32_t timecode, int id) {
	SurviveContext *ctx = so->ctx;

	FLT agm[9] = {0};
	memcpy(agm, accelgyromag, sizeof(FLT) * 9);

    scale3d(agm, agm, so->raw_acc_scale);
    scale3d(agm + 3, agm + 3, so->raw_gyro_scale);

	calibrate_acc(so, agm);
	calibrate_gyro(so, agm + 3);

	survive_recording_raw_imu_process(so, mask, accelgyromag, timecode, id);

	SURVIVE_INVOKE_HOOK_SO(imu, so, 3, agm, timecode, id);
}
void survive_default_set_imu_scale_modes(SurviveObject *so, int gyro_scale_mode, int acc_scale_mode) {
    survive_recording_imu_scales(so, gyro_scale_mode, acc_scale_mode);

    const FLT gyro_scales[] = { 250, 500, 1000, 2000 };
    const FLT acc_scales[] = { 2, 4, 8, 16 };
    if(acc_scale_mode < 4) so->raw_acc_scale = acc_scales[acc_scale_mode] / (FLT)(1 << 15);
    if(gyro_scale_mode < 4) so->raw_gyro_scale = gyro_scales[gyro_scale_mode] / (FLT)(1 << 15) * LINMATHPI / 180.;

	SurviveContext *ctx = so->ctx;
	SV_VERBOSE(100, "Setting %s gyro scale %d(%f) acc scale %d(%f)", survive_colorize_codename(so), gyro_scale_mode,
			   so->raw_acc_scale, acc_scale_mode, so->raw_gyro_scale);
}
void survive_default_imu_process(SurviveObject *so, int mask, const FLT *accelgyromag, uint32_t timecode, int id) {
	survive_long_timecode longTimecode = SurviveSensorActivations_long_timecode_imu(&so->activations, timecode);
	PoserDataIMU imu = {
		.hdr = {.pt = POSERDATA_IMU, .timecode = longTimecode},
		.datamask = mask,
		.accel = {accelgyromag[0], accelgyromag[1], accelgyromag[2]},
		.gyro = {accelgyromag[3], accelgyromag[4], accelgyromag[5]},
		.mag = {accelgyromag[6], accelgyromag[7], accelgyromag[8]},
	};

	SurviveSensorActivations_add_imu(&so->activations, &imu);

	SurviveContext *ctx = so->ctx;

	SV_VERBOSE(300, "%s %s %x (%7.3f): " Point6_format, survive_colorize(so->codename), survive_colorize("IMU"),
			   timecode, longTimecode / 48000000., LINMATH_VEC3_EXPAND(imu.accel), LINMATH_VEC3_EXPAND(imu.gyro))
	survive_kalman_tracker_integrate_imu(so->tracker, &imu);
	SURVIVE_POSER_INVOKE(so, &imu);

	survive_recording_imu_process(so, mask, accelgyromag, timecode, id);
}
