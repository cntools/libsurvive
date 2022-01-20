

#define PRI_CHANNEL PRIu8
#define PRI_PLANE PRId8
#define PRI_FLAG PRIu8
#define PRI_GEN PRIu8
#define SCN_PLANE SCNd8
#define SCN_CHANNEL SCNu8
#define SCN_FLAG SCNu8
#define SCN_GEN SCNu8

// survive_channel channel, int sensor_id, survive_timecode timecode, int8_t plane, FLT angle
#define SWEEP_ANGLE_SCANF  "%s B %"SCN_CHANNEL" %d %u %" SCN_PLANE " " FLT_sformat "\n"
#define SWEEP_ANGLE_PRINTF "%s B %"PRI_CHANNEL" %u %u %" PRI_PLANE " " FLT_format "\n"
#define SWEEP_ANGLE_SCANF_ARGS dev, &channel, &sensor_id, &timecode, &plane, &angle
#define SWEEP_ANGLE_PRINTF_ARGS dev, channel, sensor_id, timecode, plane, angle

// survive_channel channel, int sensor_id, survive_timecode timecode, bool flag
#define SWEEP_PRINTF_ARGS dev, channel, sensor_id, timecode, flag
#define SWEEP_SCANF_ARGS dev, &channel, &sensor_id, &timecode, &flag
#define SWEEP_SCANF "%s W %"SCN_CHANNEL" %d %u %"SCN_FLAG"\n"
#define SWEEP_PRINTF "%s W %"PRI_CHANNEL" %d %u %"PRI_FLAG"\n"



#define SYNC_SCANF_ARGS dev, &channel, &timecode, &ootx, &gen
#define SYNC_PRINTF_ARGS dev, channel, timecode, ootx, gen
#define SYNC_SCANF "%s Y %"SCN_CHANNEL" %u %"SCN_FLAG" %"SCN_GEN"\n"
#define SYNC_PRINTF "%s Y %"PRI_CHANNEL" %u %"PRI_FLAG" %"PRI_GEN"\n"

struct SurviveRecordingData;
SURVIVE_EXPORT void survive_recording_write_matrix(struct SurviveRecordingData *recordingData, const SurviveObject *so,
												   int lvl, const char *name, const CnMat *M);
SURVIVE_EXPORT void survive_recording_write_to_output(struct SurviveRecordingData *recordingData, const char *format,
													  ...);
SURVIVE_EXPORT void survive_recording_write_to_output_nopreamble(struct SurviveRecordingData *recordingData,
																 const char *format, ...);
void survive_destroy_recording(SurviveContext *ctx);
void survive_install_recording(SurviveContext *ctx);
void survive_recording_config_process(SurviveObject *so, char *ct0conf, int len);

void survive_recording_lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, const SurvivePose *lh_pose);
void survive_recording_lightcap(SurviveObject *so, LightcapElement *le);
void survive_recording_raw_pose_process(SurviveObject *so, uint8_t lighthouse, const SurvivePose *pose);
void survive_recording_velocity_process(SurviveObject *so, uint8_t lighthouse, const SurviveVelocity *velocity);
void survive_recording_info_process(SurviveContext *ctx, const char *fault);
void survive_recording_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
									 survive_timecode timecode, bool flag);
void survive_recording_button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
									  const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals);
void survive_recording_angle_process(struct SurviveObject *so, int sensor_id, int acode, uint32_t timecode, FLT length,
									 FLT angle, uint32_t lh);
void survive_recording_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
										   survive_timecode timecode, int8_t plane, FLT angle);
void survive_recording_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode, bool ootx,
									bool gen);

void survive_recording_external_pose_process(SurviveContext *ctx, const char *name, const SurvivePose *pose);
void survive_recording_external_velocity_process(SurviveContext *ctx, const char *name,
												 const SurviveVelocity *velocity);

void survive_recording_light_process(struct SurviveObject *so, int sensor_id, int acode, int timeinsweep,
									 uint32_t timecode, uint32_t length, uint32_t lh);

void survive_recording_imu_process(struct SurviveObject *so, int mask, const FLT *accelgyro, uint32_t timecode, int id);
void survive_recording_imu_scales(struct SurviveObject *so, int gyro_scale_mode, int acc_scale_mode);
void survive_recording_raw_imu_process(struct SurviveObject *so, int mask, const FLT *accelgyro, uint32_t timecode,
									   int id);
void survive_recording_disconnect_process(struct SurviveObject *so);