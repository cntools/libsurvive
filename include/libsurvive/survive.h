#ifndef _SURVIVE_H
#define _SURVIVE_H

#include "poser.h"
#include "survive_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


#ifdef _MSC_VER
	#define SURVIVE_EXPORT_CONSTRUCTOR SURVIVE_EXPORT
#else
	#define SURVIVE_EXPORT_CONSTRUCTOR __attribute__((constructor))
#endif


/**
 * This struct encodes what the last effective angles seen on a sensor were, and when they occured.
 */
typedef struct SurviveSensorActivations_s {
	int lh_gen;

	// Valid for gen2; somewhat different meaning though -- refers to angle of the rotor when the sweep happened.
	FLT angles[SENSORS_PER_OBJECT][NUM_GEN2_LIGHTHOUSES][2];				// 2 Axes (Angles in LH space)
	survive_timecode timecode[SENSORS_PER_OBJECT][NUM_GEN2_LIGHTHOUSES][2]; // Timecode per axis in ticks

	// Valid only for Gen1
	survive_timecode lengths[SENSORS_PER_OBJECT][NUM_GEN1_LIGHTHOUSES][2]; // Timecode per axis in ticks

	uint32_t rollover_count;
	size_t imu_init_cnt;
	survive_timecode last_imu;
	survive_long_timecode last_movement; // Tracks the timecode of the last IMU packet which saw movement.
	FLT accel[3];
	FLT gyro[3];
	FLT mag[3];
} SurviveSensorActivations;

struct PoserDataLight;
struct PoserDataIMU;

SURVIVE_EXPORT void SurviveSensorActivations_ctor(SurviveSensorActivations *self);

/**
 * Adds a lightData packet to the table.
 */
SURVIVE_EXPORT FLT SurviveSensorActivations_difference(const SurviveSensorActivations *rhs,
        const SurviveSensorActivations *lhs);
SURVIVE_EXPORT void SurviveSensorActivations_add(SurviveSensorActivations *self, struct PoserDataLightGen1 *lightData);
SURVIVE_EXPORT void SurviveSensorActivations_add_gen2(SurviveSensorActivations *self,
													  struct PoserDataLightGen2 *lightData);

SURVIVE_EXPORT void SurviveSensorActivations_add_imu(SurviveSensorActivations *self, struct PoserDataIMU *imuData);

/**
 * Returns true iff the given sensor and lighthouse at given axis were seen at most `tolerance` ticks before the given
 * `timecode_now`.
 */
SURVIVE_EXPORT bool SurviveSensorActivations_isReadingValid(const SurviveSensorActivations *self, survive_timecode tolerance,
											 survive_timecode timecode_now, uint32_t sensor_idx, int lh, int axis);

/**
 * Returns true iff both angles for the given sensor and lighthouse were seen at most `tolerance` ticks before the given
 * `timecode_now`.
 */
SURVIVE_EXPORT bool SurviveSensorActivations_isPairValid(const SurviveSensorActivations *self, survive_timecode tolerance,
										  survive_timecode timecode_now, uint32_t sensor_idx, int lh);

/**
 * Returns the amount of time stationary
 */
SURVIVE_EXPORT survive_timecode SurviveSensorActivations_stationary_time(const SurviveSensorActivations *self);

/**
 * Default tolerance that gives a somewhat accuate representation of current state.
 *
 * Don't rely on this to be a given value.
 */
SURVIVE_IMPORT extern survive_timecode SurviveSensorActivations_default_tolerance;

// DANGER: This structure may be redefined.  Note that it is logically split into 64-bit chunks
// for optimization on 32- and 64-bit systems.

struct SurviveObject {
	SurviveContext *ctx;

	char codename[4];   // 3 letters, null-terminated.  Currently HMD, WM0, WM1.
	char drivername[8]; // 8 letters for driver.  Currently "HTC"
	char serial_number[16]; // 13 letters device serial number
	void *driver;
	int32_t buttonmask;
	int16_t axis1;

	int16_t axis2;
	int16_t axis3;
	int8_t charge;
	int8_t charging : 1;
	int8_t ison : 1;
	int8_t additional_flags : 6;

	// Pose Information, also "poser" field.
	FLT PoseConfidence;						 // 0..1

	// obj2world
	SurvivePose OutPose;

	// objImu2world
	SurvivePose OutPoseIMU;

	survive_timecode OutPose_timecode;

	SurviveVelocity velocity;
	survive_timecode velocity_timecode;

	SurvivePose
		FromLHPose[NUM_GEN1_LIGHTHOUSES]; // Filled out by poser, contains computed position from each lighthouse.
	void *PoserData; // Initialized to zero, configured by poser, can be anything the poser wants.
	PoserCB PoserFn;

	// Device-specific information about the location of the sensors.  This data will be used by the poser.
	// These are stored in the IMU's coordinate frame so that posers don't have to do a ton of manipulation
	// to do sensor fusion.
	int8_t sensor_ct;	  // sensor count

	// Remaps pins as reported from device into indices in 'modelPoints'
	int *channel_map;
	FLT *sensor_locations; // size is sensor_ct*3.  Contains x,y,z values for each sensor
	FLT *sensor_normals;   // size is nrlocations*3.  cointains normal vector for each sensor

	// Timing sensitive data (mostly for disambiguation)
	int32_t timebase_hz;		  // 48,000,000 for normal vive hardware.  (checked)

	// Flood info, for calculating which laser is currently sweeping.
	void *disambiguator_data;
	int8_t oldcode;
	int8_t sync_set_number; // 0 = master, 1 = slave, -1 = fault.
	int8_t did_handle_ootx; // If unset, will send lightcap data for sync pulses next time a sensor is hit.
	survive_timecode last_time_between_sync[NUM_GEN2_LIGHTHOUSES];
	survive_timecode last_sync_time[NUM_GEN2_LIGHTHOUSES];
	survive_timecode last_sync_length[NUM_GEN1_LIGHTHOUSES];
	survive_timecode recent_sync_time;
	survive_timecode last_lighttime; // May be a 24- or 32- bit number depending on what device.

	FLT imu_freq;

	// These are from the vive config files. They are named 'trackref_from_head' and 'trackref_from_imu'
	// respectively. We name them x2trackref here to match the naming convention used elsewhere in this
	// library.
	// They are kept here for posterity more than anything -- nothing in libsurvive is ultimately represented
	// in the 'trackref' coordinate space.
	SurvivePose head2trackref;
	SurvivePose imu2trackref;
	SurvivePose head2imu;

	FLT acc_bias[3];   // size is FLT*3. contains x,y,z
	FLT acc_scale[3];  // size is FLT*3. contains x,y,z
	FLT gyro_bias[3];  // size is FLT*3. contains x,y,z
	FLT gyro_scale[3]; // size is FLT*3. contains x,y,z

	haptic_func haptic;

	SurviveSensorActivations activations;
	void *user_ptr;

	char *conf;
	size_t conf_cnt;
};

// These exports are mostly for language binding against
SURVIVE_EXPORT const char *survive_object_codename(SurviveObject *so);
SURVIVE_EXPORT const SurvivePose *survive_object_last_imu2world(const SurviveObject *so);
SURVIVE_EXPORT const char *survive_object_drivername(SurviveObject *so);
SURVIVE_EXPORT const int8_t survive_object_charge(SurviveObject *so);
SURVIVE_EXPORT const bool survive_object_charging(SurviveObject *so);

SURVIVE_EXPORT const SurvivePose *survive_object_pose(SurviveObject *so);

SURVIVE_EXPORT int8_t survive_object_sensor_ct(SurviveObject *so);
SURVIVE_EXPORT const FLT *survive_object_sensor_locations(SurviveObject *so);
SURVIVE_EXPORT const FLT *survive_object_sensor_normals(SurviveObject *so);

typedef struct BaseStationCal {
	FLT phase;
	FLT tilt;
	FLT curve;
	FLT gibpha;
	FLT gibmag;

	// Gen 2 specific cal params
	FLT ogeephase;
	FLT ogeemag;
} BaseStationCal;

struct BaseStationData {
	uint8_t PositionSet : 1;

	/**
	 * This is a transformation from lh space to world space
	 */
	SurvivePose Pose;
	uint8_t OOTXSet : 1;
	uint32_t BaseStationID;

	BaseStationCal fcal[2];

	int8_t accel[3]; //"Up" vector
	uint8_t mode;

	void *ootx_data;
	void *user_ptr;
};

struct config_group;

#define BUTTON_QUEUE_MAX_LEN 32

// note: buttonId and axisId are 1-indexed values.
// a value of 0 for an id means that no data is present in that value
// additionally, when x and y values are both present in axis data,
// axis1 will be x, axis2 will be y.
typedef struct {
	uint8_t isPopulated; // probably can remove this given the semaphore in the parent struct.   helps with debugging
	uint8_t eventType;
	uint8_t buttonId;
	uint8_t axis1Id;
	uint16_t axis1Val;
	uint8_t axis2Id;
	uint16_t axis2Val;
	SurviveObject *so;
} ButtonQueueEntry;

typedef struct {
	uint8_t nextReadIndex;  // init to 0
	uint8_t nextWriteIndex; // init to 0
	void *buttonservicesem;
	ButtonQueueEntry entry[BUTTON_QUEUE_MAX_LEN];
} ButtonQueue;

typedef enum { SURVIVE_STOPPED = 0, SURVIVE_RUNNING, SURVIVE_CLOSING, SURVIVE_STATE_MAX } SurviveState;

struct SurviveRecordingData;

enum SurviveCalFlag {
	SVCal_None = 0,
	SVCal_Phase = 1,
	SVCal_Tilt = 2,
	SVCal_Curve = 4,
	SVCal_Gib = 8,
	SVCal_All = SVCal_Gib | SVCal_Curve | SVCal_Tilt | SVCal_Phase
};

struct SurviveContext {
	int lh_version_forced;
	int lh_version; // -1 is unknown, 0 is LHv1 -- pulse, ootx, etc. 1 is LHv2 -- single motor rotated beams

#define SURVIVE_HOOK_PROCESS_DEF(hook) hook##_process_func hook##proc;
#include "survive_hooks.h"

	// Calibration data:
	int activeLighthouses;
	BaseStationData bsd[NUM_GEN2_LIGHTHOUSES];
	int8_t bsd_map[NUM_GEN2_LIGHTHOUSES]; // maps channels to idxs

	SurviveCalData *calptr;				 // If and only if the calibration subsystem is attached.
	void *disambiguator_data;			 // global disambiguator data
	struct SurviveRecordingData *recptr; // Iff recording is attached
	SurviveObject **objs;
	int objs_ct;

	void **drivers;
	DeviceDriverCb *driverpolls;
	DeviceDriverCb *drivercloses;
	DeviceDriverMagicCb *drivermagics;
	int driver_ct;

	SurviveState state;
	SurviveError currentError;

	void *buttonservicethread;
	ButtonQueue buttonQueue;

	void *user_ptr;

	int log_level;
	FILE *log_target;

	struct config_group *global_config_values;
	struct config_group *lh_config; // lighthouse configs
	struct config_group	*temporary_config_values; // Set per-session, from command-line. Not saved but override global_config_values
};

SURVIVE_EXPORT void survive_verify_FLT_size(
	uint32_t user_size); // Baked in size of FLT to verify users of the library have the correct setting.

SURVIVE_EXPORT SurviveContext *survive_init_internal(int argc, char *const *argv, void *user_ptr,
													 log_process_func log_func);

/**
 * Same as survive_init, except it allows a log_func to be installed before most of system startup.
 *
 */
static inline SurviveContext *survive_init_with_logger(int argc, char *const *argv, void *user_ptr,
													   log_process_func log_func) {
	survive_verify_FLT_size(sizeof(FLT));
	return survive_init_internal(argc, argv, user_ptr, log_func);
}

/**
 * Call survive_init to get a populated SurviveContext pointer.
 *
 * This also sets up a number of configuration values based on command line
 * arguments. Pass 0, 0 to this function if you specifically do not want
 * command line processing.
 *
 * Note that this function _can_ return null based on command line arguments,
 * notably if -h was passed in.
 */
static inline SurviveContext *survive_init(int argc, char *const *argv) {
	return survive_init_with_logger(argc, argv, 0, 0);
}

// For any of these, you may pass in 0 for the function pointer to use default behavior.
// In general unless you are doing wacky things like recording or playing back data, you won't need to use this.
#define SURVIVE_HOOK_PROCESS_DEF(hook)                                                                                 \
	SURVIVE_EXPORT void survive_install_##hook##_fn(SurviveContext *ctx, hook##_process_func fbp);
#define SURVIVE_HOOK_FEEDBACK_DEF(hook)                                                                                \
	SURVIVE_EXPORT void survive_install_##hook##_fn(SurviveContext *ctx, hook##_feedback_func fbp);
#include "survive_hooks.h"

SURVIVE_EXPORT int survive_startup(SurviveContext *ctx);
SURVIVE_EXPORT int survive_poll(SurviveContext *ctx);
SURVIVE_EXPORT void survive_close(SurviveContext *ctx);

SURVIVE_EXPORT SurviveObject *survive_get_so_by_name(SurviveContext *ctx, const char *name);

// Utilitiy functions.
SURVIVE_EXPORT int survive_simple_inflate(SurviveContext *ctx, const uint8_t *input, int inlen, uint8_t *output, int outlen);
SURVIVE_EXPORT int survive_send_magic(SurviveContext *ctx, int magic_code, void *data, int datalen);

// These functions search both the stored-general and temporary sections for a parameter and return it.
#define SC_GET 0	   // Get, only.
#define SC_SET 1	   // Set, if not present
#define SC_OVERRIDE 2  // Set, to new default value.
#define SC_SETCONFIG 4 // Set, both in-memory and config file.  Use in conjunction with SC_OVERRIDE.

SURVIVE_EXPORT bool survive_config_is_set(SurviveContext *ctx, const char *tag);
SURVIVE_EXPORT FLT survive_configf(SurviveContext *ctx, const char *tag, char flags, FLT def);
SURVIVE_EXPORT uint32_t survive_configi(SurviveContext *ctx, const char *tag, char flags, uint32_t def);
SURVIVE_EXPORT const char *survive_configs(SurviveContext *ctx, const char *tag, char flags, const char *def);

SURVIVE_EXPORT void survive_attach_configi(SurviveContext *ctx, const char *tag, int * var );
SURVIVE_EXPORT void survive_attach_configf(SurviveContext *ctx, const char *tag, FLT * var );
SURVIVE_EXPORT void survive_attach_configs(SurviveContext *ctx, const char *tag, char * var );
SURVIVE_EXPORT void survive_detach_config(SurviveContext *ctx, const char *tag, void * var );

SURVIVE_EXPORT int8_t survive_get_bsd_idx(SurviveContext *ctx, survive_channel channel);

#define STATIC_CONFIG_ITEM(variable, name, type, description, default_value)                                           \
	const char *variable##_TAG = name;                                                                                 \
	SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##variable() {                                                             \
		survive_config_bind_variable(type, name, description, default_value);                                          \
	}
SURVIVE_EXPORT void survive_config_bind_variable(char vt, const char *name, const char *description,
												 ...); // Only used at boot.

// Install the calibrator.
SURVIVE_EXPORT void survive_cal_install(SurviveContext *ctx); // XXX This will be removed if not already done so.

// Read back a human-readable string description of the calibration status
SURVIVE_EXPORT int survive_cal_get_status(SurviveContext *ctx, char *description, int description_length);

// Induce haptic feedback
SURVIVE_EXPORT int survive_haptic(SurviveObject *so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow,
								  uint16_t repeatCount);

SURVIVE_EXPORT void survive_find_ang_velocity(SurviveAngularVelocity out, FLT tdiff, const LinmathQuat from,
											  const LinmathQuat to);
SURVIVE_EXPORT void survive_apply_ang_velocity(LinmathQuat out, const SurviveAngularVelocity v, FLT t,
											   const LinmathQuat t0);
// Call these from your callback if overridden.
// Accept higher-level data.
SURVIVE_EXPORT void survive_default_log_process(struct SurviveContext *ctx, SurviveLogLevel ll, const char *fault);
SURVIVE_EXPORT void survive_default_lightcap_process(SurviveObject *so, const LightcapElement *element);
SURVIVE_EXPORT void survive_default_light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep,
												  survive_timecode timecode, survive_timecode length, uint32_t lh);
SURVIVE_EXPORT void survive_default_raw_imu_process(SurviveObject *so, int mode, FLT *accelgyro,
													survive_timecode timecode, int id);
SURVIVE_EXPORT void survive_default_imu_process(SurviveObject *so, int mode, FLT *accelgyro, survive_timecode timecode, int id);
SURVIVE_EXPORT void survive_default_angle_process(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode,
												  FLT length, FLT angle, uint32_t lh);

SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel,
												 survive_timecode timeinsweep, bool ootx, bool gen);
SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
												  survive_timecode timecode, bool flag);
SURVIVE_EXPORT void survive_default_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
														survive_timecode timecode, int8_t plane, FLT angle);

SURVIVE_EXPORT void survive_default_button_process(SurviveObject *so, uint8_t eventType, uint8_t buttonId,
												   uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id,
												   uint16_t axis2Val);
SURVIVE_EXPORT void survive_default_pose_process(SurviveObject *so, survive_timecode timecode, SurvivePose *pose);
SURVIVE_EXPORT void survive_default_velocity_process(SurviveObject *so, survive_timecode timecode,
													 const SurviveVelocity *pose);
SURVIVE_EXPORT void survive_default_external_pose_process(SurviveContext *so, const char *name,
														  const SurvivePose *pose);
SURVIVE_EXPORT void survive_default_external_velocity_process(SurviveContext *so, const char *name,
															  const SurviveVelocity *velocity);
SURVIVE_EXPORT void survive_default_lighthouse_pose_process(SurviveContext *ctx, uint8_t lighthouse,
															SurvivePose *lh_pose, SurvivePose *obj_pose);
SURVIVE_EXPORT int survive_default_config_process(SurviveObject *so, char *ct0conf, int len);
SURVIVE_EXPORT void survive_default_gen_detected_process(SurviveObject *so, int lh_version);
SURVIVE_EXPORT void survive_default_new_object_process(SurviveObject *so);

////////////////////// Survive Drivers ////////////////////////////

SURVIVE_EXPORT void RegisterDriver(const char *name, void *data);

#define REGISTER_LINKTIME(func)                                                                                        \
	SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##func() { RegisterDriver(#func, (void *)&func); }

///////////////////////// General stuff for writing drivers ///////

// For device drivers to call.  This actually attaches them.
SURVIVE_EXPORT int survive_add_object(SurviveContext *ctx, SurviveObject *obj);
SURVIVE_EXPORT void survive_remove_object(SurviveContext *ctx, SurviveObject *obj);
SURVIVE_EXPORT void survive_add_driver(SurviveContext *ctx, void *payload, DeviceDriverCb poll, DeviceDriverCb close,
						DeviceDriverMagicCb magic);

// This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
SURVIVE_EXPORT uint8_t survive_map_sensor_id(SurviveObject *so, uint8_t reported_id);
SURVIVE_EXPORT void handle_lightcap(SurviveObject *so, const LightcapElement *le);

#define SV_LOG_NULL_GUARD                                                                                              \
	if (ctx == 0) {                                                                                                    \
		fprintf(stderr, "Logging: %s\n", stbuff);                                                                      \
	} else

#define SV_WARN(...)                                                                                                   \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		sprintf(stbuff, __VA_ARGS__);                                                                                  \
		SV_LOG_NULL_GUARD ctx->logproc(ctx, SURVIVE_LOG_LEVEL_WARNING, stbuff);                                        \
	}

#define SV_INFO(...)                                                                                                   \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		sprintf(stbuff, __VA_ARGS__);                                                                                  \
		SV_LOG_NULL_GUARD ctx->logproc(ctx, SURVIVE_LOG_LEVEL_INFO, stbuff);                                           \
	}

#define SV_VERBOSE(lvl, ...)                                                                                           \
	{                                                                                                                  \
		if (ctx == 0 || ctx->log_level >= lvl) {                                                                       \
			SV_INFO(__VA_ARGS__);                                                                                      \
		}                                                                                                              \
	}

#define SV_ERROR(errorCode, ...)                                                                                       \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		sprintf(stbuff, __VA_ARGS__);                                                                                  \
		if (ctx)                                                                                                       \
			ctx->report_errorproc(ctx, errorCode);                                                                     \
		SV_LOG_NULL_GUARD ctx->logproc(ctx, SURVIVE_LOG_LEVEL_INFO, stbuff);                                           \
	}

static inline void survive_notify_gen2(struct SurviveObject *so) {
	if (so->ctx->lh_version_forced != -1 && so->ctx->lh_version_forced != 1) {
		return;
	}

	if (so->ctx->lh_version != 1) {
		so->ctx->gen_detectedproc(so, 1);
	}
}

static inline void survive_notify_gen1(struct SurviveObject *so) {
	if (so->ctx->lh_version_forced != -1 && so->ctx->lh_version_forced != 0) {
		return;
	}

	if (so->ctx->lh_version != 0) {
		so->ctx->gen_detectedproc(so, 0);
	}
}

#define SV_GENERAL_ERROR(...) SV_ERROR(SURVIVE_ERROR_GENERAL, __VA_ARGS__)

#define SV_KILL() exit(0) // XXX This should likely be re-defined.

#ifdef __cplusplus
};
#endif

#endif
