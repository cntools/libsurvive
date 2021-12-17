#ifndef _SURVIVE_H
#define _SURVIVE_H

#include "assert.h"
#include "poser.h"
#include "survive_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

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
	SurviveObject *so;
	int lh_gen;

	// Valid for gen2; somewhat different meaning though -- refers to angle of the rotor when the sweep happened.
	FLT angles[SENSORS_PER_OBJECT][NUM_GEN2_LIGHTHOUSES][2];				// 2 Axes (Angles in LH space)
	FLT angles_center_x[NUM_GEN2_LIGHTHOUSES][2];
	FLT angles_center_dev[NUM_GEN2_LIGHTHOUSES][2];
	int angles_center_cnt[NUM_GEN2_LIGHTHOUSES][2];

	FLT raw_angles[SENSORS_PER_OBJECT][NUM_GEN2_LIGHTHOUSES][2];					 // 2 Axes (Angles in LH space)
	survive_long_timecode raw_timecode[SENSORS_PER_OBJECT][NUM_GEN2_LIGHTHOUSES][2]; // Timecode per axis in ticks
	survive_long_timecode timecode[SENSORS_PER_OBJECT][NUM_GEN2_LIGHTHOUSES][2]; // Timecode per axis in ticks

	// Valid only for Gen1
	survive_timecode lengths[SENSORS_PER_OBJECT][NUM_GEN1_LIGHTHOUSES][2]; // Timecode per axis in ticks

	survive_long_timecode hits[SENSORS_PER_OBJECT][NUM_GEN2_LIGHTHOUSES][2];

	size_t imu_init_cnt;
	survive_long_timecode last_imu;
	survive_long_timecode last_light;
	survive_long_timecode last_light_change;
	survive_long_timecode last_movement; // Tracks the timecode of the last IMU packet which saw movement.

	FLT runtime_offset;

	FLT accel[3];
	FLT gyro[3];
	FLT mag[3];

	struct SurviveSensorActivations_params {
		FLT moveThresholdGyro;
		FLT moveThresholdAcc;
		FLT moveThresholdAng;
		FLT filterLightChange;
		FLT filterOutlierCriteria;
		FLT filterVarianceMin;
	} params;
} SurviveSensorActivations;

struct PoserDataLight;
struct PoserDataIMU;

SURVIVE_EXPORT void SurviveSensorActivations_reset(SurviveSensorActivations *self);
SURVIVE_EXPORT void SurviveSensorActivations_ctor(SurviveObject *so, SurviveSensorActivations *self);
SURVIVE_EXPORT void SurviveSensorActivations_dtor(SurviveObject *so);
SURVIVE_EXPORT survive_long_timecode SurviveSensorActivations_long_timecode_imu(const SurviveSensorActivations *self, survive_timecode timecode);
SURVIVE_EXPORT survive_long_timecode SurviveSensorActivations_long_timecode_light(const SurviveSensorActivations *self, survive_timecode timecode);

/**
 * Adds a lightData packet to the table.
 */
SURVIVE_EXPORT FLT SurviveSensorActivations_difference(const SurviveSensorActivations *rhs,
        const SurviveSensorActivations *lhs);
SURVIVE_EXPORT void SurviveSensorActivations_add_sync(SurviveSensorActivations *self, struct PoserDataLight *lightData);
SURVIVE_EXPORT bool SurviveSensorActivations_add(SurviveSensorActivations *self, struct PoserDataLightGen1 *lightData);
SURVIVE_EXPORT bool SurviveSensorActivations_add_gen2(SurviveSensorActivations *self,
													  struct PoserDataLightGen2 *lightData);
SURVIVE_EXPORT void SurviveSensorActivations_valid_counts(SurviveSensorActivations *self,
														  survive_long_timecode tolerance, uint32_t *meas_cnt,
														  uint32_t *lh_count, uint32_t *axis_cnt,
														  size_t *meas_for_lhs_axis);
SURVIVE_EXPORT void SurviveSensorActivations_register_runtime(SurviveSensorActivations *self, survive_long_timecode tc,
															  uint64_t runtime_clock);
SURVIVE_EXPORT uint64_t SurviveSensorActivations_runtime(SurviveSensorActivations *self, survive_long_timecode tc);
SURVIVE_EXPORT void SurviveSensorActivations_add_imu(SurviveSensorActivations *self, struct PoserDataIMU *imuData);

/**
 * Returns true iff the given sensor and lighthouse at given axis were seen at most `tolerance` ticks before the given
 * `timecode_now`.
 */
SURVIVE_EXPORT bool SurviveSensorActivations_is_reading_valid(const SurviveSensorActivations *self,
															  survive_long_timecode tolerance, uint32_t sensor_idx,
															  int lh, int axis);

SURVIVE_EXPORT survive_long_timecode SurviveSensorActivations_time_since_last_reading(const SurviveSensorActivations *self,
																				 uint32_t sensor_idx, int lh, int axis);

SURVIVE_EXPORT survive_long_timecode SurviveSensorActivations_last_reading(const SurviveSensorActivations *self,
																		   uint32_t sensor_idx, int lh, int axis);

/**
 * Returns true iff both angles for the given sensor and lighthouse were seen at most `tolerance` ticks before the given
 * `timecode_now`.
 */
SURVIVE_EXPORT bool SurviveSensorActivations_isPairValid(const SurviveSensorActivations *self, survive_timecode tolerance,
										  survive_timecode timecode_now, uint32_t sensor_idx, int lh);

/**
 * Returns the amount of time stationary
 */
SURVIVE_EXPORT survive_long_timecode SurviveSensorActivations_stationary_time(const SurviveSensorActivations *self);
SURVIVE_EXPORT survive_long_timecode SurviveSensorActivations_last_time(const SurviveSensorActivations *self);
/**
 * Default tolerance that gives a somewhat accuate representation of current state.
 *
 * Don't rely on this to be a given value.
 */
SURVIVE_IMPORT extern survive_timecode SurviveSensorActivations_default_tolerance;

struct SurviveObject {
	SurviveContext *ctx;

	char codename[4];   // 3 letters, null-terminated.  Currently HMD, WM0, WM1.
	char drivername[8]; // 8 letters for driver.  Currently "HTC"
	char serial_number[16]; // 13 letters device serial number
	void *driver;
	SurviveObjectType object_type;
	SurviveObjectSubtype object_subtype;
	uint32_t buttonmask;
	uint32_t touchmask;
	SurviveAxisVal_t axis[16];

	int8_t charge;
	uint8_t charging : 1;
	uint8_t ison : 1;
	int8_t additional_flags : 6;

	// Pose Information, also "poser" field.
	FLT PoseConfidence;						 // 0..1

	// obj2world
	SurvivePose OutPose;

	// objImu2world
	SurvivePose OutPoseIMU;
	FLT poseConfidence;
	survive_long_timecode OutPose_timecode;

	SurviveVelocity velocity;
	survive_long_timecode velocity_timecode;

	SurvivePose
		FromLHPose[NUM_GEN2_LIGHTHOUSES]; // Filled out by poser, contains computed position from each lighthouse.

	void *PoserFnData; // Initialized to zero, configured by poser, can be anything the poser wants.

	// Device-specific information about the location of the sensors.  This data will be used by the poser.
	// These are stored in the IMU's coordinate frame so that posers don't have to do a ton of manipulation
	// to do sensor fusion.
	int8_t sensor_ct;	  // sensor count

	// Remaps pins as reported from device into indices in 'modelPoints'
	int *channel_map;
	bool has_sensor_locations;
	FLT *sensor_locations; // size is sensor_ct*3.  Contains x,y,z values for each sensor
	FLT *sensor_normals;   // size is nrlocations*3.  cointains normal vector for each sensor

	// Timing sensitive data (mostly for disambiguation)
	int32_t timebase_hz;		  // 48,000,000 for normal vive hardware.  (checked)

	// Flood info, for calculating which laser is currently sweeping.
	void *disambiguator_data;
	int8_t oldcode;
	survive_timecode last_time_between_sync[NUM_GEN2_LIGHTHOUSES];
	survive_timecode last_sync_time[NUM_GEN2_LIGHTHOUSES];
	survive_timecode sync_count[NUM_GEN2_LIGHTHOUSES];

	FLT imu_freq;

	// These are from the vive config files. They are named 'trackref_from_head' and 'trackref_from_imu'
	// respectively. We name them x2trackref here to match the naming convention used elsewhere in this
	// library.
	// They are kept here for posterity more than anything -- nothing in libsurvive is ultimately represented
	// in the 'trackref' coordinate space.
	SurvivePose head2trackref;
	SurvivePose imu2trackref;
	SurvivePose head2imu;

	FLT raw_acc_scale, raw_gyro_scale;
	FLT acc_bias[3];   // size is FLT*3. contains x,y,z
	FLT acc_scale[3];  // size is FLT*3. contains x,y,z
	FLT gyro_bias[3];  // size is FLT*3. contains x,y,z
	FLT gyro_scale[3]; // size is FLT*3. contains x,y,z

	haptic_func haptic;

	SurviveSensorActivations activations;
	void *user_ptr;

	char *conf;
	size_t conf_cnt;

	struct SurviveKalmanTracker *tracker;

	struct {
		uint32_t syncs[NUM_GEN2_LIGHTHOUSES];
		uint32_t skipped_syncs[NUM_GEN2_LIGHTHOUSES];
		uint32_t bad_syncs[NUM_GEN2_LIGHTHOUSES];
		uint32_t hit_from_lhs[NUM_GEN2_LIGHTHOUSES];
		uint32_t rejected_data[NUM_GEN2_LIGHTHOUSES];
		uint32_t dropped_light[NUM_GEN2_LIGHTHOUSES];
		uint32_t sync_resets[NUM_GEN2_LIGHTHOUSES];

		uint32_t extent_hits, extent_misses, naive_hits;
		FLT min_extent, max_extent;
	} stats;
};

// These exports are mostly for language binding against
SURVIVE_EXPORT const char *survive_object_codename(SurviveObject *so);
SURVIVE_EXPORT const SurvivePose *survive_object_last_imu2world(const SurviveObject *so);
SURVIVE_EXPORT const char *survive_object_drivername(SurviveObject *so);
SURVIVE_EXPORT int8_t survive_object_charge(SurviveObject *so);
SURVIVE_EXPORT bool survive_object_charging(SurviveObject *so);

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

	uint8_t sys_unlock_count;
	LinmathPoint3d accel; //"Up" vector
	uint8_t mode;

	FLT confidence;
	void *ootx_data;
	void *user_ptr;
	bool disable;
	uint8_t OOTXChecked : 1;
	struct SurviveKalmanLighthouse *tracker;
};

struct config_group;

#define BUTTON_QUEUE_MAX_LEN 32

// note: buttonId and axisId are 1-indexed values.
// a value of 0 for an id means that no data is present in that value
// additionally, when x and y values are both present in axis data,
// axis1 will be x, axis2 will be y.
typedef struct {
	uint8_t isPopulated; // probably can remove this given the semaphore in the parent struct.   helps with debugging
	enum SurviveInputEvent eventType;
	enum SurviveButton buttonId;

	enum SurviveAxis ids[16];
	SurviveAxisVal_t axisValues[16];

	SurviveObject *so;
} ButtonQueueEntry;

typedef struct {
	uint8_t nextReadIndex;  // init to 0
	uint8_t nextWriteIndex; // init to 0
	void *buttonservicesem;
	ButtonQueueEntry entry[BUTTON_QUEUE_MAX_LEN];

	size_t processed_events;
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
	int lh_version_configed;
	int lh_version_forced;
	int lh_version; // -1 is unknown, 0 is LHv1 -- pulse, ootx, etc. 1 is LHv2 -- single motor rotated beams

#define SURVIVE_HOOK_PROCESS_DEF(hook) hook##_process_func hook##proc;
#include "survive_hooks.h"

#define SURVIVE_HOOK_PROCESS_DEF(hook)                                                                                 \
	FLT hook##_call_time;                                                                                              \
	uint32_t hook##_call_cnt;                                                                                          \
	uint32_t hook##_call_over_cnt;                                                                                     \
	FLT hook##_max_call_time;
#include "survive_hooks.h"

	// Calibration data:
	int activeLighthouses;
	BaseStationData bsd[NUM_GEN2_LIGHTHOUSES];
	int8_t bsd_map[NUM_GEN2_LIGHTHOUSES]; // maps channels to idxs

	void *disambiguator_data;			 // global disambiguator data
	struct SurviveRecordingData *recptr; // Iff recording is attached
	SurviveObject **objs;
	int objs_ct;

	PoserCB PoserFn;

	void **drivers;
	DeviceDriverCb *driverpolls;
	DeviceDriverCb *drivercloses;
	int driver_ct;

	SurviveState state;
	SurviveError currentError;

	void *buttonservicethread;
	ButtonQueue buttonQueue;

	void *user_ptr;

	int log_level;
	FILE *log_target;

	size_t poll_min_time_ms;

	struct config_group *global_config_values;
	struct config_group *lh_config; // lighthouse configs
	struct config_group	*temporary_config_values; // Set per-session, from command-line. Not saved but override global_config_values

	// Additional details that we don't want / need to expose to every single include
	void *private_members;
	bool request_floor_set;
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
	SURVIVE_EXPORT hook##_process_func survive_install_##hook##_fn(SurviveContext *ctx, hook##_process_func fbp);

#define SURVIVE_HOOK_FEEDBACK_DEF(hook)                                                                                \
	SURVIVE_EXPORT hook##_feedback_func survive_install_##hook##_fn(SurviveContext *ctx, hook##_feedback_func fbp);
#include "survive_hooks.h"

SURVIVE_EXPORT int survive_startup(SurviveContext *ctx);
SURVIVE_EXPORT int survive_poll(SurviveContext *ctx);
SURVIVE_EXPORT void survive_close(SurviveContext *ctx);
SURVIVE_EXPORT void survive_get_ctx_lock(SurviveContext *ctx);
SURVIVE_EXPORT void survive_release_ctx_lock(SurviveContext *ctx);

SURVIVE_EXPORT const char *survive_build_tag();

SURVIVE_EXPORT SurviveObject *survive_get_so_by_name(SurviveContext *ctx, const char *name);

// Utilitiy functions.
SURVIVE_EXPORT int survive_simple_inflate(SurviveContext *ctx, const uint8_t *input, int inlen, uint8_t *output, int outlen);

// These functions search both the stored-general and temporary sections for a parameter and return it.
enum survive_config_flags {
	SC_GET = 0,		 // Get, only.
	SC_SET = 1,		 // Set, if not present
	SC_OVERRIDE = 2, // Set, to new default value.
	SC_SETCONFIG = 4 // Set, both in-memory and config file.  Use in conjunction with SC_OVERRIDE.
};

SURVIVE_EXPORT bool survive_config_is_set(SurviveContext *ctx, const char *tag);
SURVIVE_EXPORT FLT survive_configf(SurviveContext *ctx, const char *tag, char flags, FLT def);
SURVIVE_EXPORT uint32_t survive_configi(SurviveContext *ctx, const char *tag, char flags, uint32_t def);
SURVIVE_EXPORT char survive_config_type(SurviveContext *ctx, const char *tag);
SURVIVE_EXPORT void survive_config_as_str(SurviveContext *ctx, char *output, size_t n, const char *tag,
										  const char *def);

SURVIVE_EXPORT const char *survive_configs(SurviveContext *ctx, const char *tag, char flags, const char *def);

SURVIVE_EXPORT void survive_attach_config(SurviveContext *ctx, const char *tag, void * var, char type);
SURVIVE_EXPORT void survive_attach_configi(SurviveContext *ctx, const char *tag, int32_t *var);
SURVIVE_EXPORT void survive_attach_configf(SurviveContext *ctx, const char *tag, FLT * var );
SURVIVE_EXPORT void survive_attach_configs(SurviveContext *ctx, const char *tag, char * var );

#ifdef COMPILER_HAS_GENERIC_SUPPORT
#define SURVIVE_ATTACH_CONFIG(ctx, name, var) _Generic((var), \
              double*: survive_attach_configf, \
              float*: survive_attach_configf,  \
              int*: survive_attach_configi  \
)(ctx, name, var);

#define SURVIVE_CONFIG_BIND_VARIABLE(name, desc, def, var) _Generic((var), \
              double*: survive_config_bind_variablef, \
              float*: survive_config_bind_variablef,  \
              int*: survive_config_bind_variablei  \
)(name, desc, def);
#else
#define SURVIVE_ATTACH_CONFIG(ctx, name, var)
#define SURVIVE_CONFIG_BIND_VARIABLE(name, desc, def, var)
#endif
SURVIVE_EXPORT void survive_detach_config(SurviveContext *ctx, const char *tag, void * var );

SURVIVE_EXPORT int8_t survive_get_bsd_idx(SurviveContext *ctx, survive_channel channel);

#define SURVIVE_INVOKE_HOOK(hook, ctx, ...)                                                                            \
	{                                                                                                                  \
		if (ctx->hook##proc) {                                                                                         \
			FLT start_time = OGRelativeTime();                                                                         \
			ctx->hook##proc(ctx, __VA_ARGS__);                                                                         \
			FLT this_time = OGRelativeTime() - start_time;                                                             \
			if (this_time > ctx->hook##_max_call_time)                                                                 \
				ctx->hook##_max_call_time = this_time;                                                                 \
			if (this_time > .001)                                                                                      \
				ctx->hook##_call_over_cnt++;                                                                           \
			ctx->hook##_call_time += this_time;                                                                        \
			ctx->hook##_call_cnt++;                                                                                    \
		}                                                                                                              \
	}

#define SURVIVE_INVOKE_HOOK_SO(hook, so, ...)                                                                          \
	{                                                                                                                  \
		if (so->ctx->hook##proc) {                                                                                     \
			FLT start_time = OGRelativeTime();                                                                         \
			so->ctx->hook##proc(so, ##__VA_ARGS__);                                                                    \
			FLT this_time = OGRelativeTime() - start_time;                                                             \
			if (this_time > so->ctx->hook##_max_call_time)                                                             \
				so->ctx->hook##_max_call_time = this_time;                                                             \
			if (this_time > .001)                                                                                      \
				so->ctx->hook##_call_over_cnt++;                                                                       \
			so->ctx->hook##_call_time += this_time;                                                                    \
			so->ctx->hook##_call_cnt++;                                                                                \
		}                                                                                                              \
	}

#define STATIC_CONFIG_ITEM(variable, name, type, description, default_value)                                           \
	const char *variable##_TAG = name;                                                                                 \
	SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##variable() {                                                             \
		survive_config_bind_variable(type, name, description, default_value, 0xcafebeef);                              \
	}

#define STRUCT_CONFIG_SECTION(type) \
static void type##_bind_variables(SurviveContext* ctx, type* t, bool ctor) {

#define STRUCT_CONFIG_ITEM(name, description, default_value, var)                                      \
	if (t && ctor) {                                                                                                           \
            var = default_value;                                                                                                           \
            SURVIVE_ATTACH_CONFIG(ctx, name, &var);                                                                                                           \
	} else if(t) {                                                                                                       \
			survive_detach_config(ctx, name, &var);                                                                    \
	} else {                                                                                                           \
		SURVIVE_CONFIG_BIND_VARIABLE(name, description, default_value, &var);                          \
	}

#define END_STRUCT_CONFIG_SECTION(type)                                                                                \
	}                                                                                                                  \
SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##type() { type##_bind_variables(0, 0, 0); }                                  \
void type##_attach_config(SurviveContext* ctx, type* t) { type##_bind_variables(ctx, t, 1); }                          \
void type##_detach_config(SurviveContext* ctx, type* t) { type##_bind_variables(ctx, t, 0); }                          \

SURVIVE_EXPORT void survive_config_bind_variable(char vt, const char *name, const char *description, ...); // Only used at boot.
SURVIVE_EXPORT void survive_config_bind_variablei(const char *name, const char *description, int def);
SURVIVE_EXPORT void survive_config_bind_variablef(const char *name, const char *description, FLT def);

// Read back a human-readable string description of the calibration status
SURVIVE_EXPORT int survive_cal_get_status(SurviveContext *ctx, char *description, int description_length);

// Induce haptic feedback
SURVIVE_EXPORT int survive_haptic(SurviveObject *so, FLT freq, FLT amp, FLT duration);
SURVIVE_EXPORT void survive_ootx_free_decoder_context(struct SurviveContext *ctx, int bsd_idx);
SURVIVE_EXPORT void survive_find_ang_velocity(SurviveAngularVelocity out, FLT tdiff, const LinmathQuat from,
											  const LinmathQuat to);
SURVIVE_EXPORT void survive_apply_ang_velocity(LinmathQuat out, const SurviveAngularVelocity v, FLT t,
											   const LinmathQuat t0);
// Call these from your callback if overridden.
// Accept higher-level data.
SURVIVE_EXPORT void survive_default_ootx_received_process(struct SurviveContext *ctx, uint8_t bsd_idx);

SURVIVE_EXPORT void survive_default_disconnect_process(struct SurviveObject *so);
SURVIVE_EXPORT int survive_default_printf_process(struct SurviveContext *ctx, const char *format, ...);
SURVIVE_EXPORT void survive_default_log_process(struct SurviveContext *ctx, SurviveLogLevel ll, const char *fault);
SURVIVE_EXPORT void survive_default_lightcap_process(SurviveObject *so, const LightcapElement *element);
SURVIVE_EXPORT void survive_default_light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep,
												  survive_timecode timecode, survive_timecode length, uint32_t lh);
SURVIVE_EXPORT void survive_default_raw_imu_process(SurviveObject *so, int mode, const FLT *accelgyro,
													survive_timecode timecode, int id);
SURVIVE_EXPORT void survive_default_set_imu_scale_modes(SurviveObject *so, int gyro_scale_mode, int acc_scale_mode);
SURVIVE_EXPORT void survive_default_imu_process(SurviveObject *so, int mode, const FLT *accelgyro,
												survive_timecode timecode, int id);
SURVIVE_EXPORT void survive_default_angle_process(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode,
												  FLT length, FLT angle, uint32_t lh);

SURVIVE_EXPORT void survive_default_light_pulse_process(SurviveObject *so, int sensor_id, int acode,
														survive_timecode timecode, FLT length, uint32_t lh);
SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel,
												 survive_timecode timeinsweep, bool ootx, bool gen);
SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
												  survive_timecode timecode, bool flag);
SURVIVE_EXPORT void survive_default_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
														survive_timecode timecode, int8_t plane, FLT angle);
SURVIVE_EXPORT void survive_default_button_process(SurviveObject *so, enum SurviveInputEvent eventType,
												   enum SurviveButton buttonId, const enum SurviveAxis *axisIds,
												   const SurviveAxisVal_t *axisVals);
SURVIVE_EXPORT void survive_default_imupose_process(SurviveObject *so, survive_long_timecode timecode,
													const SurvivePose *imu2world);
SURVIVE_EXPORT void survive_default_pose_process(SurviveObject *so, survive_long_timecode timecode,
												 const SurvivePose *pose);
SURVIVE_EXPORT void survive_default_velocity_process(SurviveObject *so, survive_long_timecode timecode,
													 const SurviveVelocity *pose);
SURVIVE_EXPORT void survive_default_external_pose_process(SurviveContext *so, const char *name,
														  const SurvivePose *pose);
SURVIVE_EXPORT void survive_default_external_velocity_process(SurviveContext *so, const char *name,
															  const SurviveVelocity *velocity);
SURVIVE_EXPORT void survive_default_lighthouse_pose_process(SurviveContext *ctx, uint8_t lighthouse,
															const SurvivePose *lh_pose);
SURVIVE_EXPORT int survive_default_config_process(SurviveObject *so, char *ct0conf, int len);
SURVIVE_EXPORT void survive_default_gen_detected_process(SurviveObject *so, int lh_version);
SURVIVE_EXPORT void survive_default_new_object_process(SurviveObject *so);
SURVIVE_EXPORT double survive_run_time(const SurviveContext *ctx);
SURVIVE_EXPORT double survive_run_time_since_epoch(const SurviveContext *ctx);

SURVIVE_EXPORT size_t survive_input_event_count(const SurviveContext *ctx);
////////////////////// Survive Drivers ////////////////////////////

SURVIVE_EXPORT void RegisterDriver(const char *name, survive_driver_fn data);
SURVIVE_EXPORT void RegisterPoserDriver(const char *name, PoserCB data);

#define REGISTER_LINKTIME(func)                                                                                        \
	SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##func() {                                                                 \
		static bool loaded = false;                                                                                    \
		if (loaded == false) {                                                                                         \
			RegisterDriver(#func, (survive_driver_fn)&func);                                                           \
		};                                                                                                             \
		loaded = true;                                                                                                 \
	}

#define REGISTER_POSER(func)                                                                                           \
	SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##func() {                                                                 \
		static bool loaded = false;                                                                                    \
		if (loaded == false) {                                                                                         \
			RegisterPoserDriver(#func, func);                                                                          \
		};                                                                                                             \
		loaded = true;                                                                                                 \
	}

///////////////////////// General stuff for writing drivers ///////

// For device drivers to call.  This actually attaches them.
SURVIVE_EXPORT int survive_add_object(SurviveContext *ctx, SurviveObject *obj);
SURVIVE_EXPORT void survive_remove_object(SurviveContext *ctx, SurviveObject *obj);
SURVIVE_EXPORT const void *survive_get_driver(const SurviveContext *ctx, DeviceDriverCb pollFn);
SURVIVE_EXPORT const void *survive_get_driver_by_closefn(const SurviveContext *ctx, DeviceDriverCb closeFn);
SURVIVE_EXPORT void survive_add_driver(SurviveContext *ctx, void *driver_data, DeviceDriverCb poll,
									   DeviceDriverCb close);
SURVIVE_EXPORT bool *survive_add_threaded_driver(SurviveContext *ctx, void *driver_data, const char *name,
												 void *(routine)(void *), DeviceDriverCb close);
SURVIVE_EXPORT char *survive_export_config(SurviveObject *so);
SURVIVE_EXPORT void survive_reset_lighthouse_positions(SurviveContext *ctx);
SURVIVE_EXPORT void survive_reset_lighthouse_position(SurviveContext *ctx, int bsd_idx);

// This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
SURVIVE_EXPORT uint8_t survive_map_sensor_id(SurviveObject *so, uint8_t reported_id);
SURVIVE_EXPORT bool handle_lightcap(SurviveObject *so, const LightcapElement *le);

SURVIVE_EXPORT const char *survive_colorize(const char *str);
SURVIVE_EXPORT const char *survive_colorize_codename(const SurviveObject *so);
SURVIVE_EXPORT uint32_t survive_hash(const uint8_t *data, size_t len);
SURVIVE_EXPORT uint32_t survive_hash_str(const char *str);
#define SV_CONST_COLOR(str, clr) "\033[0;" #clr "m" fmt "\033[0m"
#define SURVIVE_COLORIZED_FORMAT(fmt) "\033[0;%dm" fmt "\033[0m"
#define SURVIVE_COLORIZED_DATA(data) (survive_hash((uint8_t *)&(data), sizeof(data)) % 8 + 30), (data)
#define SURVIVE_COLORIZED_STR(str) (survive_hash_str(str) % 8 + 30), str

#define SV_DATA_LOG(fmt, v, n, ...)                                                                                    \
	{                                                                                                                  \
		if (so && so->ctx && so->ctx->datalogproc) {                                                                   \
			char name[128];                                                                                            \
			snprintf(name, sizeof(name) - 1, fmt, ##__VA_ARGS__);                                                      \
			SURVIVE_INVOKE_HOOK_SO(datalog, so, name, v, n);                                                           \
		}                                                                                                              \
	}

#define SV_LOG_NULL_GUARD                                                                                              \
	if (ctx == 0) {                                                                                                    \
		fprintf(stderr, "Logging: %s\n", stbuff);                                                                      \
	} else

#define SV_WARN(...)                                                                                                   \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		sprintf(stbuff, __VA_ARGS__);                                                                                  \
		SV_LOG_NULL_GUARD SURVIVE_INVOKE_HOOK(log, ctx, SURVIVE_LOG_LEVEL_WARNING, stbuff);                            \
	}

#define SV_INFO(...)                                                                                                   \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		snprintf(stbuff, sizeof(stbuff), __VA_ARGS__);                                                                 \
		SV_LOG_NULL_GUARD SURVIVE_INVOKE_HOOK(log, ctx, SURVIVE_LOG_LEVEL_INFO, stbuff);                               \
	}

#define SV_VERBOSE(lvl, ...)                                                                                           \
	{                                                                                                                  \
		if (ctx == 0 || ctx->log_level >= (lvl)) {                                                                     \
			SV_INFO(__VA_ARGS__);                                                                                      \
		}                                                                                                              \
	}

#define SV_ERROR(errorCode, ...)                                                                                       \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		sprintf(stbuff, __VA_ARGS__);                                                                                  \
		if (ctx)                                                                                                       \
			SURVIVE_INVOKE_HOOK(report_error, ctx, errorCode);                                                         \
		SV_LOG_NULL_GUARD SURVIVE_INVOKE_HOOK(log, ctx, SURVIVE_LOG_LEVEL_INFO, stbuff);                               \
		if (!ctx)                                                                                                      \
			assert(0);                                                                                                 \
	}

inline static void *sv_dynamic_ptr_check(char *file, int line, void *ptr) {
	if (ptr == NULL) {
		fprintf(stderr, "Survive: memory allocation request failed in file %s, line %d, exiting", file, line);
		exit(EXIT_FAILURE);
	}
	return ptr;
}

#define SV_MALLOC(size) sv_dynamic_ptr_check(__FILE__, __LINE__, malloc(size))
#define SV_CALLOC_N(num, size) sv_dynamic_ptr_check(__FILE__, __LINE__, calloc((num), (size)))
#define SV_CALLOC(size) SV_CALLOC_N(1, size)
#define SV_NEW(type, ...)                                                                                              \
	type##_init(((type *)sv_dynamic_ptr_check(__FILE__, __LINE__, calloc(1, (sizeof(struct type))))), ##__VA_ARGS__)
#define SV_REALLOC(ptr, size) sv_dynamic_ptr_check(__FILE__, __LINE__, realloc(ptr, (size)))

static inline void survive_notify_gen2(struct SurviveObject *so, const char *msg) {
	if (so->ctx->lh_version_forced != -1 && so->ctx->lh_version_forced != 1) {
		return;
	}

	if (so->ctx->lh_version != 1) {
		struct SurviveContext *ctx = so->ctx;
		SV_VERBOSE(100, "Gen2 reason: %s %s", survive_colorize(so->codename), msg);
		SURVIVE_INVOKE_HOOK_SO(gen_detected, so, 1);
	}
}

static inline void survive_notify_gen1(struct SurviveObject *so, const char *msg) {
	if (so->ctx->lh_version_forced != -1 && so->ctx->lh_version_forced != 0) {
		return;
	}

	if (so->ctx->lh_version != 0) {
		struct SurviveContext *ctx = so->ctx;
		SV_VERBOSE(100, "Gen1 reason: %s %s", survive_colorize(so->codename), msg);
		SURVIVE_INVOKE_HOOK_SO(gen_detected, so, 0);
	}
}

#define SV_GENERAL_ERROR(...) SV_ERROR(SURVIVE_ERROR_GENERAL, __VA_ARGS__)

#ifdef __cplusplus
};
#endif

#endif
