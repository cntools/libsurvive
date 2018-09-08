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
	FLT angles[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];		   // 2 Axes (Angles in LH space)
	survive_timecode timecode[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2]; // Timecode per axis in ticks
	survive_timecode lengths[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];  // Timecode per axis in ticks

	FLT accel[3];
	FLT gyro[3];
	FLT mag[3];
} SurviveSensorActivations;

struct PoserDataLight;
struct PoserDataIMU;

/**
 * Adds a lightData packet to the table.
 */
void SurviveSensorActivations_add(SurviveSensorActivations *self, struct PoserDataLight *lightData);

void SurviveSensorActivations_add_imu(SurviveSensorActivations *self, struct PoserDataIMU *imuData);

/**
 * Returns true iff the given sensor and lighthouse at given axis were seen at most `tolerance` ticks before the given
 * `timecode_now`.
 */
bool SurviveSensorActivations_isReadingValid(const SurviveSensorActivations *self, survive_timecode tolerance,
											 survive_timecode timecode_now, uint32_t sensor_idx, int lh, int axis);

/**
 * Returns true iff both angles for the given sensor and lighthouse were seen at most `tolerance` ticks before the given
 * `timecode_now`.
 */
bool SurviveSensorActivations_isPairValid(const SurviveSensorActivations *self, survive_timecode tolerance,
										  survive_timecode timecode_now, uint32_t sensor_idx, int lh);

/**
 * Default tolerance that gives a somewhat accuate representation of current state.
 *
 * Don't rely on this to be a given value.
 */
extern survive_timecode SurviveSensorActivations_default_tolerance;

// DANGER: This structure may be redefined.  Note that it is logically split into 64-bit chunks
// for optimization on 32- and 64-bit systems.

struct SurviveObject {
	SurviveContext *ctx;

	char codename[4];   // 3 letters, null-terminated.  Currently HMD, WM0, WM1.
	char drivername[4]; // 3 letters for driver.  Currently "HTC"
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
	SurvivePose OutPose;					 // Final pose? (some day, one can dream!)
	survive_timecode OutPose_timecode;
	SurvivePose FromLHPose[NUM_LIGHTHOUSES]; // Filled out by poser, contains computed position from each lighthouse.
	void *PoserData; // Initialized to zero, configured by poser, can be anything the poser wants.
	PoserCB PoserFn;

	// Device-specific information about the location of the sensors.  This data will be used by the poser.
	int8_t sensor_ct;	  // sensor count
	FLT *sensor_locations; // size is sensor_ct*3.  Contains x,y,z values for each sensor
	FLT *sensor_normals;   // size is nrlocations*3.  cointains normal vector for each sensor

	// Timing sensitive data (mostly for disambiguation)
	int32_t timebase_hz;		  // 48,000,000 for normal vive hardware.  (checked)

	// Flood info, for calculating which laser is currently sweeping.
	void *disambiguator_data;
	int8_t oldcode;
	int8_t sync_set_number; // 0 = master, 1 = slave, -1 = fault.
	int8_t did_handle_ootx; // If unset, will send lightcap data for sync pulses next time a sensor is hit.
	survive_timecode last_sync_time[NUM_LIGHTHOUSES];
	survive_timecode last_sync_length[NUM_LIGHTHOUSES];
	survive_timecode recent_sync_time;

	survive_timecode last_lighttime; // May be a 24- or 32- bit number depending on what device.

	FLT imu_freq;
	SurvivePose relative_imu_pose;
	FLT *acc_bias;   // size is FLT*3. contains x,y,z
	FLT *acc_scale;  // size is FLT*3. contains x,y,z
	FLT *gyro_bias;  // size is FLT*3. contains x,y,z
	FLT *gyro_scale; // size is FLT*3. contains x,y,z

	haptic_func haptic;

	SurviveSensorActivations activations;
	void *user_ptr;
	// Debug
	int tsl;
};

// These exports are mostly for language binding against
SURVIVE_EXPORT const char *survive_object_codename(SurviveObject *so);

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
};

struct config_group;

#define BUTTON_QUEUE_MAX_LEN 32
#define MAX_SHORTHAND_CONFIGS 32

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
struct survive_calibration_config;

enum SurviveCalFlag {
	SVCal_None = 0,
	SVCal_Phase = 1,
	SVCal_Tilt = 2,
	SVCal_Curve = 4,
	SVCal_Gib = 8,
	SVCal_All = SVCal_Gib | SVCal_Curve | SVCal_Tilt | SVCal_Phase
};

struct SurviveContext {
	text_feedback_func faultfunction;
	text_feedback_func notefunction;
	light_process_func lightproc;
	imu_process_func imuproc;
	angle_process_func angleproc;
	button_process_func buttonproc;
	pose_func poseproc;
	lighthouse_pose_func lighthouseposeproc;
	htc_config_func configfunction;
	handle_lightcap_func lightcapfunction;
	// Calibration data:
	int activeLighthouses;
	BaseStationData bsd[NUM_LIGHTHOUSES];
	SurviveCalData *calptr;				 // If and only if the calibration subsystem is attached.
	struct SurviveRecordingData *recptr; // Iff recording is attached
	SurviveObject **objs;
	int objs_ct;

	void **drivers;
	DeviceDriverCb *driverpolls;
	DeviceDriverCb *drivercloses;
	DeviceDriverMagicCb *drivermagics;
	int driver_ct;

	SurviveState state;

	void *buttonservicethread;
	ButtonQueue buttonQueue;

	void *user_ptr;

	struct config_group *global_config_values;
	struct config_group *lh_config; // lighthouse configs
	struct config_group	*temporary_config_values; // Set per-session, from command-line. Not saved but override global_config_values
};

SURVIVE_EXPORT void survive_verify_FLT_size(
	uint32_t user_size); // Baked in size of FLT to verify users of the library have the correct setting.

SURVIVE_EXPORT SurviveContext *survive_init_internal(int argc, char *const *argv);

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
	survive_verify_FLT_size(sizeof(FLT));
	return survive_init_internal(argc, argv);
}

// For any of these, you may pass in 0 for the function pointer to use default behavior.
// In general unless you are doing wacky things like recording or playing back data, you won't need to use this.
SURVIVE_EXPORT void survive_install_htc_config_fn(SurviveContext *ctx, htc_config_func fbp);
SURVIVE_EXPORT void survive_install_info_fn(SurviveContext *ctx, text_feedback_func fbp);
SURVIVE_EXPORT void survive_install_error_fn(SurviveContext *ctx, text_feedback_func fbp);
SURVIVE_EXPORT void survive_install_light_fn(SurviveContext *ctx, light_process_func fbp);
SURVIVE_EXPORT void survive_install_imu_fn(SurviveContext *ctx, imu_process_func fbp);
SURVIVE_EXPORT void survive_install_angle_fn(SurviveContext *ctx, angle_process_func fbp);
SURVIVE_EXPORT void survive_install_button_fn(SurviveContext *ctx, button_process_func fbp);
SURVIVE_EXPORT void survive_install_pose_fn(SurviveContext *ctx, pose_func fbp);
SURVIVE_EXPORT void survive_install_lighthouse_pose_fn(SurviveContext *ctx, lighthouse_pose_func fbp);
SURVIVE_EXPORT int survive_startup(SurviveContext *ctx);
SURVIVE_EXPORT int survive_poll(SurviveContext *ctx);
SURVIVE_EXPORT void survive_close(SurviveContext *ctx);

SURVIVE_EXPORT SurviveObject *survive_get_so_by_name(SurviveContext *ctx, const char *name);

// Utilitiy functions.
int survive_simple_inflate(SurviveContext *ctx, const uint8_t *input, int inlen, uint8_t *output, int outlen);
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

#define STATIC_CONFIG_ITEM( variable, name, type, description, default_value ) \
	SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##variable() { survive_config_bind_variable( type, name, description, default_value ); }
	void survive_config_bind_variable( char vt, const char * name, const char * description, ... ); //Only used at boot.

// Install the calibrator.
SURVIVE_EXPORT void survive_cal_install(SurviveContext *ctx); // XXX This will be removed if not already done so.

// Read back a human-readable string description of the calibration status
SURVIVE_EXPORT int survive_cal_get_status(SurviveContext *ctx, char *description, int description_length);

// Induce haptic feedback
SURVIVE_EXPORT int survive_haptic(SurviveObject *so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow,
								  uint16_t repeatCount);

// Call these from your callback if overridden.
// Accept higher-level data.
SURVIVE_EXPORT void survive_default_light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep,
												  survive_timecode timecode, survive_timecode length, uint32_t lh);
SURVIVE_EXPORT void survive_default_imu_process(SurviveObject *so, int mode, FLT *accelgyro, survive_timecode timecode, int id);
SURVIVE_EXPORT void survive_default_angle_process(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode,
												  FLT length, FLT angle, uint32_t lh);
SURVIVE_EXPORT void survive_default_button_process(SurviveObject *so, uint8_t eventType, uint8_t buttonId,
												   uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id,
												   uint16_t axis2Val);
SURVIVE_EXPORT void survive_default_raw_pose_process(SurviveObject *so, survive_timecode timecode, SurvivePose *pose);
SURVIVE_EXPORT void survive_default_lighthouse_pose_process(SurviveContext *ctx, uint8_t lighthouse,
															SurvivePose *lh_pose, SurvivePose *obj_pose);
SURVIVE_EXPORT int survive_default_htc_config_process(SurviveObject *so, char *ct0conf, int len);

////////////////////// Survive Drivers ////////////////////////////

void RegisterDriver(const char *name, void *data);

#define REGISTER_LINKTIME(func)                                                                                        \
	SURVIVE_EXPORT_CONSTRUCTOR void REGISTER##func() { RegisterDriver(#func, &func); }

///////////////////////// General stuff for writing drivers ///////

// For device drivers to call.  This actually attaches them.
int survive_add_object(SurviveContext *ctx, SurviveObject *obj);
void survive_remove_object(SurviveContext *ctx, SurviveObject *obj);
void survive_add_driver(SurviveContext *ctx, void *payload, DeviceDriverCb poll, DeviceDriverCb close,
						DeviceDriverMagicCb magic);

// This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
void handle_lightcap(SurviveObject *so, LightcapElement *le);

#define SV_INFO(...)                                                                                                   \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		sprintf(stbuff, __VA_ARGS__);                                                                                  \
		ctx->notefunction(ctx, stbuff);                                                                                \
	}
#define SV_ERROR(...)                                                                                                  \
	{                                                                                                                  \
		char stbuff[1024];                                                                                             \
		sprintf(stbuff, __VA_ARGS__);                                                                                  \
		ctx->faultfunction(ctx, stbuff);                                                                               \
	}
#define SV_KILL() exit(0) // XXX This should likely be re-defined.

#ifdef __cplusplus
};
#endif

#endif
