#ifndef _SURVIVE_TYPES_H
#define _SURVIVE_TYPES_H

#include "inttypes.h"
#include "linmath.h"
#include "stdint.h"

#ifndef SURVIVE_EXPORT
#ifdef _WIN32
#define SURVIVE_EXPORT __declspec(dllexport)
#define SURVIVE_IMPORT __declspec(dllimport)
#else
#define SURVIVE_EXPORT __attribute__((visibility("default")))
#define SURVIVE_IMPORT SURVIVE_EXPORT
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PRIu64
#define PRIu64 "l" PRIu32
#endif
  
#ifndef FLT
#ifdef USE_DOUBLE
#define FLT double
#else
#define FLT float
#endif
#endif

#define float_format "%+e"
#define double_format "%+le"

#define _FLT_format2(f) f##_format
#define _FLT_format(f) _FLT_format2(f)
#define FLT_format _FLT_format(FLT)

#define Point19_format Point18_format "   " FLT_format
#define Point18_format Point17_format "   " FLT_format
#define Point17_format Point16_format "   " FLT_format
#define Point16_format Point15_format "   " FLT_format
#define Point15_format Point14_format "   " FLT_format
#define Point14_format Point13_format "   " FLT_format
#define Point13_format Point12_format "   " FLT_format
#define Point12_format Point11_format "   " FLT_format
#define Point11_format Point10_format "   " FLT_format
#define Point10_format Point9_format "   " FLT_format
#define Point9_format Point8_format "   " FLT_format
#define Point8_format Point7_format "   " FLT_format
#define Point7_format Point6_format "   " FLT_format
#define Point6_format Point5_format "   " FLT_format
#define Point5_format Point4_format "   " FLT_format
#define Point4_format Point3_format "   " FLT_format
#define Point3_format FLT_format "   " FLT_format "   " FLT_format

#define Quat_format Point4_format
#define SurvivePose_format Point3_format "\t" Quat_format
#define SurviveVel_format Point3_format "\t" Point3_format

#define float_sformat "%f"
#define double_sformat "%lf"
#define _FLT_sformat2(f) f##_sformat
#define _FLT_sformat(f) _FLT_sformat2(f)
#define FLT_sformat _FLT_sformat(FLT)

#define Point3_sformat FLT_sformat "\t" FLT_sformat "\t" FLT_sformat
#define Quat_sformat FLT_sformat "\t" FLT_sformat "\t" FLT_sformat "\t" FLT_sformat
#define SurvivePose_sformat Point3_sformat "\t" Quat_sformat
#define SurviveVel_sformat Point3_sformat "\t" Point3_sformat

#define LINMATH_QUAT_EXPAND(q) (q)[0], (q)[1], (q)[2], (q)[3]
#define LINMATH_VEC3_EXPAND(p) (p)[0], (p)[1], (p)[2]
#define LINMATH_VEC4_EXPAND(p) LINMATH_VEC3_EXPAND(p), (p)[3]
#define LINMATH_VEC5_EXPAND(p) LINMATH_VEC4_EXPAND(p), (p)[4]
#define LINMATH_VEC6_EXPAND(p) LINMATH_VEC5_EXPAND(p), (p)[5]
#define LINMATH_VEC7_EXPAND(p) LINMATH_VEC6_EXPAND(p), (p)[6]
#define LINMATH_VEC8_EXPAND(p) LINMATH_VEC7_EXPAND(p), (p)[7]
#define LINMATH_VEC9_EXPAND(p) LINMATH_VEC8_EXPAND(p), (p)[8]
#define LINMATH_VEC10_EXPAND(p) LINMATH_VEC9_EXPAND(p), (p)[9]
#define LINMATH_VEC11_EXPAND(p) LINMATH_VEC10_EXPAND(p), (p)[10]
#define LINMATH_VEC12_EXPAND(p) LINMATH_VEC11_EXPAND(p), (p)[11]
#define LINMATH_VEC13_EXPAND(p) LINMATH_VEC12_EXPAND(p), (p)[12]
#define LINMATH_VEC14_EXPAND(p) LINMATH_VEC13_EXPAND(p), (p)[13]
#define LINMATH_VEC15_EXPAND(p) LINMATH_VEC14_EXPAND(p), (p)[14]
#define LINMATH_VEC16_EXPAND(p) LINMATH_VEC15_EXPAND(p), (p)[15]
#define LINMATH_VEC17_EXPAND(p) LINMATH_VEC16_EXPAND(p), (p)[16]
#define LINMATH_VEC18_EXPAND(p) LINMATH_VEC17_EXPAND(p), (p)[17]
#define LINMATH_VEC19_EXPAND(p) LINMATH_VEC18_EXPAND(p), (p)[18]

#define SURVIVE_VELOCITY_EXPAND(v) LINMATH_VEC3_EXPAND((v).Pos), LINMATH_VEC3_EXPAND((v).AxisAngleRot)
#define SURVIVE_POSE_EXPAND(p) (p).Pos[0], (p).Pos[1], (p).Pos[2], (p).Rot[0], (p).Rot[1], (p).Rot[2], (p).Rot[3]
#define SURVIVE_POSE_SCAN_EXPAND(p)                                                                                    \
	&(p).Pos[0], &(p).Pos[1], &(p).Pos[2], &(p).Rot[0], &(p).Rot[1], &(p).Rot[2], &(p).Rot[3]

typedef LinmathPose SurvivePose;
typedef LinmathAxisAngleMag SurviveAngularVelocity;
typedef LinmathAxisAnglePose SurviveVelocity;

typedef struct survive_kalman_model_t {
	SurvivePose Pose;
	SurviveVelocity Velocity;
	LinmathVec3d Acc;
	LinmathVec3d GyroBias;
} SurviveKalmanModel;

//Careful with this, you can't just add another one right now, would take minor changes in survive_data.c and the cal tools.
//It will also require a recompile.  TODO: revisit this and correct the comment once fixed.
#define NUM_GEN1_LIGHTHOUSES 2
#define NUM_GEN2_LIGHTHOUSES 16

#define INTBUFFSIZE 64
#define SENSORS_PER_OBJECT	32

// These are used for the eventType of button_process_func
enum SurviveInputEvent {
	SURVIVE_INPUT_EVENT_NONE = 0,
	SURVIVE_INPUT_EVENT_BUTTON_FLAG = 0x2,
	SURVIVE_INPUT_EVENT_BUTTON_DOWN = 0x3,
	SURVIVE_INPUT_EVENT_BUTTON_UP = 0x2,
	SURVIVE_INPUT_EVENT_TOUCH_FLAG = 0x4,
	SURVIVE_INPUT_EVENT_TOUCH_DOWN = 0x5,
	SURVIVE_INPUT_EVENT_TOUCH_UP = 0x4,
	SURVIVE_INPUT_EVENT_AXIS_CHANGED = 0x8,
};

enum SurviveButton {
	SURVIVE_BUTTON_UNKNOWN = 255,

	SURVIVE_BUTTON_TRIGGER = 0,
	SURVIVE_BUTTON_TRACKPAD = 1,
	SURVIVE_BUTTON_THUMBSTICK = 2,
	SURVIVE_BUTTON_SYSTEM = 3,
	SURVIVE_BUTTON_A = 4,
	SURVIVE_BUTTON_B = 5,
	SURVIVE_BUTTON_MENU = 6,
	SURVIVE_BUTTON_GRIP = 7,

	SURVIVE_BUTTON_ON_FACE = 0,
};

enum SurviveAxis {
	SURVIVE_AXIS_UNKNOWN = 255,
	SURVIVE_AXIS_TRIGGER = 1,
	SURVIVE_AXIS_TRACKPAD_X = 2,
	SURVIVE_AXIS_TRACKPAD_Y = 3,
	SURVIVE_AXIS_MIDDLE_FINGER_PROXIMITY = 4,
	SURVIVE_AXIS_RING_FINGER_PROXIMITY = 5,
	SURVIVE_AXIS_PINKY_FINGER_PROXIMITY = 6,
	SURVIVE_AXIS_TRIGGER_FINGER_PROXIMITY = 7,
	SURVIVE_AXIS_GRIP_FORCE = 8,
	SURVIVE_AXIS_TRACKPAD_FORCE = 9,
	SURVIVE_AXIS_JOYSTICK_X = 10,
	SURVIVE_AXIS_JOYSTICK_Y = 11,

	SURVIVE_AXIS_IPD = 0,
	SURVIVE_AXIS_FACE_PROXIMITY = 1
};
typedef float SurviveAxisVal_t;

typedef enum {
	SURVIVE_OBJECT_TYPE_UNKNOWN = 0,
	SURVIVE_OBJECT_TYPE_HMD,
	SURVIVE_OBJECT_TYPE_CONTROLLER,
	SURVIVE_OBJECT_TYPE_OTHER
} SurviveObjectType;

typedef enum {
	SURVIVE_OBJECT_SUBTYPE_GENERIC = 0,
	SURVIVE_OBJECT_SUBTYPE_VIVE_HMD,
	SURVIVE_OBJECT_SUBTYPE_INDEX_HMD,
	SURVIVE_OBJECT_SUBTYPE_WAND,
	SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R,
	SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L,
	SURVIVE_OBJECT_SUBTYPE_TRACKER,
	SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2,
	SURVIVE_OBJECT_SUBTYPE_COUNT
} SurviveObjectSubtype;

typedef uint32_t survive_timecode;
typedef uint64_t survive_long_timecode;

// Lighthouse gen 2 channel/mode
typedef uint8_t survive_channel;

SURVIVE_EXPORT survive_timecode survive_timecode_difference(survive_timecode most_recent, survive_timecode least_recent);

typedef struct SurviveObject SurviveObject;
typedef struct SurviveContext SurviveContext;
typedef struct BaseStationData BaseStationData;
typedef struct SurviveCalData SurviveCalData;   //XXX Warning: This may be removed.  Check at a later time for its defunctness.

typedef enum {
	SURVIVE_OK = 0,
	SURVIVE_ERROR_GENERAL = -1,
	SURVIVE_ERROR_NO_TRACKABLE_OBJECTS = -2,
	SURVIVE_ERROR_HARWARE_FAULT = -3,
	SURVIVE_ERROR_INVALID_CONFIG = -4
} SurviveError;

typedef enum {
	SURVIVE_LOG_LEVEL_ERROR = 0,
	SURVIVE_LOG_LEVEL_WARNING = 1,
	SURVIVE_LOG_LEVEL_INFO = 2,
} SurviveLogLevel;

typedef void (*survive_driver_fn)();

typedef void (*datalog_process_func)(SurviveObject *so, const char *name, const FLT *v, size_t length);
typedef int (*printf_process_func)(SurviveContext *ctx, const char *format, ...);
typedef void (*log_process_func)(SurviveContext *ctx, SurviveLogLevel logLevel, const char *fault);
typedef void (*report_error_process_func)(SurviveContext *ctx, SurviveError error);

typedef int (*config_process_func)(SurviveObject *so, char *ct0conf, int len);

// For lightcap, etc.  Don't change this structure at all.  Regular vive is dependent on it being exactly as-is.
// When you write drivers, you can use this to send survive lightcap data.
typedef struct {
	uint8_t sensor_id;
	// Length of pulse
	uint16_t length;
	// Start of pulse
	uint32_t timestamp;
} LightcapElement;

/************************************************ Hook definitions ****************************************************/
/**
 * This is called when libsurvive figures out if its looking at a gen1 or gen2 system.
 */
typedef void (*gen_detected_process_func)(SurviveObject *so, int gen);

// LH1 specific callbacks
/**
 * This processes the raw light data for lighthouse v1 systems.
 */
typedef void (*lightcap_process_func)(SurviveObject *so, const LightcapElement *le);

/**
 * This is called on disambiguated data in a v1 system; so it contains the lighthouse index of the data as well as
 * the time in sweep of the event.
 */
typedef void (*light_process_func)(SurviveObject *so, int sensor_id, int acode, int timeinsweep,
								   survive_timecode timecode, survive_timecode length, uint32_t lighthouse);

typedef void (*ootx_received_process_func)(struct SurviveContext *ctx, uint8_t bsd_idx);

typedef void (*light_pulse_process_func)(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode,
										 FLT length, uint32_t lh);

/**
 * This is called with the calculated angle and axis for a single lighthouse sensor event
 */
typedef void (*angle_process_func)(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode, FLT length,
								   FLT angle, uint32_t lh);

// LH2 specific callbacks
/**
 * Represents a sync event which shows the base station rotor was at 0 degrees at the given time. The channel is
 * the channel of the lighthouse _not_ the index of it.
 */
typedef void (*sync_process_func)(SurviveObject *so, survive_channel channel, survive_timecode timeofsync, bool ootx,
								  bool gen);
/**
 * Represents a sweep event which shows the given sensor was hit by the given channel at a given time with the channel
 * of the lighthouse _not_ the index of it.
 */
typedef void (*sweep_process_func)(SurviveObject *so, survive_channel channel, int sensor_id, survive_timecode timecode,
								   bool half_clock_flag);

/**
 * The calculated angle and plane of a gen2 event along a given sensor and channel.
 */
typedef void (*sweep_angle_process_func)(SurviveObject *so, survive_channel channel, int sensor_id,
										 survive_timecode timecode, int8_t plane, FLT angle);

/**
 * Raw accelerometer data straight from the device; with no scaling or bias applied. accelgyro is a vector of length
 * 6 in [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z ]
 */
typedef void (*raw_imu_process_func)(SurviveObject *so, int mask, FLT *accelgyro, survive_timecode timecode, int id);

/**
 * Processed accelerometer data straight from the device; with no scaling or bias applied. accelgyro is a vector of
 * length 6 in [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z ]. Acc is scaled to G's -- a stationary object should have a
 * norm of
 * 1. The gyro is in units of radians.
 */
typedef void (*imu_process_func)(SurviveObject *so, int mask, FLT *accelgyro, survive_timecode timecode, int id);

/**
 * A general button press event
 */
typedef void (*button_process_func)(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
									const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals);

/**
 * Called when a pose is solved for at a given time. Given in 'tracking' coordinate frame.
 */
typedef void (*pose_process_func)(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose);

/**
 * Called when a pose is solve for at a given time. Given in the IMU coordinate frame.
 */
typedef pose_process_func imupose_process_func;

/**
 * Called when a new velocity estimate is calculated. Velocity is in global frame.
 */
typedef void (*velocity_process_func)(SurviveObject *so, survive_long_timecode timecode, const SurviveVelocity *pose);

/**
 * External pose and velocity callbacks are called by a few drivers to expose external localizations into libsurvive.
 * The names are unique keys for that object.
 */
typedef void (*external_pose_process_func)(SurviveContext *so, const char *name, const SurvivePose *pose);
typedef void (*external_velocity_process_func)(SurviveContext *so, const char *name, const SurviveVelocity *velocity);

/**
 * Called when a lighthouse has a new estimated position.
 */
typedef void (*lighthouse_pose_process_func)(SurviveContext *ctx, uint8_t bsd_idx, SurvivePose *lighthouse_pose);

/**
 * Called when a new object is added into the system.
 */
typedef void (*new_object_process_func)(SurviveObject *so);
/************************************************ End Hook definitions ************************************************/

typedef int (*haptic_func)(SurviveObject *so, FLT freq, FLT amp, FLT duration);

//Device drivers (prefix your drivers with "DriverReg") i.e.
//		REGISTER_LINKTIME( DriverRegHTCVive );
typedef int (*DeviceDriver)( SurviveContext * ctx );
typedef enum SurviveDeviceDriverReturn {
	SURVIVE_DRIVER_NORMAL = 0, // Driver OK
	SURVIVE_DRIVER_ERROR = -1, // Indicates some form of error in loading the devices
	SURVIVE_DRIVER_PASSIVE = 1 // No errors, but driver is a passive one; still allow loading default drivers.
} SurviveDeviceDriverReturn;

typedef int (*DeviceDriverCb)(struct SurviveContext *ctx, void *driver);
typedef int (*DeviceDriverMagicCb)( struct SurviveContext * ctx, void * driver, int magic_code, void * data, int datalen );

SURVIVE_EXPORT const char *SurviveInputEventStr(enum SurviveInputEvent evt);
SURVIVE_EXPORT const char *SurviveButtonsStr(SurviveObjectSubtype objectSubtype, enum SurviveButton b);
SURVIVE_EXPORT const char *SurviveAxisStr(SurviveObjectSubtype objectSubtype, enum SurviveAxis b);

SURVIVE_EXPORT const char *SurviveObjectTypeStr(SurviveObjectType t);
SURVIVE_EXPORT const char *SurviveObjectSubtypeStr(SurviveObjectSubtype t);
#ifdef __cplusplus
};
#endif

#endif

