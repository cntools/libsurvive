#ifndef _SURVIVE_TYPES_H
#define _SURVIVE_TYPES_H

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
#define BUTTON_EVENT_BUTTON_NONE   0
#define BUTTON_EVENT_BUTTON_DOWN   1
#define BUTTON_EVENT_BUTTON_UP     2
#define BUTTON_EVENT_AXIS_CHANGED  3

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

// LH1 specific callbacks
typedef void (*lightcap_process_func)(SurviveObject *so, const LightcapElement *le);
typedef void (*light_process_func)(SurviveObject *so, int sensor_id, int acode, int timeinsweep,
								   survive_timecode timecode, survive_timecode length, uint32_t lighthouse);
typedef void (*angle_process_func)(SurviveObject *so, int sensor_id, int acode, survive_timecode timecode, FLT length,
								   FLT angle, uint32_t lh);

// LH2 specific callbacks
typedef void (*gen_detected_process_func)(SurviveObject *so, int gen);
typedef void (*sync_process_func)(SurviveObject *so, survive_channel channel, survive_timecode timeinsweep, bool ootx,
								  bool gen);
typedef void (*sweep_process_func)(SurviveObject *so, survive_channel channel, int sensor_id, survive_timecode timecode,
								   bool half_clock_flag);

// Angle is defined as the rotor angle at time of sweep
typedef void (*sweep_angle_process_func)(SurviveObject *so, survive_channel channel, int sensor_id,
										 survive_timecode timecode, int8_t plane, FLT angle);

typedef void (*raw_imu_process_func)(SurviveObject *so, int mask, FLT *accelgyro, survive_timecode timecode, int id);
typedef void (*imu_process_func)(SurviveObject *so, int mask, FLT *accelgyro, survive_timecode timecode, int id);
typedef void (*button_process_func)(SurviveObject *so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id,
									uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val);
typedef void (*pose_process_func)(SurviveObject *so, survive_timecode timecode, SurvivePose *pose);
typedef void (*velocity_process_func)(SurviveObject *so, survive_timecode timecode, const SurviveVelocity *pose);
typedef void (*external_pose_process_func)(SurviveContext *so, const char *name, const SurvivePose *pose);
typedef void (*external_velocity_process_func)(SurviveContext *so, const char *name, const SurviveVelocity *velocity);
typedef void (*lighthouse_pose_process_func)(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *lighthouse_pose,
											 SurvivePose *object_pose);

typedef int(*haptic_func)(SurviveObject * so, uint8_t reserved, uint16_t pulseHigh , uint16_t pulseLow, uint16_t repeatCount);

typedef void (*new_object_process_func)(SurviveObject *so);

//Device drivers (prefix your drivers with "DriverReg") i.e.
//		REGISTER_LINKTIME( DriverRegHTCVive );
typedef int (*DeviceDriver)( SurviveContext * ctx );
typedef enum SurviveDeviceDriverReturn {
	SURVIVE_DRIVER_NORMAL = 0, // Driver OK
	SURVIVE_DRIVER_ERROR = -1, // Indicates some form of error in loading the devices
	SURVIVE_DRIVER_PASSIVE = 1 // No errors, but driver is a passive one; still allow loading default drivers.
} SurviveDeviceDriverReturn;

typedef int (*DeviceDriverCb)( struct SurviveContext * ctx, void * driver );
typedef int (*DeviceDriverMagicCb)( struct SurviveContext * ctx, void * driver, int magic_code, void * data, int datalen );

#ifdef __cplusplus
};
#endif

#endif

