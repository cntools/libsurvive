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

#define float_format "%f"
#define double_format "%lf"
#define _FLT_format2(f) f##_format
#define _FLT_format(f) _FLT_format2(f)
#define FLT_format _FLT_format(FLT)

#define Point3_format FLT_format "\t" FLT_format "\t" FLT_format
#define Quat_format FLT_format "\t" FLT_format "\t" FLT_format "\t" FLT_format
#define SurvivePose_format Point3_format "\t" Quat_format
#define SurviveVel_format Point3_format "\t" Point3_format
#define LINMATH_QUAT_EXPAND(q) (q)[0], (q)[1], (q)[2], (q)[3]
#define LINMATH_VEC3_EXPAND(p) (p)[0], (p)[1], (p)[2]
#define SURVIVE_VELOCITY_EXPAND(v) LINMATH_VEC3_EXPAND((v).Pos), LINMATH_VEC3_EXPAND((v).EulerRot)
#define SURVIVE_POSE_EXPAND(p) (p).Pos[0], (p).Pos[1], (p).Pos[2], (p).Rot[0], (p).Rot[1], (p).Rot[2], (p).Rot[3]
typedef LinmathPose SurvivePose;
typedef LinmathEulerPose SurviveVelocity;

//Careful with this, you can't just add another one right now, would take minor changes in survive_data.c and the cal tools.
//It will also require a recompile.  TODO: revisit this and correct the comment once fixed.
#define NUM_LIGHTHOUSES 2

#define INTBUFFSIZE 64
#define SENSORS_PER_OBJECT	32

// These are used for the eventType of button_process_func
#define BUTTON_EVENT_BUTTON_NONE   0
#define BUTTON_EVENT_BUTTON_DOWN   1
#define BUTTON_EVENT_BUTTON_UP     2
#define BUTTON_EVENT_AXIS_CHANGED  3

typedef uint32_t survive_timecode;
SURVIVE_EXPORT survive_timecode survive_timecode_difference(survive_timecode most_recent, survive_timecode least_recent);

typedef struct SurviveObject SurviveObject;
typedef struct SurviveContext SurviveContext;
typedef struct BaseStationData BaseStationData;
typedef struct SurviveCalData SurviveCalData;   //XXX Warning: This may be removed.  Check at a later time for its defunctness.

typedef int (*htc_config_func)(SurviveObject *so, char *ct0conf, int len);
typedef void (*text_feedback_func)( SurviveContext * ctx, const char * fault );
typedef void (*light_process_func)( SurviveObject * so, int sensor_id, int acode, int timeinsweep, survive_timecode timecode, survive_timecode length, uint32_t lighthouse);
typedef void (*imu_process_func)( SurviveObject * so, int mask, FLT * accelgyro, survive_timecode timecode, int id );
typedef void (*angle_process_func)( SurviveObject * so, int sensor_id, int acode, survive_timecode timecode, FLT length, FLT angle, uint32_t lh);
typedef void(*button_process_func)(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val);
typedef void (*pose_func)(SurviveObject *so, survive_timecode timecode, SurvivePose *pose);
typedef void (*velocity_func)(SurviveObject *so, survive_timecode timecode, const SurviveVelocity *pose);
typedef void (*external_pose_func)(SurviveContext *so, const char *name, const SurvivePose *pose);
typedef void (*external_velocity_func)(SurviveContext *so, const char *name, const SurviveVelocity *velocity);
typedef void (*lighthouse_pose_func)(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *lighthouse_pose, SurvivePose *object_pose);

// For lightcap, etc.  Don't change this structure at all.  Regular vive is dependent on it being exactly as-is.
// When you write drivers, you can use this to send survive lightcap data.
typedef struct {
	uint8_t sensor_id;
	uint16_t length;
	uint32_t timestamp;
} LightcapElement;

typedef void (*handle_lightcap_func)(SurviveObject *so, LightcapElement *le);

typedef int(*haptic_func)(SurviveObject * so, uint8_t reserved, uint16_t pulseHigh , uint16_t pulseLow, uint16_t repeatCount);

//Device drivers (prefix your drivers with "DriverReg") i.e.
//		REGISTER_LINKTIME( DriverRegHTCVive );
typedef int (*DeviceDriver)( SurviveContext * ctx );
typedef int (*DeviceDriverCb)( struct SurviveContext * ctx, void * driver );
typedef int (*DeviceDriverMagicCb)( struct SurviveContext * ctx, void * driver, int magic_code, void * data, int datalen );

#ifdef __cplusplus
};
#endif

#endif

