#ifndef _SURVIVE_TYPES_H
#define _SURVIVE_TYPES_H

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

typedef struct SurvivePose
{
	FLT  Pos[3];
	FLT  Rot[4];
} SurvivePose;

//Careful with this, you can't just add another one right now, would take minor changes in survive_data.c and the cal tools.
//It will also require a recompile.  TODO: revisit this and correct the comment once fixed.
#define NUM_LIGHTHOUSES 2  

#define INTBUFFSIZE			64
#define SENSORS_PER_OBJECT	32

// These are used for the eventType of button_process_func
#define BUTTON_EVENT_BUTTON_NONE   0
#define BUTTON_EVENT_BUTTON_DOWN   1
#define BUTTON_EVENT_BUTTON_UP     2
#define BUTTON_EVENT_AXIS_CHANGED  3

typedef struct SurviveObject SurviveObject;
typedef struct SurviveContext SurviveContext;
typedef struct BaseStationData BaseStationData;
typedef struct SurviveCalData SurviveCalData;   //XXX Warning: This may be removed.  Check at a later time for its defunctness.

typedef void (*text_feedback_func)( SurviveContext * ctx, const char * fault );
typedef void (*light_process_func)( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lighthouse);
typedef void (*imu_process_func)( SurviveObject * so, int mask, FLT * accelgyro, uint32_t timecode, int id );
typedef void (*angle_process_func)( SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh);
typedef void (*button_process_func)(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val);

//Device drivers (prefix your drivers with "DriverReg") i.e.
//		REGISTER_LINKTIME( DriverRegHTCVive );
typedef int (*DeviceDriver)( SurviveContext * ctx );
typedef int (*DeviceDriverCb)( struct SurviveContext * ctx, void * driver );
typedef int (*DeviceDriverMagicCb)( struct SurviveContext * ctx, void * driver, int magic_code, void * data, int datalen );

#ifdef __cplusplus
};
#endif

#endif

