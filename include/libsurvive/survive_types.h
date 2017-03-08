#ifndef _SURVIVE_TYPES_H
#define _SURVIVE_TYPES_H

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

typedef struct SurviveObject SurviveObject;
typedef struct SurviveContext SurviveContext;

typedef void (*text_feedback_func)( SurviveContext * ctx, const char * fault );
typedef void (*light_process_func)( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length );
typedef void (*imu_process_func)( SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id );
typedef void (*angle_process_func)( SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle );

#endif

