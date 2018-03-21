#ifndef _LSPOSER_H
#define _LSPOSER_H

#include "survive_types.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef enum PoserType_t {
	POSERDATA_NONE = 0,
	POSERDATA_IMU,
	POSERDATA_LIGHT,		// Single lighting event.
	POSERDATA_FULL_SCENE,   // Full, statified X, Y sweep data for both lighthouses.
	POSERDATA_DISASSOCIATE, // If you get this, it doesn't contain data.  It just tells you to please disassociate from
							// the current SurviveObject and delete your poserdata.
	POSERDATA_SYNC, // Sync pulse.
} PoserType;

typedef void (*poser_raw_pose_func)(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose, void *user);
typedef void (*poser_lighthouse_pose_func)(SurviveObject *so, uint8_t lighthouse, SurvivePose *lighthouse_pose,
										   SurvivePose *object_pose, void *user);

typedef struct
{
	PoserType pt;
	poser_raw_pose_func rawposeproc;
	poser_lighthouse_pose_func lighthouseposeproc;
	void *userdata;
} PoserData;

void PoserData_poser_raw_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse, SurvivePose *pose);
void PoserData_lighthouse_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse,
									/* OUTPARAM */ SurvivePose *objUp2world, SurvivePose *lighthouse_poses,
									SurvivePose *object_pose);

typedef struct PoserDataIMU {
	PoserData hdr;
	uint8_t datamask;  //0 = accel present, 1 = gyro present, 2 = mag present.
	FLT accel[3];
	FLT gyro[3];
	FLT mag[3];
	uint32_t timecode; //In object-local ticks.
} PoserDataIMU;

typedef struct PoserDataLight {
	PoserData hdr;
	int sensor_id;
	int acode;			//OOTX Code associated with this sweep. bit 1 indicates vertical(1) or horizontal(0) sweep
	int lh;             //Lighthouse making this sweep
	uint32_t timecode;  //In object-local ticks.
	FLT length;			//In seconds
	FLT angle;			//In radians from center of lighthouse.
} PoserDataLight;

typedef struct
{
	PoserData hdr;

	//If "lengths[...]" < 0, means not a valid piece of sweep information.
	FLT  lengths[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];
	FLT  angles [SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];  //2 Axes  (Angles in LH space)
	FLT  synctimes[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES];

	PoserDataIMU lastimu;
} PoserDataFullScene;

//When you write your posers, use the following definition, and register with REGISTER_LINKTIME.
typedef int (*PoserCB)( SurviveObject * so, PoserData * pd );


#ifdef __cplusplus
};
#endif

#endif
