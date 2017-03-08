#ifndef _LSPOSER_H
#define _LSPOSER_H

#include "survive_types.h"

typedef enum PoserType_t
{
	POSERDATA_NONE = 0,
	POSERDATA_IMU,
	POSERDATA_LIGHT,		//Single lighting event.  
	POSERDATA_FULL_SCENE, 	//Full, statified X, Y sweep data for both lighthouses.
} PoserType;

typedef struct
{
	PoserType pt;
	uint8_t data[0];
} PoserData;

typedef struct
{
	PoserType pt;
	uint8_t datamask;  //0 = accel present, 1 = gyro present, 2 = mag present.
	FLT accel[3];
	FLT gyro[3];
	FLT mag[3];
} PoserDataIMU;

typedef struct
{
	PoserType pt;
	int sensor_id;
	int acode;			//OOTX Code associated with this sweep. base_station = acode >> 2;  axis = acode & 1;
	uint32_t timecode;  //In object-local ticks.
	FLT length;			//In seconds
	FLT angle;			//In radians from center of lighthouse.
} PoserDataLight;

typedef struct
{
	PoserType pt;

	//If "lengths[...]" < 0, means not a valid piece of sweep information.
	FLT  lengths[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];
	FLT  angles [SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];  //2 Axes  (Angles in LH space)

	PoserDataIMU lastimu;
} PoserDataFullScene;

//When you register your posers using the internal system, 
typedef int (*PoserCB)( SurviveObject * so, PoserData * pd );


#endif
