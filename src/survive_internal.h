//<>< (C) 2016-2017 C. N. Lohr, MOSTLY Under MIT/x11 License.
//
//Based off of https://github.com/collabora/OSVR-Vive-Libre
// Originally Copyright 2016 Philipp Zabel
// Originally Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
// Originally Copyright (C) 2013 Fredrik Hultin
// Originally Copyright (C) 2013 Jakob Bornecrantz
//
//But, re-written as best as I can to get it put under an open souce license instead of a forced-source license.
//If there are portions of the code too similar to the original, I would like to know  so they can be re-written.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.


#ifndef _SURVIVE_INTERNAL_H
#define _SURVIVE_INTERNAL_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "survive_driverman.h"
#include <zlib.h>
#include <survive.h>

#define SV_INFO( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->notefunction( ctx, stbuff ); }
#define SV_ERROR( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->faultfunction( ctx, stbuff ); }

//XXX TODO This one needs to be rewritten.
#define SV_KILL()		exit(0)

#define INTBUFFSIZE			64
#define SENSORS_PER_OBJECT	32

struct SurviveContext;
struct SurviveUSBInterface;

typedef int (*DeviceDriverCb)( struct SurviveContext * ctx, void * driver );
typedef int (*DeviceDriverMagicCb)( struct SurviveContext * ctx, void * driver, int magic_code, void * data, int datalen );

//This is defined in survive.h
struct SurviveObject;
struct SurviveCalData;

struct BaseStationData
{
	uint8_t PositionSet:1;
	FLT Position[3];
	FLT Quaternion[4];

	uint8_t OOTXSet:1;
	uint32_t BaseStationID;
	FLT fcalphase[2];
	FLT fcaltilt[2];
	FLT fcalcurve[2];
	FLT fcalgibpha[2];
	FLT fcalgibmag[2];
};

struct SurviveContext
{
	text_feedback_func faultfunction;
	text_feedback_func notefunction;
	light_process_func lightproc;
	imu_process_func imuproc;
	angle_process_func angleproc;

	//Calibration data:
	struct BaseStationData bsd[NUM_LIGHTHOUSES];

	struct SurviveCalData * calptr; //If and only if the calibration subsystem is attached.

	struct SurviveObject ** objs;
	int objs_ct;

	void ** drivers;
	DeviceDriverCb * driverpolls;
	DeviceDriverCb * drivercloses;
	DeviceDriverMagicCb * drivermagics;
	int driver_ct;
};

int survive_add_object( struct SurviveContext * ctx, struct SurviveObject * obj );

void survive_add_driver( struct SurviveContext * ctx, void * payload, DeviceDriverCb poll, DeviceDriverCb close, DeviceDriverMagicCb magic );

//For lightcap, etc.  Don't change this structure at all.  Regular vive is dependent on it being exactly as-is.
struct LightcapElement
{
	uint8_t sensor_id;
	uint8_t type;
	uint16_t length;
	uint32_t timestamp;
} __attribute__((packed));

//This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
void handle_lightcap( struct SurviveObject * so, struct LightcapElement * le );


//Accept Data from backend.
void survive_data_cb( struct SurviveUSBInterface * si );


#endif


