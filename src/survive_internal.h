//<>< (C) 2016 C. N. Lohr, MOSTLY Under MIT/x11 License.
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
#include <libusb-1.0/libusb.h>
#include <zlib.h>
#include <survive.h>

#define SV_INFO( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->notefunction( ctx, stbuff ); }
#define SV_ERROR( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->faultfunction( ctx, stbuff ); }

//XXX TODO This one needs to be rewritten.
#define SV_KILL()		exit(0)

#define USB_DEV_HMD			0
#define USB_DEV_LIGHTHOUSE	1
#define USB_DEV_WATCHMAN1	2
#define USB_DEV_WATCHMAN2	3
#define MAX_USB_DEVS				4


#define USB_IF_HMD			0
#define USB_IF_LIGHTHOUSE 	1
#define USB_IF_WATCHMAN1	2
#define USB_IF_WATCHMAN2	3
#define USB_IF_LIGHTCAP		4
#define MAX_INTERFACES				5

#define INTBUFFSIZE			64
#define SENSORS_PER_OBJECT	32

struct SurviveContext;
struct SurviveUSBInterface;

typedef void (*usb_callback)( struct SurviveUSBInterface * ti );

struct SurviveUSBInterface
{
	struct libusb_transfer * transfer;
	struct SurviveContext * ctx;
	int actual_len;
	uint8_t buffer[INTBUFFSIZE];
	usb_callback cb;
	int which_interface_am_i;	//for indexing into uiface
	const char * hname;			//human-readable names
};

//This is defined in survive.h
struct SurviveObject;
struct SurviveCalData;

struct BaseStationData
{
	uint8_t PositionSet:1;
	float Position[3];
	float Quaternion[4];

	uint8_t OOTXSet:1;
	uint32_t BaseStationID;
	float fcalphase[2];
	float fcaltilt[2];
	float fcalcurve[2];
	float fcalgibpha[2];
	float fcalgibmag[2];
};

struct SurviveContext
{
	//USB Subsystem
    struct libusb_context* usbctx;
	struct libusb_device_handle * udev[MAX_USB_DEVS];
	struct SurviveUSBInterface uiface[MAX_INTERFACES];

	text_feedback_func faultfunction;
	text_feedback_func notefunction;
	light_process_func lightproc;
	imu_process_func imuproc;
	angle_process_func angleproc;


	//Calibration data:
	struct BaseStationData bsd[NUM_LIGHTHOUSES];

	struct SurviveCalData * calptr; //If and only if the calibration subsystem is attached.

	//Data Subsystem.  These should be last, as there may be additional surviveobjects.
	struct SurviveObject headset;
	struct SurviveObject watchman[2]; //Currently only two supported watchmen.

};


//USB Subsystem 
void survive_usb_close( struct SurviveContext * t );
int survive_usb_init( struct SurviveContext * t );
int survive_usb_poll( struct SurviveContext * ctx );
int survive_get_config( char ** config, struct SurviveContext * ctx, int devno, int interface, int send_extra_magic );

//Accept Data from backend.
void survive_data_cb( struct SurviveUSBInterface * si );


#endif


