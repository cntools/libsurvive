#ifndef _SURVIVE_INTERNAL_H
#define _SURVIVE_INTERNAL_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>

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


struct SurviveObject
{
	struct SurviveContext * ctx;
	char    codename[4];
	int16_t buttonmask;
	int16_t axis1;
	int16_t axis2;
	int16_t axis3;
	int8_t  charge;
	int8_t  charging:1;
	int8_t  ison:1;
	int sensors;
};

struct SurviveContext
{
	//USB Subsystem
    struct libusb_context* usbctx;
	void(*faultfunction)( struct SurviveContext * ctx, const char * fault );
	void(*notefunction)( struct SurviveContext * ctx, const char * fault );
	struct libusb_device_handle * udev[MAX_USB_DEVS];
	struct SurviveUSBInterface uiface[MAX_INTERFACES];

	//Flood info, for calculating which laser is currently sweeping.
	int8_t oldcode;
	int32_t last_photo_time;
	short total_photos;
	int32_t total_photo_time;
	int32_t total_pulsecode_time;

	//Data Subsystem
	struct SurviveObject headset;
	struct SurviveObject watchman[2];
//	struct SurvivePhoto 
};


//USB Subsystem 
void survive_usb_close( struct SurviveContext * t );
int survive_usb_init( struct SurviveContext * t );
int survive_usb_poll( struct SurviveContext * ctx );

//Accept Data from backend.
void survive_data_cb( struct SurviveUSBInterface * si );

//Accept higher-level data.
void survive_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode );
void survive_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id );


#endif


